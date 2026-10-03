// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/tsdf/hierarchical_tsdf_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

#include "volumetric_kit/recon/core/command_batch.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"

#include "hierarchical_classify_comp.spv.hpp"
#include "hierarchical_integrate_comp.spv.hpp"

namespace volumetric_kit::recon::tsdf {
namespace {

// Translate host allocation failures as well as backend allocation errors.
// Exception-disabled consumers still compile the same public API.
template <typename F>
auto allocation_boundary(F&& operation) -> decltype(operation()) {
#if defined(__cpp_exceptions) || defined(__EXCEPTIONS) || defined(_CPPUNWIND)
  try {
    return operation();
  } catch (const std::bad_alloc&) {
    return Status::out_of_memory({});
  } catch (const std::length_error&) {
    return Status::out_of_memory({});
  }
#else
  return operation();
#endif
}

struct IntegratePush {
  volume::VoxelGridParams grid;
  std::uint32_t count;
  float max_weight;
  std::uint32_t mode;
  std::uint32_t has_color;
  std::uint32_t has_color_attr;
  std::uint32_t tick;
  std::uint32_t coverage_in_alpha;
  std::uint32_t first_leaf;
};
static_assert(sizeof(IntegratePush) == 64, "TSDF scalar push ABI");
static_assert(offsetof(IntegratePush, coverage_in_alpha) == 56,
              "TSDF scalar push ABI");

struct ClassifyPush {
  volume::VoxelGridParams root_grid;
  float finest_voxel_size;
  float surface_error;
  float noise_floor;
  std::uint32_t max_level;
  std::uint32_t node_capacity;
  std::uint32_t pixel_stride;
  std::uint32_t patch_radius;
  std::uint32_t columns;
  std::uint32_t sample_count;
};
static_assert(sizeof(ClassifyPush) == 68, "classify scalar push ABI");
static_assert(offsetof(ClassifyPush, sample_count) == 64,
              "classify scalar push ABI");

StorageInput color_input(const ColorFrame& color) {
  return color.buffer != nullptr ? StorageInput(*color.buffer)
                                 : StorageInput(color.pixels);
}

Status check_camera(const DepthCameraParams& cam) {
  if (!std::isfinite(cam.fx) || !std::isfinite(cam.fy) || cam.fx <= 0 ||
      cam.fy <= 0 || !std::isfinite(cam.cx) || !std::isfinite(cam.cy) ||
      !std::isfinite(cam.min_depth) || !std::isfinite(cam.max_depth) ||
      cam.min_depth < 0 || cam.max_depth <= cam.min_depth) {
    return Status::invalid_argument("hierarchical TSDF: invalid depth camera");
  }
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(cam.cam_to_world[c][r])) {
        return Status::invalid_argument("hierarchical TSDF: nonfinite pose");
      }
    }
  }
  return {};
}

struct FrameSizes {
  std::vector<VkDeviceSize> depth;
  std::vector<VkDeviceSize> color;
  std::vector<std::size_t> live;
};

Result<FrameSizes> check_frames(const std::vector<FrameInput>& frames,
                                const volume::HierarchicalFieldView& field,
                                VkDeviceSize max_range, bool use_color) {
  FrameSizes sizes;
  if (frames.size() > std::numeric_limits<std::uint32_t>::max()) {
    return Status::invalid_argument("hierarchical TSDF: too many frames");
  }
  sizes.depth.resize(frames.size());
  sizes.color.resize(frames.size());
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto& cam = frames[i].camera;
    const std::uint64_t pixels = std::uint64_t(cam.width) * cam.height;
    if (pixels > std::numeric_limits<std::uint32_t>::max()) {
      return Status::invalid_argument("hierarchical TSDF: image is too large");
    }
    sizes.depth[i] = pixels * sizeof(float);
    VR_TRY(frames[i].depth.check("hierarchical TSDF: depth", sizes.depth[i]));
    VR_TRY(check_storage_buffer_range("hierarchical TSDF: depth",
                                      sizes.depth[i], max_range));
    if (pixels == 0) continue;
    VR_TRY(check_camera(cam));
    sizes.live.push_back(i);
    const ColorFrame* color = frames[i].color;
    if (!use_color || color == nullptr) continue;
    if (field.color == nullptr || color->cam.width == 0 ||
        color->cam.height == 0 ||
        (color->pixels == nullptr) == (color->buffer == nullptr) ||
        !is_canonical(color->encoding) || !std::isfinite(color->cam.fx) ||
        !std::isfinite(color->cam.fy) || color->cam.fx <= 0 ||
        color->cam.fy <= 0 || !std::isfinite(color->cam.cx) ||
        !std::isfinite(color->cam.cy)) {
      return Status::invalid_argument("hierarchical TSDF: invalid color frame");
    }
    for (int c = 0; c < 4; ++c) {
      for (int r = 0; r < 4; ++r) {
        if (!std::isfinite(color->cam.cam_to_world[c][r])) {
          return Status::invalid_argument(
              "hierarchical TSDF: nonfinite color pose");
        }
      }
    }
    const std::uint64_t color_pixels =
        std::uint64_t(color->cam.width) * color->cam.height;
    if (color_pixels > std::numeric_limits<std::uint32_t>::max()) {
      return Status::invalid_argument("hierarchical TSDF: color is too large");
    }
    sizes.color[i] = color_pixels * sizeof(std::uint32_t);
    VR_TRY(
        color_input(*color).check("hierarchical TSDF: color", sizes.color[i]));
    VR_TRY(check_storage_buffer_range("hierarchical TSDF: color",
                                      sizes.color[i], max_range));
  }
  return sizes;
}

Status check_field(const volume::HierarchicalFieldView& field,
                   VkDeviceSize max_range) {
  VR_TRY(field.validate());
  for (const Buffer* b : {field.root_hash, field.nodes, field.leaf_indices,
                          field.tsdf, field.weight, field.color}) {
    if (b != nullptr) {
      VR_TRY(check_storage_buffer_range("hierarchical TSDF: field buffer",
                                        b->size(), max_range));
    }
  }
  return {};
}

}  // namespace

struct HierarchicalTsdfIntegrator::Impl {
  Device* device = nullptr;
  Allocator* allocator = nullptr;
  DescriptorPool pool;
  ComputeKernel integrate;
  ComputeKernel classify;
  KernelSets integrate_sets;
  KernelSets classify_sets;
  Buffer camera;
  Buffer color_camera;
  Buffer dummy;
  Buffer requests;
  GpuTimer timer;
  VkDeviceSize max_range = 0;
  std::uint32_t max_groups = 0;
};

HierarchicalTsdfIntegrator::HierarchicalTsdfIntegrator(
    std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}
HierarchicalTsdfIntegrator::HierarchicalTsdfIntegrator() noexcept = default;
HierarchicalTsdfIntegrator::~HierarchicalTsdfIntegrator() = default;
HierarchicalTsdfIntegrator::HierarchicalTsdfIntegrator(
    HierarchicalTsdfIntegrator&& other) noexcept = default;
HierarchicalTsdfIntegrator& HierarchicalTsdfIntegrator::operator=(
    HierarchicalTsdfIntegrator&& other) noexcept {
  if (this != &other) impl_ = std::move(other.impl_);
  return *this;
}
bool HierarchicalTsdfIntegrator::valid() const noexcept {
  return impl_ != nullptr;
}

Result<HierarchicalTsdfIntegrator> HierarchicalTsdfIntegrator::create(
    Device& device, Allocator& allocator) {
  return allocation_boundary([&]() -> Result<HierarchicalTsdfIntegrator> {
    if (device.handle() == VK_NULL_HANDLE ||
        device.physical_device() == VK_NULL_HANDLE || !allocator.valid()) {
      return Status::invalid_argument(
          "hierarchical TSDF: invalid device or allocator");
    }
    auto p = std::make_unique<Impl>();
    p->device = &device;
    p->allocator = &allocator;
    VkPushConstantRange integrate_push{};
    integrate_push.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    integrate_push.size = sizeof(IntegratePush);
    VkPushConstantRange classify_push = integrate_push;
    classify_push.size = sizeof(ClassifyPush);
    KernelSetBuilder builder(device);
    VR_TRY(builder.add(p->integrate, "hierarchical_integrate",
                       vr_hierarchical_integrate_comp_spv,
                       vr_hierarchical_integrate_comp_spv_size, 9,
                       &integrate_push));
    VR_TRY(builder.add(
        p->classify, "hierarchical_classify", vr_hierarchical_classify_comp_spv,
        vr_hierarchical_classify_comp_spv_size, 5, &classify_push));
    VR_ASSIGN(p->pool, builder.build());
    VR_ASSIGN(p->camera,
              device_storage_buffer(allocator, sizeof(DepthCameraParams)));
    VR_ASSIGN(p->color_camera,
              device_storage_buffer(allocator, sizeof(ColorCameraParams)));
    VR_ASSIGN(p->dummy,
              device_storage_buffer(allocator, sizeof(std::uint32_t)));
    VR_ASSIGN(p->timer, GpuTimer::create(device));
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(device.physical_device(), &props);
    p->max_range = props.limits.maxStorageBufferRange;
    p->max_groups = props.limits.maxComputeWorkGroupCount[0];
    return HierarchicalTsdfIntegrator(std::move(p));
  });
}

Status HierarchicalTsdfIntegrator::integrate(
    const volume::HierarchicalFieldView& field,
    const std::vector<FrameInput>& frames, float max_weight,
    IntegrationMode mode, StageMetrics* metrics) {
  return allocation_boundary([&]() -> Status {
    if (!valid()) {
      return Status::invalid_argument("hierarchical TSDF: empty integrator");
    }
    auto& p = *impl_;
    GpuStageScope stage(metrics, p.timer, "hierarchical integrate");
    VR_TRY(check_field(field, p.max_range));
    if (!std::isfinite(max_weight) || !(max_weight > 0.0f) ||
        (mode != IntegrationMode::Classic &&
         mode != IntegrationMode::Dynamic)) {
      return Status::invalid_argument(
          "hierarchical TSDF: invalid fusion controls");
    }
    VR_ASSIGN(const FrameSizes sizes,
              check_frames(frames, field, p.max_range, true));
    if (sizes.live.empty() || field.leaf_count == 0) return {};
    // Each leaf owns two workgroups; keep chunks leaf-aligned and portable to
    // devices exposing Vulkan's minimum dispatch limit.
    const std::uint32_t chunk_leaves = p.max_groups / 2u;
    if (chunk_leaves == 0)
      return Status::invalid_argument(
          "hierarchical TSDF: invalid dispatch limit");
    const auto frame_count = static_cast<std::uint32_t>(sizes.live.size());
    const std::uint64_t dispatches =
        std::uint64_t(frame_count) *
        group_count(field.leaf_count, chunk_leaves);
    if (dispatches > std::numeric_limits<std::uint32_t>::max()) {
      return Status::invalid_argument("hierarchical TSDF: too many dispatches");
    }
    VR_TRY(p.integrate_sets.reserve(*p.device, p.integrate, frame_count));
    VR_TRY(p.timer.reserve(*p.device, static_cast<std::uint32_t>(dispatches)));
    std::vector<Buffer> depth_uploads(frames.size());
    std::vector<Buffer> color_uploads(frames.size());
    CommandBatch batch(*p.device, *p.allocator);
    auto grid = field.root_grid;
    grid.voxel_size = field.finest_voxel_size;
    for (std::size_t k = 0; k < sizes.live.size(); ++k) {
      const std::size_t i = sizes.live[k];
      const auto& frame = frames[i];
      const auto& set = p.integrate_sets[k];
      VR_ASSIGN(const VkBuffer depth,
                frame.depth.buffer(batch, *p.allocator, sizes.depth[i],
                                   depth_uploads[i]));
      VR_TRY(
          batch.upload(p.camera, 0, &frame.camera, sizeof(DepthCameraParams)));
      set.write_storage_buffer(0, field.tsdf->handle(), 0, VK_WHOLE_SIZE);
      set.write_storage_buffer(1, field.weight->handle(), 0, VK_WHOLE_SIZE);
      set.write_storage_buffer(
          2, field.leaf_indices->handle(), 0,
          VkDeviceSize(field.leaf_count) * sizeof(std::uint32_t));
      set.write_storage_buffer(3, depth, 0, sizes.depth[i]);
      set.write_storage_buffer(4, p.camera.handle(), 0, VK_WHOLE_SIZE);
      set.write_storage_buffer(
          6, field.color ? field.color->handle() : p.dummy.handle(), 0,
          VK_WHOLE_SIZE);
      set.write_storage_buffer(7, p.color_camera.handle(), 0, VK_WHOLE_SIZE);
      set.write_storage_buffer(8, field.nodes->handle(), 0, VK_WHOLE_SIZE);
      if (frame.color != nullptr) {
        VR_ASSIGN(
            const VkBuffer color,
            color_input(*frame.color)
                .buffer(batch, *p.allocator, sizes.color[i], color_uploads[i]));
        set.write_storage_buffer(5, color, 0, sizes.color[i]);
        VR_TRY(batch.upload(p.color_camera, 0, &frame.color->cam,
                            sizeof(ColorCameraParams)));
      } else {
        set.write_storage_buffer(5, p.dummy.handle(), 0, VK_WHOLE_SIZE);
      }
      IntegratePush push{
          grid,
          field.leaf_count,
          max_weight,
          static_cast<std::uint32_t>(mode),
          frame.color != nullptr ? 1u : 0u,
          field.color != nullptr ? 1u : 0u,
          0u,
          frame.color != nullptr && frame.color->coverage_in_alpha ? 1u : 0u,
          0u};
      while (push.first_leaf < field.leaf_count) {
        push.count = std::min(chunk_leaves, field.leaf_count - push.first_leaf);
        VR_TRY(batch.dispatch(p.integrate, set, &push, sizeof(push),
                              push.count * 2u, p.max_groups, &stage));
        push.first_leaf += push.count;
      }
    }
    return batch.submit();
  });
}

Result<const Buffer*> HierarchicalTsdfIntegrator::classify(
    const volume::HierarchicalFieldView& field,
    const std::vector<FrameInput>& frames,
    const HierarchicalRefinementParams& params, StageMetrics* metrics) {
  return allocation_boundary([&]() -> Result<const Buffer*> {
    if (!valid()) {
      return Status::invalid_argument("hierarchical TSDF: empty integrator");
    }
    auto& p = *impl_;
    GpuStageScope stage(metrics, p.timer, "hierarchical classify");
    VR_TRY(check_field(field, p.max_range));
    if (!std::isfinite(params.surface_error) || !(params.surface_error > 0) ||
        !std::isfinite(params.noise_floor) || params.noise_floor < 0 ||
        params.pixel_stride == 0 || params.pixel_stride > 64 ||
        params.patch_radius == 0 || params.patch_radius > 64) {
      return Status::invalid_argument(
          "hierarchical TSDF: invalid refinement controls");
    }
    VR_ASSIGN(const FrameSizes sizes,
              check_frames(frames, field, p.max_range, false));
    const VkDeviceSize request_bytes = VkDeviceSize(field.node_capacity) * 4u;
    if (p.requests.size() < request_bytes) {
      VR_ASSIGN(p.requests, device_storage_buffer(*p.allocator, request_bytes));
    }
    const auto frame_count = static_cast<std::uint32_t>(sizes.live.size());
    if (frame_count != 0) {
      VR_TRY(p.classify_sets.reserve(*p.device, p.classify, frame_count));
    }
    VR_TRY(p.timer.reserve(*p.device, frame_count));
    // Validate every dispatch before recording even the request reset.
    for (std::size_t i : sizes.live) {
      const auto& cam = frames[i].camera;
      const std::uint64_t columns = group_count(cam.width, params.pixel_stride);
      const std::uint64_t rows = group_count(cam.height, params.pixel_stride);
      if (columns * rows > std::numeric_limits<std::uint32_t>::max() ||
          group_count(static_cast<std::uint32_t>(columns * rows), 256) >
              p.max_groups) {
        return Status::invalid_argument(
            "hierarchical TSDF: classifier exceeds device limit");
      }
    }
    std::vector<Buffer> uploads(frames.size());
    CommandBatch batch(*p.device, *p.allocator);
    VR_TRY(batch.fill(p.requests, 0, request_bytes, UINT32_MAX));
    for (std::size_t k = 0; k < sizes.live.size(); ++k) {
      const std::size_t i = sizes.live[k];
      const auto& frame = frames[i];
      const auto& set = p.classify_sets[k];
      VR_ASSIGN(
          const VkBuffer depth,
          frame.depth.buffer(batch, *p.allocator, sizes.depth[i], uploads[i]));
      VR_TRY(
          batch.upload(p.camera, 0, &frame.camera, sizeof(DepthCameraParams)));
      set.write_storage_buffer(0, depth, 0, sizes.depth[i]);
      set.write_storage_buffer(1, p.camera.handle(), 0, VK_WHOLE_SIZE);
      set.write_storage_buffer(2, p.requests.handle(), 0, request_bytes);
      set.write_storage_buffer(3, field.nodes->handle(), 0, VK_WHOLE_SIZE);
      set.write_storage_buffer(4, field.root_hash->handle(), 0, VK_WHOLE_SIZE);
      const auto columns = group_count(frame.camera.width, params.pixel_stride);
      const auto rows = group_count(frame.camera.height, params.pixel_stride);
      const ClassifyPush push{
          field.root_grid,     field.finest_voxel_size, params.surface_error,
          params.noise_floor,  field.max_level,         field.node_capacity,
          params.pixel_stride, params.patch_radius,     columns,
          columns * rows};
      VR_TRY(batch.dispatch(p.classify, set, &push, sizeof(push),
                            group_count(push.sample_count, 256), p.max_groups,
                            &stage));
    }
    VR_TRY(batch.submit());
    return &p.requests;
  });
}

}  // namespace volumetric_kit::recon::tsdf
