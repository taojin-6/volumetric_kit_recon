// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/volume/hierarchical_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

#include "volumetric_kit/recon/core/command_batch.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/volume/hierarchy_layout.hpp"

#include "hierarchical_leaves_comp.spv.hpp"
#include "hierarchical_merge_candidates_comp.spv.hpp"
#include "hierarchical_merge_comp.spv.hpp"
#include "hierarchical_roots_comp.spv.hpp"
#include "hierarchical_split_comp.spv.hpp"

namespace volumetric_kit::recon::volume {
namespace {
// Keep allocation failures inside the public Status/Result boundary.
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

// Keep every descendant and a neighboring root's sample arithmetic inside
// the same signed finest-cell domain used by the adaptive lookup/classifier.
constexpr std::int64_t kFinestCellLimit = std::int64_t{1} << 30;

Status check_depth_domain(const DepthCameraParams& cam,
                          const HierarchicalGridConfig& config) {
  if (cam.width == 0 || cam.height == 0) return {};
  if (!(cam.fx > 0) || !(cam.fy > 0) || !std::isfinite(cam.fx) ||
      !std::isfinite(cam.fy) || !std::isfinite(cam.cx) ||
      !std::isfinite(cam.cy) || !std::isfinite(cam.min_depth) ||
      !std::isfinite(cam.max_depth) || cam.min_depth < 0 ||
      !(cam.max_depth > cam.min_depth)) {
    return Status::invalid_argument("HierarchicalGrid: invalid depth camera");
  }
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      if (!std::isfinite(cam.cam_to_world[col][row])) {
        return Status::invalid_argument(
            "HierarchicalGrid: nonfinite camera pose");
      }
    }
  }
  const std::int64_t root_cells = std::int64_t{8} << (config.level_count - 1);
  const double spacing = config.finest.voxel_size;
  // A band is dilated in whole root blocks. Leave two extra roots for its
  // rounding/padding plus the neighboring sample queries at extraction time.
  const double limit = double(kFinestCellLimit - 4 * root_cells) * spacing -
                       double(config.finest.trunc_dist);
  if (!(limit > 0)) {
    return Status::invalid_argument(
        "HierarchicalGrid: allocation band exceeds coordinate domain");
  }
  // The complete pinhole frustum is convex. Bounding its eight corners also
  // bounds every valid depth sample, including device-resident inputs, with
  // constant host work and no depth-image readback.
  for (double z : {0.0, double(cam.max_depth)}) {
    for (double v : {0.0, double(cam.height)}) {
      for (double u : {0.0, double(cam.width)}) {
        const double local[3]{(u - double(cam.cx)) * z / double(cam.fx),
                              (v - double(cam.cy)) * z / double(cam.fy), z};
        for (int row = 0; row < 3; ++row) {
          double world = cam.cam_to_world[3][row];
          for (int col = 0; col < 3; ++col)
            world += double(cam.cam_to_world[col][row]) * local[col];
          if (!std::isfinite(world) || !(std::abs(world) < limit)) {
            return Status::invalid_argument(
                "HierarchicalGrid: depth frustum exceeds coordinate domain");
          }
        }
      }
    }
  }
  return {};
}

struct RootPush {
  std::uint32_t count;
  std::uint32_t max_level;
};
struct SplitPush {
  std::uint32_t base;
  std::uint32_t root_capacity;
  std::uint32_t group_capacity;
  std::uint32_t max_splits;
  std::uint32_t has_color;
  float weight_cap;
};
struct MergeCandidatePush {
  std::uint32_t capacity;
  std::uint32_t stable_updates;
  std::uint32_t max_merges;
};
struct MergePush {
  std::uint32_t base;
  std::uint32_t root_capacity;
  std::uint32_t has_color;
};
void bind(const ComputeKernel& kernel, std::uint32_t binding,
          const Buffer& buffer) {
  kernel.set.write_storage_buffer(binding, buffer.handle(), 0, buffer.size());
}
}  // namespace

struct HierarchicalGrid::Impl {
  Impl(Device& device_in, Allocator& allocator_in,
       const HierarchicalGridConfig& config_in, VoxelHashMap roots_in)
      : device(device_in),
        allocator(allocator_in),
        config(config_in),
        roots(std::move(roots_in)) {}
  Device& device;
  Allocator& allocator;
  HierarchicalGridConfig config;
  VoxelHashMap roots;
  Buffer nodes, leaves, tsdf, weight, color, leaf_counter, group_counter, stats;
  Buffer free_groups, merge_ages, merge_candidates, merge_dispatch;
  DescriptorPool pool;
  ComputeKernel root_kernel, leaf_kernel, split_kernel;
  ComputeKernel merge_candidate_kernel, merge_kernel;
  GpuTimer timer;
  std::uint64_t generation = 1;
  std::uint32_t capacity = 0;
  std::uint32_t leaf_count = 0;
  std::array<std::uint32_t, 4> leaf_counts{};
  std::uint32_t known_root_count = 0;
  std::uint32_t max_groups = 0;
  bool leaves_dirty = true;

  void invalidate() { generation = generation + 1 == 0 ? 1 : generation + 1; }
  HierarchicalFieldView view() const {
    auto physical_grid = roots.grid();
    physical_grid.trunc_dist = config.finest.trunc_dist;
    return HierarchicalFieldView{&roots.entries(),
                                 &nodes,
                                 &leaves,
                                 &tsdf,
                                 &weight,
                                 config.color ? &color : nullptr,
                                 physical_grid,
                                 capacity,
                                 leaf_count,
                                 config.level_count - 1,
                                 config.finest.voxel_size,
                                 generation,
                                 &generation};
  }
  Status check_requests(const Buffer& requests) const {
    if (!requests.valid() ||
        (requests.usage() & VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) == 0 ||
        requests.size() < VkDeviceSize(capacity) * sizeof(std::uint32_t)) {
      return Status::invalid_argument(
          "HierarchicalGrid: invalid desired-level buffer");
    }
    return {};
  }
  Status record_split(CommandBatch& batch, const Buffer& requests,
                      std::uint32_t max_splits, float transfer_weight_cap,
                      std::uint32_t* counters, GpuStageScope* stage) {
    Impl& p = *this;
    const VkDeviceSize request_bytes =
        VkDeviceSize(capacity) * sizeof(std::uint32_t);
    bind(p.split_kernel, 0, p.nodes);
    bind(p.split_kernel, 1, p.leaves);
    // External storage may be larger than maxStorageBufferRange. Bind only
    // its required prefix, which is smaller than the sample array checked
    // against that device limit at grid creation.
    p.split_kernel.set.write_storage_buffer(2, requests.handle(), 0,
                                            request_bytes);
    bind(p.split_kernel, 3, p.tsdf);
    bind(p.split_kernel, 4, p.weight);
    bind(p.split_kernel, 5, p.color);
    bind(p.split_kernel, 6, p.group_counter);
    bind(p.split_kernel, 7, p.stats);
    bind(p.split_kernel, 8, p.free_groups);
    bind(p.split_kernel, 9, p.merge_ages);
    VR_TRY(batch.zero(p.stats, 0, 5 * sizeof(std::uint32_t)));
    SplitPush push{0,
                   static_cast<std::uint32_t>(p.config.finest.num_blocks),
                   p.config.child_block_capacity / 8u,
                   max_splits,
                   p.config.color ? 1u : 0u,
                   transfer_weight_cap};
    while (push.base < p.leaf_count) {
      const auto groups = std::min(p.max_groups, p.leaf_count - push.base);
      VR_TRY(batch.dispatch(p.split_kernel, &push, sizeof(push), groups,
                            p.max_groups, stage));
      push.base += groups;
    }
    VR_TRY(batch.readback(p.stats, 0, 5 * sizeof(std::uint32_t), counters));
    return {};
  }
  Status record_merge(CommandBatch& batch, const Buffer& requests,
                      std::uint32_t max_merges, std::uint32_t stable_updates,
                      std::uint32_t* counters, GpuStageScope* stage) {
    Impl& p = *this;
    const VkDeviceSize request_bytes =
        VkDeviceSize(capacity) * sizeof(std::uint32_t);
    bind(p.merge_candidate_kernel, 0, p.nodes);
    p.merge_candidate_kernel.set.write_storage_buffer(1, requests.handle(), 0,
                                                      request_bytes);
    bind(p.merge_candidate_kernel, 2, p.merge_ages);
    bind(p.merge_candidate_kernel, 3, p.merge_candidates);
    bind(p.merge_candidate_kernel, 4, p.stats);
    bind(p.merge_candidate_kernel, 5, p.merge_dispatch);
    bind(p.merge_kernel, 0, p.nodes);
    bind(p.merge_kernel, 1, p.merge_candidates);
    bind(p.merge_kernel, 2, p.tsdf);
    bind(p.merge_kernel, 3, p.weight);
    bind(p.merge_kernel, 4, p.color);
    bind(p.merge_kernel, 5, p.group_counter);
    bind(p.merge_kernel, 6, p.free_groups);
    bind(p.merge_kernel, 7, p.stats);
    bind(p.merge_kernel, 8, p.merge_ages);
    VR_TRY(batch.zero(p.stats, 0, 5 * sizeof(std::uint32_t)));
    const std::uint32_t dispatch_args[3]{0, 1, 1};
    VR_TRY(batch.upload(p.merge_dispatch, 0, dispatch_args,
                        sizeof(dispatch_args)));
    // The candidate kernel bounds the indirect dispatch without a host count
    // readback. Larger requested budgets continue through later calls.
    const MergeCandidatePush candidate_push{p.capacity, stable_updates,
                                            std::min(max_merges, p.max_groups)};
    VR_TRY(batch.dispatch(p.merge_candidate_kernel, &candidate_push,
                          sizeof(candidate_push), (p.capacity + 255u) / 256u,
                          p.max_groups, stage));
    MergePush push{0, static_cast<std::uint32_t>(p.config.finest.num_blocks),
                   p.config.color ? 1u : 0u};
    VR_TRY(batch.dispatch_indirect(p.merge_kernel, &push, sizeof(push),
                                   p.merge_dispatch, 0, stage));
    VR_TRY(batch.readback(p.stats, 0, 5 * sizeof(std::uint32_t), counters));
    return {};
  }
  Status reset_storage() {
    invalidate();
    leaves_dirty = true;
    leaf_count = 0;
    leaf_counts = {};
    known_root_count = 0;
    CommandBatch batch(device, allocator);
    // ptr=-1 is the absence marker; initialization assigns every live member.
    VR_TRY(batch.fill(nodes, 0, nodes.size(), 0xffffffffu));
    VR_TRY(batch.zero(tsdf, 0, tsdf.size()));
    VR_TRY(batch.zero(weight, 0, weight.size()));
    VR_TRY(batch.zero(color, 0, color.size()));
    VR_TRY(batch.zero(leaf_counter, 0, leaf_counter.size()));
    const std::uint32_t groups = config.child_block_capacity / 8u;
    std::vector<std::uint32_t> group_ids(groups);
    for (std::uint32_t i = 0; i < groups; ++i) group_ids[i] = i;
    if (groups != 0) {
      VR_TRY(batch.upload(free_groups, 0, group_ids.data(),
                          VkDeviceSize(groups) * sizeof(std::uint32_t)));
    }
    VR_TRY(batch.upload(group_counter, 0, &groups, sizeof(groups)));
    VR_TRY(batch.zero(merge_ages, 0, merge_ages.size()));
    return batch.submit();
  }
};

HierarchicalGrid::HierarchicalGrid(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}
HierarchicalGrid::HierarchicalGrid() noexcept = default;
HierarchicalGrid::~HierarchicalGrid() = default;
HierarchicalGrid::HierarchicalGrid(HierarchicalGrid&& other) noexcept
    : impl_(std::move(other.impl_)) {
  if (impl_) impl_->invalidate();
}
HierarchicalGrid& HierarchicalGrid::operator=(
    HierarchicalGrid&& other) noexcept {
  if (this != &other) {
    impl_ = std::move(other.impl_);
    if (impl_) impl_->invalidate();
  }
  return *this;
}
bool HierarchicalGrid::valid() const noexcept { return impl_ != nullptr; }
std::uint32_t HierarchicalGrid::node_capacity() const noexcept {
  return impl_ ? impl_->capacity : 0;
}
std::array<std::uint32_t, 4> HierarchicalGrid::leaf_counts() const noexcept {
  return impl_ ? impl_->leaf_counts : std::array<std::uint32_t, 4>{};
}

Result<HierarchicalGrid> HierarchicalGrid::create(
    Device& device, Allocator& allocator,
    const HierarchicalGridConfig& config) {
  return allocation_boundary([&]() -> Result<HierarchicalGrid> {
    if (device.handle() == VK_NULL_HANDLE ||
        device.physical_device() == VK_NULL_HANDLE || !allocator.valid()) {
      return Status::invalid_argument(
          "HierarchicalGrid: invalid device or allocator");
    }
    VR_ASSIGN(const auto layout,
              HierarchyLayout::create(config.finest, config.level_count));
    if (config.level_count > 4 || config.child_block_capacity % 8 != 0) {
      return Status::invalid_argument(
          "HierarchicalGrid: at most 4 levels and whole eight-child groups "
          "required");
    }
    const std::uint64_t capacity =
        static_cast<std::uint64_t>(config.finest.num_blocks) +
        config.child_block_capacity;
    if (capacity * 512u > std::numeric_limits<std::int32_t>::max()) {
      return Status::invalid_argument(
          "HierarchicalGrid: sample-array offsets exceed int32 range");
    }
    const VkDeviceSize max_range =
        max_storage_buffer_range(device.physical_device());
    const VkDeviceSize sample_bytes =
        VkDeviceSize(capacity) * 512u * sizeof(float);
    VR_TRY(check_storage_buffer_range("HierarchicalGrid: sample array",
                                      sample_bytes, max_range));
    VR_TRY(check_storage_buffer_range(
        "HierarchicalGrid: nodes",
        VkDeviceSize(capacity) * sizeof(HierarchicalNode), max_range));
    VR_ASSIGN(auto root_grid, layout.level_grid(config.level_count - 1));
    // The existing allocator chooses a nearest-node block. Its center can be
    // half a coarse voxel beyond the point; padding that amount preserves the
    // full physical band of finer cell-centered descendants at root faces.
    root_grid.trunc_dist += 0.5f * root_grid.voxel_size;
    if (!std::isfinite(root_grid.trunc_dist)) {
      return Status::invalid_argument(
          "HierarchicalGrid: allocation band exceeds float range");
    }
    VR_ASSIGN(auto roots, VoxelHashMap::create(device, allocator, root_grid));
    auto impl = std::unique_ptr<Impl>(
        new (std::nothrow) Impl(device, allocator, config, std::move(roots)));
    if (!impl)
      return Status::out_of_memory("HierarchicalGrid: owner allocation failed");
    impl->capacity = static_cast<std::uint32_t>(capacity);
    VR_ASSIGN(impl->nodes,
              device_storage_buffer(allocator, VkDeviceSize(capacity) *
                                                   sizeof(HierarchicalNode)));
    VR_ASSIGN(impl->leaves,
              device_storage_buffer(
                  allocator, VkDeviceSize(capacity) * sizeof(std::uint32_t)));
    VR_ASSIGN(impl->tsdf, device_storage_buffer(allocator, sample_bytes));
    VR_ASSIGN(impl->weight, device_storage_buffer(allocator, sample_bytes));
    VR_ASSIGN(impl->color, device_storage_buffer(
                               allocator, config.color ? sample_bytes : 4));
    VR_ASSIGN(impl->leaf_counter,
              device_storage_buffer(allocator, 5 * sizeof(std::uint32_t)));
    VR_ASSIGN(impl->group_counter, device_storage_buffer(allocator, 4));
    VR_ASSIGN(
        impl->free_groups,
        device_storage_buffer(
            allocator, std::max<VkDeviceSize>(
                           4, VkDeviceSize(config.child_block_capacity / 8u) *
                                  sizeof(std::uint32_t))));
    VR_ASSIGN(impl->merge_ages,
              device_storage_buffer(
                  allocator, VkDeviceSize(capacity) * sizeof(std::uint32_t)));
    VR_ASSIGN(impl->merge_candidates,
              device_storage_buffer(
                  allocator, VkDeviceSize(capacity) * sizeof(std::uint32_t)));
    VR_ASSIGN(impl->merge_dispatch,
              device_storage_buffer(allocator, 3 * sizeof(std::uint32_t),
                                    VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT));
    VR_ASSIGN(impl->stats,
              device_storage_buffer(allocator, 5 * sizeof(std::uint32_t)));
    const VkPushConstantRange root_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                        sizeof(RootPush)};
    const VkPushConstantRange leaf_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                        sizeof(std::uint32_t)};
    const VkPushConstantRange split_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                         sizeof(SplitPush)};
    const VkPushConstantRange merge_candidate_push{
        VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(MergeCandidatePush)};
    const VkPushConstantRange merge_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                         sizeof(MergePush)};
    KernelSetBuilder builder(device);
    VR_TRY(builder.add(impl->root_kernel, "hierarchical_roots",
                       vr_hierarchical_roots_comp_spv,
                       vr_hierarchical_roots_comp_spv_size, 2, &root_push));
    VR_TRY(builder.add(impl->leaf_kernel, "hierarchical_leaves",
                       vr_hierarchical_leaves_comp_spv,
                       vr_hierarchical_leaves_comp_spv_size, 3, &leaf_push));
    VR_TRY(builder.add(impl->split_kernel, "hierarchical_split",
                       vr_hierarchical_split_comp_spv,
                       vr_hierarchical_split_comp_spv_size, 10, &split_push));
    VR_TRY(builder.add(impl->merge_candidate_kernel,
                       "hierarchical_merge_candidates",
                       vr_hierarchical_merge_candidates_comp_spv,
                       vr_hierarchical_merge_candidates_comp_spv_size, 6,
                       &merge_candidate_push));
    VR_TRY(builder.add(impl->merge_kernel, "hierarchical_merge",
                       vr_hierarchical_merge_comp_spv,
                       vr_hierarchical_merge_comp_spv_size, 9, &merge_push));
    VR_ASSIGN(impl->pool, builder.build());
    VR_ASSIGN(impl->timer, GpuTimer::create(device));
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(device.physical_device(), &properties);
    impl->max_groups = properties.limits.maxComputeWorkGroupCount[0];
    VR_TRY(impl->reset_storage());
    return HierarchicalGrid(std::move(impl));
  });
}

Result<std::uint32_t> HierarchicalGrid::allocate_from_depth(
    const std::vector<DepthInput>& frames, AllocFailures* out_failures,
    StageMetrics* metrics) {
  return allocation_boundary([&]() -> Result<std::uint32_t> {
    if (!valid())
      return Status::invalid_argument("HierarchicalGrid: empty grid");
    for (const auto& frame : frames)
      VR_TRY(check_depth_domain(frame.camera, impl_->config));
    impl_->invalidate();  // allocation may partially succeed even on an error
    return impl_->roots.allocate_from_depth(frames, out_failures, metrics);
  });
}
Result<std::uint32_t> HierarchicalGrid::allocate_roots(const BlockIndex* coords,
                                                       std::uint32_t count) {
  return allocation_boundary([&]() -> Result<std::uint32_t> {
    if (!valid())
      return Status::invalid_argument("HierarchicalGrid: empty grid");
    if (coords == nullptr && count != 0) {
      return Status::invalid_argument(
          "HierarchicalGrid: null root coordinates");
    }
    const std::int64_t root_cells = std::int64_t{8}
                                    << (impl_->config.level_count - 1);
    const std::int64_t max_coord = kFinestCellLimit / root_cells - 2;
    for (std::uint32_t i = 0; i < count; ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(std::int64_t(coords[i].coord[axis])) > max_coord) {
          return Status::invalid_argument(
              "HierarchicalGrid: root exceeds coordinate domain");
        }
      }
    }
    impl_->invalidate();
    return impl_->roots.allocate(coords, count);
  });
}

Result<HierarchicalFieldView> HierarchicalGrid::prepare_leaves(
    StageMetrics* metrics) {
  return allocation_boundary([&]() -> Result<HierarchicalFieldView> {
    if (!valid())
      return Status::invalid_argument("HierarchicalGrid: empty grid");
    Impl& p = *impl_;
    VR_ASSIGN(const auto roots,
              p.roots.compact_active_blocks_on_device(metrics));
    if (!p.leaves_dirty && roots.count == p.known_root_count) return p.view();
    GpuStageScope stage(metrics, p.timer, "hierarchy leaves");
    CommandBatch batch(p.device, p.allocator);
    if (roots.count != 0 && roots.count != p.known_root_count) {
      bind(p.root_kernel, 0, *roots.buffer);
      bind(p.root_kernel, 1, p.nodes);
      const RootPush push{roots.count, p.config.level_count - 1};
      VR_TRY(batch.dispatch(p.root_kernel, &push, sizeof(push),
                            (roots.count + 255u) / 256u, p.max_groups, &stage));
    }
    bind(p.leaf_kernel, 0, p.nodes);
    bind(p.leaf_kernel, 1, p.leaves);
    bind(p.leaf_kernel, 2, p.leaf_counter);
    VR_TRY(batch.zero(p.leaf_counter, 0, p.leaf_counter.size()));
    VR_TRY(batch.dispatch(p.leaf_kernel, &p.capacity, sizeof(p.capacity),
                          (p.capacity + 255u) / 256u, p.max_groups, &stage));
    std::uint32_t leaf_counts[5]{};
    VR_TRY(batch.readback(p.leaf_counter, 0, sizeof(leaf_counts), leaf_counts));
    p.invalidate();
    VR_TRY(batch.submit());
    p.leaf_count = leaf_counts[0];
    std::copy(leaf_counts + 1, leaf_counts + 5, p.leaf_counts.begin());
    p.known_root_count = roots.count;
    p.leaves_dirty = false;
    return p.view();
  });
}

Result<HierarchicalSplitStats> HierarchicalGrid::split(
    const Buffer& requests, std::uint32_t max_splits, float transfer_weight_cap,
    StageMetrics* metrics) {
  return allocation_boundary([&]() -> Result<HierarchicalSplitStats> {
    if (!valid())
      return Status::invalid_argument("HierarchicalGrid: empty grid");
    Impl& p = *impl_;
    VR_TRY(p.check_requests(requests));
    if (!(transfer_weight_cap > 0.0f) || !std::isfinite(transfer_weight_cap)) {
      return Status::invalid_argument("HierarchicalGrid: invalid split input");
    }
    if (max_splits == 0) return HierarchicalSplitStats{};
    VR_ASSIGN(const auto field, prepare_leaves(metrics));
    if (field.leaf_count == 0) return HierarchicalSplitStats{};
    GpuStageScope stage(metrics, p.timer, "hierarchy split");
    std::uint32_t counters[5]{};
    CommandBatch batch(p.device, p.allocator);
    VR_TRY(p.record_split(batch, requests, max_splits, transfer_weight_cap,
                          counters, &stage));
    p.invalidate();
    p.leaves_dirty = true;  // a failed submission may have changed topology
    VR_TRY(batch.submit());
    if (counters[0] == 0) p.leaves_dirty = false;
    return HierarchicalSplitStats{counters[0], counters[1], counters[2],
                                  counters[3]};
  });
}
Result<HierarchicalMergeStats> HierarchicalGrid::merge(
    const Buffer& requests, std::uint32_t max_merges,
    std::uint32_t stable_updates, StageMetrics* metrics) {
  return allocation_boundary([&]() -> Result<HierarchicalMergeStats> {
    if (!valid())
      return Status::invalid_argument("HierarchicalGrid: empty grid");
    Impl& p = *impl_;
    VR_TRY(p.check_requests(requests));
    if (stable_updates == 0) {
      return Status::invalid_argument("HierarchicalGrid: invalid merge input");
    }
    if (max_merges == 0) return HierarchicalMergeStats{};
    VR_ASSIGN(const auto field, prepare_leaves(metrics));
    if (field.leaf_count == 0) return HierarchicalMergeStats{};
    GpuStageScope stage(metrics, p.timer, "hierarchy merge");
    std::uint32_t counters[5]{};
    CommandBatch batch(p.device, p.allocator);
    VR_TRY(p.record_merge(batch, requests, max_merges, stable_updates, counters,
                          &stage));
    p.invalidate();
    p.leaves_dirty = true;
    VR_TRY(batch.submit());
    if (counters[0] == 0) p.leaves_dirty = false;
    return HierarchicalMergeStats{counters[0], counters[1], counters[2]};
  });
}

Result<HierarchicalTopologyStats> HierarchicalGrid::update_topology(
    const Buffer& requests, std::uint32_t max_splits, std::uint32_t max_merges,
    std::uint32_t stable_updates, float transfer_weight_cap,
    StageMetrics* metrics) {
  return allocation_boundary([&]() -> Result<HierarchicalTopologyStats> {
    if (!valid())
      return Status::invalid_argument("HierarchicalGrid: empty grid");
    Impl& p = *impl_;
    VR_TRY(p.check_requests(requests));
    if (stable_updates == 0 || !(transfer_weight_cap > 0.0f) ||
        !std::isfinite(transfer_weight_cap)) {
      return Status::invalid_argument(
          "HierarchicalGrid: invalid topology input");
    }
    if (max_splits == 0 && max_merges == 0) return HierarchicalTopologyStats{};
    VR_ASSIGN(const auto field, prepare_leaves(metrics));
    if (field.leaf_count == 0) return HierarchicalTopologyStats{};
    GpuStageScope stage(metrics, p.timer, "hierarchy update");
    std::uint32_t split_counters[5]{}, merge_counters[5]{};
    CommandBatch batch(p.device, p.allocator);
    if (max_splits != 0) {
      VR_TRY(p.record_split(batch, requests, max_splits, transfer_weight_cap,
                            split_counters, &stage));
    }
    if (max_merges != 0) {
      // The split counters are copied before their storage is reset. Both
      // copies occupy distinct staging slices and reach the host after this
      // single submission. Merge selection reads nodes, not the stale leaves.
      VR_TRY(p.record_merge(batch, requests, max_merges, stable_updates,
                            merge_counters, &stage));
    }
    p.invalidate();
    p.leaves_dirty = true;  // a failed submission may have changed topology
    VR_TRY(batch.submit());
    p.leaves_dirty = split_counters[0] != 0 || merge_counters[0] != 0;
    return HierarchicalTopologyStats{
        {split_counters[0], split_counters[1], split_counters[2],
         split_counters[3]},
        {merge_counters[0], merge_counters[1], merge_counters[2]}};
  });
}

Status HierarchicalGrid::clear() {
  return allocation_boundary([&]() -> Status {
    if (!valid())
      return Status::invalid_argument("HierarchicalGrid: empty grid");
    impl_->invalidate();
    VR_TRY(impl_->roots.clear());
    return impl_->reset_storage();
  });
}
}  // namespace volumetric_kit::recon::volume
