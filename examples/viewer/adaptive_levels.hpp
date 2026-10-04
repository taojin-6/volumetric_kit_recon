// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/adaptive_levels.hpp
/// @brief Uniform grids at halving voxel sizes, refined per block where the
///        depth is systematically off the coarser grid's surface.
///
/// Every level is an ordinary VoxelBlockGrid fused by the existing allocator
/// and integrator. The coarsest is fused from the full depth; a finer level
/// allocates only from the depth whose points fall in coarse blocks flagged
/// for refinement, and integrates the full depth so its blocks still carve
/// free space. A finer block's parent is the coarser block containing it
/// (key / 2), found through the coarser grid's own hash, so no pointer is
/// stored.
///
/// The test is how far the depth sits from a grid's iso-surface. Single
/// frames cannot answer it on a real sensor -- a ToF camera's per-pixel noise
/// is several millimetres and varies with distance, angle and material -- so
/// every set's residuals go into running sums per cell of 2 x 2 x 2 voxels,
/// and a check turns each cell into its systematic offset with the noise
/// removed (adaptive_bias.comp). A real sensor also leaves an offset no grid
/// fits -- per-pixel bias, multipath, registration steps -- about the same at
/// every level: its floor is taken each check as the median coarsest block's
/// offset (most of a room is flat). A block refines when its offset exceeds
/// floor + eps and stands clear of the noise left in the estimate; it
/// coarsens once its offset stays under floor + eps / 2.
///
/// Meshing gives each level the blocks it owns -- the finest level whose
/// ancestors are all refined -- for the existing culled extract. Seams between
/// levels are not stitched. A prototype for the live viewer.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <new>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "adaptive_bias_comp.spv.hpp"
#include "adaptive_mask_comp.spv.hpp"
#include "adaptive_residual_comp.spv.hpp"
#include "adaptive_tint_comp.spv.hpp"
#include "fuse_frame.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace adaptive {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;

/// Resolution and refinement controls.
struct Config {
  float base_voxel = 0.02f;   ///< Coarsest voxel; each finer level halves it.
  int levels = 3;             ///< 2..4 grids.
  float trunc_voxels = 4.0f;  ///< Each grid's band, in its own voxels.
  float max_weight = 20.0f;   ///< Integration weight cap.
  float eps_mm = 1.0f;        ///< Systematic offset that refines a block.
  /// Ignore points seen more obliquely; 0 keeps all. Residuals are measured
  /// along the surface normal, so the angle does not inflate them.
  float max_view_deg = 0.0f;
  std::uint32_t pixel_stride = 4;  ///< Residual sample spacing, pixels.
  std::uint32_t min_cells = 4;     ///< Sampled cells a block needs.
  int check_every = 5;             ///< Frames between checks.
  int calm_checks = 3;  ///< Consecutive calm checks before coarsening.
  /// Dynamic clears surfaces that moved away, for people in the scene.
  vr::tsdf::IntegrationMode mode = vr::tsdf::IntegrationMode::Classic;
};

/// One level's block counts, as of the last check / ownership query.
struct LevelStats {
  float voxel = 0.0f;
  std::size_t blocks = 0;         ///< Allocated.
  std::size_t refined = 0;        ///< Handed to the next level.
  std::size_t owned = 0;          ///< Meshed by this level.
  double median_offset_mm = 0.0;  ///< Median judged block's offset.
  double floor_mm = 0.0;          ///< Offset subtracted as the sensor's.
  double median_noise_mm = 0.0;   ///< Median judged block's per-sample noise.
};

/// One camera's frame of a set: depth in metres on the device, its posed
/// camera, and optional colour (host or device, as `ColorFrame` says).
struct Input {
  std::shared_ptr<const vkc::Buffer> depth;
  vr::DepthCameraParams camera{};
  std::optional<vr::tsdf::ColorFrame> color;
};

/// Block keys and ancestry.
inline std::uint64_t key(vr::Vec3i c) {
  const auto pack = [](int v) {
    return static_cast<std::uint64_t>(v + (1 << 20)) & 0x1fffffu;
  };
  return pack(c.x) | (pack(c.y) << 21) | (pack(c.z) << 42);
}

inline int floor_half(int v) { return v >= 0 ? v / 2 : -((1 - v) / 2); }

inline vr::Vec3i parent(vr::Vec3i c) {
  return {floor_half(c.x), floor_half(c.y), floor_half(c.z)};
}

class Levels {
 public:
  /// @brief Build the grids, integrator and kernels.
  static vkc::Result<std::unique_ptr<Levels>> create(vkc::Device& device,
                                                     vkc::Allocator& allocator,
                                                     const Config& config) {
    if (config.levels < 2 || config.levels > 4 || !(config.base_voxel > 0.0f) ||
        !(config.eps_mm > 0.0f) || config.check_every < 1 ||
        config.pixel_stride < 1) {
      return vkc::Status::invalid_argument("adaptive: invalid configuration");
    }
    std::unique_ptr<Levels> l(new (std::nothrow)
                                  Levels(device, allocator, config));
    if (!l) return vkc::Status::out_of_memory("adaptive: levels");
    VKC_TRY(l->init());
    return l;
  }

  /// @brief A host frame as an @ref Input: its depth copied to the device
  ///        into slot @p slot's buffer, its colour borrowed from the frame.
  vkc::Result<Input> upload(const vr::sensor::CapturedFrame& frame,
                            std::size_t slot) {
    const vr::DepthCameraParams& cam = frame.depth_camera;
    const VkDeviceSize bytes = VkDeviceSize(cam.width) * cam.height * 4u;
    if (host_depth_.size() <= slot) host_depth_.resize(slot + 1);
    std::shared_ptr<vkc::Buffer>& buffer = host_depth_[slot];
    if (!buffer || buffer->size() != bytes) {
      VKC_ASSIGN(vkc::Buffer b, vkc::device_storage_buffer(allocator_, bytes));
      buffer = std::make_shared<vkc::Buffer>(std::move(b));
    }
    vkc::CommandBatch batch(device_, allocator_);
    VKC_TRY(batch.upload(*buffer, 0, frame.depth, bytes));
    VKC_TRY(batch.submit());
    Input in{buffer, cam, std::nullopt};
    if (frame.has_color()) {
      in.color = vr::tsdf::ColorFrame{frame.color, frame.color_camera,
                                      frame.color_encoding};
    }
    return in;
  }

  /// @brief Fuse one set of frames -- a rig's cameras, or one frame -- into
  ///        every level, deciding refinement first when a check is due: per
  ///        level, one allocation and one integration for the whole set.
  vkc::Status fuse(const std::vector<Input>& set, vkc::StageMetrics* metrics) {
    if (set.empty()) return {};
    {
      vkc::StageScope scope(metrics, "adaptive upload");
      while (cameras_.size() < set.size()) {
        VKC_ASSIGN(vkc::Buffer b,
                   vkc::device_storage_buffer(allocator_,
                                              sizeof(vr::DepthCameraParams)));
        cameras_.push_back(std::move(b));
      }
      vkc::CommandBatch batch(device_, allocator_);
      for (std::size_t c = 0; c < set.size(); ++c) {
        VKC_TRY(batch.upload(cameras_[c], 0, &set[c].camera,
                             sizeof(vr::DepthCameraParams)));
      }
      VKC_TRY(batch.submit());
    }
    if (frame_ > 0 && frame_ % std::size_t(config_.check_every) == 0) {
      vkc::StageScope scope(metrics, "adaptive check");
      VKC_TRY(check());
    }
    {
      vkc::StageScope scope(metrics, "adaptive residual");
      VKC_TRY(accumulate_and_mask(set));
    }
    for (std::size_t l = 0; l < grids_.size(); ++l) {
      std::vector<vol::DepthInput> alloc;
      std::vector<vr::tsdf::FrameInput> inputs;
      for (std::size_t c = 0; c < set.size(); ++c) {
        const vkc::Buffer& own = *set[c].depth;
        const vkc::Buffer& from = l == 0 ? own : *masked_[l][c];
        alloc.push_back({vkc::StorageInput(from), set[c].camera});
        inputs.push_back({{vkc::StorageInput(own), set[c].camera},
                          set[c].color ? &*set[c].color : nullptr});
      }
      VKC_TRY(vr_example::allocate_band(grids_[l], alloc, metrics));
      VKC_TRY(integrator_->integrate(grids_[l], inputs, config_.max_weight,
                                     config_.mode, metrics));
    }
    ++frame_;
    return {};
  }

  /// @brief Each level's active blocks it meshes: those whose ancestors are
  ///        all refined (and settled) and which are not refined themselves.
  vkc::Status owned(std::vector<std::vector<vol::BlockIndex>>& out) {
    out.resize(grids_.size());
    for (std::size_t l = 0; l < grids_.size(); ++l) {
      VKC_ASSIGN(const std::vector<vol::BlockIndex> active,
                 grids_[l].map().compact_active_blocks());
      out[l].clear();
      for (const vol::BlockIndex& b : active) {
        const bool ancestors =
            l == 0 || ready_[l - 1].count(key(parent(b.coord))) != 0;
        const bool deeper =
            l + 1 < grids_.size() && ready_[l].count(key(b.coord)) != 0;
        if (ancestors && !deeper) out[l].push_back(b);
      }
      stats_[l].blocks = active.size();
      stats_[l].owned = out[l].size();
    }
    return {};
  }

  /// @brief Paint @p mesh (level @p level's extract) its level's colour.
  vkc::Status tint(const vr::mesh::DeviceMesh& mesh, std::size_t level) {
    if (mesh.vertex_count == 0 || mesh.vertices == VK_NULL_HANDLE) return {};
    static constexpr float kColor[4][3] = {{0.15f, 0.55f, 0.15f},
                                           {0.85f, 0.70f, 0.10f},
                                           {0.80f, 0.15f, 0.10f},
                                           {0.45f, 0.10f, 0.60f}};
    const TintPush push{
        {kColor[level][0], kColor[level][1], kColor[level][2], 1.0f},
        mesh.vertex_count};
    const vkc::ComputeKernel& k = tint_[level];
    k.set.write_storage_buffer(0, mesh.vertices, 0,
                               VkDeviceSize(mesh.vertex_count) * 64u);
    vkc::CommandBatch batch(device_, allocator_);
    VKC_TRY(batch.dispatch(k, &push, sizeof(push),
                           (mesh.vertex_count + 255u) / 256u, max_groups_));
    return batch.submit();
  }

  /// The threshold, changeable between frames (the viewer's slider).
  void set_eps_mm(float eps_mm) noexcept {
    if (eps_mm > 0.0f) config_.eps_mm = eps_mm;
  }
  std::size_t count() const noexcept { return grids_.size(); }
  vol::VoxelBlockGrid& grid(std::size_t l) { return grids_[l]; }
  const std::vector<LevelStats>& stats() const noexcept { return stats_; }

 private:
  struct State {
    bool refined = false;
    std::size_t since = 0;  // check frame it was refined at
    int calm = 0;
  };
  struct ResidualPush {
    vol::VoxelGridParams grid;
    std::uint32_t stride;
    std::uint32_t columns;
    std::uint32_t count;
    float min_cos;
  };
  struct BiasPush {
    std::uint32_t count;
    std::uint32_t base;
    std::uint32_t voxels_per_block;
  };
  struct MaskPush {
    vol::VoxelGridParams coarse;
  };
  struct TintPush {
    float color[4];
    std::uint32_t count;
  };
  // Residual units: 0.02 mm, so squares are in (0.02 mm)^2.
  static constexpr double kUnit2 = 2e-5 * 2e-5;
  static constexpr std::size_t kCellInts = 64 * 3;  // per block slot

  Levels(vkc::Device& device, vkc::Allocator& allocator, const Config& config)
      : device_(device), allocator_(allocator), config_(config) {}

  vkc::Status init() {
    const std::size_t levels = std::size_t(config_.levels);
    for (std::size_t l = 0; l < levels; ++l) {
      const float voxel = config_.base_voxel / float(1u << l);
      VKC_ASSIGN(vol::VoxelBlockGrid grid,
                 vr_example::create_fusion_grid(device_, allocator_, voxel,
                                                config_.trunc_voxels * voxel));
      grids_.push_back(std::move(grid));
      stats_.push_back(LevelStats{voxel, 0, 0, 0, 0.0, 0.0});
    }
    VKC_ASSIGN(vr::tsdf::TsdfIntegrator integrator,
               vr::tsdf::TsdfIntegrator::create(device_, allocator_));
    integrator_.emplace(std::move(integrator));
    masked_.resize(levels);
    residual_sets_.resize(levels - 1);
    mask_sets_.resize(levels - 1);
    accum_.resize(levels - 1);
    scores_.resize(levels - 1);
    flags_.resize(levels - 1);
    flags_host_.resize(levels - 1);
    state_.resize(levels - 1);
    refined_.resize(levels - 1);
    ready_.resize(levels - 1);

    // The builder keeps pointers to the kernels, so size them first.
    residual_.resize(levels - 1);
    bias_.resize(levels - 1);
    mask_.resize(levels - 1);
    tint_.resize(levels);
    const VkPushConstantRange residual_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                            sizeof(ResidualPush)};
    const VkPushConstantRange bias_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                        sizeof(BiasPush)};
    const VkPushConstantRange mask_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                        sizeof(MaskPush)};
    const VkPushConstantRange tint_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                        sizeof(TintPush)};
    vkc::KernelSetBuilder builder(device_);
    for (std::size_t l = 0; l + 1 < levels; ++l) {
      VKC_TRY(builder.add(
          residual_[l], "adaptive_residual", vr_adaptive_residual_comp_spv,
          vr_adaptive_residual_comp_spv_size, 6, &residual_push));
      VKC_TRY(builder.add(bias_[l], "adaptive_bias", vr_adaptive_bias_comp_spv,
                          vr_adaptive_bias_comp_spv_size, 3, &bias_push));
      VKC_TRY(builder.add(mask_[l], "adaptive_mask", vr_adaptive_mask_comp_spv,
                          vr_adaptive_mask_comp_spv_size, 5, &mask_push));
    }
    for (std::size_t l = 0; l < levels; ++l) {
      VKC_TRY(builder.add(tint_[l], "adaptive_tint", vr_adaptive_tint_comp_spv,
                          vr_adaptive_tint_comp_spv_size, 1, &tint_push));
    }
    VKC_ASSIGN(pool_, builder.build());
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(device_.physical_device(), &properties);
    max_groups_ = properties.limits.maxComputeWorkGroupCount[0];
    for (std::size_t l = 0; l + 1 < levels; ++l) VKC_TRY(fit(l));
    return {};
  }

  static void bind(const vkc::ComputeKernel& k, std::uint32_t binding,
                   const vkc::Buffer& b) {
    k.set.write_storage_buffer(binding, b.handle(), 0, b.size());
  }

  // Per-slot buffers follow the grid's heap, which grows on overflow; a grown
  // level starts its sums again.
  vkc::Status fit(std::size_t l) {
    const std::size_t slots = std::size_t(grids_[l].grid().num_blocks);
    if (flags_host_[l].size() == slots && flags_[l].valid()) return {};
    flags_host_[l].resize(slots, 0u);
    VKC_ASSIGN(flags_[l], vkc::device_storage_buffer(
                              allocator_, slots * sizeof(std::uint32_t)));
    VKC_ASSIGN(accum_[l], vkc::device_storage_buffer(
                              allocator_, slots * kCellInts * sizeof(int)));
    VKC_ASSIGN(scores_[l], vkc::device_storage_buffer(
                               allocator_, slots * 4 * sizeof(float)));
    vkc::CommandBatch batch(device_, allocator_);
    VKC_TRY(batch.upload(flags_[l], 0, flags_host_[l].data(),
                         slots * sizeof(std::uint32_t)));
    VKC_TRY(batch.zero(accum_[l], 0, accum_[l].size()));
    VKC_TRY(batch.zero(scores_[l], 0, scores_[l].size()));
    return batch.submit();
  }

  // Add the set's residuals to each coarse level's cell sums (before the set
  // is fused, so it is judged against what came before), then write each
  // camera's allocation depth for the finer levels from the last check's
  // flags. One submit; each camera's dispatches bind a set of their own.
  vkc::Status accumulate_and_mask(const std::vector<Input>& set) {
    const std::uint32_t stride = config_.pixel_stride;
    const float min_cos =
        config_.max_view_deg > 0.0f
            ? std::cos(config_.max_view_deg * 3.14159265f / 180.0f)
            : 0.0f;
    const std::uint32_t n = std::uint32_t(set.size());
    for (std::size_t l = 0; l + 1 < grids_.size(); ++l) {
      VKC_TRY(fit(l));
      VKC_TRY(residual_sets_[l].reserve(device_, residual_[l], n));
      VKC_TRY(mask_sets_[l].reserve(device_, mask_[l], n));
      std::vector<std::shared_ptr<vkc::Buffer>>& masked = masked_[l + 1];
      if (masked.size() < n) masked.resize(n);
      for (std::size_t c = 0; c < n; ++c) {
        const VkDeviceSize bytes = set[c].depth->size();
        if (!masked[c] || masked[c]->size() != bytes) {
          VKC_ASSIGN(vkc::Buffer b,
                     vkc::device_storage_buffer(allocator_, bytes));
          masked[c] = std::make_shared<vkc::Buffer>(std::move(b));
        }
      }
    }
    vkc::CommandBatch batch(device_, allocator_);
    for (std::size_t l = 0; l + 1 < grids_.size(); ++l) {
      vol::VoxelBlockGrid& g = grids_[l];
      VKC_ASSIGN(const vol::AttributeView tsdf, g.attribute("tsdf"));
      VKC_ASSIGN(const vol::AttributeView weight, g.attribute("weight"));
      for (std::size_t c = 0; c < n; ++c) {
        const vr::DepthCameraParams& cam = set[c].camera;
        const std::uint32_t columns = (cam.width + stride - 1) / stride;
        const std::uint32_t count =
            columns * ((cam.height + stride - 1) / stride);
        const vkc::DescriptorSet& r = residual_sets_[l][c];
        write(r, 0, *set[c].depth);
        r.write_storage_buffer(1, g.map().entries_buffer(), 0,
                               g.map().entries_buffer_size());
        write(r, 2, *tsdf.buffer);
        write(r, 3, *weight.buffer);
        write(r, 4, accum_[l]);
        write(r, 5, cameras_[c]);
        const ResidualPush rpush{g.grid(), stride, columns, count, min_cos};
        VKC_TRY(batch.dispatch(residual_[l], r, &rpush, sizeof(rpush),
                               (count + 255u) / 256u, max_groups_));

        const vkc::DescriptorSet& m = mask_sets_[l][c];
        write(m, 0, *set[c].depth);
        m.write_storage_buffer(1, g.map().entries_buffer(), 0,
                               g.map().entries_buffer_size());
        write(m, 2, flags_[l]);
        write(m, 3, *masked_[l + 1][c]);
        write(m, 4, cameras_[c]);
        const MaskPush push{g.grid()};
        const std::uint32_t pixels = cam.width * cam.height;
        VKC_TRY(batch.dispatch(mask_[l], m, &push, sizeof(push),
                               (pixels + 255u) / 256u, max_groups_));
      }
    }
    return batch.submit();
  }

  static void write(const vkc::DescriptorSet& set, std::uint32_t binding,
                    const vkc::Buffer& b) {
    set.write_storage_buffer(binding, b.handle(), 0, b.size());
  }

  vkc::Status check() {
    const std::size_t levels = grids_.size();
    // Each coarse level's active blocks report their offsets, and fade.
    std::vector<std::vector<vol::BlockIndex>> active(levels);
    std::vector<std::vector<float>> scores(levels - 1);
    for (std::size_t l = 0; l + 1 < levels; ++l) {
      VKC_TRY(fit(l));
      VKC_ASSIGN(const vol::DeviceBlockList list,
                 grids_[l].map().compact_active_blocks_on_device());
      if (list.count == 0) continue;
      const vkc::ComputeKernel& k = bias_[l];
      k.set.write_storage_buffer(
          0, list.buffer->handle(), 0,
          VkDeviceSize(list.count) * sizeof(vol::BlockIndex));
      bind(k, 1, accum_[l]);
      bind(k, 2, scores_[l]);
      scores[l].resize(flags_host_[l].size() * 4);
      vkc::CommandBatch batch(device_, allocator_);
      for (std::uint32_t base = 0; base < list.count; base += max_groups_) {
        const BiasPush push{list.count, base, 512u};
        VKC_TRY(batch.dispatch(k, &push, sizeof(push),
                               std::min(max_groups_, list.count - base),
                               max_groups_));
      }
      VKC_TRY(batch.readback(scores_[l], 0, scores[l].size() * sizeof(float),
                             scores[l].data()));
      VKC_TRY(batch.submit());
    }
    for (std::size_t l = 0; l < levels; ++l) {
      VKC_ASSIGN(active[l], grids_[l].map().compact_active_blocks());
    }

    // The sensor's floor: the median coarsest block's offset.
    const double eps = double(config_.eps_mm) * 1e-3;
    double floor2 = 0.0;
    if (!scores[0].empty()) {
      std::vector<double> all;
      for (const vol::BlockIndex& b : active[0]) {
        const float* sc = &scores[0][4 * (std::size_t(b.ptr) / 512u)];
        if (sc[3] >= float(config_.min_cells)) all.push_back(sc[0] * kUnit2);
      }
      if (!all.empty()) {
        std::nth_element(all.begin(), all.begin() + all.size() / 2, all.end());
        floor2 = std::max(0.0, all[all.size() / 2]);
      }
    }
    // Decide per block, keyed by coordinate (heap slots are reused).
    for (std::size_t l = 0; l + 1 < levels; ++l) {
      stats_[l].floor_mm = 1000.0 * std::sqrt(floor2);
      if (scores[l].empty()) continue;
      std::unordered_map<std::uint64_t, State> next;
      next.reserve(active[l].size());
      std::vector<double> offsets, noises;
      for (const vol::BlockIndex& b : active[l]) {
        const std::uint64_t k = key(b.coord);
        const auto it = state_[l].find(k);
        State s = it == state_[l].end() ? State{} : it->second;
        const float* sc = &scores[l][4 * (std::size_t(b.ptr) / 512u)];
        if (sc[3] >= float(config_.min_cells)) {
          const double offset2 = double(sc[0]) * kUnit2;
          const double left2 = double(sc[1]) * kUnit2;
          offsets.push_back(std::sqrt(std::max(0.0, offset2)));
          noises.push_back(std::sqrt(double(sc[2]) * kUnit2));
          const double excess =
              std::sqrt(std::max(0.0, offset2)) - std::sqrt(floor2);
          if (!s.refined) {
            // Off by more than eps past the floor, and clear of the noise.
            if (excess > eps && excess > 2.0 * std::sqrt(left2)) {
              s = State{true, frame_, 0};
            }
          } else {
            s.calm = excess < eps / 2 ? s.calm + 1 : 0;
            if (s.calm >= config_.calm_checks) s = State{};
          }
        }
        if (s.refined) next.emplace(k, s);
      }
      state_[l] = std::move(next);
      const auto median = [](std::vector<double>& v) {
        if (v.empty()) return 0.0;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return 1000.0 * v[v.size() / 2];
      };
      stats_[l].median_offset_mm = median(offsets);
      stats_[l].median_noise_mm = median(noises);
    }
    // A level counts as refined only below refined ancestors, and as ready --
    // owning the mesh -- once it has fused for at least one check interval.
    for (std::size_t l = 0; l + 1 < levels; ++l) {
      refined_[l].clear();
      ready_[l].clear();
      std::fill(flags_host_[l].begin(), flags_host_[l].end(), 0u);
      for (const vol::BlockIndex& b : active[l]) {
        const std::uint64_t k = key(b.coord);
        const auto it = state_[l].find(k);
        if (it == state_[l].end()) continue;
        if (l > 0 && refined_[l - 1].count(key(parent(b.coord))) == 0) {
          continue;
        }
        refined_[l].insert(k);
        flags_host_[l][std::size_t(b.ptr) / 512u] = 1u;
        const bool up_ready =
            l == 0 || ready_[l - 1].count(key(parent(b.coord))) != 0;
        if (up_ready && it->second.since < frame_) ready_[l].insert(k);
      }
      stats_[l].refined = refined_[l].size();
    }
    {
      vkc::CommandBatch batch(device_, allocator_);
      for (std::size_t l = 0; l + 1 < levels; ++l) {
        VKC_TRY(batch.upload(flags_[l], 0, flags_host_[l].data(),
                             flags_host_[l].size() * sizeof(std::uint32_t)));
      }
      VKC_TRY(batch.submit());
    }
    // Drop finer blocks more than one block from a refined region: what a
    // coarsened region leaves behind. One block of margin is kept, since the
    // band allocates that far past a region and its meshing reads it.
    for (std::size_t l = 1; l < levels; ++l) {
      std::vector<vol::BlockIndex> stale;
      for (const vol::BlockIndex& b : active[l]) {
        bool near = false;
        for (int dz = -1; dz <= 1 && !near; ++dz)
          for (int dy = -1; dy <= 1 && !near; ++dy)
            for (int dx = -1; dx <= 1 && !near; ++dx)
              near = refined_[l - 1].count(
                         key(parent(b.coord + vr::Vec3i(dx, dy, dz)))) != 0;
        if (!near) stale.push_back(b);
      }
      if (!stale.empty()) {
        VKC_TRY(grids_[l]
                    .remove(stale.data(), std::uint32_t(stale.size()))
                    .status());
      }
    }
    return {};
  }

  vkc::Device& device_;
  vkc::Allocator& allocator_;
  Config config_;
  std::size_t frame_ = 0;  // sets fused
  std::uint32_t max_groups_ = 0;
  std::vector<vol::VoxelBlockGrid> grids_;
  std::optional<vr::tsdf::TsdfIntegrator> integrator_;
  std::vector<vkc::Buffer> cameras_;  // [c]: camera c of the set
  std::vector<std::shared_ptr<vkc::Buffer>> host_depth_;  // uploaded frames
  // [l][c]: camera c's allocation depth for level l >= 1.
  std::vector<std::vector<std::shared_ptr<vkc::Buffer>>> masked_;
  // [l], per slot of level l < last: cell sums, block scores, refine flags.
  std::vector<vkc::Buffer> accum_, scores_, flags_;
  std::vector<std::vector<std::uint32_t>> flags_host_;
  std::vector<std::unordered_map<std::uint64_t, State>> state_;
  std::vector<std::unordered_set<std::uint64_t>> refined_, ready_;
  std::vector<LevelStats> stats_;
  // Declared before the kernels so it outlives their sets.
  vkc::DescriptorPool pool_;
  std::vector<vkc::ComputeKernel> residual_, bias_, mask_, tint_;
  // [l]: per-camera sets for one submit over a whole set.
  std::vector<vkc::KernelSets> residual_sets_, mask_sets_;
};

}  // namespace adaptive
