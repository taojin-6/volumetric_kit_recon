// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/tsdf/adaptive_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "adaptive_mask_comp.spv.hpp"
#include "adaptive_offset_comp.spv.hpp"
#include "adaptive_residual_comp.spv.hpp"

namespace volumetric_kit::recon::tsdf {
namespace {

// A block coordinate as one hash key; coordinates span +-2^20.
std::uint64_t key(Vec3i c) {
  const auto pack = [](int v) {
    return static_cast<std::uint64_t>(v + (1 << 20)) & 0x1fffffu;
  };
  return pack(c.x) | (pack(c.y) << 21) | (pack(c.z) << 42);
}

// The coarser block containing `c`: its coordinate halved, rounding down.
Vec3i parent(Vec3i c) {
  const auto half = [](int v) { return v >= 0 ? v / 2 : -((1 - v) / 2); };
  return {half(c.x), half(c.y), half(c.z)};
}

struct ResidualPush {
  volume::VoxelGridParams grid;
  std::uint32_t stride;
  std::uint32_t columns;
  std::uint32_t count;
};
struct OffsetPush {
  std::uint32_t count;
  std::uint32_t base;
  std::uint32_t voxels_per_block;
  float voxel_size;
};
struct MaskPush {
  volume::VoxelGridParams coarse;
};

// Residuals are summed in 0.02 mm units; 64 cells of three ints per block.
constexpr double kUnit2 = 2e-5 * 2e-5;
constexpr std::size_t kCellInts = 64 * 3;

void bind(const core::DescriptorSet& set, std::uint32_t binding,
          const core::Buffer& b) {
  set.write_storage_buffer(binding, b.handle(), 0, b.size());
}

core::Status check_config(const AdaptiveGridConfig& c) {
  const auto positive = [](float v) { return std::isfinite(v) && v > 0.0f; };
  if (c.levels < 2 || c.levels > 4 || !positive(c.voxel_size) ||
      !positive(c.trunc_voxels) || !positive(c.max_weight) ||
      !positive(c.refine_offset) || c.pixel_stride == 0 || c.check_every == 0 ||
      c.calm_checks == 0 || c.num_buckets < 1 ||
      std::int64_t{c.num_buckets} * 8 >
          std::numeric_limits<std::int32_t>::max()) {
    return core::Status::invalid_argument(
        "AdaptiveGrid: invalid configuration");
  }
  return {};
}

}  // namespace

struct AdaptiveGrid::Impl {
  struct State {
    bool refined = false;
    std::size_t since = 0;  // the set count it was refined at
    std::uint32_t calm = 0;
  };

  Impl(core::Device& device_in, core::Allocator& allocator_in,
       const AdaptiveGridConfig& config_in)
      : device(device_in), allocator(allocator_in), config(config_in) {}

  core::Device& device;
  core::Allocator& allocator;
  AdaptiveGridConfig config;
  std::size_t sets = 0;
  std::uint32_t max_groups = 0;
  float floor = 0.0f;
  std::vector<volume::VoxelBlockGrid> levels;
  std::optional<TsdfIntegrator> integrator;
  std::vector<core::Buffer> cameras;  // [c]: the set's camera c
  std::vector<core::Buffer> uploads;  // [c]: camera c's host depth, staged
  // [l][c]: camera c's allocation depth for level l >= 1.
  std::vector<std::vector<core::Buffer>> masked;
  // [l], per block slot of level l < last: cell sums, block scores, flags.
  std::vector<core::Buffer> accum, scores, flags;
  std::vector<std::vector<std::uint32_t>> flags_host;
  std::vector<std::unordered_map<std::uint64_t, State>> state;
  std::vector<std::unordered_set<std::uint64_t>> refined, ready;
  std::vector<AdaptiveLevelStats> stats;
  // Declared before the kernels so it outlives their sets.
  core::DescriptorPool pool;
  std::vector<core::ComputeKernel> residual, offset, mask;
  // [l]: one set per camera, so a whole set is one submit.
  std::vector<core::KernelSets> residual_sets, mask_sets;

  core::Status init() {
    const std::size_t n = config.levels;
    const volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                           {"weight", sizeof(float)},
                                           {"color", sizeof(std::uint32_t)}};
    for (std::size_t l = 0; l < n; ++l) {
      const float voxel = std::ldexp(config.voxel_size, -int(l));
      volume::VoxelGridParams grid{};
      grid.voxel_size = voxel;
      grid.block_size = 8;
      grid.voxels_per_block = 512;
      grid.trunc_dist = config.trunc_voxels * voxel;
      grid.bucket_size = 8;
      grid.num_buckets = config.num_buckets;
      grid.num_blocks = grid.bucket_size * grid.num_buckets;
      grid.max_chain = 128;
      VKC_ASSIGN(volume::VoxelBlockGrid level,
                 volume::VoxelBlockGrid::create(device, allocator, grid, attrs,
                                                config.color ? 3u : 2u));
      levels.push_back(std::move(level));
      AdaptiveLevelStats s;
      s.voxel_size = voxel;
      stats.push_back(s);
    }
    VKC_ASSIGN(TsdfIntegrator made, TsdfIntegrator::create(device, allocator));
    integrator.emplace(std::move(made));
    masked.resize(n);
    accum.resize(n - 1);
    scores.resize(n - 1);
    flags.resize(n - 1);
    flags_host.resize(n - 1);
    state.resize(n - 1);
    refined.resize(n - 1);
    ready.resize(n - 1);
    residual.resize(n - 1);
    offset.resize(n - 1);
    mask.resize(n - 1);
    residual_sets.resize(n - 1);
    mask_sets.resize(n - 1);
    const VkPushConstantRange residual_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                            sizeof(ResidualPush)};
    const VkPushConstantRange offset_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                          sizeof(OffsetPush)};
    const VkPushConstantRange mask_push{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                        sizeof(MaskPush)};
    core::KernelSetBuilder builder(device);
    for (std::size_t l = 0; l + 1 < n; ++l) {
      VKC_TRY(builder.add(
          residual[l], "adaptive_residual", vr_adaptive_residual_comp_spv,
          vr_adaptive_residual_comp_spv_size, 6, &residual_push));
      VKC_TRY(builder.add(offset[l], "adaptive_offset",
                          vr_adaptive_offset_comp_spv,
                          vr_adaptive_offset_comp_spv_size, 5, &offset_push));
      VKC_TRY(builder.add(mask[l], "adaptive_mask", vr_adaptive_mask_comp_spv,
                          vr_adaptive_mask_comp_spv_size, 5, &mask_push));
    }
    VKC_ASSIGN(pool, builder.build());
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(device.physical_device(), &properties);
    max_groups = properties.limits.maxComputeWorkGroupCount[0];
    for (std::size_t l = 0; l + 1 < n; ++l) VKC_TRY(fit(l));
    return {};
  }

  // Per-slot buffers follow the level's heap, which grows on overflow; a
  // grown level starts its sums again.
  core::Status fit(std::size_t l) {
    const std::size_t slots = std::size_t(levels[l].grid().num_blocks);
    if (flags_host[l].size() == slots && flags[l].valid()) return {};
    flags_host[l].resize(slots, 0u);
    VKC_ASSIGN(flags[l], core::device_storage_buffer(
                             allocator, slots * sizeof(std::uint32_t)));
    VKC_ASSIGN(accum[l], core::device_storage_buffer(
                             allocator, slots * kCellInts * sizeof(int)));
    VKC_ASSIGN(scores[l], core::device_storage_buffer(
                              allocator, slots * 8 * sizeof(float)));
    core::CommandBatch batch(device, allocator);
    VKC_TRY(batch.upload(flags[l], 0, flags_host[l].data(),
                         slots * sizeof(std::uint32_t)));
    VKC_TRY(batch.zero(accum[l], 0, accum[l].size()));
    VKC_TRY(batch.zero(scores[l], 0, scores[l].size()));
    return batch.submit();
  }

  // Allocate a level's band for the set, doubling its table when it runs out
  // of room and retrying rounds that only lost bucket-lock races.
  core::Status allocate(std::size_t l,
                        const std::vector<volume::DepthInput>& depths,
                        core::StageMetrics* metrics) {
    for (int round = 0; round < 5; ++round) {
      volume::AllocFailures failures;
      VKC_ASSIGN(
          const std::uint32_t failed,
          levels[l].map().allocate_from_depth(depths, &failures, metrics));
      if (failed == 0) return {};
      if (!failures.capacity_limited()) continue;
      const std::int64_t grown = std::int64_t{levels[l].grid().num_buckets} * 2;
      if (grown * levels[l].grid().bucket_size >
          std::numeric_limits<std::int32_t>::max()) {
        return core::Status::out_of_memory("AdaptiveGrid: a level cannot grow");
      }
      VKC_TRY(levels[l].resize(std::int32_t(grown)));
    }
    return core::Status::out_of_memory("AdaptiveGrid: allocation kept failing");
  }

  // Stage the set's cameras and host depth, add its residuals to each coarse
  // level's cell sums -- before it is fused, so it is not judged against
  // itself -- and write each camera's allocation depth for the finer levels
  // from the last check's flags. One submit.
  core::Status accumulate_and_mask(const std::vector<FrameInput>& frames) {
    const std::uint32_t n = std::uint32_t(frames.size());
    while (cameras.size() < n) {
      VKC_ASSIGN(core::Buffer b, core::device_storage_buffer(
                                     allocator, sizeof(DepthCameraParams)));
      cameras.push_back(std::move(b));
    }
    if (uploads.size() < n) uploads.resize(n);
    for (std::size_t l = 0; l + 1 < levels.size(); ++l) {
      VKC_TRY(fit(l));
      VKC_TRY(residual_sets[l].reserve(device, residual[l], n));
      VKC_TRY(mask_sets[l].reserve(device, mask[l], n));
      std::vector<core::Buffer>& out = masked[l + 1];
      if (out.size() < n) out.resize(n);
      for (std::size_t c = 0; c < n; ++c) {
        const VkDeviceSize bytes =
            VkDeviceSize(frames[c].camera.width) * frames[c].camera.height * 4;
        if (!out[c].valid() || out[c].size() != bytes) {
          VKC_ASSIGN(out[c], core::device_storage_buffer(allocator, bytes));
        }
      }
    }
    core::CommandBatch batch(device, allocator);
    std::vector<VkBuffer> depth(n);
    for (std::size_t c = 0; c < n; ++c) {
      const DepthCameraParams& cam = frames[c].camera;
      VKC_TRY(batch.upload(cameras[c], 0, &cam, sizeof(cam)));
      VKC_ASSIGN(depth[c],
                 frames[c].depth.buffer(
                     batch, allocator,
                     VkDeviceSize(cam.width) * cam.height * sizeof(float),
                     uploads[c]));
    }
    const std::uint32_t stride = config.pixel_stride;
    for (std::size_t l = 0; l + 1 < levels.size(); ++l) {
      volume::VoxelBlockGrid& g = levels[l];
      VKC_ASSIGN(const volume::AttributeView tsdf, g.attribute("tsdf"));
      VKC_ASSIGN(const volume::AttributeView weight, g.attribute("weight"));
      for (std::size_t c = 0; c < n; ++c) {
        const DepthCameraParams& cam = frames[c].camera;
        const VkDeviceSize depth_bytes =
            VkDeviceSize(cam.width) * cam.height * sizeof(float);
        const std::uint32_t columns = (cam.width + stride - 1) / stride;
        const std::uint32_t count =
            columns * ((cam.height + stride - 1) / stride);
        const core::DescriptorSet& r = residual_sets[l][c];
        r.write_storage_buffer(0, depth[c], 0, depth_bytes);
        r.write_storage_buffer(1, g.map().entries_buffer(), 0,
                               g.map().entries_buffer_size());
        bind(r, 2, *tsdf.buffer);
        bind(r, 3, *weight.buffer);
        bind(r, 4, accum[l]);
        bind(r, 5, cameras[c]);
        const ResidualPush rpush{g.grid(), stride, columns, count};
        VKC_TRY(batch.dispatch(residual[l], r, &rpush, sizeof(rpush),
                               (count + 255u) / 256u, max_groups));

        const core::DescriptorSet& m = mask_sets[l][c];
        m.write_storage_buffer(0, depth[c], 0, depth_bytes);
        m.write_storage_buffer(1, g.map().entries_buffer(), 0,
                               g.map().entries_buffer_size());
        bind(m, 2, flags[l]);
        bind(m, 3, masked[l + 1][c]);
        bind(m, 4, cameras[c]);
        const MaskPush push{g.grid()};
        const std::uint32_t pixels = cam.width * cam.height;
        VKC_TRY(batch.dispatch(mask[l], m, &push, sizeof(push),
                               (pixels + 255u) / 256u, max_groups));
      }
    }
    return batch.submit();
  }

  // Turn the cell sums into block offsets, decide refinement per block, and
  // drop finer blocks a coarsened region left behind.
  core::Status check() {
    const std::size_t n = levels.size();
    std::vector<std::vector<float>> score(n - 1);
    for (std::size_t l = 0; l + 1 < n; ++l) {
      VKC_TRY(fit(l));
      VKC_ASSIGN(const volume::DeviceBlockList list,
                 levels[l].map().compact_active_blocks_on_device());
      if (list.count == 0) continue;
      const core::ComputeKernel& k = offset[l];
      k.set.write_storage_buffer(
          0, list.buffer->handle(), 0,
          VkDeviceSize(list.count) * sizeof(volume::BlockIndex));
      VKC_ASSIGN(const volume::AttributeView tsdf, levels[l].attribute("tsdf"));
      VKC_ASSIGN(const volume::AttributeView weight,
                 levels[l].attribute("weight"));
      bind(k.set, 1, accum[l]);
      bind(k.set, 2, scores[l]);
      bind(k.set, 3, *tsdf.buffer);
      bind(k.set, 4, *weight.buffer);
      score[l].resize(flags_host[l].size() * 8);
      core::CommandBatch batch(device, allocator);
      for (std::uint32_t base = 0; base < list.count; base += max_groups) {
        const OffsetPush push{list.count, base, 512u,
                              levels[l].grid().voxel_size};
        VKC_TRY(batch.dispatch(k, &push, sizeof(push),
                               std::min(max_groups, list.count - base),
                               max_groups));
      }
      VKC_TRY(batch.readback(scores[l], 0, score[l].size() * sizeof(float),
                             score[l].data()));
      VKC_TRY(batch.submit());
    }
    std::vector<std::vector<volume::BlockIndex>> active(n);
    for (std::size_t l = 0; l < n; ++l) {
      VKC_ASSIGN(active[l], levels[l].map().compact_active_blocks());
      stats[l].blocks = std::uint32_t(active[l].size());
    }
    const auto scored = [&](std::size_t l, const volume::BlockIndex& b) {
      return &score[l][8 * (std::size_t(b.ptr) / 512u)];
    };
    const auto judged = [&](std::size_t l, const volume::BlockIndex& b) {
      const float* sc = scored(l, b);
      return sc[3] >= float(config.min_cells) ? sc : nullptr;
    };
    const auto median = [](std::vector<double>& v) {
      if (v.empty()) return 0.0;
      std::nth_element(v.begin(), v.begin() + std::ptrdiff_t(v.size() / 2),
                       v.end());
      return v[v.size() / 2];
    };

    // The sensor's floor: the median coarsest block's offset.
    double floor2 = 0.0;
    if (!score[0].empty()) {
      std::vector<double> all;
      for (const volume::BlockIndex& b : active[0]) {
        if (const float* sc = judged(0, b)) all.push_back(sc[0] * kUnit2);
      }
      floor2 = std::max(0.0, median(all));
    }
    floor = float(std::sqrt(floor2));

    const double eps = config.refine_offset;
    for (std::size_t l = 0; l + 1 < n; ++l) {
      if (score[l].empty()) continue;
      std::unordered_map<std::uint64_t, State> next;
      next.reserve(active[l].size());
      std::vector<double> offsets, noises;
      for (const volume::BlockIndex& b : active[l]) {
        const std::uint64_t k = key(b.coord);
        const auto it = state[l].find(k);
        State s = it == state[l].end() ? State{} : it->second;
        if (const float* sc = judged(l, b)) {
          const double offset2 = double(sc[0]) * kUnit2;
          const double left2 = double(sc[1]) * kUnit2;
          offsets.push_back(std::sqrt(std::max(0.0, offset2)));
          noises.push_back(std::sqrt(double(sc[2]) * kUnit2));
          const double excess = std::sqrt(std::max(0.0, offset2)) - floor;
          if (!s.refined) {
            if (excess > eps && excess > 2.0 * std::sqrt(left2)) {
              s = State{true, sets, 0};
            }
          } else {
            s.calm = excess < eps / 2 ? s.calm + 1 : 0;
          }
        } else if (s.refined && scored(l, b)[4] == 0.0f) {
          ++s.calm;  // its surface is gone, not just out of view
        }
        if (s.refined && s.calm >= config.calm_checks) s = State{};
        if (s.refined) next.emplace(k, s);
      }
      state[l] = std::move(next);
      stats[l].median_offset = float(median(offsets));
      stats[l].median_noise = float(median(noises));
    }
    // Refined only below refined ancestors; ready -- owning the mesh -- once
    // it has fused for a check.
    for (std::size_t l = 0; l + 1 < n; ++l) {
      refined[l].clear();
      ready[l].clear();
      std::fill(flags_host[l].begin(), flags_host[l].end(), 0u);
      for (const volume::BlockIndex& b : active[l]) {
        const std::uint64_t k = key(b.coord);
        const auto it = state[l].find(k);
        if (it == state[l].end()) continue;
        const std::uint64_t up = key(parent(b.coord));
        if (l > 0 && refined[l - 1].count(up) == 0) continue;
        refined[l].insert(k);
        flags_host[l][std::size_t(b.ptr) / 512u] = 1u;
        if ((l == 0 || ready[l - 1].count(up) != 0) &&
            it->second.since < sets) {
          ready[l].insert(k);
        }
      }
      stats[l].refined = std::uint32_t(refined[l].size());
    }
    {
      core::CommandBatch batch(device, allocator);
      for (std::size_t l = 0; l + 1 < n; ++l) {
        VKC_TRY(batch.upload(flags[l], 0, flags_host[l].data(),
                             flags_host[l].size() * sizeof(std::uint32_t)));
      }
      VKC_TRY(batch.submit());
    }
    // Finer blocks more than one block from a refined region go. One block of
    // margin stays: the band allocates that far past a region, and meshing
    // its edge reads it.
    for (std::size_t l = 1; l < n; ++l) {
      std::vector<volume::BlockIndex> stale;
      for (const volume::BlockIndex& b : active[l]) {
        bool near = false;
        for (int dz = -1; dz <= 1 && !near; ++dz) {
          for (int dy = -1; dy <= 1 && !near; ++dy) {
            for (int dx = -1; dx <= 1 && !near; ++dx) {
              near = refined[l - 1].count(
                         key(parent(b.coord + Vec3i(dx, dy, dz)))) != 0;
            }
          }
        }
        if (!near) stale.push_back(b);
      }
      if (!stale.empty()) {
        VKC_TRY(levels[l]
                    .remove(stale.data(), std::uint32_t(stale.size()))
                    .status());
      }
    }
    return {};
  }
};

AdaptiveGrid::AdaptiveGrid(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}
AdaptiveGrid::AdaptiveGrid() noexcept = default;
AdaptiveGrid::~AdaptiveGrid() = default;
AdaptiveGrid::AdaptiveGrid(AdaptiveGrid&& other) noexcept = default;
AdaptiveGrid& AdaptiveGrid::operator=(AdaptiveGrid&& other) noexcept = default;

core::Result<AdaptiveGrid> AdaptiveGrid::create(
    core::Device& device, core::Allocator& allocator,
    const AdaptiveGridConfig& config) {
  if (device.handle() == VK_NULL_HANDLE || !allocator.valid()) {
    return core::Status::invalid_argument(
        "AdaptiveGrid: invalid device or allocator");
  }
  VKC_TRY(check_device_requirements(device, "AdaptiveGrid::create"));
  VKC_TRY(check_config(config));
  std::unique_ptr<Impl> impl(new (std::nothrow)
                                 Impl(device, allocator, config));
  if (!impl)
    return core::Status::out_of_memory("AdaptiveGrid: owner allocation");
  VKC_TRY(impl->init());
  return AdaptiveGrid(std::move(impl));
}

bool AdaptiveGrid::valid() const noexcept { return impl_ != nullptr; }

core::Status AdaptiveGrid::fuse(const std::vector<FrameInput>& frames,
                                core::StageMetrics* metrics) {
  if (!valid())
    return core::Status::invalid_argument("AdaptiveGrid: empty grid");
  if (frames.empty()) return {};
  Impl& p = *impl_;
  for (const FrameInput& f : frames) {
    const DepthCameraParams& cam = f.camera;
    if (cam.width == 0 || cam.height == 0 || !(cam.fx > 0.0f) ||
        !(cam.fy > 0.0f)) {
      return core::Status::invalid_argument(
          "AdaptiveGrid: invalid depth camera");
    }
    VKC_TRY(
        f.depth.check("AdaptiveGrid: depth",
                      VkDeviceSize(cam.width) * cam.height * sizeof(float)));
  }
  if (p.sets > 0 && p.sets % p.config.check_every == 0) {
    core::StageScope scope(metrics, "adaptive check");
    VKC_TRY(p.check());
  }
  {
    core::StageScope scope(metrics, "adaptive residual");
    VKC_TRY(p.accumulate_and_mask(frames));
  }
  const std::vector<volume::DepthInput> depths(frames.begin(), frames.end());
  // Without colour attributes, the frames' colour is not fused.
  std::vector<FrameInput> inputs = frames;
  if (!p.config.color) {
    for (FrameInput& f : inputs) f.color = nullptr;
  }
  for (std::size_t l = 0; l < p.levels.size(); ++l) {
    if (l == 0) {
      VKC_TRY(p.allocate(0, depths, metrics));
    } else {
      std::vector<volume::DepthInput> from;
      for (std::size_t c = 0; c < frames.size(); ++c) {
        from.push_back({core::StorageInput(p.masked[l][c]), frames[c].camera});
      }
      VKC_TRY(p.allocate(l, from, metrics));
    }
    VKC_TRY(p.integrator->integrate(p.levels[l], inputs, p.config.max_weight,
                                    p.config.mode, metrics));
  }
  ++p.sets;
  return {};
}

core::Result<std::vector<volume::BlockIndex>> AdaptiveGrid::owned_blocks(
    std::uint32_t level) {
  if (!valid() || level >= impl_->levels.size()) {
    return core::Status::invalid_argument("AdaptiveGrid: no such level");
  }
  Impl& p = *impl_;
  VKC_ASSIGN(const std::vector<volume::BlockIndex> active,
             p.levels[level].map().compact_active_blocks());
  std::vector<volume::BlockIndex> out;
  const bool finest = level + 1 == p.levels.size();
  for (const volume::BlockIndex& b : active) {
    const bool ancestors =
        level == 0 || p.ready[level - 1].count(key(parent(b.coord))) != 0;
    const bool deeper = !finest && p.ready[level].count(key(b.coord)) != 0;
    if (ancestors && !deeper) out.push_back(b);
  }
  p.stats[level].blocks = std::uint32_t(active.size());
  p.stats[level].owned = std::uint32_t(out.size());
  return out;
}

std::uint32_t AdaptiveGrid::level_count() const noexcept {
  return valid() ? std::uint32_t(impl_->levels.size()) : 0u;
}

volume::VoxelBlockGrid& AdaptiveGrid::level(std::uint32_t l) {
  VKC_CHECK(valid() && l < impl_->levels.size(), "AdaptiveGrid: no such level");
  return impl_->levels[l];
}

const volume::VoxelBlockGrid& AdaptiveGrid::level(std::uint32_t l) const {
  VKC_CHECK(valid() && l < impl_->levels.size(), "AdaptiveGrid: no such level");
  return impl_->levels[l];
}

void AdaptiveGrid::set_refine_offset(float metres) noexcept {
  if (valid() && std::isfinite(metres) && metres > 0.0f) {
    impl_->config.refine_offset = metres;
  }
}

float AdaptiveGrid::sensor_floor() const noexcept {
  return valid() ? impl_->floor : 0.0f;
}

AdaptiveLevelStats AdaptiveGrid::stats(std::uint32_t level) const noexcept {
  if (!valid() || level >= impl_->stats.size()) return {};
  return impl_->stats[level];
}

}  // namespace volumetric_kit::recon::tsdf
