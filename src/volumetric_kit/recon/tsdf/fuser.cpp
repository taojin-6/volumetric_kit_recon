// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/tsdf/fuser.hpp"

#include <new>
#include <utility>
#include <vector>

#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"

namespace volumetric_kit::recon::tsdf {
namespace {

constexpr std::uint32_t kGrowthRetryTicks = 60;
constexpr int kLockRetries = 4;

bool out_of_memory(const core::Status& status) {
  if (status.domain() == core::Status::Code::OutOfMemory) return true;
  const auto vk = core::vk_result(status);
  return vk && (*vk == VK_ERROR_OUT_OF_DEVICE_MEMORY ||
                *vk == VK_ERROR_OUT_OF_HOST_MEMORY);
}

}  // namespace

core::Result<Fuser> Fuser::create(core::Device& device,
                                  core::Allocator& allocator,
                                  FuserConfig config) try {
  if (config.max_grows_per_set < 0) {
    return core::Status::invalid_argument(
        "Fuser::create: max_grows_per_set must be >= 0");
  }
  VKC_ASSIGN(TsdfIntegrator integrator,
             TsdfIntegrator::create(device, allocator));
  return Fuser(std::move(integrator), config);
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Fuser::create: host allocation failed");
}

Fuser::Fuser(TsdfIntegrator integrator, FuserConfig config)
    : integrator_(std::move(integrator)), config_(config) {}

core::Result<FuseReport> Fuser::fuse(volume::VoxelBlockGrid& grid,
                                     const std::vector<FrameInput>& frames,
                                     float max_weight, IntegrationMode mode,
                                     core::StageMetrics* metrics) try {
  if (!valid()) {
    return core::Status::invalid_argument("Fuser::fuse: moved-from fuser");
  }
  FuseReport report;
  VKC_ASSIGN(report.load_factor, grid.map().load_factor());
  report.from_buckets = report.to_buckets = grid.grid().num_buckets;
  if (frames.empty()) return report;

  const std::vector<volume::DepthInput> depths(frames.begin(), frames.end());
  bool may_grow =
      !(config_.allow_partial && refused_at_ == grid.grid().num_buckets &&
        std::uint32_t(grid.map().tick() - refused_tick_) < kGrowthRetryTicks);
  const auto grow = [&]() -> core::Result<bool> {
    if (!may_grow || report.grows >= config_.max_grows_per_set) return false;
    core::Status status = volume::grow_grid(grid, 0, metrics);
    if (!status.ok()) {
      if (!out_of_memory(status)) return status;
      // Grow-ahead is opportunistic: allocation may still complete in the
      // current grid. Strict mode reports this error only if it cannot.
      report.growth_error = std::move(status);
      may_grow = false;
      if (config_.allow_partial) {
        refused_at_ = grid.grid().num_buckets;
        refused_tick_ = grid.map().tick();
      }
      return false;
    }
    refused_at_ = 0;
    ++report.grows;
    return true;
  };

  if (report.load_factor > volume::VoxelHashMap::kGrowThreshold) {
    const auto ahead = grow();
    if (!ahead) return ahead.status();
  }
  int lock_retries = 0;
  for (;;) {
    VKC_ASSIGN(report.dropped, grid.map().allocate_from_depth(
                                   depths, &report.failures, metrics));
    if (report.dropped == 0) break;
    if (report.failures.capacity_limited()) {
      VKC_ASSIGN(const bool grew, grow());
      if (grew) continue;
    } else if (!config_.allow_partial && lock_retries++ < kLockRetries) {
      continue;
    }
    if (!config_.allow_partial) {
      if (!report.growth_error.ok()) return report.growth_error;
      return core::Status::out_of_memory(
          "Fuser::fuse: could not allocate the complete truncation band");
    }
    break;
  }

  VKC_TRY(integrator_.integrate(grid, frames, max_weight, mode, metrics));
  report.to_buckets = grid.grid().num_buckets;
  VKC_ASSIGN(report.load_factor, grid.map().load_factor());
  return report;
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Fuser::fuse: host allocation failed");
}

}  // namespace volumetric_kit::recon::tsdf
