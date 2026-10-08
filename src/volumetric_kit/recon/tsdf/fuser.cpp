// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/tsdf/fuser.hpp"

#include <utility>
#include <vector>

namespace volumetric_kit::recon::tsdf {

core::Result<Fuser> Fuser::create(core::Device& device,
                                  core::Allocator& allocator,
                                  FuserConfig config) {
  if (config.max_grows_per_set < 0) {
    return core::Status::invalid_argument(
        "Fuser::create: max_grows_per_set must be >= 0");
  }
  if (config.growth.max_buckets < 0) {
    return core::Status::invalid_argument(
        "Fuser::create: growth.max_buckets must be >= 0");
  }
  // Written so a NaN fails too.
  if (!(config.refuse_allocation_above > 0.0f &&
        config.refuse_allocation_above <= 1.0f)) {
    return core::Status::invalid_argument(
        "Fuser::create: refuse_allocation_above must be in (0, 1]");
  }
  if (config.growth.enabled &&
      config.refuse_allocation_above < volume::VoxelHashMap::kGrowThreshold) {
    return core::Status::invalid_argument(
        "Fuser::create: with growth on, refuse_allocation_above must be >= "
        "VoxelHashMap::kGrowThreshold");
  }
  VKC_ASSIGN(TsdfIntegrator integrator,
             TsdfIntegrator::create(device, allocator));
  return Fuser(std::move(integrator), std::move(config));
}

Fuser::Fuser(TsdfIntegrator integrator, FuserConfig config)
    : integrator_(std::move(integrator)),
      config_(std::move(config)),
      growth_(config_.growth) {}

core::Result<FuseReport> Fuser::fuse(volume::VoxelBlockGrid& grid,
                                     const std::vector<FrameInput>& frames,
                                     float max_weight, IntegrationMode mode,
                                     core::StageMetrics* metrics) {
  if (!valid()) {
    return core::Status::invalid_argument("Fuser::fuse: moved-from fuser");
  }
  FuseReport report;
  if (config_.max_grows_per_set > 0) {
    VKC_ASSIGN(report.ahead, growth_.grow_ahead(grid, metrics));
    report.grows = report.ahead.grew() ? 1 : 0;
  }

  VKC_ASSIGN(const float load, grid.map().load_factor());
  if (load > config_.refuse_allocation_above) {
    report.allocation_refused = true;
  } else {
    // Each input's depth half, sliced off.
    const std::vector<volume::DepthInput> depths(frames.begin(), frames.end());
    VKC_ASSIGN(report.dropped, grid.map().allocate_from_depth(
                                   depths, &report.failures, metrics));
    while (report.dropped > 0 && report.failures.capacity_limited() &&
           report.grows < config_.max_grows_per_set) {
      VKC_ASSIGN(report.grow, growth_.grow(grid, 0, metrics));
      if (!report.grow.grew()) break;
      ++report.grows;
      VKC_ASSIGN(report.dropped, grid.map().allocate_from_depth(
                                     depths, &report.failures, metrics));
    }
  }

  VKC_TRY(integrator_.integrate(grid, frames, max_weight, mode, metrics));
  VKC_ASSIGN(report.load_factor, grid.map().load_factor());
  return report;
}

}  // namespace volumetric_kit::recon::tsdf
