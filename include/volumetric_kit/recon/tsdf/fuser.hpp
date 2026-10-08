// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/fuser.hpp
/// @brief A set of frames fused into a grid: the grid grown ahead of need,
///        every frame's truncation band allocated, the grid grown again if
///        that ran out of room, and the frames integrated.

#include <cstdint>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief Options for @ref Fuser::create.
struct FuserConfig {
  /// When the grid grows: the ceiling, the memory, and whether at all.
  volume::GrowthPolicy growth;
  /// The most times one set may grow the grid, ahead of need and for a
  /// capacity limit together. Each grow for a capacity limit is followed by
  /// one more allocation, so a set allocates at most this many times plus
  /// one. Lost bucket-lock races are not retried here: the allocation
  /// re-dispatches while they resolve, and what is left is asked for again
  /// by the next set.
  int max_grows_per_set = 2;
  /// The occupancy past which a set allocates nothing new, so a grid that
  /// cannot grow stops feeding the allocator's overflow scan; the set's
  /// frames still fuse into the blocks already there. 1 (the default)
  /// never stops.
  float refuse_allocation_above = 1.0f;
};

/// @brief What @ref Fuser::fuse did to the grid, for a caller to report.
struct FuseReport {
  /// The grow ahead of need, before allocating.
  volume::GrowthEvent ahead;
  /// The last grow for a capacity limit; `NotDue` when the set needed none.
  volume::GrowthEvent grow;
  /// How many times the set grew the grid.
  int grows = 0;
  /// The grid was past @ref FuserConfig::refuse_allocation_above, so the set
  /// allocated nothing new.
  bool allocation_refused = false;
  /// Blocks the set's allocation could not place, and why
  /// (@ref failures). The set fused into the rest. A capacity limit is left
  /// when the grid could not grow (@ref grow says why) or the set spent
  /// @ref FuserConfig::max_grows_per_set; lost lock races, when the
  /// allocation's own rounds ran out.
  std::uint32_t dropped = 0;
  /// The last allocation's failures by reason.
  volume::AllocFailures failures;
  /// The map's occupancy after the set.
  float load_factor = 0.0f;
};

/// @brief Fuses sets of frames into a @ref volume::VoxelBlockGrid: the one
///        fusion driver, so every caller grows, allocates and integrates
///        alike.
///
/// A set is one @ref fuse: the grid grows ahead of need
/// (@ref volume::GridGrowth::grow_ahead); every frame's band is allocated in
/// one call (@ref volume::VoxelHashMap::allocate_from_depth); if that hits a
/// capacity limit, the grid grows (@ref volume::GridGrowth::grow) and the set
/// allocates again, up to @ref FuserConfig::max_grows_per_set; then every
/// frame is integrated in one call (@ref TsdfIntegrator::integrate). Blocks
/// that could not be placed are reported, not an error: the set fuses into
/// the rest, as a scan that outgrew its memory keeps refining what it has.
///
/// It remembers a refused grow for the grid it fuses (@ref GridGrowth), so
/// one fuser fuses one grid.
///
/// @warning The `Device` and `Allocator` passed to @ref create must outlive
///          this object; its integrator stores references to them.
class VR_TSDF_API Fuser {
 public:
  /// @brief Build the integrator and check @p config.
  /// @param device     The compute device (must outlive this object).
  /// @param allocator  The allocator (must outlive this object).
  /// @param config     The growth policy and the per-set bounds.
  /// @return The fuser; `Status::Code::InvalidArgument` for a negative
  ///         @ref FuserConfig::max_grows_per_set or
  ///         @ref volume::GrowthPolicy::max_buckets, or a
  ///         @ref FuserConfig::refuse_allocation_above outside (0, 1]; or
  ///         what @ref TsdfIntegrator::create returns.
  static core::Result<Fuser> create(core::Device& device,
                                    core::Allocator& allocator,
                                    FuserConfig config = {});

  ~Fuser() = default;
  Fuser(Fuser&&) noexcept = default;
  Fuser& operator=(Fuser&&) = delete;
  Fuser(const Fuser&) = delete;
  Fuser& operator=(const Fuser&) = delete;

  /// @brief Fuse a set of frames into @p grid.
  /// @param grid        The grid, as @ref TsdfIntegrator::integrate takes it.
  /// @param frames      Each camera's frame; one is `{frame}`.
  /// @param max_weight  As @ref TsdfIntegrator::integrate.
  /// @param mode        As @ref TsdfIntegrator::integrate.
  /// @param metrics     Optional rows: `"resize"` around a grow, beside the
  ///                    allocation's `"allocate"` and the integration's
  ///                    `"integrate"`.
  /// @return What the set did to the grid; `Status::Code::InvalidArgument`
  ///         for a moved-from fuser; or the first error of the grow, the
  ///         allocation or the integration.
  core::Result<FuseReport> fuse(volume::VoxelBlockGrid& grid,
                                const std::vector<FrameInput>& frames,
                                float max_weight = 5.0f,
                                IntegrationMode mode = IntegrationMode::Classic,
                                core::StageMetrics* metrics = nullptr);

  /// @return The configuration this was created with.
  const FuserConfig& config() const noexcept { return config_; }
  /// @return `true` if this owns a live integrator (`false` when moved-from).
  bool valid() const noexcept { return integrator_.valid(); }

 private:
  Fuser(TsdfIntegrator integrator, FuserConfig config);

  TsdfIntegrator integrator_;
  FuserConfig config_;
  volume::GridGrowth growth_;
};

}  // namespace volumetric_kit::recon::tsdf
