// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/fuser.hpp
/// @brief Grow, allocate a set's truncation bands, then integrate its frames.

#include <cstdint>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief Options for @ref Fuser::create.
struct FuserConfig {
  /// Maximum successful grows per set, including grow-ahead. Zero fixes
  /// capacity. Four lets the default complete an initial band needing four
  /// doublings, as the former example allocation loop did.
  int max_grows_per_set = 4;
  /// Opt into integrating an incomplete band after allocation retries or
  /// capacity are exhausted. Intended for live viewers; offline callers
  /// default to returning an error before integration. Partial mode leaves
  /// residual lock contention to the next set and backs off a failed grow
  /// for 60 map ticks; it still reports other errors.
  bool allow_partial = false;
};

/// @brief The result of a successful @ref Fuser::fuse.
struct FuseReport {
  /// Successful grows, and the bucket counts before and after the set.
  int grows = 0;
  std::int32_t from_buckets = 0;
  std::int32_t to_buckets = 0;
  /// Failed allocation requests, which may name the same block more than
  /// once. Nonzero only with @ref FuserConfig::allow_partial.
  std::uint32_t dropped = 0;
  /// The last allocation's failures by reason.
  volume::AllocFailures failures;
  /// A capacity or memory error that prevented growth. The band may still
  /// fit without growing; otherwise only partial mode can return this report.
  core::Status growth_error;
  /// The map's occupancy after the set.
  float load_factor = 0.0f;
};

/// @brief The shared fusion sequence: grow ahead, allocate, grow for capacity
///        if needed, then integrate the set.
///
/// By default every frame's band must be allocated before integration. A
/// failed call can grow the grid or add zeroed blocks but does not integrate
/// an incomplete band. Pure lock contention is retried up to four times
/// beyond the allocation tier's own rounds. Live callers can explicitly
/// accept partial coverage through @ref FuserConfig::allow_partial.
///
/// Partial mode remembers failed growth for one grid; use one fuser per grid.
/// @warning The `Device` and `Allocator` must outlive this object.
class VR_TSDF_API Fuser {
 public:
  /// @brief Create the integrator and validate @p config.
  /// @param device     Compute device; must outlive this object.
  /// @param allocator  Its allocator; must outlive this object.
  /// @param config     Growth bound and explicit partial-coverage choice.
  /// @return The fuser; `Status::Code::InvalidArgument` for a negative bound;
  ///         `Status::Code::OutOfMemory` on host allocation failure; or
  ///         @ref TsdfIntegrator::create's error.
  static core::Result<Fuser> create(core::Device& device,
                                    core::Allocator& allocator,
                                    FuserConfig config = {});

  ~Fuser() = default;
  Fuser(Fuser&&) noexcept = default;
  Fuser& operator=(Fuser&&) = delete;
  Fuser(const Fuser&) = delete;
  Fuser& operator=(const Fuser&) = delete;

  /// @brief Fuse one set of frames into @p grid.
  /// @param grid        The grid accepted by @ref TsdfIntegrator::integrate.
  /// @param frames      Each camera's input; an empty set does nothing.
  /// @param max_weight  As @ref TsdfIntegrator::integrate.
  /// @param mode        As @ref TsdfIntegrator::integrate.
  /// @param metrics     Optional `"resize"`, `"allocate"`, `"integrate"` rows.
  /// @return A report; `Status::Code::OutOfMemory` for an incomplete band in
  ///         strict mode or host allocation failure; a resize's backend OOM
  ///         when it prevents completion; or another tier's error.
  core::Result<FuseReport> fuse(volume::VoxelBlockGrid& grid,
                                const std::vector<FrameInput>& frames,
                                float max_weight = 5.0f,
                                IntegrationMode mode = IntegrationMode::Classic,
                                core::StageMetrics* metrics = nullptr);

  /// @return The configuration this was created with.
  const FuserConfig& config() const noexcept { return config_; }
  /// @return Whether this owns a live integrator; false when moved-from.
  bool valid() const noexcept { return integrator_.valid(); }

 private:
  Fuser(TsdfIntegrator integrator, FuserConfig config);

  TsdfIntegrator integrator_;
  FuserConfig config_;
  std::int32_t refused_at_ = 0;
  std::uint32_t refused_tick_ = 0;
};

}  // namespace volumetric_kit::recon::tsdf
