// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/fuser.hpp
/// @brief Grow, allocate a set's truncation bands, then integrate its frames.

#include <cstdint>
#include <memory>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/tsdf/change_detector.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

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
  /// for 60 map ticks; it still reports other errors. Once the grid holds a
  /// block, a partial set is one allocation round recorded with its fuse
  /// (@ref Fuser::record_fuse), ungated, and a grow for a capacity limit made
  /// after it, so what the set could not allocate lands with the next.
  bool allow_partial = false;
  /// How @ref IntegrationMode::Dynamic's change detector tests a pixel
  /// (@ref TsdfIntegrator::create).
  ChangeDetectorConfig change{};
};

/// @brief The result of a successful @ref Fuser::fuse or @ref Fuser::finish.
struct FuseReport {
  /// Successful grows, and the bucket counts before and after the set.
  int grows = 0;
  std::int32_t from_buckets = 0;
  std::int32_t to_buckets = 0;
  /// Failed allocation requests, which may name the same block more than
  /// once. Nonzero only with @ref FuserConfig::allow_partial.
  std::uint32_t dropped = 0;
  /// The last allocation's failures by reason.
  volumetric_kit::grid::volume::AllocFailures failures;
  /// A capacity or memory error that prevented growth. The band may still
  /// fit without growing; otherwise only partial mode can return this report.
  core::Status growth_error;
  /// The map's occupancy after the set.
  float load_factor = 0.0f;
  /// The device left the band incomplete where the set needed it whole -- a
  /// strict fuser, or a grid that held no block -- so @ref Fuser::finish
  /// allocated and fused the set again, blocking, at the map's tick the set
  /// was recorded at. Whatever the set's batch recorded after the fuse saw
  /// the grid without it.
  bool redone = false;
};

/// @brief A set's fusion recorded into a caller's batch
///        (@ref Fuser::record_fuse), for @ref Fuser::finish or
///        @ref Fuser::abandon once that batch has resolved.
///
/// A token: the fuser keeps what finishing needs -- the round's result slot,
/// the frames, whether the fuse was gated -- and its next record supersedes
/// an unfinished one.
///
/// @code
/// VKC_ASSIGN(const tsdf::PendingFuse fused,
///            fuser.record_fuse(batch, grid, frames, 5.0f, mode));
/// VKC_TRY(batch.submit());
/// VKC_ASSIGN(const tsdf::FuseReport report, fuser.finish(fused, grid));
/// @endcode
struct PendingFuse {
  /// Which record this is, unique in the process; 0 for none.
  std::uint64_t serial = 0;
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
/// A set records into a caller's batch (@ref record_fuse): one allocation
/// round and the fuse, every count on the device. Where the set needs its
/// whole band -- a strict fuser, or a grid holding no block -- the fuse is
/// gated on the round, and @ref finish fuses a set the round left incomplete
/// again, blocking. @ref fuse is the record, one submit and @ref finish.
///
/// Partial mode remembers failed growth for one grid; use one fuser per grid.
/// @warning The `Device` and `Allocator` must outlive this object.
class VR_TSDF_API Fuser {
 public:
  /// @brief Create the integrator and validate @p config.
  /// @param device     Compute device; must outlive this object.
  /// @param allocator  Its allocator; must outlive this object.
  /// @param config     Growth bound and explicit partial-coverage choice.
  /// @return The fuser; `Status::Code::InvalidArgument` for a negative bound
  ///         or a change detector configuration
  ///         @ref ChangeDetector::create refuses;
  ///         `Status::Code::OutOfMemory` on host allocation failure; or
  ///         @ref TsdfIntegrator::create's error.
  static core::Result<Fuser> create(core::Device& device,
                                    core::Allocator& allocator,
                                    FuserConfig config = {});

  ~Fuser();
  Fuser(Fuser&&) noexcept;
  Fuser& operator=(Fuser&&) = delete;
  Fuser(const Fuser&) = delete;
  Fuser& operator=(const Fuser&) = delete;

  /// @brief Fuse one set of frames into @p grid: @ref record_fuse, one submit
  ///        and @ref finish.
  /// @param grid        The grid accepted by @ref TsdfIntegrator::integrate.
  /// @param frames      Each camera's input; an empty set does nothing.
  /// @param max_weight  As @ref TsdfIntegrator::integrate.
  /// @param mode        As @ref TsdfIntegrator::integrate.
  /// @param metrics     Optional rows, as @ref record_fuse and @ref finish
  ///                    report them, the batch's wait inside `"integrate"`.
  /// @return A report; `Status::Code::OutOfMemory` for an incomplete band in
  ///         strict mode or host allocation failure; a resize's backend OOM
  ///         when it prevents completion; or another tier's error.
  core::Result<FuseReport> fuse(
      volumetric_kit::grid::volume::VoxelBlockGrid& grid,
      const std::vector<FrameInput>& frames, float max_weight = 5.0f,
      IntegrationMode mode = IntegrationMode::Classic,
      core::StageMetrics* metrics = nullptr);

  /// @brief Record @p frames' fusion into @p batch, after what it holds (the
  ///        set's preparation): one allocation round of their bands, then the
  ///        fuse, every count on the device.
  ///
  /// A grow ahead of need (`VoxelHashMap::kGrowThreshold`) runs first,
  /// blocking (`"resize"`). With @ref FuserConfig::allow_partial, once the
  /// grid holds a block, the fuse takes what the round placed. Otherwise the
  /// fuse is gated on the whole band: a round that leaves a block out runs no
  /// fuse, and @ref finish fuses the set again (@ref FuseReport::redone).
  /// @param batch       Records the fusion; the caller submits it.
  /// @param grid        As @ref fuse; nothing but @p batch changes it until
  ///                    the batch has resolved.
  /// @param frames      As @ref fuse; their buffers, and the colour frames
  ///                    they point to, unchanged until @ref finish or
  ///                    @ref abandon, which may fuse them again.
  /// @param max_weight  As @ref fuse.
  /// @param mode        As @ref fuse.
  /// @param metrics     `"integrate"` over `"  ..allocate"` and
  ///                    `"  ..active set"` (both the device half alone), and
  ///                    a grow's `"resize"`: host halves now, device halves
  ///                    from `PendingBatch::device_rows`.
  /// @return The pending set (one recording nothing for no frames);
  ///         @ref fuse's refusals, with nothing recorded or grown and the
  ///         last pending set still this fuser's; or a grow's failure.
  /// @pre The batch of this fuser's last record has resolved, and nothing
  ///      @p batch holds yet reads @p grid: a grow ahead replaces its buffers.
  core::Result<PendingFuse> record_fuse(
      core::CommandBatch& batch,
      volumetric_kit::grid::volume::VoxelBlockGrid& grid,
      const std::vector<FrameInput>& frames, float max_weight = 5.0f,
      IntegrationMode mode = IntegrationMode::Classic,
      core::StageMetrics* metrics = nullptr);

  /// @brief Once @p pending's batch has resolved: settle the round and report
  ///        the set, fusing it again where the device refused it.
  ///
  /// A gated set whose round left a block out is allocated and fused again,
  /// blocking, as @ref fuse fused one before recording: lock races retried,
  /// the grid grown for a capacity limit, the tick the gated fuse took
  /// taken back first (`VoxelHashMap::rewind_tick`), so the set takes one.
  /// A partial set that met a capacity limit grows the grid for the next, as
  /// before.
  /// @param pending  From this fuser's last @ref record_fuse.
  /// @param grid     The grid it recorded into.
  /// @param metrics  Optional: a fuse again's rows (`"allocate"`,
  ///                 `"integrate"`), and a grow's `"resize"`.
  /// @return The report; `Status::Code::InvalidArgument` for a pending this
  ///         fuser no longer holds or another grid; or what @ref fuse returns
  ///         for a set fused again or a grow.
  core::Result<FuseReport> finish(
      const PendingFuse& pending,
      volumetric_kit::grid::volume::VoxelBlockGrid& grid,
      core::StageMetrics* metrics = nullptr);

  /// @brief Drop @p pending unjudged: its batch failed or was never submitted.
  ///        The grid holds what the batch left, and the heap's host copy
  ///        waits for the next settle.
  /// @param pending  From this fuser; one it no longer holds is ignored.
  void abandon(const PendingFuse& pending) noexcept;

  /// @return The configuration this was created with.
  const FuserConfig& config() const noexcept { return config_; }
  /// @return Whether this owns a live integrator; false when moved-from.
  bool valid() const noexcept { return integrator_.valid(); }

 private:
  // What finish needs of a recorded set (fuser.cpp).
  struct Recorded;

  Fuser(core::Device& device, core::Allocator& allocator,
        TsdfIntegrator integrator, FuserConfig config);
  // Grow `grid` once for `report`, when this set may: false when it may not
  // or the grow ran out of memory (kept in the report), or another failure.
  core::Result<bool> grow(volumetric_kit::grid::volume::VoxelBlockGrid& grid,
                          FuseReport& report, bool& may_grow,
                          core::StageMetrics* metrics);
  // The blocking fusion: allocation rounds until the band is in or as far as
  // the config allows, grows for capacity, then the integrate.
  core::Status fuse_blocking(volumetric_kit::grid::volume::VoxelBlockGrid& grid,
                             const std::vector<FrameInput>& frames,
                             float max_weight, IntegrationMode mode,
                             FuseReport& report, bool& may_grow,
                             core::StageMetrics* metrics);
  // Whether `pending` is this fuser's unfinished last record.
  bool holds(const PendingFuse& pending) const noexcept;

  core::Device* device_ = nullptr;
  core::Allocator* allocator_ = nullptr;
  TsdfIntegrator integrator_;
  FuserConfig config_;
  std::int32_t refused_at_ = 0;
  std::uint32_t refused_tick_ = 0;
  // The word a strict set's fuse is gated on, set to 1 by each record and
  // closed by its round on any failure; device-local.
  core::Buffer gate_;
  // Device spans for a set's "integrate"; idle until a caller asks.
  core::GpuTimer timer_;
  std::unique_ptr<Recorded> pending_;
};

}  // namespace volumetric_kit::recon::tsdf
