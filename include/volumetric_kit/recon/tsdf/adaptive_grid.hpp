// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/adaptive_grid.hpp
/// @brief Uniform grids at halving voxel sizes, a block refined where the
///        depth is systematically off the coarser grid's surface.

#include <cstdint>
#include <memory>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief Resolution, fusion and refinement controls for @ref AdaptiveGrid.
struct AdaptiveGridConfig {
  /// Coarsest voxel edge, metres; each finer level halves it.
  float voxel_size = 0.02f;
  /// Levels, 2 to 4: 2 cm, 1 cm and 5 mm by default.
  std::uint32_t levels = 3;
  /// Each level's truncation band, in that level's voxels.
  float trunc_voxels = 4.0f;
  /// Each level's initial hash buckets (8 blocks each); a level grows when
  /// allocation runs out of room.
  std::int32_t num_buckets = 4096;
  /// Fuse colour too: every level carries a `color` attribute. Without it,
  /// frames' colour is ignored.
  bool color = true;
  /// Per-voxel integration weight cap (one per observation).
  float max_weight = 20.0f;
  /// Classic keeps free space ahead of a surface; Dynamic clears it.
  IntegrationMode mode = IntegrationMode::Classic;
  /// How far, in metres, a block's surface must sit off the depth beyond the
  /// sensor's own floor for it to refine.
  float refine_offset = 0.001f;
  /// Residuals are sampled at one pixel in each stride x stride square.
  std::uint32_t pixel_stride = 4;
  /// Sampled cells (of 2 x 2 x 2 voxels) a block needs to be judged.
  std::uint32_t min_cells = 2;
  /// Frame sets between refinement checks.
  std::uint32_t check_every = 5;
  /// Consecutive checks a refined block must stay calm to coarsen.
  std::uint32_t calm_checks = 3;
};

/// @brief One level's state as of the last check (and, for @ref owned, the
///        last @ref AdaptiveGrid::owned_blocks).
struct AdaptiveLevelStats {
  float voxel_size = 0.0f;     ///< This level's voxel edge, metres.
  std::uint32_t blocks = 0;    ///< Allocated blocks.
  std::uint32_t refined = 0;   ///< Blocks handed to the next level.
  std::uint32_t owned = 0;     ///< Blocks this level meshes.
  float median_offset = 0.0f;  ///< Median judged block's offset, metres.
  float median_noise = 0.0f;   ///< Median judged block's per-sample noise.
};

/// @brief Adaptive resolution from ordinary voxel block grids: a stack of
///        @ref volume::VoxelBlockGrid levels whose voxel halves per level, a
///        block refined where the depth is systematically off the coarser
///        level's surface.
///
/// The coarsest level fuses every frame. A finer level allocates only from
/// the depth whose points fall in refined blocks of the level above, and
/// integrates the whole frame so it still carves free space. A block's parent
/// is the coarser block containing it (its coordinate halved), found through
/// the coarser level's own hash; nothing stores a pointer. Each level is a
/// complete grid: stamps, GC, extraction and the codec work on it unchanged.
///
/// Refinement is decided from the depth itself. A sampled point's residual is
/// the coarser level's TSDF there over the TSDF's gradient length: fused
/// distances run along camera rays and read 1 / cos(t) too long at view angle
/// t, and the gradient is steeper by the same factor, so the ratio is the
/// distance along the normal. Residuals add up per cell of 2 x 2 x 2 voxels
/// across frames (halved at each check), and a cell's systematic offset is
/// estimated as mean^2 - variance / n, so frame-to-frame sensor noise cancels.
/// A real sensor also leaves an offset no voxel size fits; the median
/// coarsest block's offset is taken as that floor. A block refines when its
/// offset exceeds floor + @ref AdaptiveGridConfig::refine_offset and twice the
/// noise left in its estimate, and coarsens after
/// @ref AdaptiveGridConfig::calm_checks checks below floor + half of it, or
/// once its surface is gone (a Dynamic fusion cleared it); a block merely out
/// of view keeps its detail. Coarsening removes the finer blocks it leaves
/// behind.
///
/// A level meshes the blocks it @ref owned_blocks: those whose ancestors are
/// refined and have fused for a check, and which are not refined themselves.
/// Seams between levels are not stitched.
///
/// Calls are externally serialized; the levels' grids belong to this object.
/// Device and allocator passed to @ref create must outlive it.
class VR_TSDF_API AdaptiveGrid {
 public:
  /// @brief Build the levels, the integrator and the refinement kernels.
  /// @param device     Compute device that outlives the grid.
  /// @param allocator  Allocator that outlives the grid.
  /// @param config     Validated before anything is created.
  /// @return The grid; `Status::Code::InvalidArgument` for an empty device
  ///         or allocator, or a configuration out of range or not finite; or
  ///         a backend or allocation error.
  static core::Result<AdaptiveGrid> create(core::Device& device,
                                           core::Allocator& allocator,
                                           const AdaptiveGridConfig& config);

  AdaptiveGrid() noexcept;
  ~AdaptiveGrid();
  AdaptiveGrid(AdaptiveGrid&& other) noexcept;
  AdaptiveGrid& operator=(AdaptiveGrid&& other) noexcept;
  AdaptiveGrid(const AdaptiveGrid&) = delete;
  AdaptiveGrid& operator=(const AdaptiveGrid&) = delete;

  /// @return Whether this owns live levels; false after a move.
  bool valid() const noexcept;

  /// @brief Fuse one set of frames -- a rig's cameras, or a single frame --
  ///        into every level, after deciding refinement when a check is due.
  ///
  /// The set's residuals are taken against the levels as they were before it,
  /// so a frame is never judged against itself. Each level allocates and
  /// integrates the whole set at once, growing its map when it runs out of
  /// room.
  /// @param frames   Posed depth (host or device) with optional colour, as
  ///                 @ref TsdfIntegrator::integrate takes them. Empty fuses
  ///                 nothing.
  /// @param metrics  Optional host/device stage rows.
  /// @return OK; `Status::Code::InvalidArgument` for an empty grid or a
  ///         frame its inputs refuse, before anything is fused; or a backend
  ///         or allocation error.
  core::Status fuse(const std::vector<FrameInput>& frames,
                    core::StageMetrics* metrics = nullptr);

  /// @brief The active blocks @p level meshes: its ancestors are all refined
  ///        and have fused for a check, and it is not refined itself.
  ///
  /// Pass them to `mesh::MarchingCubes::extract_device` with
  /// `level(l).block_list(...)`; every level's other blocks stay readable as
  /// neighbours.
  /// @param level  Less than @ref level_count.
  /// @return The blocks, or `Status::Code::InvalidArgument` for an empty
  ///         grid or a level out of range, or a compaction error.
  core::Result<std::vector<volume::BlockIndex>> owned_blocks(
      std::uint32_t level);

  /// @return Levels; 0 for an empty grid.
  std::uint32_t level_count() const noexcept;

  /// @brief One level's grid, finest last.
  /// @pre @ref valid and @p l is less than @ref level_count.
  volume::VoxelBlockGrid& level(std::uint32_t l);
  /// @copydoc level
  const volume::VoxelBlockGrid& level(std::uint32_t l) const;

  /// @brief Change @ref AdaptiveGridConfig::refine_offset from the next check
  ///        on; a value that is not finite and positive is ignored.
  void set_refine_offset(float metres) noexcept;

  /// @return The sensor floor the last check subtracted, metres; 0 before the
  ///         first check or for an empty grid.
  float sensor_floor() const noexcept;

  /// @return @p level's state, or a zero record for an empty grid or a level
  ///         out of range.
  AdaptiveLevelStats stats(std::uint32_t level) const noexcept;

 private:
  struct Impl;
  explicit AdaptiveGrid(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::tsdf
