// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file volume/hierarchical_grid.hpp
/// @brief Device-resident forest of fixed-size adaptive TSDF blocks.

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/volume/export.hpp"
#include "volumetric_kit/recon/volume/hierarchical_field_view.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace volumetric_kit::recon::volume {

/// @brief Fixed capacities and physical resolution for a hierarchical grid.
struct HierarchicalGridConfig {
  /// Finest spacing, physical truncation distance, and root hash capacity.
  /// Every payload is 8 cubed. Root capacity never grows: its slots are the
  /// first nodes of the forest and child indices follow them.
  VoxelGridParams finest = VoxelGridParams::defaults();
  std::uint32_t level_count = 2;  ///< 1..4; level 0 is finest.
  std::uint32_t child_block_capacity =
      0;               ///< Additional nodes, divisible by 8.
  bool color = false;  ///< Allocate packed encoded RGB too.
};

/// @brief Counts from one bounded online topology update.
struct HierarchicalSplitStats {
  std::uint32_t split = 0;      ///< Leaves replaced by eight children.
  std::uint32_t deferred = 0;   ///< Eligible requests beyond the event budget.
  std::uint32_t exhausted = 0;  ///< Eligible requests with no child group free.
  std::uint32_t rejected = 0;   ///< Child coordinates would overflow int32.
};

/// @brief Counts from one conservative online coarsening update.
struct HierarchicalMergeStats {
  std::uint32_t merged = 0;  ///< Eight sibling leaves replaced by their parent.
  std::uint32_t deferred = 0;  ///< Stable candidates beyond the event budget.
  std::uint32_t pending =
      0;  ///< Observed candidates still below the age limit.
};

/// @brief A sparse root hash and an octree of device-resident 8-cubed blocks.
///
/// Only leaves are integrated or meshed. Parent payloads remain available for
/// transfer. Interior child samples interpolate metric TSDF from eight observed
/// parent samples and interpolate color in linear light when all colors exist.
/// Unsupported or boundary samples retain the nearest parent's value. Inherited
/// confidence is bounded by every contributing weight and the caller's cap.
/// Interior transfer preserves an observed affine field but cannot recover
/// lost detail; subsequent depth observations must refine the field.
///
/// All nodes, leaf indices, and attributes stay on the device. Operations read
/// back control counts only. Capacities are explicit and fixed; failed root
/// allocation reports its residue and a full child pool leaves requested
/// parents intact. No caller can mutate the root map separately.
///
/// Scalar samples are at cell centers: `(8 * block.coord + local + 0.5) *
/// spacing`. This is a separate field from the existing node-sampled uniform
/// VoxelBlockGrid. All topology calls invalidate previously returned views.
/// Calls and borrowed-buffer consumers must be externally serialized.
///
/// Desired levels describe one topology-update snapshot. Inactive node slots
/// must contain UINT32_MAX. One update may call split once and then merge once
/// with that same buffer: new children have no request, and merge only frees
/// slots. Regenerate requests before the next update, or after allocation or
/// clear. In particular, merge followed by split must use fresh requests,
/// because split can reuse node indices freed by merge. Acquire a fresh field
/// view after the update before integration or meshing.
class VR_VOLUME_API HierarchicalGrid {
 public:
  /// @brief Construct a grid and zero its device-local storage.
  /// @param device Device that outlives the grid.
  /// @param allocator Allocator that outlives the grid.
  /// @param config Resolution and fixed capacity; validates before allocation.
  /// @return The grid, or invalid arguments, unsupported device limits, or a
  ///         backend/allocation error. No roots are allocated initially.
  static Result<HierarchicalGrid> create(Device& device, Allocator& allocator,
                                         const HierarchicalGridConfig& config);

  HierarchicalGrid() noexcept;
  ~HierarchicalGrid();
  HierarchicalGrid(HierarchicalGrid&& other) noexcept;
  HierarchicalGrid& operator=(HierarchicalGrid&& other) noexcept;
  HierarchicalGrid(const HierarchicalGrid&) = delete;
  HierarchicalGrid& operator=(const HierarchicalGrid&) = delete;

  /// @return Whether this owns a live grid; false after a move.
  bool valid() const noexcept;
  /// @return Total reserved root and child nodes, or zero when empty.
  std::uint32_t node_capacity() const noexcept;
  /// @return Leaf populations by level from the last successful
  ///         @ref prepare_leaves, all zero when empty. Prepare after topology
  ///         changes before using these counts; this accessor does no work.
  std::array<std::uint32_t, 4> leaf_counts() const noexcept;

  /// @brief Allocate coarse root regions from posed depth frames.
  ///
  /// Reuses the root allocator's conservative truncation-band dilation, with
  /// half a root voxel of allocation-only padding for its nearest-node center
  /// selection. The field's physical truncation distance remains unchanged.
  /// Existing refined roots retain their descendants and attributes.
  /// The declared camera frustum and allocation band must fit the signed
  /// finest-cell domain (-2^30, 2^30), with neighbor-query headroom. Validation
  /// checks camera metadata before dispatch and never reads depth to the host.
  /// @param frames Host or device depth inputs in metres.
  /// @param out_failures Optional root allocation failure details.
  /// @param metrics Optional host/device stage rows.
  /// @return Root allocations still failing after retries, or a non-OK status.
  Result<std::uint32_t> allocate_from_depth(
      const std::vector<DepthInput>& frames,
      AllocFailures* out_failures = nullptr, StageMetrics* metrics = nullptr);

  /// @brief Allocate prescribed coarse root coordinates.
  /// @param coords Root-level coordinates; ptr is ignored. Every coordinate
  ///               must have magnitude <= 2^30 / (8 * 2^max_level) - 2,
  ///               leaving headroom for descendant and neighbor arithmetic.
  /// @param count Number of coordinates; zero accepts a null pointer.
  /// @return Unsuccessful root allocations after retries, or a non-OK status.
  Result<std::uint32_t> allocate_roots(const BlockIndex* coords,
                                       std::uint32_t count);

  /// @brief Prepare the device leaf list and borrow the field buffers.
  /// @param metrics Optional host/device stage rows.
  /// @return A borrowed field view guarded by the topology generation. An
  ///         unchanged topology reuses its prepared list without dispatch or
  ///         readback. Only total and per-level leaf counts are read back when
  ///         the list is rebuilt.
  Result<HierarchicalFieldView> prepare_leaves(StageMetrics* metrics = nullptr);

  /// @brief Split requested leaves, subject to capacity and an event budget.
  ///
  /// A request is one desired level per node; UINT32_MAX means no request and
  /// is required in inactive slots. Follow the class's single-update ordering
  /// when sharing a request buffer with merge.
  /// Only current leaves with a finer desired level qualify. A complete group
  /// of eight children is reserved before the parent
  /// publishes it; the parent's payload is retained. Requests are not reset.
  /// @param requests Desired-level buffer with at least node_capacity uints.
  /// @param max_splits Maximum parents split by this call; zero does no work.
  /// @param transfer_weight_cap Maximum inherited observation weight, finite
  ///                            and positive. Interpolated confidence is also
  ///                            bounded by the least confident contributor.
  /// @param metrics Optional host/device stage rows.
  /// @return Completed, budget-deferred, and capacity-exhausted event counts,
  ///         or a non-OK status. Capacity exhaustion leaves leaves intact.
  Result<HierarchicalSplitStats> split(const Buffer& requests,
                                       std::uint32_t max_splits,
                                       float transfer_weight_cap = 1.0f,
                                       StageMetrics* metrics = nullptr);

  /// @brief Coarsen stable sibling leaves and recycle their child groups.
  ///
  /// All eight children must be leaves with observed desired levels at least
  /// as coarse as their parent. UINT32_MAX means unseen and resets the parent's
  /// consecutive-update age, as does a fine request from any child. Candidate
  /// selection precedes mutation, so one call cannot merge overlapping levels.
  /// Requests express the caller's geometry policy; this operation does not
  /// independently establish an error tolerance for discarding fine detail.
  ///
  /// A coarse sample is the weighted mean of its eight fine samples, with
  /// their mean weight, only if every fine sample is observed; otherwise it
  /// becomes unobserved. Color is averaged in linear light when all eight
  /// colors are observed. Returned groups are reusable by the next split.
  /// @param requests Desired levels, one uint per node as in @ref split.
  /// @param max_merges Maximum parents merged by this call; zero does no work.
  /// @param stable_updates Required consecutive qualifying calls, at least 1.
  /// @param metrics Optional host/device stage rows.
  /// @return Merge, deferred, and pending counts, or a non-OK status.
  Result<HierarchicalMergeStats> merge(const Buffer& requests,
                                       std::uint32_t max_merges,
                                       std::uint32_t stable_updates = 8,
                                       StageMetrics* metrics = nullptr);

  /// @brief Discard all roots, descendants, and attributes and reset the pool.
  /// @return OK, or a non-OK status on an empty grid or backend failure.
  Status clear();

 private:
  struct Impl;
  explicit HierarchicalGrid(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::volume
