// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file volume/hierarchical_grid.hpp
/// @brief Device-resident forest of fixed-size adaptive TSDF blocks.

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

/// @brief A sparse root hash and an octree of device-resident 8-cubed blocks.
///
/// Only leaves are integrated or meshed. Parent payloads remain available for
/// transfer; children inherit nearest-parent samples with weight capped by the
/// caller. Such transfer preserves observations approximately and introduces
/// no new detail; subsequent depth observations must refine the field.
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

  HierarchicalGrid() noexcept = default;
  ~HierarchicalGrid();
  HierarchicalGrid(HierarchicalGrid&& other) noexcept;
  HierarchicalGrid& operator=(HierarchicalGrid&& other) noexcept;
  HierarchicalGrid(const HierarchicalGrid&) = delete;
  HierarchicalGrid& operator=(const HierarchicalGrid&) = delete;

  /// @return Whether this owns a live grid; false after a move.
  bool valid() const noexcept;
  /// @return Total reserved root and child nodes, or zero when empty.
  std::uint32_t node_capacity() const noexcept;

  /// @brief Allocate coarse root regions from posed depth frames.
  ///
  /// Reuses the root allocator's conservative truncation-band dilation, with
  /// half a root voxel of allocation-only padding for its nearest-node center
  /// selection. The field's physical truncation distance remains unchanged.
  /// Existing refined roots retain their descendants and attributes.
  /// @param frames Host or device depth inputs in metres.
  /// @param out_failures Optional root allocation failure details.
  /// @param metrics Optional host/device stage rows.
  /// @return Root allocations still failing after retries, or a non-OK status.
  Result<std::uint32_t> allocate_from_depth(
      const std::vector<DepthInput>& frames,
      AllocFailures* out_failures = nullptr, StageMetrics* metrics = nullptr);

  /// @brief Allocate prescribed coarse root coordinates.
  /// @param coords Root-level coordinates; ptr is ignored.
  /// @param count Number of coordinates; zero accepts a null pointer.
  /// @return Unsuccessful root allocations after retries, or a non-OK status.
  Result<std::uint32_t> allocate_roots(const BlockIndex* coords,
                                       std::uint32_t count);

  /// @brief Prepare the device leaf list and borrow the field buffers.
  /// @param metrics Optional host/device stage rows.
  /// @return A checked-lifetime field view. An unchanged topology reuses its
  ///         prepared list without dispatch or readback. Only the leaf count
  ///         is read back when the list is rebuilt.
  Result<HierarchicalFieldView> prepare_leaves(StageMetrics* metrics = nullptr);

  /// @brief Split requested leaves, subject to capacity and an event budget.
  ///
  /// A request is one desired level per node; UINT32_MAX means no request.
  /// Only current leaves with a finer desired level qualify. A complete group
  /// of eight children is reserved before the parent
  /// publishes it; the parent's payload is retained. Requests are not reset.
  /// @param requests Desired-level buffer with at least node_capacity uints.
  /// @param max_splits Maximum parents split by this call; zero does no work.
  /// @param transfer_weight_cap Maximum inherited observation weight, finite
  ///                            and positive. New observations correct it.
  /// @param metrics Optional host/device stage rows.
  /// @return Completed, budget-deferred, and capacity-exhausted event counts,
  ///         or a non-OK status. Capacity exhaustion leaves leaves intact.
  Result<HierarchicalSplitStats> split(const Buffer& requests,
                                       std::uint32_t max_splits,
                                       float transfer_weight_cap = 1.0f,
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
