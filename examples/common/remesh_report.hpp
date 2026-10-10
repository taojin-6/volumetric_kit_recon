// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/remesh_report.hpp
/// @brief What re-meshing a fused grid costs, and how much of it changed:
///        `fuse_replica`'s extract reports and its `--dirty-every` survey.
///        Header-only; compiled into the executables that mesh.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <unordered_set>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// The active blocks stamped changed after a tick, and how many blocks those
/// put back to marching cubes.
struct ChangedBlocks {
  std::uint32_t changed = 0;
  std::uint32_t remesh = 0;
};

/// @brief Count @p active's blocks stamped changed after @p since, and the
///        blocks a changed-only re-mesh would redo: a cell reads corners at
///        `base + {0,1}^3`, so a block re-meshes when any block of its own
///        `+{0,1}^3` neighbourhood changed. A slot past the stamps counts as
///        changed.
/// @return The counts, or the stamps' readback error; InvalidArgument for a
///         grid with no voxels per block.
inline vkc::Result<ChangedBlocks> changed_since(
    const vr::volume::VoxelBlockGrid& grid,
    const std::vector<vr::volume::BlockIndex>& active, std::uint32_t since) {
  const auto vpb = static_cast<std::uint32_t>(grid.grid().voxels_per_block);
  if (vpb == 0) {
    return vkc::Status::invalid_argument(
        "changed_since: voxels_per_block is 0");
  }
  VKC_ASSIGN(const std::vector<vr::volume::BlockStamp> stamps,
             grid.map().read_block_stamps());
  // 21 bits an axis, through unsigned casts since coordinates go negative.
  const auto key = [](const vr::Vec3i& c) {
    constexpr std::uint64_t kMask = (std::uint64_t{1} << 21) - 1;
    return ((std::uint64_t{static_cast<std::uint32_t>(c.x)} & kMask) << 42) |
           ((std::uint64_t{static_cast<std::uint32_t>(c.y)} & kMask) << 21) |
           (std::uint64_t{static_cast<std::uint32_t>(c.z)} & kMask);
  };
  std::unordered_set<std::uint64_t> changed;
  for (const vr::volume::BlockIndex& b : active) {
    const std::size_t slot = static_cast<std::uint32_t>(b.ptr) / vpb;
    if (slot >= stamps.size() ||
        vr::volume::tick_after(stamps[slot].changed, since)) {
      changed.insert(key(b.coord));
    }
  }
  ChangedBlocks out;
  out.changed = static_cast<std::uint32_t>(changed.size());
  for (const vr::volume::BlockIndex& b : active) {
    bool hit = false;
    for (int d = 0; d < 8 && !hit; ++d) {
      hit = changed.count(
                key(b.coord + vr::Vec3i(d & 1, (d >> 1) & 1, d >> 2))) != 0;
    }
    out.remesh += hit ? 1u : 0u;
  }
  return out;
}

/// @brief `--dirty-every`: the changed and re-mesh fractions of the active
///        blocks, over windows of fused frames, against a tick of its own.
///
/// A window's sample is the union of its frames' changes, which is what a
/// changed-only re-mesh at that cadence would redo. The report divides the
/// summed counts, so each window weighs by its own active set: the first one
/// builds the map from nothing, every block in it allocated and written, and
/// averaging per-window ratios would give it a steady-state window's vote.
class DirtySurvey {
 public:
  /// Starts the first window at @p grid's tick now.
  explicit DirtySurvey(const vr::volume::VoxelBlockGrid& grid)
      : since_(grid.map().tick()) {}

  /// @brief Close the window: count what changed since it opened, and open
  ///        the next at the grid's tick now.
  vkc::Status sample(vr::volume::VoxelBlockGrid& grid) {
    VKC_ASSIGN(const std::vector<vr::volume::BlockIndex> active,
               grid.map().compact_active_blocks());
    VKC_ASSIGN(const ChangedBlocks blocks, changed_since(grid, active, since_));
    since_ = grid.map().tick();
    if (active.empty()) return {};
    sum_changed_ += blocks.changed;
    sum_remesh_ += blocks.remesh;
    sum_active_ += active.size();
    last_ = blocks;
    last_active_ = static_cast<std::uint32_t>(active.size());
    ++windows_;
    return {};
  }

  /// @brief Print the fractions, or nothing before a window with blocks.
  ///
  /// Not a speedup: only the marching-cubes dispatch scales with the block
  /// count. Compaction walks every table slot regardless, the arena is sized
  /// by the whole surface, and a readback copies all of it, so the share is
  /// the ceiling a changed-only re-mesh could aim at.
  /// @param every  Frames a window, for the heading.
  void print(int every) const {
    if (windows_ == 0) return;
    const auto pct = [](std::uint64_t num, std::uint64_t den) {
      return den > 0
                 ? 100.0 * static_cast<double>(num) / static_cast<double>(den)
                 : 0.0;
    };
    std::printf(
        "dirty     %zu windows of %d frame(s)\n"
        "  changed %.2f%% of active blocks\n"
        "  remesh  %.2f%% once dilated into -x/-y/-z (the real set)\n"
        "  final   %u changed -> %u to re-mesh of %u active (dilation %.2fx)\n",
        windows_, every, pct(sum_changed_, sum_active_),
        pct(sum_remesh_, sum_active_), last_.changed, last_.remesh,
        last_active_,
        last_.changed > 0 ? static_cast<double>(last_.remesh) / last_.changed
                          : 1.0);
  }

  /// @return The windows sampled with any active block.
  std::size_t windows() const noexcept { return windows_; }

  /// @return The last such window's counts.
  const ChangedBlocks& last() const noexcept { return last_; }

 private:
  std::uint32_t since_;
  std::size_t windows_ = 0;
  std::uint64_t sum_changed_ = 0, sum_remesh_ = 0, sum_active_ = 0;
  ChangedBlocks last_;
  std::uint32_t last_active_ = 0;
};

/// @brief Every periodic re-mesh's phases, summed: the first extract of a run
///        faults in its arena and is not the steady state, so no one sample
///        stands for the rest.
class ExtractTotals {
 public:
  /// Add one extract's timings.
  void add(const vr::mesh::ExtractTimings& t) {
    ++extracts_;
    total_ms_ += t.total_ms();
    compact_ms_ += t.compact_ms;
    arena_ms_ += t.arena_alloc_ms;
    dispatch_ms_ += t.dispatch_ms;
    read_ms_ += t.readback_ms;
    dispatches_ += t.dispatches;
    last_ = t;
  }

  /// @brief Print the means and the last extract's size, or nothing before
  ///        an extract.
  /// @param path              How the extracts ran, for the heading.
  /// @param voxels_per_block  The grid's, for the cell count.
  void print(const char* path, std::int32_t voxels_per_block) const {
    if (extracts_ == 0) return;
    const double n = static_cast<double>(extracts_);
    const double cells = static_cast<double>(last_.active_blocks) *
                         static_cast<double>(voxels_per_block);
    std::printf(
        "remesh    %zu extracts, mean %.2f ms  (%s)\n"
        "  phases  compact %.2f  arena %.2f  dispatch %.2f  read %.2f\n"
        "  final   %u blocks -> %.2fM cells, %u tris (%.2f%% of cells), "
        "%.2f dispatches/extract\n",
        extracts_, total_ms_ / n, path, compact_ms_ / n, arena_ms_ / n,
        dispatch_ms_ / n, read_ms_ / n, last_.active_blocks, cells / 1e6,
        last_.emitted_triangles,
        cells > 0.0 ? 100.0 * last_.emitted_triangles / cells : 0.0,
        static_cast<double>(dispatches_) / n);
  }

 private:
  std::size_t extracts_ = 0;
  std::uint64_t dispatches_ = 0;
  double total_ms_ = 0.0, compact_ms_ = 0.0, arena_ms_ = 0.0;
  double dispatch_ms_ = 0.0, read_ms_ = 0.0;
  vr::mesh::ExtractTimings last_{};
};

/// @brief Print one extract's phases, its cells (one workgroup an active
///        block, striding its voxels) beside the triangles they emitted --
///        a low emit rate is a kernel gathering cells that produce nothing --
///        and the arena's fill.
/// @param voxels_per_block  The grid's, for the cell count.
inline void print_extract(const vr::mesh::ExtractTimings& t,
                          std::int32_t voxels_per_block) {
  const double cells = static_cast<double>(t.active_blocks) *
                       static_cast<double>(voxels_per_block);
  std::printf(
      "extract   %.1f ms in %u dispatch(es)\n"
      "  phases  compact %.2f  arena %.2f  desc %.2f  "
      "dispatch %.2f  read %.2f\n"
      "  blocks  %u active -> %.2fM cells, %u tris emitted (%.2f%% of cells)\n"
      "  arena   %.1f MB resident, %u tris planned (%.1f%% full)\n",
      t.total_ms(), t.dispatches, t.compact_ms, t.arena_alloc_ms,
      t.descriptor_ms, t.dispatch_ms, t.readback_ms, t.active_blocks,
      cells / 1e6, t.emitted_triangles,
      cells > 0.0 ? 100.0 * t.emitted_triangles / cells : 0.0,
      static_cast<double>(t.arena_bytes) / (1024.0 * 1024.0),
      t.triangle_capacity,
      t.triangle_capacity > 0
          ? 100.0 * static_cast<double>(t.emitted_triangles) /
                t.triangle_capacity
          : 0.0);
}

}  // namespace vr_example
