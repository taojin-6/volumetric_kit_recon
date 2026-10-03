// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file volume/hierarchy_layout.hpp
/// @brief Dyadic block addresses and physical resolution for an adaptive grid.

#include <cstdint>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/volume/export.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace volumetric_kit::recon::volume {

/// @brief A block address qualified by resolution; level 0 is finest.
///
/// Coordinates count blocks at this key's level, rather than finest-level
/// voxels. A parent's coordinate is floor(child.coord / 2), including on
/// negative axes. Two keys with equal coordinates and different levels name
/// different blocks. This is a host-side address, not an uploaded shader ABI.
struct LevelBlockKey {
  Vec3i coord{0};           ///< Block coordinate at @ref level.
  std::uint32_t level = 0;  ///< Resolution level; spacing doubles each level.
};

/// @brief The geometry of a finite set of dyadic block resolutions.
///
/// Every block has an 8 x 8 x 8 payload. At level l its spacing is
/// `finest.voxel_size * 2^l` and its extent is eight times that spacing. A
/// key owns the Cartesian product of half-open intervals
/// `[coord * extent, (coord + 1) * extent)`, so its eight children partition
/// exactly the same region, including across the world origin.
///
/// Ownership describes a spatial region, not the placement of its scalar
/// samples. In particular, @ref owning_block floors a world position into a
/// region; the existing uniform @ref world_to_block instead chooses the block
/// of the nearest voxel. These operations are deliberately distinct.
///
/// This value object owns no voxel storage and changes no existing allocation,
/// integration, meshing, or codec behavior. Sample placement, active-leaf
/// ownership, and GPU mutation belong to the adaptive grid that uses it.
class VR_VOLUME_API HierarchyLayout {
 public:
  /// The first adaptive layout uses the existing production 8-cubed payload.
  static constexpr std::int32_t kBlockSize = 8;
  /// At most sixteen levels; level indices are 0 through 15.
  static constexpr std::uint32_t kMaxLevels = 16;

  /// @brief Validate and construct a hierarchy's physical layout.
  /// @param finest      Finest-level spacing, common physical truncation
  ///                    distance, and hash-table capacity. Its block size must
  ///                    be 8; all other @ref VoxelGridParams rules still apply.
  /// @param level_count Number of levels, in [1, @ref kMaxLevels].
  /// @return The layout, or @ref Status::invalid_argument for invalid grid
  ///         parameters, non-finite spacing/truncation, or a coarsest block
  ///         extent exceeding a finite float. No device work is performed.
  static Result<HierarchyLayout> create(const VoxelGridParams& finest,
                                        std::uint32_t level_count);

  /// @return Number of resolution levels in this layout.
  std::uint32_t level_count() const noexcept { return level_count_; }

  /// @brief Describe one level using the existing grid parameter vocabulary.
  ///
  /// Only voxel_size changes. The physical truncation distance, payload shape,
  /// and table capacity are copied from the finest level. This describes the
  /// resolution; creating uniform grids from it does not establish hierarchical
  /// ownership, nor change their sampling convention.
  /// @param level Level index, less than @ref level_count.
  /// @return The level parameters, or @ref Status::invalid_argument if @p level
  ///         is outside this layout.
  Result<VoxelGridParams> level_grid(std::uint32_t level) const;

  /// @brief Find the half-open block region containing a world position.
  ///
  /// Evaluates floor(world / block_extent), with double intermediates and
  /// bounds checks before converting to a signed block coordinate. It neither
  /// looks up allocated blocks nor chooses a resolution automatically.
  /// @param world World position in metres; all components must be finite.
  /// @param level Level index, less than @ref level_count.
  /// @return The key, or @ref Status::invalid_argument for an invalid level,
  ///         non-finite world position, or coordinate outside int32 range.
  Result<LevelBlockKey> owning_block(Vec3f world, std::uint32_t level) const;

  /// @brief Find the next-coarser block containing @p child.
  /// @param child A key at a level strictly below the coarsest level.
  /// @return The parent, using floor division on negative coordinates too, or
  ///         @ref Status::invalid_argument if the level is invalid or has no
  ///         parent in this layout.
  Result<LevelBlockKey> parent(LevelBlockKey child) const;

  /// @brief Find one of a block's eight next-finer children.
  /// @param parent A key at a valid level other than the finest level.
  /// @param octant Child index in [0, 7]: x is bit 0, y bit 1, z bit 2.
  /// @return The child, or @ref Status::invalid_argument for an invalid level,
  ///         octant, or child coordinate outside int32 range. Overflow is
  ///         checked before narrowing; the input key is never modified.
  Result<LevelBlockKey> child(LevelBlockKey parent, std::uint32_t octant) const;

 private:
  HierarchyLayout(const VoxelGridParams& finest, std::uint32_t level_count)
      : finest_(finest), level_count_(level_count) {}

  VoxelGridParams finest_;
  std::uint32_t level_count_;
};

}  // namespace volumetric_kit::recon::volume
