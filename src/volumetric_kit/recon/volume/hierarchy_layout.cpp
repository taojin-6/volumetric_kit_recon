// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/volume/hierarchy_layout.hpp"

#include <cmath>
#include <limits>

namespace volumetric_kit::recon::volume {

Result<HierarchyLayout> HierarchyLayout::create(const VoxelGridParams& finest,
                                                std::uint32_t level_count) {
  VR_TRY(finest.validate());
  if (finest.block_size != kBlockSize) {
    return Status::invalid_argument("HierarchyLayout: block_size must be 8");
  }
  if (level_count == 0 || level_count > kMaxLevels) {
    return Status::invalid_argument(
        "HierarchyLayout: level_count must be 1..16");
  }
  if (!std::isfinite(finest.voxel_size) || !std::isfinite(finest.trunc_dist)) {
    return Status::invalid_argument(
        "HierarchyLayout: spacing and truncation must be finite");
  }
  const double coarsest_extent =
      std::ldexp(static_cast<double>(finest.voxel_size),
                 static_cast<int>(level_count - 1)) *
      kBlockSize;
  if (coarsest_extent > std::numeric_limits<float>::max()) {
    return Status::invalid_argument(
        "HierarchyLayout: coarsest block extent exceeds float range");
  }
  return HierarchyLayout(finest, level_count);
}

Result<VoxelGridParams> HierarchyLayout::level_grid(std::uint32_t level) const {
  if (level >= level_count_) {
    return Status::invalid_argument("HierarchyLayout: level is out of range");
  }
  VoxelGridParams grid = finest_;
  grid.voxel_size = std::ldexp(finest_.voxel_size, static_cast<int>(level));
  return grid;
}

Result<LevelBlockKey> HierarchyLayout::owning_block(Vec3f world,
                                                    std::uint32_t level) const {
  if (level >= level_count_) {
    return Status::invalid_argument("HierarchyLayout: level is out of range");
  }
  const double extent = std::ldexp(static_cast<double>(finest_.voxel_size),
                                   static_cast<int>(level)) *
                        kBlockSize;
  LevelBlockKey key{Vec3i(0), level};
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(world[axis])) {
      return Status::invalid_argument(
          "HierarchyLayout: world position must be finite");
    }
    const double coordinate =
        std::floor(static_cast<double>(world[axis]) / extent);
    if (coordinate < std::numeric_limits<std::int32_t>::min() ||
        coordinate > std::numeric_limits<std::int32_t>::max()) {
      return Status::invalid_argument(
          "HierarchyLayout: block coordinate exceeds int32 range");
    }
    key.coord[axis] = static_cast<std::int32_t>(coordinate);
  }
  return key;
}

Result<LevelBlockKey> HierarchyLayout::parent(LevelBlockKey child) const {
  // Test before incrementing: even UINT32_MAX is a rejected input, not a level
  // that wraps to zero and looks valid again.
  if (child.level >= level_count_ - 1) {
    return Status::invalid_argument(
        "HierarchyLayout: key has no parent in this layout");
  }
  LevelBlockKey result{Vec3i(0), child.level + 1};
  for (int axis = 0; axis < 3; ++axis) {
    const std::int32_t coordinate = child.coord[axis];
    // Truncation toward zero needs one correction on negative odd values.
    // Unlike subtracting one before division, this also handles INT32_MIN.
    result.coord[axis] = coordinate / 2 - (coordinate % 2 < 0 ? 1 : 0);
  }
  return result;
}

Result<LevelBlockKey> HierarchyLayout::child(LevelBlockKey parent,
                                             std::uint32_t octant) const {
  if (parent.level == 0 || parent.level >= level_count_ || octant >= 8) {
    return Status::invalid_argument(
        "HierarchyLayout: invalid parent level or child octant");
  }
  LevelBlockKey result{Vec3i(0), parent.level - 1};
  for (int axis = 0; axis < 3; ++axis) {
    const std::int64_t coordinate =
        static_cast<std::int64_t>(parent.coord[axis]) * 2 +
        static_cast<std::int64_t>((octant >> static_cast<unsigned>(axis)) & 1u);
    if (coordinate < std::numeric_limits<std::int32_t>::min() ||
        coordinate > std::numeric_limits<std::int32_t>::max()) {
      return Status::invalid_argument(
          "HierarchyLayout: child coordinate exceeds int32 range");
    }
    result.coord[axis] = static_cast<std::int32_t>(coordinate);
  }
  return result;
}

}  // namespace volumetric_kit::recon::volume
