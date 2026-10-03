// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file mesh/hierarchical_geometry.hpp
/// @brief Integer geometry and ownership for the adaptive dual-cell extractor.

#include <array>
#include <cmath>
#include <limits>

#include "volumetric_kit/recon/volume/hierarchical_field_view.hpp"

namespace volumetric_kit::recon::mesh {

/// @brief Compute a leaf's primal vertex in finest-cell integer coordinates.
/// @param node The leaf's coordinate and level; sample storage is not read.
/// @param corner Local primal vertex, each component in [0,8].
/// @return Finest-cell coordinate, or InvalidArgument for unsupported levels,
///         invalid local coordinates, or overflow including the -1 incident
///         cell query. Uses the same conservative bounds as the device kernel.
inline Result<Vec3i> hierarchical_dual_vertex(
    const volume::HierarchicalNode& node, Vec3i corner) {
  if (node.level > 3 || corner.x < 0 || corner.x > 8 || corner.y < 0 ||
      corner.y > 8 || corner.z < 0 || corner.z > 8) {
    return Status::invalid_argument(
        "hierarchical dual vertex: invalid geometry");
  }
  const std::int32_t scale = 1 << node.level;
  const auto limit =
      (std::numeric_limits<std::int32_t>::max() - 1) / (8 * scale) - 1;
  if (node.coord.x < -limit || node.coord.x > limit || node.coord.y < -limit ||
      node.coord.y > limit || node.coord.z < -limit || node.coord.z > limit) {
    return Status::invalid_argument(
        "hierarchical dual vertex: coordinate overflow");
  }
  return Vec3i((node.coord.x * 8 + corner.x) * scale,
               (node.coord.y * 8 + corner.y) * scale,
               (node.coord.z * 8 + corner.z) * scale);
}

/// @brief Return a leaf sample's cell-centered world position.
/// @param node Leaf geometry, as for hierarchical_dual_vertex.
/// @param local Local sample coordinate, each component in [0,7].
/// @param finest_voxel_size Finest spacing in metres, finite and positive.
/// @return World center, or InvalidArgument for invalid geometry or spacing.
inline Result<Vec3f> hierarchical_sample_center(
    const volume::HierarchicalNode& node, Vec3i local,
    float finest_voxel_size) {
  if (local.x > 7 || local.y > 7 || local.z > 7 ||
      !std::isfinite(finest_voxel_size) || !(finest_voxel_size > 0.0f)) {
    return Status::invalid_argument(
        "hierarchical sample center: invalid geometry");
  }
  VR_ASSIGN(Vec3i lower, hierarchical_dual_vertex(node, local));
  const float half = 0.5f * float(1u << node.level);
  const Vec3f result((float(lower.x) + half) * finest_voxel_size,
                     (float(lower.y) + half) * finest_voxel_size,
                     (float(lower.z) + half) * finest_voxel_size);
  if (!std::isfinite(result.x) || !std::isfinite(result.y) ||
      !std::isfinite(result.z)) {
    return Status::invalid_argument(
        "hierarchical sample center: non-finite position");
  }
  return result;
}

/// @brief Decide which incident leaf emits a primal vertex's dual cell.
/// @param owner Candidate leaf, enumerating each of its 9 cubed vertices once.
/// @param incident The eight actual leaves adjacent to that primal vertex;
///                 repeated leaves are allowed, absent samples use ptr=-1.
/// @return True only for the finest incident leaf, then lowest (z,y,x) block
///         coordinate among ties. At least one incident leaf must be owner.
/// @note Mirrors shader gather's ownership rule; does not access sample values.
inline bool owns_hierarchical_dual_cell(
    const volume::HierarchicalNode& owner,
    const std::array<volume::HierarchicalNode, 8>& incident) noexcept {
  if (owner.ptr < 0 || owner.children != 0) return false;
  bool present = false;
  for (const auto& node : incident) {
    if (node.ptr < 0 || node.children != 0 || node.level < owner.level ||
        (node.level == owner.level &&
         volume::coord_less(node.coord, owner.coord))) {
      return false;
    }
    present |= node.level == owner.level && node.coord.x == owner.coord.x &&
               node.coord.y == owner.coord.y && node.coord.z == owner.coord.z;
  }
  return present;
}

}  // namespace volumetric_kit::recon::mesh
