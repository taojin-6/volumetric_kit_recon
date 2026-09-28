// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file mesh_distance.hpp
/// @brief How far one triangle mesh is from another: the codec example's
///        quality metric.
///
/// Two directions, named as the codec's ground-truth test names them (so the
/// figures compare across the two):
///  - **accuracy**: each vertex of the mesh under test, its distance to the
///    reference surface -- is what was decoded where the surface was?
///  - **coverage**: each vertex of the reference, its distance to the surface
///    under test -- is all of the surface still there?
///
/// Distances are to the nearest point on any triangle, not to the nearest
/// vertex, and they are measured up to a `reach`: a point with nothing within
/// it reads as `reach` and is counted apart, so a hole shows up as a count
/// rather than as an unbounded mean. Header-only so the unit test in `tests/`
/// can include it without linking the examples' library.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief The point of segment `ab` nearest to `p`; `a` itself when the
///        segment has no length.
inline vr::Vec3f closest_point_on_segment(vr::Vec3f p, vr::Vec3f a,
                                          vr::Vec3f b) {
  const vr::Vec3f ab = b - a;
  const float len2 = vr::dot(ab, ab);
  if (!(len2 > 0.0f)) return a;
  return a + ab * std::clamp(vr::dot(p - a, ab) / len2, 0.0f, 1.0f);
}

/// @brief The point of triangle `abc` nearest to `p`: its projection onto the
///        face when that lands inside, and otherwise the nearest point of the
///        three edges.
///
/// Not Ericson's region test ("Real-Time Collision Detection", 5.1.5), which
/// divides by an edge's squared length and by the area: with two corners
/// coincident it returns NaN (for half of all queries when `a == b`) or a
/// point millimetres off (`b == c`), and for a micron sliver NaN now and
/// then. Marching cubes can emit both -- a voxel exactly at the iso level
/// puts every vertex on its edges at its corner, and one just off it makes
/// slivers -- and a NaN drops the triangle from the search without a word
/// (`std::min` keeps the other operand). The face is used only while its
/// normal is far above the rounding in the cross product that makes it (a
/// sine of 1e-4 at `a`, a thousand times float epsilon); a thinner triangle
/// is measured by its edges, which are within its height -- a micron at
/// 1 cm -- of its face.
inline vr::Vec3f closest_point_on_triangle(vr::Vec3f p, vr::Vec3f a,
                                           vr::Vec3f b, vr::Vec3f c) {
  const vr::Vec3f ab = b - a;
  const vr::Vec3f bc = c - b;
  const vr::Vec3f ca = a - c;
  const vr::Vec3f n = vr::cross(ab, c - a);
  const float n2 = vr::dot(n, n);
  if (n2 > 1e-8f * vr::dot(ab, ab) * vr::dot(ca, ca)) {
    const vr::Vec3f q = p - n * (vr::dot(p - a, n) / n2);
    if (vr::dot(vr::cross(ab, q - a), n) >= 0.0f &&
        vr::dot(vr::cross(bc, q - b), n) >= 0.0f &&
        vr::dot(vr::cross(ca, q - c), n) >= 0.0f) {
      return q;
    }
  }
  const vr::Vec3f edges[] = {closest_point_on_segment(p, a, b),
                             closest_point_on_segment(p, b, c),
                             closest_point_on_segment(p, c, a)};
  vr::Vec3f best = edges[0];
  for (const vr::Vec3f& e : edges) {
    if (vr::dot(p - e, p - e) < vr::dot(p - best, p - best)) best = e;
  }
  return best;
}

/// @brief Distance from points to one triangle mesh's surface, up to a reach,
///        through a hash of cells half the reach on a side.
///
/// Each triangle is filed under every cell its bounding box touches, so its
/// nearest point to a query lies in one of them. A query scans its own cell
/// first, then only the neighbours whose box is nearer than the best found
/// so far -- with a best under a millimetre, few of them. Keeps a reference
/// to the mesh, which must outlive it.
class MeshDistance {
 public:
  /// @param mesh   The surface, by vertices and indices.
  /// @param reach  The farthest distance measured, metres (> 0).
  MeshDistance(const vr::mesh::Mesh& mesh, float reach)
      : mesh_(mesh), reach_(reach), cell_(reach / float(kRings)) {
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
      const vr::Vec3f& a = mesh.vertices[mesh.indices[t]].position;
      const vr::Vec3f& b = mesh.vertices[mesh.indices[t + 1]].position;
      const vr::Vec3f& c = mesh.vertices[mesh.indices[t + 2]].position;
      const vr::Vec3i lo = cell_of(glm::min(a, glm::min(b, c)));
      const vr::Vec3i hi = cell_of(glm::max(a, glm::max(b, c)));
      for (int z = lo.z; z <= hi.z; ++z) {
        for (int y = lo.y; y <= hi.y; ++y) {
          for (int x = lo.x; x <= hi.x; ++x) {
            cells_[key(vr::Vec3i(x, y, z))].push_back(
                static_cast<std::uint32_t>(t));
          }
        }
      }
    }
  }

  /// @return The distance from @p p to the nearest point of the surface, or
  ///         exactly `reach` when nothing lies nearer.
  float distance(vr::Vec3f p) const {
    const float reach2 = reach_ * reach_;
    float best2 = reach2;
    const vr::Vec3i centre = cell_of(p);
    scan(centre, p, best2);
    // Anything within reach is within kRings cells of the centre.
    for (int dz = -kRings; dz <= kRings; ++dz) {
      const float gz = gap2(p.z, centre.z + dz);
      if (gz >= best2) continue;
      for (int dy = -kRings; dy <= kRings; ++dy) {
        const float gy = gz + gap2(p.y, centre.y + dy);
        if (gy >= best2) continue;
        for (int dx = -kRings; dx <= kRings; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          if (gy + gap2(p.x, centre.x + dx) >= best2) continue;
          scan(centre + vr::Vec3i(dx, dy, dz), p, best2);
        }
      }
    }
    return best2 < reach2 ? std::sqrt(best2) : reach_;
  }

  /// @return The mesh this measures to.
  const vr::mesh::Mesh& mesh() const noexcept { return mesh_; }

  /// @return The reach this was built with.
  float reach() const noexcept { return reach_; }

 private:
  static constexpr int kRings = 2;  // cells per reach

  vr::Vec3i cell_of(vr::Vec3f p) const {
    return vr::Vec3i(glm::floor(p / cell_));
  }
  // The squared distance from coordinate v to cell i's slab along one axis.
  float gap2(float v, int i) const {
    const float lo = float(i) * cell_;
    const float d = v < lo ? lo - v : v - (lo + cell_);
    return d > 0.0f ? d * d : 0.0f;
  }
  // Lower best2 to the nearest triangle filed under cell c, if nearer.
  void scan(vr::Vec3i c, vr::Vec3f p, float& best2) const {
    const auto it = cells_.find(key(c));
    if (it == cells_.end()) return;
    for (std::uint32_t t : it->second) {
      const vr::Vec3f d =
          p - closest_point_on_triangle(
                  p, mesh_.vertices[mesh_.indices[t]].position,
                  mesh_.vertices[mesh_.indices[t + 1]].position,
                  mesh_.vertices[mesh_.indices[t + 2]].position);
      best2 = std::min(best2, vr::dot(d, d));
    }
  }
  // 21 bits per axis, offset to unsigned: +-2^20 cells, ~10 km at 1 cm.
  static std::uint64_t key(vr::Vec3i c) {
    const auto bits = [](int v) {
      return static_cast<std::uint64_t>(v + (1 << 20)) & 0x1FFFFFu;
    };
    return bits(c.x) | (bits(c.y) << 21) | (bits(c.z) << 42);
  }

  const vr::mesh::Mesh& mesh_;
  float reach_;
  float cell_;
  std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> cells_;
};

/// @brief A distance distribution: over the points within reach, and a count
///        of those beyond it.
struct DistanceStats {
  std::size_t count = 0;         ///< Points measured.
  std::size_t beyond_reach = 0;  ///< Of those, nothing within reach.
  double mean = 0.0;             ///< Metres, over the points within reach.
  double rms = 0.0;              ///< Metres, over the points within reach.
  /// Metres, over the points within reach: the nearest-rank 95th percentile.
  double p95 = 0.0;
  /// Metres, over the points within reach, so under `reach` by construction;
  /// @ref beyond_reach counts the rest.
  double max = 0.0;
};

/// @brief Summarize distances measured with @p reach.
/// @param distances  Consumed (sorted in place).
inline DistanceStats summarize(std::vector<float> distances, float reach) {
  DistanceStats s;
  s.count = distances.size();
  std::sort(distances.begin(), distances.end());
  std::size_t within = 0;
  while (within < distances.size() && distances[within] < reach) {
    ++within;
  }
  s.beyond_reach = distances.size() - within;
  if (within == 0) {
    return s;
  }
  for (std::size_t i = 0; i < within; ++i) {
    s.mean += distances[i];
    s.rms += double(distances[i]) * distances[i];
  }
  s.mean /= double(within);
  s.rms = std::sqrt(s.rms / double(within));
  s.p95 = distances[(within * 95 + 99) / 100 - 1];  // ceil(0.95 n)-th
  s.max = distances[within - 1];
  return s;
}

/// @brief The distance from every @p stride-th vertex of @p points to
///        @p surface, summarized.
inline DistanceStats measure_distances(const vr::mesh::Mesh& points,
                                       const MeshDistance& surface,
                                       std::size_t stride) {
  stride = std::max<std::size_t>(stride, 1);
  std::vector<float> out;
  out.reserve(points.vertices.size() / stride + 1);
  for (std::size_t i = 0; i < points.vertices.size(); i += stride) {
    out.push_back(surface.distance(points.vertices[i].position));
  }
  return summarize(std::move(out), surface.reach());
}

/// @brief Accuracy and coverage of @p test against @p reference.
struct MeshComparison {
  DistanceStats accuracy;  ///< @p test's vertices to @p reference's surface.
  DistanceStats coverage;  ///< @p reference's vertices to @p test's surface.
};

/// @brief Compare two meshes both ways.
/// @param reference  The mesh taken as the truth.
/// @param test       The mesh being judged.
/// @param reach      Farthest distance measured, metres.
/// @param stride     Measure every `stride`-th vertex in each direction (at
///                   least 1): a room-sized mesh has a million vertices, and
///                   a regular subsample estimates the same distribution.
inline MeshComparison compare_meshes(const MeshDistance& reference,
                                     const vr::mesh::Mesh& test,
                                     std::size_t stride = 1) {
  const MeshDistance to_test(test, reference.reach());
  return MeshComparison{measure_distances(test, reference, stride),
                        measure_distances(reference.mesh(), to_test, stride)};
}

/// @brief Compare two meshes both ways, hashing @p reference for this one
///        comparison.
/// @param reach  Farthest distance measured, metres.
inline MeshComparison compare_meshes(const vr::mesh::Mesh& reference,
                                     const vr::mesh::Mesh& test, float reach,
                                     std::size_t stride = 1) {
  return compare_meshes(MeshDistance(reference, reach), test, stride);
}

}  // namespace vr_example
