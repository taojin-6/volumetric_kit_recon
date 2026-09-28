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

/// @brief The point of triangle `abc` nearest to `p` (Ericson, "Real-Time
///        Collision Detection", 5.1.5): vertex, edge or face region.
inline vr::Vec3f closest_point_on_triangle(vr::Vec3f p, vr::Vec3f a,
                                           vr::Vec3f b, vr::Vec3f c) {
  const vr::Vec3f ab = b - a;
  const vr::Vec3f ac = c - a;
  const vr::Vec3f ap = p - a;
  const float d1 = vr::dot(ab, ap);
  const float d2 = vr::dot(ac, ap);
  if (d1 <= 0.0f && d2 <= 0.0f) return a;
  const vr::Vec3f bp = p - b;
  const float d3 = vr::dot(ab, bp);
  const float d4 = vr::dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3) return b;
  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    return a + ab * (d1 / (d1 - d3));
  }
  const vr::Vec3f cp = p - c;
  const float d5 = vr::dot(ab, cp);
  const float d6 = vr::dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6) return c;
  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    return a + ac * (d2 / (d2 - d6));
  }
  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
  }
  // Inside the face. A degenerate (zero-area) triangle never gets here: it
  // falls into one of the vertex or edge regions above.
  const float denom = 1.0f / (va + vb + vc);
  return a + ab * (vb * denom) + ac * (vc * denom);
}

/// @brief Distance from points to one triangle mesh's surface, up to a reach,
///        through a hash of `reach`-sized cells.
///
/// Each triangle is filed under every cell its bounding box touches, so a
/// query needs only the 27 cells around its own to see every triangle within
/// `reach`. Keeps a reference to the mesh, which must outlive it.
class MeshDistance {
 public:
  /// @param mesh   The surface, by vertices and indices.
  /// @param reach  The farthest distance measured, metres (> 0). Also the cell
  ///               edge; it should exceed the mesh's triangle size.
  MeshDistance(const vr::mesh::Mesh& mesh, float reach)
      : mesh_(mesh), reach_(reach) {
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
  ///         exactly `reach` when nothing lies within it.
  float distance(vr::Vec3f p) const {
    float best = reach_;
    const vr::Vec3i centre = cell_of(p);
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const auto it = cells_.find(key(centre + vr::Vec3i(dx, dy, dz)));
          if (it == cells_.end()) {
            continue;
          }
          for (std::uint32_t t : it->second) {
            const vr::Vec3f& a = mesh_.vertices[mesh_.indices[t]].position;
            const vr::Vec3f& b = mesh_.vertices[mesh_.indices[t + 1]].position;
            const vr::Vec3f& c = mesh_.vertices[mesh_.indices[t + 2]].position;
            best = std::min(
                best, vr::length(p - closest_point_on_triangle(p, a, b, c)));
          }
        }
      }
    }
    return best;
  }

  /// @return The reach this was built with.
  float reach() const noexcept { return reach_; }

 private:
  vr::Vec3i cell_of(vr::Vec3f p) const {
    return vr::Vec3i(glm::floor(p / reach_));
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
  std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> cells_;
};

/// @brief A distance distribution: over the points within reach, and a count
///        of those beyond it.
struct DistanceStats {
  std::size_t count = 0;         ///< Points measured.
  std::size_t beyond_reach = 0;  ///< Of those, nothing within reach.
  double mean = 0.0;             ///< Metres, over the points within reach.
  double rms = 0.0;              ///< Metres, over the points within reach.
  double p95 = 0.0;              ///< Metres, over the points within reach.
  double max = 0.0;              ///< Metres, over the points within reach.
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
  s.p95 = distances[std::min(within - 1, (within * 95) / 100)];
  s.max = distances[within - 1];
  return s;
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
inline MeshComparison compare_meshes(const vr::mesh::Mesh& reference,
                                     const vr::mesh::Mesh& test, float reach,
                                     std::size_t stride = 1) {
  stride = std::max<std::size_t>(stride, 1);
  auto measure = [&](const vr::mesh::Mesh& points,
                     const vr::mesh::Mesh& surface) {
    const MeshDistance d(surface, reach);
    std::vector<float> out;
    out.reserve(points.vertices.size() / stride + 1);
    for (std::size_t i = 0; i < points.vertices.size(); i += stride) {
      out.push_back(d.distance(points.vertices[i].position));
    }
    return summarize(std::move(out), reach);
  };
  return MeshComparison{measure(test, reference), measure(reference, test)};
}

}  // namespace vr_example
