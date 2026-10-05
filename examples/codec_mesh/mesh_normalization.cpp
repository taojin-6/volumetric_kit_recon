// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "mesh_normalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <utility>

namespace vkc = volumetric_kit::core;

namespace vr_example {
namespace {

Point3d point(const vr::Vec3f& p) { return {p.x, p.y, p.z}; }
Point3d subtract(const Point3d& a, const Point3d& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
double dot(const Point3d& a, const Point3d& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Point3d cross(const Point3d& a, const Point3d& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
Point3d unit(const Point3d& v) {
  const double length = std::sqrt(dot(v, v));
  return {v[0] / length, v[1] / length, v[2] / length};
}
MeshBounds bounds(const std::vector<vr::Vec3f>& positions) {
  MeshBounds b{point(positions.front()), point(positions.front())};
  for (const auto& p : positions) {
    const auto v = point(p);
    for (std::size_t a = 0; a < 3; ++a) {
      b.min[a] = std::min(b.min[a], v[a]);
      b.max[a] = std::max(b.max[a], v[a]);
    }
  }
  return b;
}

}  // namespace

bool MeshTopology::supports_signed() const noexcept {
  return components > 0 && boundary_edges == 0 && nonmanifold_edges == 0 &&
         inconsistent_edges == 0 && degenerate_triangles == 0 &&
         nonpositive_components == 0;
}

vkc::Result<MeshNormalization> normalize_mesh_height(
    vr::io::TriangleMesh& geometry, double height, const Point3d& up) {
  const double norm = dot(up, up);
  if (geometry.positions.empty() || !std::isfinite(height) || !(height > 0.0) ||
      !std::isfinite(norm) || !(norm > 0.0)) {
    return vkc::Status::invalid_argument(
        "normalization needs geometry, positive height and finite nonzero up");
  }
  MeshNormalization result;
  result.original = bounds(geometry.positions);
  result.up = unit(up);
  const Point3d reference = std::abs(result.up[2]) < 0.9
                                ? Point3d{0.0, 0.0, 1.0}
                                : Point3d{1.0, 0.0, 0.0};
  const Point3d x = unit(cross(result.up, reference));
  const Point3d z = cross(x, result.up);
  std::vector<Point3d> rotated;
  rotated.reserve(geometry.positions.size());
  Point3d lo{std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity()};
  Point3d hi{-lo[0], -lo[1], -lo[2]};
  for (const auto& position : geometry.positions) {
    const auto p = point(position);
    const Point3d r{dot(x, p), dot(result.up, p), dot(z, p)};
    for (std::size_t a = 0; a < 3; ++a) {
      if (!std::isfinite(r[a])) {
        return vkc::Status::invalid_argument(
            "normalization: nonfinite position");
      }
      lo[a] = std::min(lo[a], r[a]);
      hi[a] = std::max(hi[a], r[a]);
    }
    rotated.push_back(r);
  }
  result.original_height = hi[1] - lo[1];
  if (!(result.original_height > 0.0)) {
    return vkc::Status::invalid_argument(
        "normalization: zero height along up vector");
  }
  result.metres_per_unit = height / result.original_height;
  const Point3d origin{0.5 * (lo[0] + hi[0]), lo[1], 0.5 * (lo[2] + hi[2])};
  std::vector<vr::Vec3f> positions;
  positions.reserve(rotated.size());
  for (const auto& p : rotated) {
    Point3d normalized{};
    for (std::size_t a = 0; a < 3; ++a) {
      normalized[a] = (p[a] - origin[a]) * result.metres_per_unit;
      if (!std::isfinite(normalized[a]) ||
          std::abs(normalized[a]) > std::numeric_limits<float>::max()) {
        return vkc::Status::invalid_argument(
            "normalization: output overflows float");
      }
    }
    positions.push_back(
        {float(normalized[0]), float(normalized[1]), float(normalized[2])});
  }
  result.normalized = bounds(positions);
  if (!(result.normalized.max[1] > result.normalized.min[1])) {
    return vkc::Status::invalid_argument(
        "normalization: height collapses at float precision");
  }
  geometry.positions = std::move(positions);
  return result;
}

MeshTopology audit_mesh_topology(const vr::io::TriangleMesh& geometry) {
  struct Edge {
    std::size_t count = 0;
    int direction = 0;
  };
  std::map<std::pair<std::uint32_t, std::uint32_t>, Edge> edges;
  std::vector<std::uint32_t> parents(geometry.positions.size());
  std::iota(parents.begin(), parents.end(), 0u);
  auto root = [&parents](std::uint32_t v) {
    while (parents[v] != v) {
      parents[v] = parents[parents[v]];
      v = parents[v];
    }
    return v;
  };
  MeshTopology result;
  for (std::size_t t = 0; t < geometry.indices.size(); t += 3) {
    const auto a = geometry.indices[t];
    const auto b = geometry.indices[t + 1];
    const auto c = geometry.indices[t + 2];
    parents[root(b)] = root(a);
    parents[root(c)] = root(a);
    const auto ab =
        subtract(point(geometry.positions[b]), point(geometry.positions[a]));
    const auto ac =
        subtract(point(geometry.positions[c]), point(geometry.positions[a]));
    const auto area = cross(ab, ac);
    if (!(dot(area, area) > 0.0)) ++result.degenerate_triangles;
    for (std::size_t e = 0; e < 3; ++e) {
      const auto from = geometry.indices[t + e];
      const auto to = geometry.indices[t + (e + 1) % 3];
      auto& edge = edges[std::minmax(from, to)];
      ++edge.count;
      edge.direction += from < to ? 1 : -1;
    }
  }
  result.edges = edges.size();
  for (const auto& item : edges) {
    const auto& edge = item.second;
    if (edge.count == 1) ++result.boundary_edges;
    if (edge.count > 2) ++result.nonmanifold_edges;
    if (edge.count == 2 && edge.direction != 0) ++result.inconsistent_edges;
  }
  std::map<std::uint32_t, double> volumes;
  for (std::size_t t = 0; t < geometry.indices.size(); t += 3) {
    const auto a = geometry.indices[t];
    const auto b = geometry.indices[t + 1];
    const auto c = geometry.indices[t + 2];
    // Translate to one component vertex to avoid cancellation far from origin.
    const auto component = root(a);
    const auto origin = point(geometry.positions[component]);
    const auto p = subtract(point(geometry.positions[a]), origin);
    const auto q = subtract(point(geometry.positions[b]), origin);
    const auto r = subtract(point(geometry.positions[c]), origin);
    volumes[component] += dot(p, cross(q, r)) / 6.0;
  }
  result.components = volumes.size();
  for (const auto& volume : volumes) {
    result.signed_volume += volume.second;
    if (!(volume.second > 0.0)) ++result.nonpositive_components;
  }
  return result;
}

vr::mesh::Mesh geometry_mesh(const vr::io::TriangleMesh& geometry) {
  vr::mesh::Mesh mesh;
  mesh.indices = geometry.indices;
  mesh.vertices.reserve(geometry.positions.size());
  std::vector<Point3d> normals(geometry.positions.size());
  for (std::size_t t = 0; t < geometry.indices.size(); t += 3) {
    const auto a = geometry.indices[t];
    const auto b = geometry.indices[t + 1];
    const auto c = geometry.indices[t + 2];
    const auto normal = cross(
        subtract(point(geometry.positions[b]), point(geometry.positions[a])),
        subtract(point(geometry.positions[c]), point(geometry.positions[a])));
    for (const auto index : {a, b, c}) {
      for (std::size_t axis = 0; axis < 3; ++axis)
        normals[index][axis] += normal[axis];
    }
  }
  std::size_t index = 0;
  for (const auto& p : geometry.positions) {
    const auto normal = dot(normals[index], normals[index]) > 0.0
                            ? unit(normals[index])
                            : Point3d{0, 1, 0};
    mesh.vertices.push_back(
        {p,
         {float(normal[0]), float(normal[1]), float(normal[2])},
         {1, 0, 0, 1},
         {-1, -1},
         {1, 1, 1, 1}});
    ++index;
  }
  return mesh;
}

}  // namespace vr_example
