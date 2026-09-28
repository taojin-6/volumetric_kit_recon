// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "mesh_topology.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <unordered_map>

namespace volumetric_kit::recon::tsdf::detail {
namespace {

// A position's exact bit pattern, with -0.0 folded onto +0.0 so the two zeros
// weld. Exact rather than within a tolerance: a tolerance would weld two
// vertices a modeller placed apart on purpose, and a soup's duplicates are
// bit-identical copies anyway.
struct PositionKey {
  std::uint32_t x, y, z;
  bool operator==(const PositionKey& o) const noexcept {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct PositionKeyHash {
  std::size_t operator()(const PositionKey& k) const noexcept {
    std::uint64_t h = k.x;
    h = h * 0x9E3779B97F4A7C15ull ^ k.y;
    h = h * 0x9E3779B97F4A7C15ull ^ k.z;
    return static_cast<std::size_t>(h ^ (h >> 29));
  }
};

std::uint32_t float_bits(float f) {
  f += 0.0f;  // -0.0 + 0.0 == +0.0
  std::uint32_t bits = 0;
  std::memcpy(&bits, &f, sizeof(bits));
  return bits;
}

PositionKey key_of(const Vec3f& v) {
  return {float_bits(v.x), float_bits(v.y), float_bits(v.z)};
}

bool all_finite(const Vec3f& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// One undirected edge's accumulation across the triangles that share it.
struct EdgeAccum {
  double nx = 0.0, ny = 0.0, nz = 0.0;  // sum of the unit face normals
  std::uint32_t count = 0;              // triangles that have this edge
  std::uint32_t forward = 0;            // of those, how many run low -> high
  // The corner (3t + k) at each end of the first two triangles to have this
  // edge: [i][0] at the lower vertex index, [i][1] at the higher. What joins
  // two triangles' corners into one fan around a vertex.
  std::uint32_t corner[2][2] = {{0, 0}, {0, 0}};
};

// Union-find over corners, for counting the fans around each vertex.
struct CornerSets {
  std::vector<std::uint32_t> parent;
  explicit CornerSets(std::size_t n) : parent(n) {
    for (std::size_t i = 0; i < n; ++i)
      parent[i] = static_cast<std::uint32_t>(i);
  }
  std::uint32_t find(std::uint32_t c) {
    while (parent[c] != c) {
      parent[c] = parent[parent[c]];
      c = parent[c];
    }
    return c;
  }
  void join(std::uint32_t a, std::uint32_t b) { parent[find(a)] = find(b); }
};

std::uint64_t edge_key(std::uint32_t a, std::uint32_t b) {
  const std::uint32_t lo = std::min(a, b);
  const std::uint32_t hi = std::max(a, b);
  return (static_cast<std::uint64_t>(lo) << 32) | hi;
}

// Normalize a double accumulation into a float direction, or zero when it has
// none to give (the faces around it cancel).
Vec3f unit_or_zero(double x, double y, double z) {
  const double len = std::sqrt(x * x + y * y + z * z);
  if (!(len > 1e-12)) {
    return Vec3f(0.0f);
  }
  return Vec3f(static_cast<float>(x / len), static_cast<float>(y / len),
               static_cast<float>(z / len));
}

}  // namespace

MeshTopology build_mesh_topology(const Vec3f* vertices,
                                 std::uint32_t vertex_count,
                                 const std::uint32_t* indices,
                                 std::uint32_t triangle_count) {
  MeshTopology topo;
  const std::size_t corners = 3 * std::size_t{triangle_count};
  topo.indices.assign(indices, indices + corners);
  topo.vertex_normals.assign(vertex_count, Vec3f(0.0f));
  topo.edge_normals.assign(corners, Vec3f(0.0f));

  // Weld: redirect every corner of a finite triangle to the first vertex at its
  // position. Then keep the triangles whose corners are still three vertices;
  // one that welded onto fewer has collapsed to an edge or a point, traverses
  // what edge it has both ways, and counting it would make a manifold edge
  // read as a non-manifold one. A triangle left out keeps its indices; no
  // kernel reads it, since it has zero area.
  std::vector<bool> in_mesh(triangle_count, false);
  std::unordered_map<PositionKey, std::uint32_t, PositionKeyHash> first_at;
  first_at.reserve(vertex_count);
  for (std::uint32_t t = 0; t < triangle_count; ++t) {
    std::uint32_t* w = &topo.indices[3 * std::size_t{t}];
    if (!all_finite(vertices[w[0]]) || !all_finite(vertices[w[1]]) ||
        !all_finite(vertices[w[2]])) {
      continue;
    }
    for (int k = 0; k < 3; ++k) {
      w[k] = first_at.emplace(key_of(vertices[w[k]]), w[k]).first->second;
    }
    in_mesh[t] = w[0] != w[1] && w[1] != w[2] && w[2] != w[0];
  }

  // Accumulate each face's unit normal into its three edges, and into its three
  // corners weighted by the angle there. Double precision, because a vertex
  // normal is a sum over every face around it and a float sum of many small
  // angles loses the ones that matter at a sharp corner. The enclosed volume
  // is summed about one of the mesh's own vertices, which keeps a mesh far
  // from the origin from cancelling its terms away.
  std::vector<double> vertex_accum(3 * std::size_t{vertex_count}, 0.0);
  std::unordered_map<std::uint64_t, EdgeAccum> edges;
  edges.reserve(corners);
  bool have_origin = false;
  double origin[3] = {0.0, 0.0, 0.0};
  for (std::uint32_t t = 0; t < triangle_count; ++t) {
    if (!in_mesh[t]) continue;
    const std::uint32_t* w = &topo.indices[3 * std::size_t{t}];
    double p[3][3];
    for (int k = 0; k < 3; ++k) {
      const Vec3f& v = vertices[w[k]];
      p[k][0] = v.x;
      p[k][1] = v.y;
      p[k][2] = v.z;
    }
    const double e1[3] = {p[1][0] - p[0][0], p[1][1] - p[0][1],
                          p[1][2] - p[0][2]};
    const double e2[3] = {p[2][0] - p[0][0], p[2][1] - p[0][1],
                          p[2][2] - p[0][2]};
    double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                   e1[0] * e2[1] - e1[1] * e2[0]};
    if (!have_origin) {
      std::copy(p[0], p[0] + 3, origin);
      have_origin = true;
    }
    const double r[3] = {p[0][0] - origin[0], p[0][1] - origin[1],
                         p[0][2] - origin[2]};
    topo.volume += (r[0] * n[0] + r[1] * n[1] + r[2] * n[2]) / 6.0;
    // Zero for three distinct corners in a line: a sliver the kernels skip,
    // kept for its adjacency, or a triangle whose float area survived only on
    // an FMA's rounding residue. Either contributes no normal -- dividing
    // would put a NaN into every edge and corner it touches.
    const double nlen = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    for (double& c : n) c = nlen > 0.0 ? c / nlen : 0.0;

    for (int k = 0; k < 3; ++k) {
      // The interior angle at corner k, between its two outgoing edges.
      const double* o = p[k];
      const double* a = p[(k + 1) % 3];
      const double* b = p[(k + 2) % 3];
      const double u[3] = {a[0] - o[0], a[1] - o[1], a[2] - o[2]};
      const double v[3] = {b[0] - o[0], b[1] - o[1], b[2] - o[2]};
      const double uu = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
      const double vv = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
      double angle = 0.0;
      if (uu > 0.0 && vv > 0.0) {
        const double cosine =
            (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]) / (uu * vv);
        angle = std::acos(std::clamp(cosine, -1.0, 1.0));
      }
      double* acc = &vertex_accum[3 * std::size_t{w[k]}];
      acc[0] += angle * n[0];
      acc[1] += angle * n[1];
      acc[2] += angle * n[2];

      const std::uint32_t head = w[k];
      const std::uint32_t tail = w[(k + 1) % 3];
      EdgeAccum& e = edges[edge_key(head, tail)];
      e.nx += n[0];
      e.ny += n[1];
      e.nz += n[2];
      if (e.count < 2) {
        const auto at_head = static_cast<std::uint32_t>(3 * std::size_t{t} + k);
        const auto at_tail =
            static_cast<std::uint32_t>(3 * std::size_t{t} + (k + 1) % 3);
        e.corner[e.count][0] = head < tail ? at_head : at_tail;
        e.corner[e.count][1] = head < tail ? at_tail : at_head;
      }
      ++e.count;
      if (head < tail) ++e.forward;
    }
  }

  // Classify every edge once, and mark the vertices on the rim: a voxel nearest
  // such a vertex lies past the edge of the surface, where there is no side.
  // A manifold edge joins its two triangles' corners at each end into one fan.
  std::vector<bool> boundary_vertex(vertex_count, false);
  std::vector<bool> on_nonmanifold_edge(vertex_count, false);
  CornerSets fans(corners);
  for (const auto& [key, e] : edges) {
    if (e.count == 1) {
      ++topo.boundary_edges;
      boundary_vertex[static_cast<std::uint32_t>(key >> 32)] = true;
      boundary_vertex[static_cast<std::uint32_t>(key & 0xFFFFFFFFu)] = true;
    } else if (e.count > 2) {
      ++topo.nonmanifold_edges;
      on_nonmanifold_edge[static_cast<std::uint32_t>(key >> 32)] = true;
      on_nonmanifold_edge[static_cast<std::uint32_t>(key & 0xFFFFFFFFu)] = true;
    } else {
      if (e.forward != 1) ++topo.inconsistent_edges;
      fans.join(e.corner[0][0], e.corner[1][0]);
      fans.join(e.corner[0][1], e.corner[1][1]);
    }
  }

  // A vertex whose corners fall into more than one fan is a pinch -- two sheets
  // touching at a point -- and its pseudonormal sums normals from surfaces that
  // do not bound the same side. Only the pinch the edge count cannot see is
  // counted: an end of a non-manifold edge splits into fans too, and is
  // already reported as that edge. A rim vertex is not a pinch worth refusing:
  // its pseudonormal is zeroed below, so no voxel is signed through it.
  constexpr std::uint32_t kNoFan = 0xFFFFFFFFu;
  std::vector<std::uint32_t> fan_of(vertex_count, kNoFan);
  std::vector<bool> pinched(vertex_count, false);
  for (std::uint32_t t = 0; t < triangle_count; ++t) {
    if (!in_mesh[t]) continue;
    for (int k = 0; k < 3; ++k) {
      const auto c = static_cast<std::uint32_t>(3 * std::size_t{t} + k);
      const std::uint32_t v = topo.indices[c];
      if (on_nonmanifold_edge[v] || boundary_vertex[v]) continue;
      const std::uint32_t root = fans.find(c);
      if (fan_of[v] == kNoFan) {
        fan_of[v] = root;
      } else if (fan_of[v] != root && !pinched[v]) {
        pinched[v] = true;
        ++topo.nonmanifold_vertices;
      }
    }
  }

  for (std::uint32_t t = 0; t < triangle_count; ++t) {
    if (!in_mesh[t]) continue;
    const std::uint32_t* w = &topo.indices[3 * std::size_t{t}];
    for (int k = 0; k < 3; ++k) {
      const EdgeAccum& e = edges.at(edge_key(w[k], w[(k + 1) % 3]));
      if (e.count == 2 && e.forward == 1) {
        topo.edge_normals[3 * std::size_t{t} + k] =
            unit_or_zero(e.nx, e.ny, e.nz);
      }
    }
  }
  for (std::uint32_t v = 0; v < vertex_count; ++v) {
    if (boundary_vertex[v]) continue;
    const double* acc = &vertex_accum[3 * std::size_t{v}];
    topo.vertex_normals[v] = unit_or_zero(acc[0], acc[1], acc[2]);
  }
  return topo;
}

}  // namespace volumetric_kit::recon::tsdf::detail
