// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Host-only test for examples/common/mesh_distance.hpp, the codec example's
// quality metric: the closest point in every region of a triangle and on
// degenerate ones, the pruned cell search against brute force, the reach
// clamp, the summary statistics, and accuracy / coverage between meshes whose
// true distance is known -- a plane against a copy of itself shifted by a
// known amount, and against one with half its triangles gone. CPU-only.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "mesh_distance.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"

namespace vr = volumetric_kit::recon;
namespace mesh = volumetric_kit::recon::mesh;
using vr_example::closest_point_on_segment;
using vr_example::closest_point_on_triangle;
using vr_example::compare_meshes;
using vr_example::MeshDistance;
using vr_example::summarize;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

bool near(vr::Vec3f a, vr::Vec3f b, float eps = 1e-6f) {
  return vr::length(a - b) <= eps;
}

bool near(double a, double b, double eps = 1e-6) {
  return std::fabs(a - b) <= eps;
}

// An n x n grid of `cell`-sized squares in the plane z = `z`, two triangles
// each, starting at the origin. `keep_x` drops every square starting at or past
// that x.
mesh::Mesh plane(int n, float cell, float z, float keep_x = 1e9f) {
  mesh::Mesh m;
  for (int j = 0; j <= n; ++j) {
    for (int i = 0; i <= n; ++i) {
      mesh::Vertex v{};
      v.position = vr::Vec3f(float(i) * cell, float(j) * cell, z);
      m.vertices.push_back(v);
    }
  }
  const auto at = [n](int i, int j) {
    return static_cast<std::uint32_t>(j * (n + 1) + i);
  };
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      if (float(i) * cell >= keep_x) {
        continue;
      }
      m.indices.insert(m.indices.end(), {at(i, j), at(i + 1, j), at(i, j + 1)});
      m.indices.insert(m.indices.end(),
                       {at(i + 1, j), at(i + 1, j + 1), at(i, j + 1)});
    }
  }
  return m;
}

// A deterministic stream of floats in [lo, hi): a fixed LCG, so every
// standard library draws the same points.
struct Lcg {
  std::uint32_t state = 12345u;
  float operator()(float lo, float hi) {
    state = state * 1664525u + 1013904223u;
    return lo + (hi - lo) * float(state >> 8) / float(1u << 24);
  }
};

// The distance from p to triangle abc in double, where a sliver at room
// coordinates is still well conditioned: to the face when p projects inside
// it, else to the nearest edge.
double triangle_distance(const vr::Vec3f& pf, const vr::Vec3f& af,
                         const vr::Vec3f& bf, const vr::Vec3f& cf) {
  const glm::dvec3 p(pf), a(af), b(bf), c(cf);
  const auto segment = [&p](const glm::dvec3& u, const glm::dvec3& v) {
    const glm::dvec3 uv = v - u;
    const double len2 = glm::dot(uv, uv);
    const double t =
        len2 > 0.0 ? std::clamp(glm::dot(p - u, uv) / len2, 0.0, 1.0) : 0.0;
    return glm::length(p - (u + uv * t));
  };
  const glm::dvec3 n = glm::cross(b - a, c - a);
  const double n2 = glm::dot(n, n);
  if (n2 > 0.0) {
    const glm::dvec3 q = p - n * (glm::dot(p - a, n) / n2);
    if (glm::dot(glm::cross(b - a, q - a), n) >= 0.0 &&
        glm::dot(glm::cross(c - b, q - b), n) >= 0.0 &&
        glm::dot(glm::cross(a - c, q - c), n) >= 0.0) {
      return glm::length(p - q);
    }
  }
  return std::min({segment(a, b), segment(b, c), segment(c, a)});
}

int closest_point_case() {
  const vr::Vec3f a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
  // Face interior: straight down.
  CHECK(near(closest_point_on_triangle({0.25f, 0.25f, 1.0f}, a, b, c),
             {0.25f, 0.25f, 0.0f}));
  // Each vertex region.
  CHECK(near(closest_point_on_triangle({-1, -1, 0.5f}, a, b, c), a));
  CHECK(near(closest_point_on_triangle({2, -1, 0}, a, b, c), b));
  CHECK(near(closest_point_on_triangle({-1, 2, 0}, a, b, c), c));
  // Each edge region.
  CHECK(near(closest_point_on_triangle({0.5f, -1, 0}, a, b, c),
             {0.5f, 0.0f, 0.0f}));
  CHECK(near(closest_point_on_triangle({-1, 0.5f, 0}, a, b, c),
             {0.0f, 0.5f, 0.0f}));
  CHECK(
      near(closest_point_on_triangle({1, 1, 0}, a, b, c), {0.5f, 0.5f, 0.0f}));
  // A point on the triangle is its own closest point.
  CHECK(near(closest_point_on_triangle({0.2f, 0.3f, 0}, a, b, c),
             {0.2f, 0.3f, 0.0f}));
  return 0;
}

// Marching cubes emits triangles with coincident corners, whenever a voxel
// sits exactly at the iso level, and slivers beside them. Each is measured
// by its edges: never NaN, and never farther than the nearest edge.
int degenerate_case() {
  const vr::Vec3f a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
  const vr::Vec3f queries[] = {{0.5f, 0.5f, 1.0f}, {2, -1, 0},
                               {-1, 0.5f, 0.3f},   {0.25f, 0.0f, -2.0f},
                               {0.9f, 0.2f, 0.1f}, {0.1f, 0.9f, 0.0f}};
  for (const vr::Vec3f& p : queries) {
    // Two corners coincident, each pair: the triangle is the segment.
    CHECK(near(closest_point_on_triangle(p, a, a, c),
               closest_point_on_segment(p, a, c)));
    CHECK(near(closest_point_on_triangle(p, a, b, b),
               closest_point_on_segment(p, a, b)));
    CHECK(near(closest_point_on_triangle(p, a, b, a),
               closest_point_on_segment(p, a, b)));
    // All three: the point.
    CHECK(near(closest_point_on_triangle(p, b, b, b), b));
    // Collinear: the segment spanning them.
    CHECK(near(closest_point_on_triangle(p, a, b, vr::Vec3f(0.5f, 0, 0)),
               closest_point_on_segment(p, a, b)));
  }
  CHECK(near(closest_point_on_segment({0.3f, 1.0f, 0}, a, a), a));

  // Slivers at room coordinates, one corner a few ulps from another and the
  // third a voxel away, where the region test returned NaN now and then: the
  // double-precision distance, to float rounding at 3 m.
  Lcg rng;
  const vr::Vec3f base(3.2f, 1.7f, 2.9f);
  for (int i = 0; i < 2000; ++i) {
    const vr::Vec3f p0 =
        base +
        vr::Vec3f(rng(-0.01f, 0.01f), rng(-0.01f, 0.01f), rng(-0.01f, 0.01f));
    const vr::Vec3f p1 = p0 + vr::Vec3f(rng(-1e-6f, 1e-6f), rng(-1e-6f, 1e-6f),
                                        rng(-1e-6f, 1e-6f));
    const vr::Vec3f p2 = p0 + vr::Vec3f(rng(-0.01f, 0.01f), rng(-0.01f, 0.01f),
                                        rng(-0.01f, 0.01f));
    const vr::Vec3f p = base + vr::Vec3f(rng(-0.04f, 0.04f), rng(-0.04f, 0.04f),
                                         rng(-0.04f, 0.04f));
    const vr::Vec3f q = closest_point_on_triangle(p, p0, p1, p2);
    CHECK(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z));
    CHECK(std::fabs(double(vr::length(p - q)) -
                    triangle_distance(p, p0, p1, p2)) <= 1e-6);
  }
  return 0;
}

// The cell search prunes neighbours by distance; it must find what a scan of
// every triangle finds. A bumpy sheet of 1 cm triangles, some of them
// degenerate, against points scattered within and beyond the reach.
int search_case() {
  Lcg rng;
  mesh::Mesh m = plane(30, 0.01f, 0.0f);
  for (mesh::Vertex& v : m.vertices) {
    v.position += vr::Vec3f(rng(-0.004f, 0.004f), rng(-0.004f, 0.004f),
                            rng(-0.006f, 0.006f));
  }
  for (std::size_t t = 0; t < m.indices.size(); t += 3 * 17) {
    m.indices[t + 1] = m.indices[t];  // a zero-area triangle
  }
  const float reach = 0.04f;
  const MeshDistance d(m, reach);
  int within = 0;
  for (int i = 0; i < 3000; ++i) {
    const vr::Vec3f p(rng(-0.05f, 0.35f), rng(-0.05f, 0.35f),
                      rng(-0.05f, 0.05f));
    float best = reach;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
      best = std::min(
          best, vr::length(p - closest_point_on_triangle(
                                   p, m.vertices[m.indices[t]].position,
                                   m.vertices[m.indices[t + 1]].position,
                                   m.vertices[m.indices[t + 2]].position)));
    }
    CHECK(near(double(d.distance(p)), double(best), 1e-7));
    within += best < reach ? 1 : 0;
  }
  CHECK(within > 1000 && within < 3000);  // both sides of the reach
  return 0;
}

int distance_case() {
  const mesh::Mesh m = plane(20, 0.01f, 0.0f);  // 0.2 m square
  const MeshDistance d(m, 0.05f);
  CHECK(near(double(d.distance({0.03f, 0.07f, 0.02f})), 0.02, 1e-6));
  CHECK(near(double(d.distance({0.1f, 0.1f, -0.013f})), 0.013, 1e-6));
  // Off the edge of the square: to the edge, not to the plane.
  CHECK(near(double(d.distance({0.23f, 0.1f, 0.0f})), 0.03, 1e-6));
  // Nothing within reach reads as the reach, exactly.
  CHECK(d.distance({0.1f, 0.1f, 0.2f}) == 0.05f);
  CHECK(d.distance({5.0f, 5.0f, 0.0f}) == 0.05f);
  return 0;
}

int summarize_case() {
  const vr_example::DistanceStats s =
      summarize({0.4f, 0.1f, 0.3f, 0.2f, 1.0f, 1.0f}, 1.0f);
  CHECK(s.count == 6);
  CHECK(s.beyond_reach == 2);  // the two at the reach
  CHECK(near(s.mean, 0.25, 1e-6));
  CHECK(near(s.rms, std::sqrt(0.075), 1e-6));
  CHECK(near(s.max, 0.4, 1e-6));
  CHECK(near(s.p95, 0.4, 1e-6));
  const vr_example::DistanceStats none = summarize({}, 1.0f);
  CHECK(none.count == 0 && none.mean == 0.0);
  // p95 is the nearest rank, ceil(0.95 n): of 20, the 19th, not the max.
  std::vector<float> twenty;
  for (int i = 1; i <= 20; ++i) twenty.push_back(0.01f * float(i));
  const vr_example::DistanceStats r = summarize(twenty, 1.0f);
  CHECK(near(r.p95, 0.19, 1e-6));
  CHECK(near(r.max, 0.20, 1e-6));
  return 0;
}

int compare_case() {
  const mesh::Mesh reference = plane(40, 0.005f, 0.0f);  // 0.2 m, 5 mm cells
  // The same plane 3 mm up: every point of either is exactly 3 mm from the
  // other, in both directions (the vertices sit over the other's interior).
  const mesh::Mesh shifted = plane(40, 0.005f, 0.003f);
  const vr_example::MeshComparison c =
      compare_meshes(reference, shifted, 0.02f);
  for (const vr_example::DistanceStats& s : {c.accuracy, c.coverage}) {
    CHECK(s.count == reference.vertices.size());
    CHECK(s.beyond_reach == 0);
    CHECK(near(s.mean, 0.003, 1e-6));
    CHECK(near(s.max, 0.003, 1e-6));
  }
  // Identical: zero everywhere.
  const vr_example::MeshComparison same =
      compare_meshes(reference, reference, 0.02f);
  CHECK(same.accuracy.max == 0.0 && same.coverage.max == 0.0);

  // Half the test mesh gone (every square at x >= 0.1 m): what remains is
  // still exactly on the surface (accuracy 0, and its unused vertices past
  // the cut too -- they lie on the reference plane), but the reference past
  // the cut plus the reach is uncovered.
  const mesh::Mesh half = plane(40, 0.005f, 0.0f, 0.1f);
  const vr_example::MeshComparison h = compare_meshes(reference, half, 0.02f);
  CHECK(h.accuracy.max == 0.0);
  CHECK(h.coverage.beyond_reach > reference.vertices.size() / 3);
  CHECK(h.coverage.beyond_reach < reference.vertices.size() / 2);

  // A stride measures every stride-th vertex.
  const vr_example::MeshComparison sparse =
      compare_meshes(reference, shifted, 0.02f, 7);
  CHECK(sparse.accuracy.count == (reference.vertices.size() + 6) / 7);
  CHECK(near(sparse.accuracy.mean, 0.003, 1e-6));

  // A reference hashed once, for many comparisons: the same figures.
  const MeshDistance hashed(reference, 0.02f);
  const vr_example::MeshComparison again = compare_meshes(hashed, half);
  CHECK(again.accuracy.count == h.accuracy.count);
  CHECK(again.coverage.beyond_reach == h.coverage.beyond_reach);
  CHECK(again.coverage.rms == h.coverage.rms);
  return 0;
}

}  // namespace

int main() {
  if (closest_point_case() != 0) return 1;
  if (degenerate_case() != 0) return 1;
  if (search_case() != 0) return 1;
  if (distance_case() != 0) return 1;
  if (summarize_case() != 0) return 1;
  if (compare_case() != 0) return 1;
  std::printf("mesh distance: OK\n");
  return 0;
}
