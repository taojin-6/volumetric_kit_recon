// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Host-only test for examples/common/mesh_distance.hpp, the codec example's
// quality metric: the closest point in every region of a triangle, the reach
// clamp, the summary statistics, and accuracy / coverage between meshes whose
// true distance is known -- a plane against a copy of itself shifted by a
// known amount, and against one with half its triangles gone. CPU-only.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "mesh_distance.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"

namespace vr = volumetric_kit::recon;
namespace mesh = volumetric_kit::recon::mesh;
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
  return 0;
}

}  // namespace

int main() {
  if (closest_point_case() != 0) return 1;
  if (distance_case() != 0) return 1;
  if (summarize_case() != 0) return 1;
  if (compare_case() != 0) return 1;
  std::printf("mesh distance: OK\n");
  return 0;
}
