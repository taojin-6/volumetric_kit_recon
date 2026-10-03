// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "mesh_normalization.hpp"

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {
volumetric_kit::recon::io::TriangleMesh tetrahedron() {
  return {{{10, 20, 30}, {12, 20, 30}, {10, 22, 30}, {10, 20, 32}},
          {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3}};
}

double distance(const volumetric_kit::recon::Vec3f& a,
                const volumetric_kit::recon::Vec3f& b) {
  const double dx = double(a.x) - b.x;
  const double dy = double(a.y) - b.y;
  const double dz = double(a.z) - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

int main() {
  auto mesh = tetrahedron();
  const auto audit = vr_example::audit_mesh_topology(mesh);
  CHECK(audit.supports_signed() && audit.edges == 6 && audit.components == 1);
  CHECK(std::abs(audit.signed_volume - 8.0 / 6.0) < 1e-12);

  // Changing the declared input units must not change the normalized result.
  auto millimetres = mesh;
  for (auto& p : millimetres.positions) {
    p.x *= 1000;
    p.y *= 1000;
    p.z *= 1000;
  }
  const auto metric = vr_example::normalize_mesh_height(mesh, 1.7, {0, 1, 0});
  const auto milli =
      vr_example::normalize_mesh_height(millimetres, 1.7, {0, 1, 0});
  CHECK(metric.ok() && milli.ok());
  CHECK(std::abs(metric.value().original_height - 2.0) < 1e-12);
  CHECK(std::abs(metric.value().metres_per_unit - 0.85) < 1e-12);
  CHECK(std::abs(milli.value().metres_per_unit - 0.00085) < 1e-12);
  CHECK(metric.value().normalized.min[1] == 0.0);
  CHECK(std::abs(metric.value().normalized.max[1] - 1.7) < 1e-6);
  for (std::size_t i = 0; i < mesh.positions.size(); ++i) {
    CHECK(distance(mesh.positions[i], millimetres.positions[i]) < 1e-6);
  }

  // An oblique, unnormalised head-up vector exercises rotation, all pairwise
  // lengths and positive orientation, not only an axis-aligned special case.
  auto oblique = tetrahedron();
  const auto before = oblique;
  const auto n = vr_example::normalize_mesh_height(oblique, 1.7, {-2, 1, -3});
  CHECK(n.ok());
  CHECK(std::abs(n.value().normalized.max[1] - 1.7) < 1e-6);
  CHECK(n.value().normalized.min[1] == 0.0);
  CHECK(std::abs(n.value().normalized.max[0] + n.value().normalized.min[0]) <
        1e-6);
  CHECK(std::abs(n.value().normalized.max[2] + n.value().normalized.min[2]) <
        1e-6);
  for (std::size_t i = 0; i < oblique.positions.size(); ++i) {
    for (std::size_t j = i + 1; j < oblique.positions.size(); ++j) {
      CHECK(std::abs(distance(oblique.positions[i], oblique.positions[j]) -
                     distance(before.positions[i], before.positions[j]) *
                         n.value().metres_per_unit) < 1e-6);
    }
  }
  const auto rotated_audit = vr_example::audit_mesh_topology(oblique);
  CHECK(rotated_audit.supports_signed());
  CHECK(std::abs(rotated_audit.signed_volume -
                 audit.signed_volume * std::pow(n.value().metres_per_unit, 3)) <
        1e-6);
  auto z_up = tetrahedron();
  CHECK(vr_example::normalize_mesh_height(z_up, 1.7, {0, 0, -1}).ok());
  CHECK(vr_example::audit_mesh_topology(z_up).supports_signed());

  auto invalid = tetrahedron();
  CHECK(!vr_example::normalize_mesh_height(invalid, 1.7, {0, 0, 0}).ok());
  CHECK(!vr_example::normalize_mesh_height(invalid, -1, {0, 1, 0}).ok());
  CHECK(!vr_example::normalize_mesh_height(invalid, 1e-100, {0, 1, 0}).ok());
  CHECK(!vr_example::normalize_mesh_height(
             invalid, 1.7, {0, std::numeric_limits<double>::infinity(), 0})
             .ok());
  CHECK(invalid.positions.front().x == 10.0f);  // failures do not mutate
  auto flat = tetrahedron();
  for (auto& p : flat.positions) p.y = 0.0f;
  CHECK(!vr_example::normalize_mesh_height(flat, 1.7, {0, 1, 0}).ok());

  auto open = tetrahedron();
  open.indices.resize(9);
  CHECK(vr_example::audit_mesh_topology(open).boundary_edges == 3);
  CHECK(!vr_example::audit_mesh_topology(open).supports_signed());
  auto flipped = tetrahedron();
  std::swap(flipped.indices[0], flipped.indices[1]);
  CHECK(vr_example::audit_mesh_topology(flipped).inconsistent_edges == 3);
  auto inward = tetrahedron();
  for (std::size_t t = 0; t < inward.indices.size(); t += 3) {
    std::swap(inward.indices[t], inward.indices[t + 1]);
  }
  CHECK(vr_example::audit_mesh_topology(inward).nonpositive_components == 1);
  auto duplicate = tetrahedron();
  duplicate.indices.insert(duplicate.indices.end(), {0, 2, 1});
  CHECK(vr_example::audit_mesh_topology(duplicate).nonmanifold_edges == 3);
  auto degenerate = tetrahedron();
  degenerate.indices.insert(degenerate.indices.end(), {0, 0, 1});
  CHECK(vr_example::audit_mesh_topology(degenerate).degenerate_triangles == 1);

  std::puts("mesh normalization tests passed");
  return 0;
}
