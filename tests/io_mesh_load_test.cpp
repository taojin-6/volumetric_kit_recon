// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Host-only public API tests: geometry is checked in world coordinates, with
// oriented faces and independent area/volume/topology expectations. No Assimp
// headers, Vulkan instance, input parser or transform implementation is shared.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/io/mesh_io.hpp"

namespace vr = volumetric_kit::recon;
namespace io = vr::io;

#define CHECK(condition)                                           \
  do {                                                             \
    if (!(condition)) {                                            \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                   #condition);                                    \
      return 1;                                                    \
    }                                                              \
  } while (0)

namespace {
using Point = std::array<float, 3>;
using Triangle = std::array<Point, 3>;

std::string path(const char* name) {
  return std::string(VR_IO_MESH_DATA) + "/" + name;
}

vr::Result<io::TriangleMesh> load(const char* name) {
  auto result = io::load_mesh(path(name));
  if (!result.ok()) {
    std::fprintf(stderr, "%s: %s\n", name, result.status().message().c_str());
  }
  return result;
}

bool valid(const io::TriangleMesh& mesh) {
  if (mesh.positions.empty() || mesh.indices.empty() ||
      mesh.indices.size() % 3 != 0)
    return false;
  for (const auto& p : mesh.positions) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      return false;
  }
  for (std::uint32_t i : mesh.indices) {
    if (i >= mesh.positions.size()) return false;
  }
  return true;
}

Point point(const vr::Vec3f& p) { return {p.x, p.y, p.z}; }

// Ignore vertex numbering, triangle ordering and cyclic rotations, but keep
// winding: reversing a triangle must never compare equal.
std::vector<Triangle> canonical(std::vector<Triangle> triangles) {
  for (Triangle& t : triangles) {
    const auto first = std::min_element(t.begin(), t.end());
    std::rotate(t.begin(), first, t.end());
  }
  std::sort(triangles.begin(), triangles.end());
  return triangles;
}

std::vector<Triangle> triangles(const io::TriangleMesh& mesh) {
  std::vector<Triangle> result;
  for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
    result.push_back({point(mesh.positions[mesh.indices[i]]),
                      point(mesh.positions[mesh.indices[i + 1]]),
                      point(mesh.positions[mesh.indices[i + 2]])});
  }
  return canonical(std::move(result));
}

bool same_positions(const io::TriangleMesh& mesh, std::vector<Point> expected) {
  std::vector<Point> actual;
  for (const auto& p : mesh.positions) actual.push_back(point(p));
  std::sort(actual.begin(), actual.end());
  std::sort(expected.begin(), expected.end());
  return actual == expected;
}

double signed_volume(const io::TriangleMesh& mesh) {
  double six_volume = 0.0;
  for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
    const auto a = mesh.positions[mesh.indices[i]];
    const auto b = mesh.positions[mesh.indices[i + 1]];
    const auto c = mesh.positions[mesh.indices[i + 2]];
    six_volume += double(a.x) * (double(b.y) * c.z - double(b.z) * c.y) +
                  double(a.y) * (double(b.z) * c.x - double(b.x) * c.z) +
                  double(a.z) * (double(b.x) * c.y - double(b.y) * c.x);
  }
  return six_volume / 6.0;
}

int basic_formats() {
  const std::vector<Point> expected_positions{{1, 2, 3}, {5, 2, 3}, {1, 8, 3}};
  const auto expected_triangles =
      canonical({Triangle{{{1, 2, 3}, {5, 2, 3}, {1, 8, 3}}}});
  // A zero-scaled helper node without meshes must not refuse the asset.
  std::vector<const char*> filenames{"triangle.obj", "triangle.ply",
                                     "triangle.gltf",
                                     "hidden_singular_node.gltf"};
#if VR_ASSIMP_SKIPS_COLLADA_UNIT
  // Neither its Z-up axis nor its centimetre unit is applied.
  filenames.push_back("triangle_zup_cm.dae");
#endif
  for (const char* filename : filenames) {
    const auto result = load(filename);
    CHECK(result.ok());
    const auto& mesh = result.value();
    CHECK(valid(mesh));
    CHECK(same_positions(mesh, expected_positions));
    CHECK(triangles(mesh) == expected_triangles);
  }
  // Coordinates, origin and dimensions survive loading: neither unit inference
  // nor the example's 1.7-metre body normalization belongs to this API.
  return 0;
}

int triangulated_quad() {
  const auto result = load("quad.ply");
  CHECK(result.ok());
  const auto& mesh = result.value();
  CHECK(valid(mesh));
  CHECK(mesh.positions.size() == 4);
  CHECK(mesh.indices.size() == 6);
  CHECK(same_positions(
      mesh, {{10, 20, 30}, {14, 20, 30}, {14, 26, 30}, {10, 26, 30}}));
  double area = 0;
  for (const auto& t : triangles(mesh)) {
    const double cross_z =
        (double(t[1][0]) - t[0][0]) * (double(t[2][1]) - t[0][1]) -
        (double(t[1][1]) - t[0][1]) * (double(t[2][0]) - t[0][0]);
    CHECK(cross_z > 0);
    area += cross_z / 2.0;
  }
  CHECK(area == 24.0);
  return 0;
}

int material_and_attribute_seams() {
  const auto result = load("seams.obj");
  CHECK(result.ok());
  const auto& mesh = result.value();
  CHECK(valid(mesh));
  CHECK(mesh.positions.size() == 8);
  CHECK(mesh.indices.size() == 36);
  CHECK(same_positions(mesh, {{0, 0, 0},
                              {2, 0, 0},
                              {2, 3, 0},
                              {0, 3, 0},
                              {0, 0, 5},
                              {2, 0, 5},
                              {2, 3, 5},
                              {0, 3, 5}}));
  CHECK(signed_volume(mesh) == 30.0);
  // Every geometric edge (including each face's triangulation diagonal) has
  // two opposite uses. UV/normal/material splits must not leave boundaries.
  using Edge = std::pair<std::uint32_t, std::uint32_t>;
  std::map<Edge, std::pair<int, int>> edges;
  for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
    for (std::size_t k = 0; k < 3; ++k) {
      const auto a = mesh.indices[i + k];
      const auto b = mesh.indices[i + (k + 1) % 3];
      CHECK(a != b);
      auto& counts = edges[{std::min(a, b), std::max(a, b)}];
      ++counts.first;
      counts.second += a < b ? 1 : -1;
    }
  }
  CHECK(edges.size() == 18);
  for (const auto& edge : edges) {
    CHECK(edge.second.first == 2);
    CHECK(edge.second.second == 0);
  }
  return 0;
}

int exact_join() {
  const auto result = load("near_positions.obj");
  CHECK(result.ok());
  const auto& mesh = result.value();
  CHECK(valid(mesh));
  CHECK(mesh.indices.size() == 6);
  CHECK(same_positions(
      mesh,
      {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1.000001f, 0, 0}, {0, 1.000001f, 0}}));
  const auto expected =
      canonical({Triangle{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}},
                 Triangle{{{0, 0, 0}, {1.000001f, 0, 0}, {0, 1.000001f, 0}}}});
  CHECK(triangles(mesh) == expected);
  return 0;
}

int no_implicit_repair() {
  const auto result = load("unrepaired.obj");
  CHECK(result.ok());
  const auto& mesh = result.value();
  CHECK(valid(mesh));
  CHECK(mesh.positions.size() == 4);
  CHECK(mesh.indices.size() == 9);
  const auto expected =
      canonical({Triangle{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}},
                 Triangle{{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}},
                 Triangle{{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}}});
  CHECK(triangles(mesh) == expected);
  return 0;
}

int transformed_instances() {
  const auto result = load("instances.gltf");
  CHECK(result.ok());
  const auto& mesh = result.value();
  CHECK(valid(mesh));
  CHECK(mesh.positions.size() == 9);
  CHECK(mesh.indices.size() == 9);
  // A +90-degree Z rotation and translation on the parent, composed with
  // child translations/nonuniform scales. The middle instance mirrors X;
  // the last has two nested X reflections. Every oriented face still has +Z
  // normal, and each instance of the one source mesh appears once.
  const auto expected =
      canonical({Triangle{{{10, 22, 30}, {10, 24, 30}, {7, 22, 30}}},
                 Triangle{{{10, 18, 30}, {7, 18, 30}, {10, 16, 30}}},
                 Triangle{{{10, 19, 40}, {10, 21, 40}, {7, 19, 40}}}});
  CHECK(triangles(mesh) == expected);
  return 0;
}

int refusals() {
  for (const char* filename :
       {"does_not_exist.obj", "empty.obj", "malformed.obj", "bad_index.obj",
        "bad_index.gltf", "points.ply", "lines.obj", "nonfinite.gltf",
        "singular_transform.gltf", "projective_transform.gltf",
        "overflow_transform.gltf"}) {
    const auto result = io::load_mesh(path(filename));
    if (result.ok())
      std::fprintf(stderr, "unexpected acceptance: %s\n", filename);
    CHECK(!result.ok());
    CHECK(!result.status().message().empty());
  }
  CHECK(!io::load_mesh("").ok());
  for (const char* filename : {"animated.gltf", "morph.gltf", "skinned.gltf"}) {
    const auto result = io::load_mesh(path(filename));
    if (result.ok())
      std::fprintf(stderr, "unexpected acceptance: %s\n", filename);
    CHECK(!result.ok());
    if (result.status().domain() != vr::Status::Code::Unsupported)
      std::fprintf(stderr, "%s: %s\n", filename,
                   result.status().message().c_str());
    CHECK(result.status().domain() == vr::Status::Code::Unsupported);
  }
  // A failed load must not poison the next call or a previously returned mesh.
  const auto before = load("triangle.obj");
  CHECK(before.ok());
  const auto saved = triangles(before.value());
  CHECK(!io::load_mesh(path("bad_index.obj")).ok());
  const auto after = load("triangle.obj");
  CHECK(after.ok());
  CHECK(triangles(before.value()) == saved);
  CHECK(triangles(after.value()) == saved);
  return 0;
}
}  // namespace

int main() {
  if (basic_formats() || triangulated_quad() ||
      material_and_attribute_seams() || exact_join() || no_implicit_repair() ||
      transformed_instances() || refusals())
    return 1;
  std::puts("io_mesh_load: OK");
  return 0;
}
