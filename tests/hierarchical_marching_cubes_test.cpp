// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "buffer_readback.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/mesh/hierarchical_geometry.hpp"
#include "volumetric_kit/recon/mesh/hierarchical_marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes_tables.hpp"
#include "volumetric_kit/recon/volume/hash.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = vr::volume;
namespace mesh = vr::mesh;
#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {
bool consistent_faces() {
  using Segments = std::vector<std::pair<int, int>>;
  for (int axis = 0; axis < 3; ++axis) {
    std::map<int, Segments> expected;
    for (int side = 0; side < 2; ++side)
      for (int code = 0; code < 256; ++code) {
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        int pattern = 0;
        for (int c = 0; c < 8; ++c)
          if (mesh::kCornerOffset[c][axis] == side && (code & (1 << c))) {
            pattern |= 1 << (mesh::kCornerOffset[c][u] +
                             2 * mesh::kCornerOffset[c][v]);
          }
        Segments segments;
        for (int t = 0; t < 5 && mesh::kTriTable[code][3 * t] >= 0; ++t) {
          std::vector<int> face;
          for (int k = 0; k < 3; ++k) {
            const int edge = mesh::kTriTable[code][3 * t + k];
            const int a = mesh::kEdgeToVert[edge][0],
                      b = mesh::kEdgeToVert[edge][1];
            if (mesh::kCornerOffset[a][axis] == side &&
                mesh::kCornerOffset[b][axis] == side) {
              face.push_back(
                  mesh::kCornerOffset[a][u] + mesh::kCornerOffset[b][u] +
                  3 * (mesh::kCornerOffset[a][v] + mesh::kCornerOffset[b][v]));
            }
          }
          if (face.size() == 2)
            segments.push_back(std::minmax(face[0], face[1]));
        }
        std::sort(segments.begin(), segments.end());
        if (expected.count(pattern) && expected[pattern] != segments) {
          std::fprintf(stderr,
                       "MC face mismatch axis%d side%d code%d pattern%d\n",
                       axis, side, code, pattern);
          return false;
        }
        expected[pattern] = segments;
      }
  }
  return true;
}
constexpr float kH = 0.02f;
constexpr float kRadius = 0.431f;
constexpr std::uint32_t kCapacity = 512;
struct Fixture {
  vr::Buffer roots, nodes, leaves, distances, weights;
  std::vector<vol::HierarchicalNode> host_nodes;
  std::vector<std::uint32_t> host_leaves;
  std::uint64_t generation = 1;
  vol::VoxelGridParams grid{kH * 4, 8, 512, 0.08f, 8, 16, 128, 128};
  vol::HierarchicalFieldView view() const {
    vol::HierarchicalFieldView v;
    v.root_hash = &roots;
    v.nodes = &nodes;
    v.leaf_indices = &leaves;
    v.tsdf = &distances;
    v.weight = &weights;
    v.root_grid = grid;
    v.node_capacity = kCapacity;
    v.leaf_count = std::uint32_t(host_leaves.size());
    v.max_level = 2;
    v.finest_voxel_size = kH;
    v.generation = generation;
    v.live_generation = &generation;
    return v;
  }
};

vr::Status build(Fixture& f, const vr_test::Gpu& gpu, bool adaptive) {
  std::vector<vol::HashEntry> entries(128);
  for (auto& e : entries) {
    e.ptr = -1;
    e.offset = 0;
  }
  f.host_nodes.resize(kCapacity);
  f.host_leaves.clear();
  std::uint32_t child_cursor = 128;
  for (int z = -1; z <= 0; ++z)
    for (int y = -1; y <= 0; ++y)
      for (int x = -1; x <= 0; ++x) {
        const std::uint32_t id =
            std::uint32_t(x + 1 + 2 * (y + 1) + 4 * (z + 1));
        auto& root = f.host_nodes[id];
        root = {vr::Vec3i(x, y, z), std::int32_t(id * 512), 2, 0};
        auto slot = vol::hash_bucket(root.coord, f.grid.num_buckets) * 8;
        while (entries[slot].ptr != -1) ++slot;
        entries[slot] = {std::int32_t(id * 512), root.coord, 0};
        if (!adaptive || x == 0) {
          f.host_leaves.push_back(id);
          continue;
        }
        root.children = child_cursor + 1;
        const auto first = child_cursor;
        child_cursor += 8;
        for (std::uint32_t oct = 0; oct < 8; ++oct) {
          const auto node = first + oct;
          auto& child = f.host_nodes[node];
          child = {vr::Vec3i(x * 2 + int(oct & 1), y * 2 + int((oct >> 1) & 1),
                             z * 2 + int((oct >> 2) & 1)),
                   std::int32_t(node * 512), 1, 0};
          if ((oct & 6) != 0) {
            f.host_leaves.push_back(node);
            continue;
          }
          child.children = child_cursor + 1;
          const auto grandchildren = child_cursor;
          child_cursor += 8;
          for (std::uint32_t g = 0; g < 8; ++g) {
            const auto gc = grandchildren + g;
            f.host_nodes[gc] = {
                vr::Vec3i(child.coord.x * 2 + int(g & 1),
                          child.coord.y * 2 + int((g >> 1) & 1),
                          child.coord.z * 2 + int((g >> 2) & 1)),
                std::int32_t(gc * 512), 0, 0};
            f.host_leaves.push_back(gc);
          }
        }
      }
  std::vector<float> sdf(kCapacity * 512), weights(kCapacity * 512);
  for (const auto id : f.host_leaves) {
    const auto& node = f.host_nodes[id];
    for (int z = 0; z < 8; ++z)
      for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x) {
          VR_ASSIGN(const vr::Vec3f p, mesh::hierarchical_sample_center(
                                           node, vr::Vec3i(x, y, z), kH));
          const float px = p.x, py = p.y, pz = p.z;
          const auto index = std::uint32_t(node.ptr + x + 8 * (y + 8 * z));
          sdf[index] = std::sqrt(px * px + py * py + pz * pz) - kRadius;
          weights[index] = 1.0f;
        }
  }
  VR_ASSIGN(f.roots, vr::device_storage_buffer(
                         gpu.allocator, entries.size() * sizeof(entries[0])));
  VR_ASSIGN(f.nodes,
            vr::device_storage_buffer(
                gpu.allocator, f.host_nodes.size() * sizeof(f.host_nodes[0])));
  VR_ASSIGN(f.leaves, vr::device_storage_buffer(
                          gpu.allocator,
                          f.host_leaves.size() * sizeof(f.host_leaves[0])));
  VR_ASSIGN(f.distances, vr::device_storage_buffer(gpu.allocator,
                                                   sdf.size() * sizeof(float)));
  VR_ASSIGN(f.weights, vr::device_storage_buffer(
                           gpu.allocator, weights.size() * sizeof(float)));
  vr::CommandBatch batch(gpu.device, gpu.allocator);
  VR_TRY(batch.upload(f.roots, 0, entries.data(), f.roots.size()));
  VR_TRY(batch.upload(f.nodes, 0, f.host_nodes.data(), f.nodes.size()));
  VR_TRY(batch.upload(f.leaves, 0, f.host_leaves.data(), f.leaves.size()));
  VR_TRY(batch.upload(f.distances, 0, sdf.data(), f.distances.size()));
  VR_TRY(batch.upload(f.weights, 0, weights.data(), f.weights.size()));
  VR_TRY(batch.submit());
  ++f.generation;
  return {};
}

using Position = std::array<std::uint32_t, 3>;
Position key(vr::Vec3f p) {
  Position result{};
  float components[] = {p.x == 0 ? 0 : p.x, p.y == 0 ? 0 : p.y,
                        p.z == 0 ? 0 : p.z};
  std::memcpy(result.data(), components, sizeof(components));
  return result;
}
bool closed_sphere(const mesh::Mesh& m, bool check_sphere = true) {
  std::map<std::pair<Position, Position>, unsigned> edges;
  if (m.indices.empty()) return false;
  for (const auto& vertex : m.vertices) {
    if (!check_sphere) continue;
    const auto& p = vertex.position;
    const float radius = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (!std::isfinite(radius) || std::abs(radius - kRadius) > 0.02f) {
      std::fprintf(stderr, "radius %g at %g,%g,%g color %g,%g,%g\n",
                   double(radius), double(p.x), double(p.y), double(p.z),
                   double(vertex.color.x), double(vertex.color.y),
                   double(vertex.color.z));
      return false;
    }
    if (p.x * vertex.normal.x + p.y * vertex.normal.y + p.z * vertex.normal.z <=
        0) {
      std::fprintf(stderr, "inward at %g,%g,%g normal %g,%g,%g\n", double(p.x),
                   double(p.y), double(p.z), double(vertex.normal.x),
                   double(vertex.normal.y), double(vertex.normal.z));
      return false;
    }
  }
  for (std::size_t t = 0; t < m.indices.size(); t += 3) {
    for (std::size_t e = 0; e < 3; ++e) {
      auto a = key(m.vertices[m.indices[t + e]].position);
      auto b = key(m.vertices[m.indices[t + (e + 1) % 3]].position);
      if (a == b) {
        auto p = m.vertices[m.indices[t + e]].position;
        std::fprintf(stderr, "degenerate edge tri%zu at %g,%g,%g\n", t / 3,
                     double(p.x), double(p.y), double(p.z));
        return false;
      }
      if (b < a) std::swap(a, b);
      ++edges[{a, b}];
    }
  }
  std::size_t bad = 0;
  for (const auto& edge : edges)
    if (edge.second != 2) ++bad;
  if (bad)
    std::fprintf(stderr, "%zu nonmanifold/unmatched edges of %zu\n", bad,
                 edges.size());
  return bad == 0;
}

template <typename Function>
vr::Status change_field(Fixture& f, const vr_test::Gpu& gpu,
                        Function function) {
  std::vector<float> distances(kCapacity * 512);
  for (const auto id : f.host_leaves) {
    const auto& node = f.host_nodes[id];
    for (int z = 0; z < 8; ++z)
      for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x) {
          VR_ASSIGN(const auto p, mesh::hierarchical_sample_center(
                                      node, vr::Vec3i(x, y, z), kH));
          distances[std::size_t(node.ptr + x + 8 * (y + 8 * z))] = function(p);
        }
  }
  return vr_test::write_back(gpu.device, gpu.allocator, f.distances, distances);
}
}  // namespace

int run_tests() {
  CHECK(consistent_faces());
  vol::HierarchicalNode owner{vr::Vec3i(-1, 0, 0), 0, 1, 0};
  auto vertex = mesh::hierarchical_dual_vertex(owner, vr::Vec3i(8, 0, 0));
  CHECK(vertex && vertex.value().x == 0 && vertex.value().y == 0);
  auto center =
      mesh::hierarchical_sample_center(owner, vr::Vec3i(7, 0, 0), 0.5f);
  CHECK(center && center.value().x == -0.5f && center.value().y == 0.5f);
  CHECK(!mesh::hierarchical_sample_center(owner, vr::Vec3i(8, 0, 0), kH));
  std::array<vol::HierarchicalNode, 8> incident;
  incident.fill(owner);
  CHECK(mesh::owns_hierarchical_dual_cell(owner, incident));
  incident[1] = {vr::Vec3i(0, 0, 0), 512, 2, 0};
  CHECK(mesh::owns_hierarchical_dual_cell(owner, incident));
  CHECK(!mesh::owns_hierarchical_dual_cell(incident[1], incident));
  incident[2] = {vr::Vec3i(-2, 0, 0), 1024, 1, 0};
  CHECK(!mesh::owns_hierarchical_dual_cell(owner, incident));
  owner.coord.x = std::numeric_limits<std::int32_t>::min();
  CHECK(!mesh::hierarchical_dual_vertex(owner, vr::Vec3i(0, 0, 0)));
  vr::InstanceConfig ic;
  ic.enable_validation = true;
  auto instance = vr::Instance::create(ic);
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance; skipping\n");
    return 77;
  }
  auto physical = instance.value().select_physical_device();
  if (!physical) return 77;
  auto device = vr::Device::create(instance.value(), physical.value(), {});
  CHECK(device);
  auto allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator);
  vr_test::Gpu gpu{device.value(), allocator.value()};
  Fixture field;
  CHECK(build(field, gpu, false).ok());
  CHECK(field.view().validate().ok());
  auto made =
      mesh::HierarchicalMarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(made);
  auto extractor = std::move(made.value());
  CHECK(!made.value().valid());
  mesh::ExtractTimings timings;
  auto uniform = extractor.extract_host(field.view(), 0.0f, &timings);
  if (!uniform)
    std::fprintf(stderr, "%s\n", uniform.status().message().c_str());
  CHECK(uniform);
  CHECK(closed_sphere(uniform.value()));
  CHECK(timings.emitted_triangles > 0 && !timings.incremental);
  CHECK(build(field, gpu, true).ok());
  auto adaptive = extractor.extract_host(field.view(), 0.0f, &timings);
  if (!adaptive)
    std::fprintf(stderr, "%s\n", adaptive.status().message().c_str());
  CHECK(adaptive);
  CHECK(closed_sphere(adaptive.value()));
  CHECK(adaptive.value().indices.size() > uniform.value().indices.size());
  for (const auto& v : adaptive.value().vertices) {
    CHECK(v.color.x == 1.0f && v.color.y == 1.0f && v.color.z == 1.0f &&
          v.color.w == 1.0f);
  }
  auto colors = vr::device_storage_buffer(
      gpu.allocator, VkDeviceSize(kCapacity) * 512 * sizeof(std::uint32_t));
  CHECK(colors);
  {
    vr::CommandBatch batch(gpu.device, gpu.allocator);
    CHECK(
        batch.fill(colors.value(), 0, colors.value().size(), 0xff808080u).ok());
    CHECK(batch.submit().ok());
  }
  auto colored_field = field.view();
  colored_field.color = &colors.value();
  auto colored = extractor.extract_host(colored_field);
  CHECK(colored);
  for (const auto& v : colored.value().vertices) {
    CHECK(std::abs(v.color.x - 0.2158605f) < 1e-6f &&
          std::abs(v.color.y - 0.2158605f) < 1e-6f &&
          std::abs(v.color.z - 0.2158605f) < 1e-6f && v.color.w == 1.0f);
  }
  auto empty_field = field.view();
  empty_field.leaf_count = 0;
  auto empty = extractor.extract_host(empty_field);
  CHECK(empty && empty.value().vertices.empty() &&
        empty.value().indices.empty());
  // Analytic planes cross every orientation, including nearly tangent faces.
  for (const auto n : {vr::Vec3f(1, 0.001f, 0.17f), vr::Vec3f(0.17f, 1, 0.001f),
                       vr::Vec3f(0.001f, 0.17f, 1)}) {
    CHECK(change_field(field, gpu, [n](vr::Vec3f p) {
            return p.x * n.x + p.y * n.y + p.z * n.z - 0.031f;
          }).ok());
    auto plane = extractor.extract_host(field.view());
    CHECK(plane);
    CHECK(!plane.value().indices.empty());
    for (const auto& v : plane.value().vertices) {
      const auto p = v.position;
      CHECK(std::abs(p.x * n.x + p.y * n.y + p.z * n.z - 0.031f) < 2e-6f);
      CHECK(v.normal.x * n.x + v.normal.y * n.y + v.normal.z * n.z > 0.99f);
    }
  }
  // Arbitrary signs exercise ambiguous MC cases and collapsed dual cells,
  // with a positive outer shell so the extracted surface must close exactly.
  CHECK(change_field(field, gpu, [](vr::Vec3f p) {
          if (std::max({std::abs(p.x), std::abs(p.y), std::abs(p.z)}) > 0.46f)
            return 0.05f;
          const auto x = std::uint32_t(std::lround(p.x * 100));
          const auto y = std::uint32_t(std::lround(p.y * 100));
          const auto z = std::uint32_t(std::lround(p.z * 100));
          const auto value =
              (x * 73856093u) ^ (y * 19349669u) ^ (z * 83492791u);
          return ((float(value % 2048u) + 0.5f) / 2048.0f - 0.5f) * 0.06f;
        }).ok());
  auto signs = extractor.extract_host(field.view());
  CHECK(signs);
  CHECK(closed_sphere(signs.value(), false));
  auto stale = field.view();
  ++field.generation;
  CHECK(!extractor.extract_device(stale));
  auto moved = std::move(extractor);
  CHECK(!extractor.valid() && moved.valid());
  auto* self = &moved;
  moved = std::move(*self);
  CHECK(moved.valid());
  auto replacement =
      mesh::HierarchicalMarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(replacement);
  replacement.value() = std::move(moved);
  CHECK(!moved.valid());

  mesh::MarchingCubesConfig config;
  config.slot_count = 2;
  auto ring = mesh::HierarchicalMarchingCubes::create(gpu.device, gpu.allocator,
                                                      config);
  CHECK(ring);
  auto first = ring.value().extract_device(field.view());
  CHECK(first);
  auto second = ring.value().extract_device(field.view());
  CHECK(second);
  CHECK(first.value().vertices != second.value().vertices);
  CHECK(!ring.value().extract_device(field.view()));
  CHECK(second.value().is_current());
  CHECK(!ring.value().download(first.value()));
  ring.value().release_through(first.value().generation);
  auto third = ring.value().extract_device(field.view());
  CHECK(third);
  CHECK(third.value().vertices == first.value().vertices);
  ring.value().release_through(third.value().generation);
  auto host = ring.value().extract_host(field.view());
  CHECK(host);
  auto host2 = ring.value().extract_host(field.view());
  CHECK(host2);
  config.share_vertices = true;
  CHECK(!mesh::HierarchicalMarchingCubes::create(gpu.device, gpu.allocator,
                                                 config));

  // Topology growth alone must not reallocate an arena whose actual output
  // still fits. Unobserved leaves have no surface at either resolution.
  Fixture unobserved;
  auto retained =
      mesh::HierarchicalMarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(retained);
  VkBuffer previous_vertices = VK_NULL_HANDLE;
  std::uint64_t previous_capacity = 0;
  for (const bool refine : {false, true}) {
    CHECK(build(unobserved, gpu, refine).ok());
    vr::CommandBatch batch(gpu.device, gpu.allocator);
    CHECK(batch.fill(unobserved.weights, 0, unobserved.weights.size(), 0).ok());
    CHECK(batch.submit().ok());
    auto view =
        retained.value().extract_device(unobserved.view(), 0.0f, &timings);
    CHECK(view && view.value().triangle_count == 0);
    if (refine) {
      CHECK(view.value().vertices == previous_vertices);
      CHECK(timings.triangle_capacity == previous_capacity);
    }
    previous_vertices = view.value().vertices;
    previous_capacity = timings.triangle_capacity;
  }

  auto bad = field.view();
  bad.node_capacity *= 2;
  CHECK(!bad.validate().ok());
  bad = field.view();
  bad.root_grid.trunc_dist = std::numeric_limits<float>::infinity();
  CHECK(!bad.validate().ok());
  field.host_nodes[field.host_leaves[0]].children = 1;
  CHECK(vr_test::write_back(gpu.device, gpu.allocator, field.nodes,
                            field.host_nodes)
            .ok());
  CHECK(!ring.value().extract_device(field.view()));
  std::printf(
      "hierarchical mesh: uniform %zu, adaptive %zu triangles; exact closed "
      "seams passed\n",
      uniform.value().indices.size() / 3, adaptive.value().indices.size() / 3);
  return 0;
}

int main() {
  std::atomic<unsigned> validation_errors{0};
  vr::set_log_handler([&](vr::LogLevel level, std::string_view message) {
    if (level == vr::LogLevel::Error) ++validation_errors;
    if (level == vr::LogLevel::Warning || level == vr::LogLevel::Error) {
      std::fprintf(stderr, "%.*s\n", int(message.size()), message.data());
    }
  });
  // run_tests destroys every Vulkan resource before the layer's error count
  // is checked, so errors during both dispatch and destruction fail the test.
  const int result = run_tests();
  vr::set_log_handler({});
  if (result == 0 && validation_errors.load() != 0) {
    std::fprintf(stderr, "FAIL: %u validation errors\n",
                 validation_errors.load());
    return 1;
  }
  return result;
}
