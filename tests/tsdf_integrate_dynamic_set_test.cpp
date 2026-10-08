// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A Dynamic set's cameras act as one instant: two cameras that disagree give
// the same grid in either order, and the surface one of them sees survives
// the other's free space. F looks down world +z at a wall 1.005 m away, in
// colour. G sits 2 mm in front of that wall's surface and looks along it, at
// a far wall: its rays pass through F's band on the near side, which G sees
// as free space far past the band, and never reach F's wall. Each order is
// fused as a set twice, so the second set clears the first's history. The two
// grids must agree bit for bit, so their meshes must too, and F's surface
// must be whole: as many triangles on it as F fused alone gives, and F's
// colour in it. Fusing each camera after the other, G erased F's band where it
// looked and the mesh had a hole there in one order only. A third set, F's
// wall receded 10 cm, must clear the old surface in a set as well. Exits 0
// (skip) where no device is present.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "grid_readback.hpp"
#include "no_device.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr std::uint32_t kWidth = 160;
constexpr std::uint32_t kHeight = 120;
constexpr float kVoxel = 0.01f;
constexpr float kWall = 1.005f;  // F's wall, between two voxel planes
constexpr float kRecede = 0.1f;  // how far it moves back in the third set
constexpr std::uint32_t kColor = 0xFF3366CCu;

using Coord = std::tuple<int, int, int>;

vol::VoxelGridParams grid_params() {
  vol::VoxelGridParams grid{};
  grid.voxel_size = kVoxel;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 4096;
  grid.num_blocks = 4096 * 8;
  grid.max_chain = 128;
  return grid;
}

vkc::Result<vol::VoxelBlockGrid> make_grid(vkc::Device& device,
                                           vkc::Allocator& allocator) {
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)},
                                      {"color", sizeof(std::uint32_t)}};
  return vol::VoxelBlockGrid::create(device, allocator, grid_params(), attrs,
                                     3);
}

// A camera and the constant depth it sees: a wall square to its axis.
struct View {
  vr::DepthCameraParams cam{};
  std::vector<float> depth;
};

View view(const vr::Mat4f& cam_to_world, float f, float cy, float depth) {
  View v;
  v.cam.fx = f;
  v.cam.fy = f;
  v.cam.cx = 79.5f;
  v.cam.cy = cy;
  v.cam.min_depth = 0.1f;
  v.cam.max_depth = 5.0f;
  v.cam.width = kWidth;
  v.cam.height = kHeight;
  v.cam.cam_to_world = cam_to_world;
  v.depth.assign(kWidth * kHeight, depth);
  return v;
}

// F: at the origin looking down +z at its wall, `depth` away.
View frontal(float depth) {
  return view(vr::Mat4f(1.0f), 140.0f, 59.5f, depth);
}

// G: 2 mm in front of F's wall (z = 1.003, past the voxel plane z = 1.00) at
// x = -1, looking down +x at a wall at x = 2. Its image's down axis is world
// -z and its principal point is the top row, so every ray runs level or away
// from F's wall: the voxels of F's near band (z = 0.97 to 1.00) inside its
// view lie metres short of its depth of 3 m, free space far past its band.
View grazing() {
  const vr::Vec3f x(0.0f, -1.0f, 0.0f);
  const vr::Vec3f y(0.0f, 0.0f, -1.0f);
  const vr::Vec3f z(1.0f, 0.0f, 0.0f);
  const vr::Mat4f pose(vr::Vec4f(x, 0.0f), vr::Vec4f(y, 0.0f),
                       vr::Vec4f(z, 0.0f),
                       vr::Vec4f(-1.0f, 0.0f, 1.003f, 1.0f));
  return view(pose, 280.0f, 0.0f, 3.0f);
}

// Allocate every frame's band, retrying rounds that only lost bucket-lock
// races (as examples/common/fuse_frame.hpp does).
int allocate(vol::VoxelBlockGrid& grid, const std::vector<View>& views) {
  std::vector<vol::DepthInput> depths;
  for (const View& v : views) {
    depths.push_back({vkc::StorageInput(v.depth.data()), v.cam});
  }
  for (int round = 0; round < 5; ++round) {
    vol::AllocFailures why;
    auto failed = grid.map().allocate_from_depth(depths, &why);
    CHECK(failed.ok() && !why.capacity_limited());
    if (failed.value() == 0) return 0;
  }
  std::fprintf(stderr, "FAIL: allocation kept losing lock races\n");
  return 1;
}

// Allocate for `views`, then fuse them as one Dynamic set; F (always the one
// with the frontal camera's pose) brings its colour.
int fuse(tsdf::TsdfIntegrator& integrator, vol::VoxelBlockGrid& grid,
         const std::vector<View>& views, const tsdf::ColorFrame& color) {
  if (allocate(grid, views) != 0) return 1;
  std::vector<tsdf::FrameInput> frames;
  for (const View& v : views) {
    const bool is_f = v.cam.cam_to_world == vr::Mat4f(1.0f);
    frames.push_back(
        {{vkc::StorageInput(v.depth.data()), v.cam}, is_f ? &color : nullptr});
  }
  CHECK(integrator.integrate(grid, frames, 5.0f, tsdf::IntegrationMode::Dynamic)
            .ok());
  return 0;
}

// Every active block's coordinate and first voxel (`ptr` is a voxel offset).
vkc::Result<std::map<Coord, std::int32_t>> blocks_of(vol::VoxelBlockGrid& g) {
  VKC_ASSIGN(std::vector<vol::BlockIndex> active,
             g.map().compact_active_blocks());
  std::map<Coord, std::int32_t> out;
  for (const vol::BlockIndex& b : active) {
    out[Coord{b.coord.x, b.coord.y, b.coord.z}] = b.ptr;
  }
  return out;
}

// The same blocks, and in each the same bits of weight, tsdf and colour in
// every voxel. A slot can differ: allocation order is the GPU's.
int check_same(vkc::Device& device, vkc::Allocator& allocator,
               vol::VoxelBlockGrid& a, vol::VoxelBlockGrid& b) {
  auto ba = blocks_of(a);
  auto bb = blocks_of(b);
  CHECK(ba.ok() && bb.ok());
  CHECK(!ba.value().empty());
  CHECK(ba.value().size() == bb.value().size());
  const auto read = [&](vol::VoxelBlockGrid& g, const char* name) {
    return vr_test::read_attribute<std::uint32_t>(device, allocator, g, name)
        .value();
  };
  const std::array<const char*, 3> names{"weight", "tsdf", "color"};
  for (const char* name : names) {
    const std::vector<std::uint32_t> va = read(a, name);
    const std::vector<std::uint32_t> vb = read(b, name);
    for (const auto& [coord, ptr] : ba.value()) {
      const auto other = bb.value().find(coord);
      CHECK(other != bb.value().end());
      for (std::size_t k = 0; k < 512; ++k) {
        CHECK(va[static_cast<std::size_t>(ptr) + k] ==
              vb[static_cast<std::size_t>(other->second) + k]);
      }
    }
  }
  return 0;
}

// A mesh's vertices sorted by position, and how many of its triangles lie on
// a wall at z = `wall` in F's view (every corner within a voxel of it), which
// leaves out G's far wall.
struct MeshSummary {
  std::vector<std::array<float, 3>> vertices;
  std::size_t on_wall = 0;
};

vkc::Result<MeshSummary> summarize(mesh::MarchingCubes& mc,
                                   vol::VoxelBlockGrid& grid, float wall) {
  VKC_ASSIGN(const mesh::Mesh m, mc.extract_host(grid));
  MeshSummary s;
  for (const mesh::Vertex& v : m.vertices) {
    s.vertices.push_back({v.position.x, v.position.y, v.position.z});
  }
  std::sort(s.vertices.begin(), s.vertices.end());
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    bool on = true;
    for (std::size_t c = 0; c < 3; ++c) {
      const vr::Vec3f& p = m.vertices[m.indices[3 * t + c]].position;
      on = on && std::fabs(p.z - wall) < kVoxel && std::fabs(p.x) < 1.0f;
    }
    s.on_wall += on ? 1 : 0;
  }
  return s;
}

// The voxel at world (x, y, z), as an element of `g`'s attributes, or -1.
std::int64_t voxel_at(const std::map<Coord, std::int32_t>& blocks, float x,
                      float y, float z) {
  const auto node = [](float w) {
    return static_cast<int>(std::lround(w / kVoxel));
  };
  const int v[3] = {node(x), node(y), node(z)};
  int b[3];
  int l[3];
  for (int i = 0; i < 3; ++i) {
    b[i] = v[i] >= 0 ? v[i] / 8 : (v[i] - 7) / 8;
    l[i] = v[i] - 8 * b[i];
  }
  const auto it = blocks.find(Coord{b[0], b[1], b[2]});
  if (it == blocks.end()) return -1;
  return it->second + l[0] + 8 * l[1] + 64 * l[2];
}

}  // namespace

int main() {
  vkc::Result<vkc::Instance> instance = vkc::Instance::create({});
  if (!instance) {
    return vr_test::no_device("no Vulkan instance",
                              instance.status().message());
  }
  vkc::Result<vkc::PhysicalDeviceInfo> gpu =
      instance.value().select_physical_device(vr::device_requirements());
  if (!gpu) {
    return vr_test::no_device("no compute-capable device",
                              gpu.status().message());
  }
  vkc::Result<vkc::Device> device = vkc::Device::create(
      instance.value(), gpu.value(), vr::device_requirements());
  CHECK(device.ok());
  vkc::Result<vkc::Allocator> allocator =
      vkc::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  vkc::Device& dev = device.value();
  vkc::Allocator& alloc = allocator.value();

  auto integrator = tsdf::TsdfIntegrator::create(dev, alloc);
  CHECK(integrator.ok());
  auto mc = mesh::MarchingCubes::create(dev, alloc);
  CHECK(mc.ok());

  const View f = frontal(kWall);
  const View g = grazing();
  const std::vector<std::uint32_t> pixels(kWidth * kHeight, kColor);
  const tsdf::ColorFrame color{pixels.data(),
                               {f.cam.fx, f.cam.fy, f.cam.cx, f.cam.cy, kWidth,
                                kHeight, f.cam.cam_to_world},
                               {}};

  // F alone, the surface G must leave whole.
  auto alone = make_grid(dev, alloc);
  CHECK(alone.ok());
  for (int set = 0; set < 2; ++set) {
    CHECK(fuse(integrator.value(), alone.value(), {f}, color) == 0);
  }
  auto reference = summarize(mc.value(), alone.value(), kWall);
  CHECK(reference.ok());
  std::printf("F alone: %zu triangles on its wall\n", reference->on_wall);
  CHECK(reference->on_wall > 1000);

  // The set in both orders, twice.
  auto fg = make_grid(dev, alloc);
  auto gf = make_grid(dev, alloc);
  CHECK(fg.ok() && gf.ok());
  for (int set = 0; set < 2; ++set) {
    CHECK(fuse(integrator.value(), fg.value(), {f, g}, color) == 0);
    CHECK(fuse(integrator.value(), gf.value(), {g, f}, color) == 0);
  }
  if (check_same(dev, alloc, fg.value(), gf.value()) != 0) return 1;
  auto mesh_fg = summarize(mc.value(), fg.value(), kWall);
  auto mesh_gf = summarize(mc.value(), gf.value(), kWall);
  CHECK(mesh_fg.ok() && mesh_gf.ok());
  std::printf("F, G: %zu triangles on F's wall; G, F: %zu\n", mesh_fg->on_wall,
              mesh_gf->on_wall);
  CHECK(mesh_fg->vertices == mesh_gf->vertices);
  CHECK(mesh_fg->on_wall == reference->on_wall);

  // Where G looks -- x = 0, y = 0, a metre down its axis -- F's near band
  // holds weight and F's colour.
  {
    auto blocks = blocks_of(fg.value());
    CHECK(blocks.ok());
    auto weight =
        vr_test::read_attribute<float>(dev, alloc, fg.value(), "weight");
    auto colors =
        vr_test::read_attribute<std::uint32_t>(dev, alloc, fg.value(), "color");
    CHECK(weight.ok() && colors.ok());
    for (const float z : {0.97f, 0.98f, 0.99f, 1.00f}) {
      const std::int64_t i = voxel_at(blocks.value(), 0.0f, 0.0f, z);
      CHECK(i >= 0);
      CHECK(weight.value()[static_cast<std::size_t>(i)] > 0.0f);
      CHECK(colors.value()[static_cast<std::size_t>(i)] == kColor);
    }
  }

  // F's wall recedes: the set clears the old surface, in both orders.
  const View back = frontal(kWall + kRecede);
  CHECK(fuse(integrator.value(), fg.value(), {back, g}, color) == 0);
  CHECK(fuse(integrator.value(), gf.value(), {g, back}, color) == 0);
  if (check_same(dev, alloc, fg.value(), gf.value()) != 0) return 1;
  auto old_wall = summarize(mc.value(), fg.value(), kWall);
  auto new_wall = summarize(mc.value(), fg.value(), kWall + kRecede);
  CHECK(old_wall.ok() && new_wall.ok());
  std::printf("receded: %zu triangles on the old wall, %zu on the new\n",
              old_wall->on_wall, new_wall->on_wall);
  CHECK(old_wall->on_wall == 0);
  CHECK(new_wall->on_wall > 1000);

  std::puts("tsdf_integrate_dynamic_set: OK");
  return 0;
}
