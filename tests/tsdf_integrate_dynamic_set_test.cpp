// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A Dynamic set's cameras act as one instant: two cameras that disagree give
// the same grid in either order, and the surface one of them sees survives
// the other's free space. F looks down world +z at a wall 1.005 m away, in
// colour. G sits 2 mm in front of that wall's surface and looks along it, at
// a far wall: its rays pass through F's band on the near side, which G sees
// as free space far past the band, and never reach F's wall. Each order is
// fused as a set twice, each camera keeping its slot. The two grids must
// agree bit for bit, so their meshes must too, and F's surface must be whole:
// as many triangles on it as F fused alone gives, and F's colour in it. G's
// free space only averages into F's band, at a tenth of the weight, so it
// moves F's surface by under a millimetre and evicts nothing; the former
// Dynamic cleared F's band there each set, and fused one camera after the
// other erased it, leaving a hole in one order only. And G's far wall must
// mesh. A third set, F's wall receded 10 cm, is a change on every one of F's
// rays (the change test runs from a pixel's second sample here): the set
// resets the old surface, in either order and in the set it is seen.
// Skips where no device is present.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/grid/mesh/marching_cubes.hpp"
#include "volumetric_kit/grid/volume/hash_types.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::grid::mesh;

namespace {

constexpr std::uint32_t kWidth = 160;
constexpr std::uint32_t kHeight = 120;
constexpr float kVoxel = 0.01f;
constexpr float kWall = 1.005f;   // F's wall, between two voxel planes
constexpr float kRecede = 0.1f;   // how far it moves back in the third set
constexpr float kFarWall = 2.0f;  // G's wall, x = 2
constexpr std::uint32_t kColor = 0xFF3366CCu;

using Coord = std::tuple<int, int, int>;

vol::VoxelGridParams grid_params() {
  vol::VoxelGridParams grid{};
  grid.voxel_size = kVoxel;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 512;  // 4,096 blocks: the sets take at most 2,376
  grid.num_blocks = 512 * 8;
  grid.max_chain = 128;
  return grid;
}

vkc::Result<vol::VoxelBlockGrid> make_grid(vkc::Device& device,
                                           vkc::Allocator& allocator) {
  const vol::AttributeSpec attrs[] = {
      {vol::kTsdfAttribute, sizeof(float)},
      {vol::kWeightAttribute, sizeof(float)},
      {vol::kColorAttribute, sizeof(std::uint32_t)}};
  return vol::VoxelBlockGrid::create(device, allocator, grid_params(), attrs,
                                     3);
}

// A camera and the constant depth it sees: a wall square to its axis.
struct View {
  sensor::DepthCameraParams cam{};
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
// x = -1, looking down +x at its wall at x = 2. Its image's down axis is world
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
  return view(pose, 280.0f, 0.0f, kFarWall + 1.0f);
}

// Allocate every frame's band, retrying rounds that only lost bucket-lock
// races (as examples/common/fuse_frame.hpp does).
int allocate(vol::VoxelBlockGrid& grid, const std::vector<View>& views) {
  std::vector<vol::DepthInput> depths;
  for (const View& v : views) {
    depths.push_back(
        {vkc::StorageInput(v.depth.data()), tsdf::to_depth_pinhole(v.cam)});
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

// Allocate for `views`, then fuse them under Dynamic as one set; F (always
// the one with the frontal camera's pose) brings its colour and has slot 0,
// G slot 1.
int fuse(tsdf::TsdfIntegrator& integrator, vol::VoxelBlockGrid& grid,
         const std::vector<View>& views, const tsdf::ColorFrame& color) {
  constexpr tsdf::IntegrationMode kDynamic = tsdf::IntegrationMode::Dynamic;
  if (allocate(grid, views) != 0) return 1;
  std::vector<tsdf::FrameInput> frames;
  for (const View& v : views) {
    const bool is_f = v.cam.cam_to_world == vr::Mat4f(1.0f);
    frames.push_back({vkc::StorageInput(v.depth.data()), v.cam,
                      is_f ? &color : nullptr, is_f ? 0u : 1u});
  }
  CHECK(integrator.integrate(grid, frames, 5.0f, kDynamic).ok());
  return 0;
}

// An integrator a grid, so each keeps its own change histories, whose test
// runs from a pixel's second sample.
vkc::Result<tsdf::TsdfIntegrator> make_integrator(vkc::Device& device,
                                                  vkc::Allocator& allocator) {
  tsdf::ChangeDetectorConfig change;
  change.warm_up = 1;
  return tsdf::TsdfIntegrator::create(device, allocator, change);
}

// The same nonempty block set and every attribute bit, regardless of slots.
int check_same(vkc::Device& device, vkc::Allocator& allocator,
               vol::VoxelBlockGrid& a, vol::VoxelBlockGrid& b) {
  std::size_t blocks = 0;
  CHECK(vgrid_test::same_grids({device, allocator}, a, b, nullptr, &blocks));
  CHECK(blocks != 0);
  return 0;
}

// A mesh's vertices sorted by position, how many of its triangles lie on a
// wall at z = `wall` in F's view (every corner within a voxel of it), which
// leaves out G's far wall, and how many lie on G's far wall.
struct MeshSummary {
  std::vector<std::array<float, 3>> vertices;
  std::size_t on_wall = 0;
  std::size_t on_far_wall = 0;
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
    bool on_g = true;
    for (std::size_t c = 0; c < 3; ++c) {
      const vr::Vec3f& p = m.vertices[m.indices[3 * t + c]].position;
      on = on && std::fabs(p.z - wall) < kVoxel && std::fabs(p.x) < 1.0f;
      on_g = on_g && std::fabs(p.x - kFarWall) < kVoxel;
    }
    s.on_wall += on ? 1 : 0;
    s.on_far_wall += on_g ? 1 : 0;
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

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::Device& dev = gpu.device;
  vkc::Allocator& alloc = gpu.allocator;

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
  auto alone_integrator = make_integrator(dev, alloc);
  CHECK(alone.ok() && alone_integrator.ok());
  for (int set = 0; set < 2; ++set) {
    CHECK(fuse(alone_integrator.value(), alone.value(), {f}, color) == 0);
  }
  auto reference = summarize(mc.value(), alone.value(), kWall);
  CHECK(reference.ok());
  std::printf("F alone: %zu triangles on its wall\n", reference->on_wall);
  CHECK(reference->on_wall > 1000);

  // The set in both orders, twice.
  auto fg = make_grid(dev, alloc);
  auto gf = make_grid(dev, alloc);
  auto fg_integrator = make_integrator(dev, alloc);
  auto gf_integrator = make_integrator(dev, alloc);
  CHECK(fg.ok() && gf.ok() && fg_integrator.ok() && gf_integrator.ok());
  for (int set = 0; set < 2; ++set) {
    CHECK(fuse(fg_integrator.value(), fg.value(), {f, g}, color) == 0);
    CHECK(fuse(gf_integrator.value(), gf.value(), {g, f}, color) == 0);
  }
  if (check_same(dev, alloc, fg.value(), gf.value()) != 0) return 1;
  auto mesh_fg = summarize(mc.value(), fg.value(), kWall);
  auto mesh_gf = summarize(mc.value(), gf.value(), kWall);
  CHECK(mesh_fg.ok() && mesh_gf.ok());
  std::printf("F, G: %zu triangles on F's wall; G, F: %zu; %zu on G's\n",
              mesh_fg->on_wall, mesh_gf->on_wall, mesh_fg->on_far_wall);
  CHECK(mesh_fg->vertices == mesh_gf->vertices);
  CHECK(mesh_fg->on_wall == reference->on_wall);
  CHECK(mesh_fg->on_far_wall > 1000);

  // Where G looks -- x = 0, y = 0, a metre down its axis -- F's near band
  // holds weight and F's colour.
  {
    auto blocks = vgrid_test::blocks_of(fg.value());
    CHECK(blocks.ok());
    auto weight = vgrid_test::read_attribute<float>(dev, alloc, fg.value(),
                                                    vol::kWeightAttribute);
    auto colors = vgrid_test::read_attribute<std::uint32_t>(
        dev, alloc, fg.value(), vol::kColorAttribute);
    CHECK(weight.ok() && colors.ok());
    for (const float z : {0.97f, 0.98f, 0.99f, 1.00f}) {
      const std::int64_t i = voxel_at(blocks.value(), 0.0f, 0.0f, z);
      CHECK(i >= 0);
      CHECK(weight.value()[static_cast<std::size_t>(i)] > 0.0f);
      CHECK(colors.value()[static_cast<std::size_t>(i)] == kColor);
    }
  }

  // F's wall recedes: the set resets the old surface, in both orders.
  const View back = frontal(kWall + kRecede);
  CHECK(fuse(fg_integrator.value(), fg.value(), {back, g}, color) == 0);
  CHECK(fuse(gf_integrator.value(), gf.value(), {g, back}, color) == 0);
  if (check_same(dev, alloc, fg.value(), gf.value()) != 0) return 1;
  auto old_wall = summarize(mc.value(), fg.value(), kWall);
  auto new_wall = summarize(mc.value(), fg.value(), kWall + kRecede);
  CHECK(old_wall.ok() && new_wall.ok());
  std::printf("receded: %zu triangles on the old wall, %zu on the new\n",
              old_wall->on_wall, new_wall->on_wall);
  CHECK(old_wall->on_wall == 0);
  CHECK(new_wall->on_wall > 1000);
  CHECK(new_wall->on_far_wall == mesh_fg->on_far_wall);

  std::puts("tsdf_integrate_dynamic_set: OK");
  return 0;
}

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
