// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// TsdfIntegrator::integrate fuses only the blocks its frames' frusta reach, and
// that must change nothing. Each scenario fuses the same frames into two grids;
// B's calls also open and close with a frame of zero depth whose frustum
// reaches every block -- it fuses nothing, but it makes B's list the whole
// active set, which is what every call fused before the cull. The grids must
// then agree bit for bit: tsdf, weight, colour and block stamps, by block, and
// the tick. The scenarios are the reach's edges: a surface receding under
// dynamic (cleared free space) and classic, a block past max_depth but within
// the band, free space nearer than min_depth, two cameras seeing different
// places, and a principal point outside the image. Exits 0 (skip) where no
// device is present.

#include <array>
#include <cstdint>
#include <cstdio>
#include <map>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/frustum.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "grid_readback.hpp"
#include "no_device.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr std::uint32_t kWidth = 64;
constexpr std::uint32_t kHeight = 48;
constexpr float kTrunc = 0.04f;

using Coord = std::array<int, 3>;

// 1 cm voxels in 8 cm blocks: block z covers world z [0.08 z - 0.005,
// 0.08 z + 0.075] (node-centred voxels, half a voxel either side).
vol::VoxelGridParams grid_params() {
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.01f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = kTrunc;
  grid.bucket_size = 8;
  grid.num_buckets = 1024;
  grid.num_blocks = 1024 * 8;
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

// A camera looking down world +z from `eye`, seeing a wall at depth `z`, in
// one colour.
struct Frame {
  vr::DepthCameraParams cam{};
  std::vector<float> depth;
  std::vector<std::uint32_t> color;
};

Frame wall(float z, vr::Vec3f eye = vr::Vec3f(0.0f), float min_depth = 0.1f,
           float max_depth = 5.0f, float cx = 31.5f) {
  Frame f;
  f.cam.fx = 60.0f;
  f.cam.fy = 60.0f;
  f.cam.cx = cx;
  f.cam.cy = 23.5f;
  f.cam.min_depth = min_depth;
  f.cam.max_depth = max_depth;
  f.cam.width = kWidth;
  f.cam.height = kHeight;
  f.cam.cam_to_world = vr::Mat4f(1.0f);
  f.cam.cam_to_world[3] = vr::Vec4f(eye, 1.0f);
  f.depth.assign(kWidth * kHeight, z);
  f.color.assign(kWidth * kHeight,
                 0xFF000000u | static_cast<std::uint32_t>(z * 200.0f));
  return f;
}

// 50 m back with a near-hemisphere view and no depth: its frustum reaches
// every block here, and it fuses nothing.
Frame witness() {
  Frame f;
  f.cam.fx = 1.0f;
  f.cam.fy = 1.0f;
  f.cam.cx = 100.0f;
  f.cam.cy = 100.0f;
  f.cam.min_depth = 0.1f;
  f.cam.max_depth = 1000.0f;
  f.cam.width = 200;
  f.cam.height = 200;
  f.cam.cam_to_world = vr::Mat4f(1.0f);
  f.cam.cam_to_world[3] = vr::Vec4f(0.0f, 0.0f, -50.0f, 1.0f);
  f.depth.assign(200 * 200, 0.0f);
  return f;
}

// The frustum integrate culls `cam` to.
vol::FrustumPlanes reach(const vr::DepthCameraParams& cam) {
  return vol::make_frustum_planes(cam.fx, cam.fy, cam.cx, cam.cy, cam.width,
                                  cam.height, 0.0f, cam.max_depth + kTrunc,
                                  cam.cam_to_world);
}

// Each block's voxels and stamp, by coordinate: slots are the GPU's choice.
struct Block {
  std::vector<std::uint32_t> tsdf, weight, color;
  vol::BlockStamp stamp;
};

vkc::Result<std::map<Coord, Block>> blocks_of(const vr_test::Gpu& ctx,
                                              vol::VoxelBlockGrid& g) {
  VKC_ASSIGN(const std::vector<vol::BlockIndex> active,
             g.map().compact_active_blocks());
  const auto read = [&](const char* name) {
    return vr_test::read_attribute<std::uint32_t>(ctx.device, ctx.allocator, g,
                                                  name);
  };
  VKC_ASSIGN(const std::vector<std::uint32_t> tsdf, read("tsdf"));
  VKC_ASSIGN(const std::vector<std::uint32_t> weight, read("weight"));
  VKC_ASSIGN(const std::vector<std::uint32_t> color, read("color"));
  VKC_ASSIGN(const std::vector<vol::BlockStamp> stamps,
             g.map().read_block_stamps());
  const std::size_t vpb = grid_params().voxels_per_block;
  std::map<Coord, Block> out;
  for (const vol::BlockIndex& b : active) {
    const auto first = tsdf.begin() + b.ptr;
    Block& block = out[Coord{b.coord.x, b.coord.y, b.coord.z}];
    block.tsdf.assign(first, first + vpb);
    block.weight.assign(weight.begin() + b.ptr, weight.begin() + b.ptr + vpb);
    block.color.assign(color.begin() + b.ptr, color.begin() + b.ptr + vpb);
    block.stamp = stamps[static_cast<std::size_t>(b.ptr) / vpb];
  }
  return out;
}

// Allocate, retrying rounds that only lost bucket-lock races.
int settle(vol::VoxelBlockGrid& g, const std::vector<vol::DepthInput>& in) {
  for (int round = 0; round < 5; ++round) {
    vol::AllocFailures why;
    auto failed = g.map().allocate_from_depth(in, &why);
    CHECK(failed.ok() && !why.capacity_limited());
    if (failed.value() == 0) return 0;
  }
  std::fprintf(stderr, "FAIL: allocation kept losing lock races\n");
  return 1;
}

// A layer of blocks: those whose coordinate on `axis` is `index`.
struct Layer {
  int axis;
  int index;
};
constexpr int kX = 0;
constexpr int kZ = 2;

// Fuse `steps` -- each a set of frames, allocated and then integrated as one
// call -- into A and B, and compare. `fused` is a layer that must end up with
// weighted voxels, which pins that the scenario reaches the edge it is about.
int run(const vr_test::Gpu& ctx, tsdf::TsdfIntegrator& integrator,
        const char* name, tsdf::IntegrationMode mode,
        const std::vector<std::vector<Frame>>& steps, Layer fused) {
  auto a = make_grid(ctx.device, ctx.allocator);
  auto b = make_grid(ctx.device, ctx.allocator);
  CHECK(a.ok() && b.ok());
  const Frame all = witness();
  for (const std::vector<Frame>& frames : steps) {
    std::vector<vol::DepthInput> depths;
    std::vector<tsdf::ColorFrame> colors;
    colors.reserve(frames.size());
    for (const Frame& f : frames) {
      depths.push_back({vkc::StorageInput(f.depth.data()), f.cam});
      colors.push_back(tsdf::ColorFrame{
          f.color.data(),
          vr::ColorCameraParams{f.cam.fx, f.cam.fy, f.cam.cx, f.cam.cy,
                                f.cam.width, f.cam.height, f.cam.cam_to_world},
          {}});
    }
    CHECK(settle(a.value(), depths) == 0);
    CHECK(settle(b.value(), depths) == 0);
    // First and last, so a cull built from the first or the last frame's
    // frustum alone is the witness's.
    std::vector<tsdf::FrameInput> fa;
    std::vector<tsdf::FrameInput> fb{
        {{vkc::StorageInput(all.depth.data()), all.cam}, nullptr}};
    for (std::size_t i = 0; i < frames.size(); ++i) {
      fa.push_back({depths[i], &colors[i]});
      fb.push_back({depths[i], &colors[i]});
    }
    fb.push_back(fb.front());
    auto active = b->map().compact_active_blocks();
    auto reached =
        b->map().compact_active_blocks_in_frusta_on_device({reach(all.cam)});
    CHECK(active.ok() && reached.ok());
    CHECK(reached.value().count == active.value().size());
    CHECK(integrator.integrate(a.value(), fa, 5.0f, mode).ok());
    CHECK(integrator.integrate(b.value(), fb, 5.0f, mode).ok());
  }
  CHECK(a->map().tick() == b->map().tick());
  auto ba = blocks_of(ctx, a.value());
  auto bb = blocks_of(ctx, b.value());
  CHECK(ba.ok() && bb.ok());
  CHECK(ba.value().size() == bb.value().size());
  std::size_t observed = 0;
  std::size_t in_layer = 0;
  for (const auto& [coord, x] : ba.value()) {
    const auto other = bb.value().find(coord);
    CHECK(other != bb.value().end());
    const Block& y = other->second;
    CHECK(x.tsdf == y.tsdf);
    CHECK(x.weight == y.weight);
    CHECK(x.color == y.color);
    CHECK(x.stamp.requested == y.stamp.requested);
    CHECK(x.stamp.weighted == y.stamp.weighted);
    CHECK(x.stamp.changed == y.stamp.changed);
    for (const std::uint32_t w : x.weight) {
      observed += w != 0 ? 1 : 0;
      in_layer += (w != 0 && coord[fused.axis] == fused.index) ? 1 : 0;
    }
  }
  std::printf("  %s: %zu blocks, %zu voxels observed, %zu in the layer\n", name,
              ba.value().size(), observed, in_layer);
  CHECK(in_layer > 0);
  return 0;
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
  const vr_test::Gpu ctx{device.value(), allocator.value()};
  auto integrator =
      tsdf::TsdfIntegrator::create(device.value(), allocator.value());
  CHECK(integrator.ok());
  tsdf::TsdfIntegrator& integ = integrator.value();
  using Mode = tsdf::IntegrationMode;

  // A wall fused twice in place.
  if (run(ctx, integ, "static", Mode::Classic, {{wall(1.0f)}, {wall(1.0f)}},
          Layer{kZ, 12}) != 0) {
    return 1;
  }
  // The wall recedes from 0.6 m to 1.2 m: dynamic clears the free space it
  // left, classic fuses it.
  for (const Mode mode : {Mode::Dynamic, Mode::Classic}) {
    if (run(ctx, integ,
            mode == Mode::Dynamic ? "receding, dynamic" : "receding, classic",
            mode, {{wall(0.6f)}, {wall(1.2f)}}, Layer{kZ, 15}) != 0) {
      return 1;
    }
  }
  // A wall at 1.03 m with max_depth 1.032: layer 13 starts at 1.035, past
  // max_depth, and its first voxels, 1 cm behind the wall, are fused.
  if (run(ctx, integ, "past max_depth", Mode::Classic,
          {{wall(1.03f, vr::Vec3f(0.0f), 0.1f, 1.032f)}}, Layer{kZ, 13}) != 0) {
    return 1;
  }
  // A wall at 0.3 m, then one at 1.0 m seen with min_depth 0.5: the free space
  // at 0.3 m, nearer than min_depth, is still cleared.
  if (run(ctx, integ, "nearer than min_depth", Mode::Dynamic,
          {{wall(0.3f)}, {wall(1.0f, vr::Vec3f(0.0f), 0.5f)}},
          Layer{kZ, 12}) != 0) {
    return 1;
  }
  // Two cameras 2 m apart, each seeing a wall of its own; the second's is in
  // layer 17, which the first camera's frustum does not reach.
  const std::vector<Frame> pair{wall(1.0f),
                                wall(1.4f, vr::Vec3f(2.0f, 0.0f, 0.0f))};
  if (run(ctx, integ, "two cameras", Mode::Classic, {pair, pair},
          Layer{kZ, 17}) != 0) {
    return 1;
  }
  // A principal point 30 px left of the image, which the side widening does
  // not cover: the camera sees x >= 0.5 z, but the widened left plane keeps
  // only x >= 0.556 z, which culls block (5, y, 11) -- x up to 0.475, z from
  // 0.875 -- whose free space at x 0.47, z 0.88 the camera sees. So the
  // integrate fuses every block.
  if (run(ctx, integ, "principal point outside", Mode::Classic,
          {{wall(1.0f, vr::Vec3f(0.0f), 0.1f, 5.0f, -30.0f)}},
          Layer{kX, 5}) != 0) {
    return 1;
  }

  // The cull culls: one camera of the pair reaches part of the map, and both
  // reach more of it, though not the band blocks the allocation dilated past
  // the images' sides.
  {
    auto g = make_grid(ctx.device, ctx.allocator);
    CHECK(g.ok());
    std::vector<vol::DepthInput> depths;
    for (const Frame& f : pair) {
      depths.push_back({vkc::StorageInput(f.depth.data()), f.cam});
    }
    CHECK(settle(g.value(), depths) == 0);
    auto active = g->map().compact_active_blocks();
    auto first = g->map().compact_active_blocks_in_frusta_on_device(
        {reach(pair[0].cam)});
    auto both = g->map().compact_active_blocks_in_frusta_on_device(
        {reach(pair[0].cam), reach(pair[1].cam)});
    CHECK(active.ok() && first.ok() && both.ok());
    std::printf("  cull: %u of %zu blocks for one camera, %u for both\n",
                first.value().count, active.value().size(), both.value().count);
    CHECK(first.value().count > 0);
    CHECK(first.value().count < active.value().size());
    CHECK(both.value().count > first.value().count);

    // A frame reaching none of the blocks still ticks the map, as it did when
    // every call fused the whole set; an empty grid does not tick.
    const Frame away = wall(1.0f, vr::Vec3f(0.0f, 0.0f, 10.0f));
    const std::vector<tsdf::FrameInput> lone{
        {{vkc::StorageInput(away.depth.data()), away.cam}, nullptr}};
    auto missed =
        g->map().compact_active_blocks_in_frusta_on_device({reach(away.cam)});
    CHECK(missed.ok() && missed.value().count == 0);
    const std::uint32_t tick = g->map().tick();
    CHECK(integ.integrate(g.value(), lone).ok());
    CHECK(g->map().tick() == tick + 1);
    auto empty = make_grid(ctx.device, ctx.allocator);
    CHECK(empty.ok());
    const std::uint32_t empty_tick = empty->map().tick();
    CHECK(integ.integrate(empty.value(), lone).ok());
    CHECK(empty->map().tick() == empty_tick);
  }

  std::printf("recon tsdf integrate cull test passed\n");
  return 0;
}
