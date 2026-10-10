// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The several-camera overloads of VoxelHashMap::allocate_from_depth and
// TsdfIntegrator::integrate against the one-frame ones. Three cameras round a
// sphere, fused twice into one grid one frame at a time and into another as a
// set, must give, classic, the same blocks and the same tsdf, weight and colour
// bit for bit: each frame is a dispatch of its own in both, in the same order.
// Dynamic, a set is one instant, every frame's clear before any frame's fuse,
// and the cameras disagree near their silhouettes, so the reference is the set
// in the other order: the same blocks, the same voxels observed and coloured,
// and the same field but for the running average's rounding. The frames mix
// host arrays and storage buffers, colour and none, and the set carries an
// empty frame, which both calls skip. The set grows the map's and the
// integrator's sets past an earlier call's, and stamps changed the blocks the
// reference changes, so the stamps are bound on every set. And a set with one
// bad frame is refused before anything is fused. Skips where no
// device is present.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "test_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "gpu_test.hpp"
#include "grid_readback.hpp"
#include "sphere_scene.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;

namespace {

constexpr std::uint32_t kWidth = vr_test::kSphereWidth;
constexpr std::uint32_t kHeight = vr_test::kSphereHeight;
constexpr int kCameras = 3;

vol::VoxelGridParams grid_params() {
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.01f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
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

// The blocks of `g` stamped changed after `since`.
vkc::Result<std::uint32_t> changed_after(const vol::VoxelBlockGrid& g,
                                         std::uint32_t since) {
  VKC_ASSIGN(const std::vector<vol::BlockStamp> stamps,
             g.map().read_block_stamps());
  std::uint32_t n = 0;
  for (const vol::BlockStamp& st : stamps) {
    n += vol::tick_after(st.changed, since) ? 1u : 0u;
  }
  return n;
}

// A float attribute's element from its bits.
float bits_float(std::uint32_t bits) {
  float f = 0.0f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

// The same blocks, and in each the same bits of weight, tsdf and colour in
// every voxel; or, not `exact`, the same voxels observed and coloured, and
// weight and tsdf within rounding. A slot can differ: allocation order is the
// GPU's.
int check_same(const vr_test::Gpu& ctx, vol::VoxelBlockGrid& a,
               vol::VoxelBlockGrid& b, bool exact) {
  std::size_t observed = 0;
  std::size_t blocks = 0;
  CHECK(vr_test::compare_grids(
      ctx, a, b,
      [exact](vr_test::VoxelWords x, vr_test::VoxelWords y) {
        if (exact) {
          return x.weight == y.weight && x.tsdf == y.tsdf && x.color == y.color;
        }
        return (x.weight == 0) == (y.weight == 0) &&
               (x.color == 0) == (y.color == 0) &&
               std::fabs(bits_float(x.weight) - bits_float(y.weight)) <=
                   1e-5f * bits_float(x.weight) &&
               std::fabs(bits_float(x.tsdf) - bits_float(y.tsdf)) <= 1e-6f;
      },
      &observed, &blocks));
  std::printf("  %zu blocks, %zu voxels observed\n", blocks, observed);
  CHECK(observed > 10000);
  return 0;
}

// Allocate, retrying rounds that only lost bucket-lock races, so both grids
// hold every block.
template <typename Allocate>
int settle(Allocate&& allocate) {
  for (int round = 0; round < 5; ++round) {
    vol::AllocFailures why;
    auto failed = allocate(&why);
    CHECK(failed.ok() && !why.capacity_limited());
    if (failed.value() == 0) return 0;
  }
  std::fprintf(stderr, "FAIL: allocation kept losing lock races\n");
  return 1;
}

int gpu_main(vr_test::GpuContext& gpu) {
  vkc::Device& dev = gpu.device;
  vkc::Allocator& alloc = gpu.allocator;

  std::vector<vr_test::SphereView> views;
  for (int c = 0; c < kCameras; ++c) views.push_back(vr_test::sphere_view(c));
  // Camera 0 marks its coverage: a band of its colour says it has none.
  for (std::uint32_t row = 0; row < kHeight; ++row) {
    for (std::uint32_t col = 60; col < 80; ++col) {
      views[0].color[row * kWidth + col] = 0u;
    }
  }
  // Cameras 0 and 2 are on the device, 1 on the host; 2 has no colour.
  std::vector<vkc::Buffer> depth_bufs;
  for (int c = 0; c < kCameras; ++c) {
    auto buf =
        vr_test::upload_device_buffer(dev, alloc, views[c].depth.data(),
                                      views[c].depth.size() * sizeof(float));
    CHECK(buf.ok());
    CHECK(buf.value().is_device_local() && buf.value().mapped() == nullptr);
    depth_bufs.push_back(std::move(buf).value());
  }
  auto color0 = vr_test::upload_device_buffer(
      dev, alloc, views[0].color.data(),
      views[0].color.size() * sizeof(std::uint32_t));
  CHECK(color0.ok());
  const auto color_cam = [&](const vr_test::SphereView& v) {
    return vr::ColorCameraParams{v.cam.fx,          v.cam.fy,    v.cam.cx,
                                 v.cam.cy,          v.cam.width, v.cam.height,
                                 v.cam.cam_to_world};
  };
  tsdf::ColorFrame c0{};
  c0.buffer = &color0.value();
  c0.cam = color_cam(views[0]);
  c0.coverage_in_alpha = true;
  tsdf::ColorFrame c1{views[1].color.data(), color_cam(views[1]), {}};

  const std::vector<tsdf::FrameInput> frames{
      {{vkc::StorageInput(depth_bufs[0]), views[0].cam}, &c0},
      {{vkc::StorageInput(views[1].depth.data()), views[1].cam}, &c1},
      {{vkc::StorageInput(depth_bufs[2]), views[2].cam}, nullptr}};
  // The set, with an empty frame second, and the list it allocates from.
  vr::DepthCameraParams empty_cam = views[1].cam;
  empty_cam.width = 0;
  const std::vector<tsdf::FrameInput> set_frames{
      frames[0],
      {{vkc::StorageInput(views[1].depth.data()), empty_cam}, &c1},
      frames[1],
      frames[2]};
  const std::vector<vol::DepthInput> depths(set_frames.begin(),
                                            set_frames.end());

  // The set in the other order, dynamic's reference.
  const std::vector<tsdf::FrameInput> reversed(set_frames.rbegin(),
                                               set_frames.rend());
  const std::vector<vol::DepthInput> reversed_depths(reversed.begin(),
                                                     reversed.end());

  auto integrator = tsdf::TsdfIntegrator::create(dev, alloc);
  CHECK(integrator.ok());
  for (const tsdf::IntegrationMode mode :
       {tsdf::IntegrationMode::Classic, tsdf::IntegrationMode::Dynamic}) {
    const bool classic = mode == tsdf::IntegrationMode::Classic;
    // The other order moves the running average wherever the weight cap cuts
    // it, so dynamic's cap lies past what two passes reach.
    const float max_weight = classic ? 5.0f : 100.0f;
    auto one = make_grid(dev, alloc);  // the reference
    auto set = make_grid(dev, alloc);
    CHECK(one.ok() && set.ok());
    // Twice, so the running average and the colour blend run too. Each pass
    // allocates every frame's band before fusing any, as the set does, and
    // counts the blocks each grid stamped changed after its tick going in: a
    // tick a frame for one frame at a time, one for a set.
    for (int pass = 0; pass < 2; ++pass) {
      const std::uint32_t one_since = one->map().tick();
      const std::uint32_t set_since = set->map().tick();
      if (classic) {
        for (int c = 0; c < kCameras; ++c) {
          CHECK(settle([&](vol::AllocFailures* why) {
                  return c == 1 ? one->map().allocate_from_depth(
                                      views[1].depth.data(), views[1].cam, why)
                                : one->map().allocate_from_depth(
                                      depth_bufs[c], views[c].cam, why);
                }) == 0);
        }
        for (int c = 0; c < kCameras; ++c) {
          const tsdf::FrameInput& f = frames[c];
          const vkc::Status fused =
              c == 1
                  ? integrator->integrate(one.value(), views[1].depth.data(),
                                          f.camera, max_weight, mode, f.color)
                  : integrator->integrate(one.value(), depth_bufs[c], f.camera,
                                          max_weight, mode, f.color);
          CHECK(fused.ok());
        }
        CHECK(one->map().tick() == one_since + std::uint32_t{kCameras});
      } else {
        CHECK(settle([&](vol::AllocFailures* why) {
                return one->map().allocate_from_depth(reversed_depths, why);
              }) == 0);
        CHECK(integrator->integrate(one.value(), reversed, max_weight, mode)
                  .ok());
        CHECK(one->map().tick() == one_since + 1);
      }
      auto dirty_one = changed_after(one.value(), one_since);
      CHECK(dirty_one.ok() && dirty_one.value() > 0);
      if (pass == 0) {
        // One frame first, so the full set grows the map's sets.
        CHECK(settle([&](vol::AllocFailures* why) {
                return set->map().allocate_from_depth(
                    std::vector<vol::DepthInput>{depths[0]}, why);
              }) == 0);
      }
      CHECK(settle([&](vol::AllocFailures* why) {
              return set->map().allocate_from_depth(depths, why);
            }) == 0);
      CHECK(integrator->integrate(set.value(), set_frames, max_weight, mode)
                .ok());
      CHECK(set->map().tick() == set_since + 1);
      auto dirty_set = changed_after(set.value(), set_since);
      CHECK(dirty_set.ok());
      CHECK(dirty_set.value() == dirty_one.value());
    }
    std::printf("%s:\n", classic ? "classic" : "dynamic");
    if (check_same(gpu, one.value(), set.value(), classic) != 0) return 1;
  }

  // A set with one bad frame -- here the last, its depth smaller than its
  // image -- is refused before anything is allocated or fused.
  {
    auto grid = make_grid(dev, alloc);
    CHECK(grid.ok());
    auto small = vr_test::upload_device_buffer(
        dev, alloc, views[2].depth.data(), sizeof(float) * kWidth);
    CHECK(small.ok());
    std::vector<vol::DepthInput> bad_depths(frames.begin(), frames.end());
    bad_depths[2] = {vkc::StorageInput(small.value()), views[2].cam};
    CHECK(grid->map().allocate_from_depth(bad_depths).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    auto none = vr_test::blocks_of(grid.value());
    CHECK(none.ok() && none.value().empty());

    CHECK(settle([&](vol::AllocFailures* why) {
            return grid->map().allocate_from_depth(depths, why);
          }) == 0);
    std::vector<tsdf::FrameInput> bad = frames;
    bad[2] = {{vkc::StorageInput(small.value()), views[2].cam}, nullptr};
    CHECK(integrator->integrate(grid.value(), bad).domain() ==
          vkc::Status::Code::InvalidArgument);
    auto weight =
        vr_test::read_attribute<float>(dev, alloc, grid.value(), "weight");
    CHECK(weight.ok());
    for (const float w : weight.value()) CHECK(w == 0.0f);

    // And an empty set does nothing, without refusing.
    CHECK(grid->map().allocate_from_depth(std::vector<vol::DepthInput>{}).ok());
    CHECK(integrator->integrate(grid.value(), std::vector<tsdf::FrameInput>{})
              .ok());
  }

  std::puts("tsdf_integrate_set: OK");
  return 0;
}

}  // namespace

int main() { return vr_test::run_on_gpu(gpu_main); }
