// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Complete fusion is the default; live callers explicitly accept partial
// bands. Growth, grow-ahead, and a bounded failure preserve the expected
// voxel data and integration tick. Skips when no device is available.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "test_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
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

constexpr int kCameras = 3;

vol::VoxelGridParams params(std::int32_t num_buckets) {
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.01f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = num_buckets;
  grid.num_blocks = 8 * num_buckets;
  grid.max_chain = 128;
  return grid;
}

vkc::Result<vol::VoxelBlockGrid> make_grid(vkc::Device& device,
                                           vkc::Allocator& allocator,
                                           const vol::VoxelGridParams& grid) {
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)},
                                      {"color", sizeof(std::uint32_t)}};
  return vol::VoxelBlockGrid::create(device, allocator, grid, attrs, 3);
}

vkc::Result<tsdf::Fuser> make_fuser(vkc::Device& device,
                                    vkc::Allocator& allocator,
                                    tsdf::FuserConfig config = {}) {
  return tsdf::Fuser::create(device, allocator, std::move(config));
}

// Whether any voxel of `grid` holds weight: the set was integrated.
vkc::Result<bool> fused_any(const vr_test::Gpu& gpu,
                            const vol::VoxelBlockGrid& grid) {
  VKC_ASSIGN(const std::vector<float> weight,
             vr_test::read_attribute<float>(gpu.device, gpu.allocator, grid,
                                            "weight"));
  for (const float w : weight) {
    if (w > 0.0f) return true;
  }
  return false;
}

}  // namespace

int gpu_main(vr_test::GpuContext& gpu) {
  vkc::Device& dev = gpu.device;
  vkc::Allocator& alloc = gpu.allocator;
  const vr_test::Gpu ctx{dev, alloc};

  // The set: three cameras, depth and colour on the host.
  std::vector<vr_test::SphereView> views;
  for (int c = 0; c < kCameras; ++c) views.push_back(vr_test::sphere_view(c));
  std::vector<tsdf::ColorFrame> colors;
  for (const vr_test::SphereView& v : views) {
    colors.push_back({v.color.data(),
                      {v.cam.fx, v.cam.fy, v.cam.cx, v.cam.cy, v.cam.width,
                       v.cam.height, v.cam.cam_to_world},
                      {}});
  }
  std::vector<tsdf::FrameInput> set;
  for (int c = 0; c < kCameras; ++c) {
    set.push_back({{vkc::StorageInput(views[c].depth.data()), views[c].cam},
                   &colors[std::size_t(c)]});
  }

  // The reference: the tier calls on a grid with room for the set, twice, so
  // the running average and the colour blend run too.
  auto integrator = tsdf::TsdfIntegrator::create(dev, alloc);
  auto reference = make_grid(dev, alloc, params(1024));
  CHECK(integrator.ok() && reference.ok());
  const std::vector<vol::DepthInput> depths(set.begin(), set.end());
  for (int pass = 0; pass < 2; ++pass) {
    auto failed = reference->map().allocate_from_depth(depths);
    CHECK(failed.ok() && failed.value() == 0);
    CHECK(integrator->integrate(*reference, set).ok());
  }
  auto reference_blocks = vr_test::blocks_of(*reference);
  CHECK(reference_blocks.ok());
  const auto blocks = std::int32_t(reference_blocks->size());
  std::printf("  the set allocates %d blocks\n", blocks);
  CHECK(blocks > 256);

  // Default settings complete a band requiring four doublings. The former
  // two-grow default returned success with only 256 of these 604 blocks.
  {
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(8));
    CHECK(fuser.ok() && grid.ok());
    for (int pass = 0; pass < 2; ++pass) {
      auto report = fuser->fuse(*grid, set);
      CHECK(report.ok() && report->dropped == 0);
      CHECK(report->load_factor == grid->map().load_factor().value());
      if (pass == 0) {
        CHECK(report->grows == 4);
        CHECK(report->from_buckets == 8 && report->to_buckets == 128);
      }
    }
    std::size_t observed = 0;
    CHECK(vr_test::same_grids(ctx, *reference, *grid, &observed));
    CHECK(observed > 10000);
  }

  // Grow ahead before a failure, retaining all previously fused voxels.
  {
    const std::int32_t buckets = (blocks + 5) / 6;
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(buckets));
    CHECK(fuser.ok() && grid.ok());
    auto report = fuser->fuse(*grid, set);
    CHECK(report.ok() && report->grows == 0 && report->dropped == 0);
    CHECK(report->load_factor > vol::VoxelHashMap::kGrowThreshold);
    report = fuser->fuse(*grid, set);
    CHECK(report.ok() && report->grows == 1 && report->dropped == 0);
    CHECK(report->from_buckets == buckets && report->to_buckets == 2 * buckets);
    CHECK(vr_test::same_grids(ctx, *reference, *grid));
    const std::uint32_t tick = grid->map().tick();
    report = fuser->fuse(*grid, {});
    CHECK(report.ok() && report->grows == 0 && grid->map().tick() == tick);
  }

  // An incomplete band is an error before integration. A later fuser with
  // enough growth allowance can complete the same grid and frame.
  {
    tsdf::FuserConfig config;
    config.max_grows_per_set = 2;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(8));
    CHECK(fuser.ok() && grid.ok());
    const std::uint32_t tick = grid->map().tick();
    auto refused = fuser->fuse(*grid, set);
    CHECK(!refused.ok() &&
          refused.status().domain() == vkc::Status::Code::OutOfMemory);
    CHECK(grid->map().tick() == tick);
    auto any = fused_any(ctx, *grid);
    CHECK(any.ok() && !any.value());
    auto retry = make_fuser(dev, alloc);
    CHECK(retry.ok());
    auto report = retry->fuse(*grid, set);
    CHECK(report.ok() && report->dropped == 0);
    CHECK(retry->fuse(*grid, set).ok());
    CHECK(vr_test::same_grids(ctx, *reference, *grid));
  }

  // Live callers opt in: each bound includes the next set's grow-ahead,
  // still integrates what fits, and reports the allocation failures.
  for (const int bound : {0, 1, 2}) {
    tsdf::FuserConfig config;
    config.max_grows_per_set = bound;
    config.allow_partial = true;
    auto fuser = make_fuser(dev, alloc, config);
    auto grid = make_grid(dev, alloc, params(1));
    CHECK(fuser.ok() && grid.ok());
    for (int pass = 1; pass <= 2; ++pass) {
      auto report = fuser->fuse(*grid, set);
      CHECK(report.ok() && report->grows == bound);
      CHECK(report->dropped > 0 && report->failures.capacity_limited());
      CHECK(grid->grid().num_buckets == 1 << (bound * pass));
      auto any = fused_any(ctx, *grid);
      CHECK(any.ok() && any.value());
    }
    auto invalid = set;
    invalid[0].depth = vkc::StorageInput(static_cast<const float*>(nullptr));
    CHECK(fuser->fuse(*grid, invalid).status().domain() ==
          vkc::Status::Code::InvalidArgument);
  }

  // Reject invalid configuration and moved-from objects.
  {
    tsdf::FuserConfig bad;
    bad.max_grows_per_set = -1;
    CHECK(make_fuser(dev, alloc, bad).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    auto fuser = make_fuser(dev, alloc);
    auto grid = make_grid(dev, alloc, params(1024));
    CHECK(fuser.ok() && grid.ok());
    tsdf::Fuser moved(std::move(fuser).value());
    CHECK(!fuser->valid() && moved.valid());
    CHECK(fuser->fuse(*grid, set).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    CHECK(moved.fuse(*grid, set).ok());
  }

  std::puts("tsdf_fuser: OK");
  return 0;
}

int main() { return vr_test::run_on_gpu(gpu_main); }
