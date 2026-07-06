// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for TsdfIntegrator: fuse a synthetic constant-depth plane into a
// VoxelBlockGrid and verify the classic projective TSDF math per voxel --
// sdf = depth - Zc, truncation, inverse-square weight with behind-surface
// dropoff, and the running-average weight cap across two frames. Runs on the
// real driver (MoltenVK / NVIDIA); exits 0 (skip) where no device is present.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr = volumetric_kit::recon;
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

bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

int local_index(int lx, int ly, int lz, int bs) {
  return lx + ly * bs + lz * bs * bs;
}

std::int32_t find_ptr(const std::vector<vol::BlockIndex>& active,
                      vr::Vec3i coord) {
  for (const vol::BlockIndex& b : active) {
    if (b.coord.x == coord.x && b.coord.y == coord.y && b.coord.z == coord.z) {
      return b.ptr;
    }
  }
  return -1;
}

}  // namespace

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance (%s); skipping\n",
                 instance.status().message().c_str());
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device (%s); skipping\n",
                 gpu.status().message().c_str());
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  if (!device) {
    std::fprintf(stderr, "device create failed: %s\n",
                 device.status().message().c_str());
    return 1;
  }
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  if (!allocator) {
    std::fprintf(stderr, "allocator create failed: %s\n",
                 allocator.status().message().c_str());
    return 1;
  }

  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.005f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 256;
  grid.num_blocks = 256 * 8;
  grid.max_chain = 128;
  const int bs = grid.block_size;

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vr::Result<vol::VoxelBlockGrid> grid_result = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, attrs, 2);
  if (!grid_result) {
    std::fprintf(stderr, "VoxelBlockGrid::create failed: %s\n",
                 grid_result.status().message().c_str());
    return 1;
  }
  vol::VoxelBlockGrid vbg = std::move(grid_result).value();

  vr::Result<tsdf::TsdfIntegrator> integ_result =
      tsdf::TsdfIntegrator::create(device.value(), allocator.value());
  if (!integ_result) {
    std::fprintf(stderr, "TsdfIntegrator::create failed: %s\n",
                 integ_result.status().message().c_str());
    return 1;
  }
  tsdf::TsdfIntegrator integ = std::move(integ_result).value();

  // Allocate a column of blocks straddling a plane at z = 0.5 m. Block z-coord
  // 12 spans voxels z in [96,103] (world 0.48..0.515), block 13 spans [104,111]
  // (0.52..0.555) -- so the plane at 0.5 falls inside block 12.
  std::vector<vol::BlockIndex> blocks;
  for (int cz = 11; cz <= 14; ++cz) {
    vol::BlockIndex b{};
    b.coord = vr::Vec3i(0, 0, cz);
    blocks.push_back(b);
  }
  CHECK(vbg.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> active =
      vbg.map().compact_active_blocks();
  CHECK(active.ok() && active.value().size() == blocks.size());

  // Camera at the world origin looking down +Z (identity pose), a pinhole with
  // a 640x480 image; the plane fills the view at a constant 0.5 m.
  vol::DepthCameraParams cam{};
  cam.fx = 525.0f;
  cam.fy = 525.0f;
  cam.cx = 320.0f;
  cam.cy = 240.0f;
  cam.min_depth = 0.1f;
  cam.max_depth = 10.0f;
  cam.width = 640;
  cam.height = 480;
  cam.cam_to_world = vr::Mat4f(1.0f);
  const float plane_z = 0.5f;
  std::vector<float> depth(static_cast<std::size_t>(cam.width) * cam.height,
                           plane_z);

  CHECK(integ.integrate(vbg, depth.data(), cam, /*max_weight=*/5.0f).ok());

  vr::Result<vol::AttributeView> tsdf_view = vbg.attribute("tsdf");
  vr::Result<vol::AttributeView> weight_view = vbg.attribute("weight");
  CHECK(tsdf_view.ok() && weight_view.ok());
  const auto* tsdf_data =
      static_cast<const float*>(tsdf_view.value().buffer->mapped());
  const auto* weight_data =
      static_cast<const float*>(weight_view.value().buffer->mapped());

  const std::int32_t p12 = find_ptr(active.value(), vr::Vec3i(0, 0, 12));
  const std::int32_t p13 = find_ptr(active.value(), vr::Vec3i(0, 0, 13));
  CHECK(p12 >= 0 && p13 >= 0);

  // On-axis voxels (x=y=0) project to the principal point, sampling depth 0.5;
  // with the identity pose Zc equals the voxel's world z, so sdf = 0.5 - z.

  // (1) In front of the surface: voxel z=97 -> world 0.485, sdf = +0.015
  // (within the band). First observation, so weight = 1 / Zc^2 (no behind
  // dropoff).
  const std::size_t front =
      static_cast<std::size_t>(p12) + local_index(0, 0, 1, bs);
  CHECK(approx(tsdf_data[front], 0.015f, 1e-3f));
  CHECK(approx(weight_data[front], 1.0f / (0.485f * 0.485f), 2e-2f));

  // (2) Behind the surface but within the band: voxel z=104 -> world 0.52,
  // sdf = -0.02; weight = (1/Zc^2) * (trunc - behind)/trunc with behind = 0.02.
  const std::size_t behind =
      static_cast<std::size_t>(p13) + local_index(0, 0, 0, bs);
  CHECK(approx(tsdf_data[behind], -0.02f, 1e-3f));
  const float w_behind = (1.0f / (0.52f * 0.52f)) * ((0.04f - 0.02f) / 0.04f);
  CHECK(approx(weight_data[behind], w_behind, 2e-2f));

  // (3) Occluded (sdf < -trunc): voxel z=110 -> world 0.55, sdf = -0.05 -> the
  // voxel is skipped and stays untouched (zero weight).
  const std::size_t occluded =
      static_cast<std::size_t>(p13) + local_index(0, 0, 6, bs);
  CHECK(tsdf_data[occluded] == 0.0f && weight_data[occluded] == 0.0f);

  // Aggregate: some voxels were fused, and every fused sdf is inside the band.
  std::size_t touched = 0;
  for (std::uint64_t i = 0; i < tsdf_view.value().element_count; ++i) {
    if (weight_data[i] > 0.0f) {
      ++touched;
      CHECK(tsdf_data[i] >= -grid.trunc_dist - 1e-4f &&
            tsdf_data[i] <= grid.trunc_dist + 1e-4f);
    }
  }
  CHECK(touched > 0);

  // A second identical frame: the same observation, so sdf is unchanged, but
  // the weight accumulates and saturates at the cap (4.25 + 4.25 -> min(8.5,
  // 5)).
  CHECK(integ.integrate(vbg, depth.data(), cam, /*max_weight=*/5.0f).ok());
  CHECK(approx(tsdf_data[front], 0.015f, 1e-3f));
  CHECK(approx(weight_data[front], 5.0f, 1e-3f));

  std::printf(
      "recon tsdf integrate test passed: classic projective fusion of a 0.5 m "
      "plane, %zu voxels integrated, weight caps at 5.0\n",
      touched);
  return 0;
}
