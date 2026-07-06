// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for TsdfIntegrator: fuse a synthetic constant-depth plane into a
// VoxelBlockGrid and verify the classic projective TSDF math per voxel --
// sdf = depth - Zc, truncation, inverse-square weight with behind-surface
// dropoff, and the running-average weight cap across two frames. Runs on the
// real driver (MoltenVK / NVIDIA); exits 0 (skip) where no device is present.

#include <algorithm>
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

  // A non-identity rigid pose exercises the in-shader world->camera transform
  // (R^T (world - t)); the identity pose above cannot tell it apart from a
  // no-op. Fresh grid + one frame, then cross-check every fused voxel's sdf
  // against an independent glm::inverse projection -- the general inverse the
  // rigid R^T must equal.
  vr::Result<vol::VoxelBlockGrid> grid2 = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, attrs, 2);
  CHECK(grid2.ok());
  vol::VoxelBlockGrid vbg2 = std::move(grid2).value();
  CHECK(vbg2.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);

  const float theta = 0.2f;  // ~11 deg about +Y
  const float ct = std::cos(theta);
  const float st = std::sin(theta);
  vol::DepthCameraParams cam2 = cam;  // same intrinsics, range, and 0.5 m plane
  cam2.cam_to_world = vr::Mat4f(1.0f);
  cam2.cam_to_world[0] = vr::Vec4f(ct, 0.0f, -st, 0.0f);
  cam2.cam_to_world[2] = vr::Vec4f(st, 0.0f, ct, 0.0f);
  cam2.cam_to_world[3] = vr::Vec4f(0.02f, -0.01f, 0.03f, 1.0f);
  CHECK(integ.integrate(vbg2, depth.data(), cam2, /*max_weight=*/5.0f).ok());

  const vr::Mat4f world_to_cam = glm::inverse(cam2.cam_to_world);
  vr::Result<std::vector<vol::BlockIndex>> active2 =
      vbg2.map().compact_active_blocks();
  CHECK(active2.ok());
  const auto* tsdf2 = static_cast<const float*>(
      vbg2.attribute("tsdf").value().buffer->mapped());
  const auto* weight2 = static_cast<const float*>(
      vbg2.attribute("weight").value().buffer->mapped());
  std::size_t cross_checked = 0;
  for (const vol::BlockIndex& b : active2.value()) {
    for (int lz = 0; lz < bs; ++lz) {
      for (int ly = 0; ly < bs; ++ly) {
        for (int lx = 0; lx < bs; ++lx) {
          const std::size_t idx =
              static_cast<std::size_t>(b.ptr) + local_index(lx, ly, lz, bs);
          if (weight2[idx] <= 0.0f) {
            continue;
          }
          const vr::Vec3f world =
              vr::Vec3f(vr::Vec3i(b.coord.x * bs + lx, b.coord.y * bs + ly,
                                  b.coord.z * bs + lz)) *
              grid.voxel_size;
          const vr::Vec4f p_cam = world_to_cam * vr::Vec4f(world, 1.0f);
          const float expected =
              std::clamp(plane_z - p_cam.z, -grid.trunc_dist, grid.trunc_dist);
          CHECK(approx(tsdf2[idx], expected, 2e-3f));
          ++cross_checked;
        }
      }
    }
  }
  CHECK(cross_checked > 0);

  // Dynamic integration clears stale free-space voxels. A voxel at world z=0.48
  // sits just in front of a plane at 0.5 (sdf = +0.02, fused with weight). Move
  // the plane to 0.6: that voxel is now free space well in front of the surface
  // (sdf = 0.12 > trunc). Dynamic clears it; classic keeps it (clamped).
  std::vector<float> depth_far(depth.size(), 0.6f);
  const std::size_t corner_local =
      static_cast<std::size_t>(local_index(0, 0, 0, bs));

  vr::Result<vol::VoxelBlockGrid> dyn = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, attrs, 2);
  CHECK(dyn.ok());
  vol::VoxelBlockGrid vbg_dyn = std::move(dyn).value();
  CHECK(vbg_dyn.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> dyn_active =
      vbg_dyn.map().compact_active_blocks();
  CHECK(dyn_active.ok());
  const std::int32_t pd = find_ptr(dyn_active.value(), vr::Vec3i(0, 0, 12));
  CHECK(pd >= 0);
  const std::size_t vd = static_cast<std::size_t>(pd) + corner_local;
  const auto* dyn_tsdf = static_cast<const float*>(
      vbg_dyn.attribute("tsdf").value().buffer->mapped());
  const auto* dyn_weight = static_cast<const float*>(
      vbg_dyn.attribute("weight").value().buffer->mapped());
  CHECK(integ.integrate(vbg_dyn, depth.data(), cam, 5.0f).ok());
  CHECK(dyn_weight[vd] > 0.0f);  // fused at sdf = +0.02
  CHECK(integ
            .integrate(vbg_dyn, depth_far.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(dyn_weight[vd] == 0.0f && dyn_tsdf[vd] == 0.0f);  // stale voxel cleared

  // Dynamic clears only free space *past* the band, never near-surface
  // geometry: a voxel at world z=0.58 is 0.02 m in front of the receded 0.6 m
  // plane (sdf < trunc), so the dynamic frame fuses it rather than clearing it.
  // An over-clearing regression keyed on sdf>0 instead of sdf>trunc would hit
  // the clear branch and early-return, leaving this voxel at weight 0.
  const std::int32_t pd14 = find_ptr(dyn_active.value(), vr::Vec3i(0, 0, 14));
  CHECK(pd14 >= 0);
  const std::size_t vn = static_cast<std::size_t>(pd14) +
                         static_cast<std::size_t>(local_index(0, 0, 4, bs));
  CHECK(dyn_weight[vn] > 0.0f);  // near-surface voxel fused, not over-cleared

  // Classic keeps the same free-space voxel (fresh grid, same two frames).
  vr::Result<vol::VoxelBlockGrid> cls = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, attrs, 2);
  CHECK(cls.ok());
  vol::VoxelBlockGrid vbg_cls = std::move(cls).value();
  CHECK(vbg_cls.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> cls_active =
      vbg_cls.map().compact_active_blocks();
  CHECK(cls_active.ok());
  const std::int32_t pcl = find_ptr(cls_active.value(), vr::Vec3i(0, 0, 12));
  CHECK(pcl >= 0);
  const std::size_t vc = static_cast<std::size_t>(pcl) + corner_local;
  const auto* cls_weight = static_cast<const float*>(
      vbg_cls.attribute("weight").value().buffer->mapped());
  CHECK(integ.integrate(vbg_cls, depth.data(), cam, 5.0f).ok());
  CHECK(integ.integrate(vbg_cls, depth_far.data(), cam, 5.0f).ok());  // classic
  CHECK(cls_weight[vc] > 0.0f);  // kept (clamped to +trunc, fused)

  // Bilinear depth sampling. cx = 321.0 puts the on-axis voxel's projection at
  // pixel u = 321.0, so the bilinear taps straddle columns 320 and 321 at
  // fx = 0.5 (a 50/50 blend). The on-axis voxel is at world z = 0.48.
  vol::DepthCameraParams bcam = cam;
  bcam.cx = 321.0f;
  const std::size_t bw = bcam.width;
  const std::size_t on_axis =
      static_cast<std::size_t>(local_index(0, 0, 0, bs));

  // (interp) A 0.48 / 0.50 step across the taps (< trunc) blends to 0.49, so
  // sdf = 0.49 - 0.48 = 0.01 -- distinct from the nearest sample (0.50 ->
  // 0.02).
  std::vector<float> depth_interp(depth.size(), 0.48f);
  for (std::uint32_t y = 0; y < bcam.height; ++y) {
    depth_interp[static_cast<std::size_t>(y) * bw + 321] = 0.50f;
  }
  vr::Result<vol::VoxelBlockGrid> bi = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, attrs, 2);
  CHECK(bi.ok());
  vol::VoxelBlockGrid vbg_bi = std::move(bi).value();
  CHECK(vbg_bi.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> bi_active =
      vbg_bi.map().compact_active_blocks();
  CHECK(bi_active.ok());
  const std::int32_t pbi = find_ptr(bi_active.value(), vr::Vec3i(0, 0, 12));
  CHECK(pbi >= 0);
  const std::size_t vbi = static_cast<std::size_t>(pbi) + on_axis;
  CHECK(integ.integrate(vbg_bi, depth_interp.data(), bcam, 5.0f).ok());
  const auto* bi_tsdf = static_cast<const float*>(
      vbg_bi.attribute("tsdf").value().buffer->mapped());
  const auto* bi_weight = static_cast<const float*>(
      vbg_bi.attribute("weight").value().buffer->mapped());
  CHECK(bi_weight[vbi] > 0.0f);
  CHECK(approx(bi_tsdf[vbi], 0.01f, 1e-3f));   // bilinear: 0.49 - 0.48
  CHECK(!approx(bi_tsdf[vbi], 0.02f, 5e-3f));  // NOT nearest: 0.50 - 0.48

  // (discontinuity) A 0.48 / 0.58 step across the taps (> trunc) is a depth
  // edge: the sampler falls back to the nearest sample (0.48) instead of
  // blending to 0.53, which would exceed the band and drop the voxel. So it
  // fuses at sdf ~ 0.
  std::vector<float> depth_edge(depth.size(), 0.48f);
  for (std::uint32_t y = 0; y < bcam.height; ++y) {
    depth_edge[static_cast<std::size_t>(y) * bw + 320] = 0.58f;
  }
  vr::Result<vol::VoxelBlockGrid> ed = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, attrs, 2);
  CHECK(ed.ok());
  vol::VoxelBlockGrid vbg_ed = std::move(ed).value();
  CHECK(vbg_ed.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> ed_active =
      vbg_ed.map().compact_active_blocks();
  CHECK(ed_active.ok());
  const std::int32_t ped = find_ptr(ed_active.value(), vr::Vec3i(0, 0, 12));
  CHECK(ped >= 0);
  const std::size_t ved = static_cast<std::size_t>(ped) + on_axis;
  CHECK(integ.integrate(vbg_ed, depth_edge.data(), bcam, 5.0f).ok());
  const auto* ed_tsdf = static_cast<const float*>(
      vbg_ed.attribute("tsdf").value().buffer->mapped());
  const auto* ed_weight = static_cast<const float*>(
      vbg_ed.attribute("weight").value().buffer->mapped());
  CHECK(ed_weight[ved] > 0.0f);  // fused via the nearest fallback...
  CHECK(approx(ed_tsdf[ved], 0.0f,
               1e-3f));  // ...at 0.48, not blended out of band

  std::printf(
      "recon tsdf integrate test passed: classic fusion of a 0.5 m plane (%zu "
      "voxels), %zu cross-checked under a rotated pose, dynamic cleared a "
      "stale "
      "voxel, bilinear blended a step and fell back across a depth edge\n",
      touched, cross_checked);
  return 0;
}
