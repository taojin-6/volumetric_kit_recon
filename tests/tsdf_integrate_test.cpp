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
#include <cstdlib>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
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

// A color camera sharing a depth camera's intrinsics + pose (registered capture
// in these tests); the color params simply drop the depth-range fields.
vr::ColorCameraParams color_cam_of(const vr::DepthCameraParams& d) {
  return vr::ColorCameraParams{d.fx,    d.fy,     d.cx,          d.cy,
                               d.width, d.height, d.cam_to_world};
}

// --- Dirty-block tracking --------------------------------------------------
//
// Its own fixture, because the geometry decides whether the assertions can fail
// at all. A dirty set that is a contiguous 1-D run cannot tell a correct
// dilation from a broken one -- its -z neighbours are already in the set, so
// dropping the dilation, dropping the existence filter, inverting the octant
// and deleting the feature outright all agree.
//
// So: a 2x2x2 cube of blocks straddling the band, plus one isolated block that
// no ray reaches.
//
//   (0..1, 0..1, 13)  the band       -- written        -> dirty   = 4
//   (0..1, 0..1, 12)  ahead of it    -- never written  -> remesh  = 8
//   (5, 5, 5)         off-camera     -- never written  -> active  = 9
//
// Every quantity differs from every other, which is what gives the equalities
// teeth: dropping the dilation gives 4, dropping the existence filter gives 18,
// inverting the octant to +x/+y/+z gives 4, and returning the active count
// gives 9.
//
// Dynamic mode throughout, deliberately. In classic mode the free-space cone
// ahead of the surface is fused too (that is the point of classic), so the
// first fuse legitimately marks the bz=12 blocks as well and the fixture loses
// the separation it is built on.
int dirty_blocks_case(vr::Device& device, vr::Allocator& allocator) {
  vol::VoxelGridParams gp{};
  gp.voxel_size = 0.005f;
  gp.block_size = 8;
  gp.voxels_per_block = 512;
  gp.trunc_dist = 0.01f;
  gp.bucket_size = 8;
  gp.num_buckets = 256;
  gp.num_blocks = 256 * 8;
  gp.max_chain = 128;
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};

  vr::Result<vol::VoxelBlockGrid> grid_a =
      vol::VoxelBlockGrid::create(device, allocator, gp, attrs, 2);
  CHECK(grid_a.ok());
  vol::VoxelBlockGrid a = std::move(grid_a).value();

  std::vector<vol::BlockIndex> blocks;
  for (int bz = 12; bz <= 13; ++bz) {
    for (int by = 0; by <= 1; ++by) {
      for (int bx = 0; bx <= 1; ++bx) {
        vol::BlockIndex b{};
        b.coord = vr::Vec3i(bx, by, bz);
        blocks.push_back(b);
      }
    }
  }
  vol::BlockIndex far{};
  far.coord = vr::Vec3i(5, 5, 5);  // projects outside the image
  blocks.push_back(far);
  CHECK(a.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> active_a =
      a.map().compact_active_blocks();
  CHECK(active_a.ok() && active_a.value().size() == 9);

  vr::DepthCameraParams cam{};
  cam.fx = 525.0f;
  cam.fy = 525.0f;
  cam.cx = 320.0f;
  cam.cy = 240.0f;
  cam.min_depth = 0.1f;
  cam.max_depth = 10.0f;
  cam.width = 640;
  cam.height = 480;
  cam.cam_to_world = vr::Mat4f(1.0f);
  const std::size_t px = static_cast<std::size_t>(cam.width) * cam.height;
  const std::vector<float> depth_band(px, 0.54f);  // band lands in bz = 13
  const std::vector<float> depth_near(px, 0.50f);  // band lands in bz = 12
  const std::vector<float> depth_far(px, 0.60f);   // both recede -> clear
  const std::vector<float> depth_none(px, 0.0f);   // all invalid -> no writes

  // --- Tracking is opt-in, and off costs nothing ---------------------------
  //
  // The default integrator must not allocate a flag array or store a flag, and
  // must say so rather than answer 0 as though nothing were dirty.
  {
    vr::Result<tsdf::TsdfIntegrator> untracked =
        tsdf::TsdfIntegrator::create(device, allocator);
    CHECK(untracked.ok());
    CHECK(untracked.value()
              .integrate(a, depth_band.data(), cam, 5.0f,
                         tsdf::IntegrationMode::Dynamic)
              .ok());
    CHECK(untracked.value().dirty_block_count() == 0);
    CHECK(!untracked.value()
               .dirty_remesh_blocks(a, active_a.value().data(),
                                    active_a.value().size())
               .ok());
  }
  // That fuse wrote real values into `a`, so start the tracked cases from a
  // clean grid rather than from whatever it left behind.
  CHECK(a.clear().ok());
  CHECK(a.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  active_a = a.map().compact_active_blocks();
  CHECK(active_a.ok() && active_a.value().size() == 9);

  tsdf::TsdfIntegratorConfig cfg{};
  cfg.track_dirty_blocks = true;
  vr::Result<tsdf::TsdfIntegrator> integ_result =
      tsdf::TsdfIntegrator::create(device, allocator, cfg);
  CHECK(integ_result.ok());
  tsdf::TsdfIntegrator integ = std::move(integ_result).value();

  CHECK(integ.dirty_block_count() == 0);

  // --- What was written, and what that means for a re-mesh -----------------
  CHECK(integ
            .integrate(a, depth_band.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(integ.dirty_block_count() == 4);

  vr::Result<std::vector<vr::Vec3i>> remesh = integ.dirty_remesh_blocks(
      a, active_a.value().data(), active_a.value().size());
  CHECK(remesh.ok());
  CHECK(remesh.value().size() == 8);
  // And it is the cube, not any 8 of the 9: the isolated block is the one that
  // must not appear.
  for (const vr::Vec3i& c : remesh.value()) {
    CHECK(!(c.x == 5 && c.y == 5 && c.z == 5));
  }

  // --- Flags ACCUMULATE across fuses ---------------------------------------
  //
  // The header promises this (a consumer fuses several frames per remesh), and
  // nothing pinned it: an integrator that zeroed the array at the top of every
  // integrate -- per-frame instead of cumulative, the one semantic a consumer
  // must not get wrong -- passed every other assertion here.
  CHECK(integ
            .integrate(a, depth_near.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(integ.dirty_block_count() == 8);

  // --- A map grow carries the flags forward --------------------------------
  //
  // VoxelHashMap::resize preserves each block's index, so a slot means the same
  // block on both sides of the grow and the flags stay true. Reallocating and
  // zeroing instead loses every block fused since the last reset -- silently,
  // and on exactly the frames a growing scan brings in the most new surface.
  // The depth here is all-invalid, so the fuse itself writes nothing and the
  // count that survives is entirely the carried-forward one.
  CHECK(a.resize(gp.num_buckets * 2).ok());
  CHECK(integ
            .integrate(a, depth_none.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(integ.dirty_block_count() == 8);

  // --- The dynamic clear marks too -----------------------------------------
  //
  // A receded surface that leaves a stale mesh behind is the whole reason
  // dynamic mode exists, so the clear has to report itself. Deleting that one
  // mark left the entire suite green before this case existed. The band at
  // 0.60 m falls in blocks nobody allocated, so every flag below comes from the
  // clear.
  integ.reset_dirty();
  CHECK(integ.dirty_block_count() == 0);
  CHECK(integ
            .integrate(a, depth_far.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(integ.dirty_block_count() == 8);

  // --- A flag means the field CHANGED, not that a store happened -----------
  //
  // max_weight 0 pins the fused weight at 0 forever, which makes the stored
  // tsdf a pure function of THIS frame: re-fusing an identical frame recomputes
  // bit-identical numbers. That is the exact case a scan revisiting converged
  // surface lives in, and a flag set by the act of storing reports every block
  // again, every frame, for as long as the camera can see it.
  integ.reset_dirty();
  CHECK(integ
            .integrate(a, depth_band.data(), cam, 0.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(integ.dirty_block_count() == 4);
  integ.reset_dirty();
  CHECK(integ
            .integrate(a, depth_band.data(), cam, 0.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(integ.dirty_block_count() == 0);

  // --- One integrator, a second grid ---------------------------------------
  //
  // A flag is keyed by block SLOT, and a slot means nothing across grids -- the
  // heap hands the same index to a block at a different coordinate. Driving one
  // integrator over several grids is already how this suite is written, so this
  // is not hypothetical; what it used to do was OR the two grids' flags
  // together.
  integ.reset_dirty();
  CHECK(integ
            .integrate(a, depth_band.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  CHECK(integ.dirty_block_count() == 4);

  vr::Result<vol::VoxelBlockGrid> grid_b =
      vol::VoxelBlockGrid::create(device, allocator, gp, attrs, 2);
  CHECK(grid_b.ok());
  vol::VoxelBlockGrid b = std::move(grid_b).value();
  vol::BlockIndex one{};
  one.coord = vr::Vec3i(0, 0, 13);
  CHECK(b.map().allocate(&one, 1).value() == 0);
  CHECK(integ
            .integrate(b, depth_band.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());
  // Re-anchored on `b`: its single block, not `a`'s four OR-ed underneath.
  CHECK(integ.dirty_block_count() == 1);
  // And `a`'s flags are gone, so asking about `a` is refused rather than
  // answered out of `b`'s array.
  CHECK(!integ
             .dirty_remesh_blocks(a, active_a.value().data(),
                                  active_a.value().size())
             .ok());

  // --- Removing blocks invalidates the set ---------------------------------
  //
  // remove() changes the field from the host, and no flag can say so: the freed
  // slot goes back to a LIFO heap and is re-drawn for a different block. The
  // dirty set cannot describe geometry that went away, so the answer is a
  // refusal (re-mesh everything) rather than a confidently wrong subset.
  vr::Result<std::vector<vol::BlockIndex>> active_b =
      b.map().compact_active_blocks();
  CHECK(active_b.ok());
  CHECK(integ
            .dirty_remesh_blocks(b, active_b.value().data(),
                                 active_b.value().size())
            .ok());
  CHECK(b.remove(&one, 1).ok());
  CHECK(b.topology_epoch() == 1);
  vr::Result<std::vector<vol::BlockIndex>> after_remove =
      b.map().compact_active_blocks();
  CHECK(after_remove.ok());
  CHECK(!integ
             .dirty_remesh_blocks(b, after_remove.value().data(),
                                  after_remove.value().size())
             .ok());
  // reset_dirty() re-arms it: nothing is accumulated, so nothing is stale.
  integ.reset_dirty();
  vr::Result<std::vector<vr::Vec3i>> rearmed = integ.dirty_remesh_blocks(
      b, after_remove.value().data(), after_remove.value().size());
  CHECK(rearmed.ok() && rearmed.value().empty());

  return 0;
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
  vr::DepthCameraParams cam{};
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

  // Dirty-block tracking has its own fixture (see dirty_blocks_case): the
  // geometry a plane fixture produces is a contiguous run, which cannot tell a
  // correct dilation from a broken one. This integrator carries no tracking, so
  // the per-voxel numerics below are asserted against the shape every other
  // consumer in the repo runs.
  CHECK(dirty_blocks_case(device.value(), allocator.value()) == 0);

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
  vr::DepthCameraParams cam2 = cam;  // same intrinsics, range, and 0.5 m plane
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
  vr::DepthCameraParams bcam = cam;
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
  // edge, so the sampler falls back to the nearest sample (0.48) rather than
  // blending to 0.53. This runs in classic mode, where a regression that DID
  // blend across the edge would not drop the voxel: sdf = 0.53 - 0.48 = 0.05 >
  // trunc is clamped to +trunc and fused at tsdf ~ +0.04. So the tsdf ~ 0 check
  // below is what separates the fallback (0.48 -> sdf 0) from a cross-edge
  // blend (tsdf ~ +0.04); the weight check only confirms the voxel fused at
  // all.
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
  CHECK(ed_weight[ved] > 0.0f);  // fused (fallback kept it in-band)
  CHECK(approx(ed_tsdf[ved], 0.0f,
               1e-3f));  // 0.48 - 0.48; not a cross-edge blend

  // Color fusion through the separate color camera (RGB packed in a uint's low
  // bytes, the mesh tier's layout). A grid with a `color` attribute + a
  // constant color frame: a first-observation voxel takes the sampled RGB; an
  // occluded voxel (never depth-fused) keeps zero.
  const vol::AttributeSpec cattrs[] = {{"tsdf", sizeof(float)},
                                       {"weight", sizeof(float)},
                                       {"color", sizeof(std::uint32_t)}};
  vr::Result<vol::VoxelBlockGrid> cg = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, cattrs, 3);
  CHECK(cg.ok());
  vol::VoxelBlockGrid vbg_c = std::move(cg).value();
  CHECK(vbg_c.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> c_active =
      vbg_c.map().compact_active_blocks();
  CHECK(c_active.ok());
  const std::int32_t cp12 = find_ptr(c_active.value(), vr::Vec3i(0, 0, 12));
  const std::int32_t cp13 = find_ptr(c_active.value(), vr::Vec3i(0, 0, 13));
  CHECK(cp12 >= 0 && cp13 >= 0);
  const std::size_t c_front =
      static_cast<std::size_t>(cp12) + local_index(0, 0, 1, bs);  // fused
  const std::size_t c_occ =
      static_cast<std::size_t>(cp13) + local_index(0, 0, 6, bs);  // occluded

  const std::uint32_t rgb =
      200u | (100u << 8) | (50u << 16);  // R=200 G=100 B=50
  std::vector<std::uint32_t> color_img(depth.size(), rgb);
  const tsdf::ColorFrame frame{color_img.data(),
                               color_cam_of(cam)};  // registered (= depth)
  CHECK(integ
            .integrate(vbg_c, depth.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Classic, &frame)
            .ok());
  const auto* c_color = static_cast<const std::uint32_t*>(
      vbg_c.attribute("color").value().buffer->mapped());
  const auto* c_weight = static_cast<const float*>(
      vbg_c.attribute("weight").value().buffer->mapped());
  CHECK(c_weight[c_front] > 0.0f);
  CHECK((c_color[c_front] & 0xFFu) == 200u);         // R
  CHECK(((c_color[c_front] >> 8) & 0xFFu) == 100u);  // G
  CHECK(((c_color[c_front] >> 16) & 0xFFu) == 50u);  // B
  CHECK(c_color[c_occ] == 0u);  // occluded: never fused, color untouched

  // A dynamic frame that recedes the surface clears the fused color along with
  // the geometry (the shader zeroes color in the same stale-clear branch):
  // re-fuse the front voxel as free space well past the band (plane at 0.6 m,
  // voxel at 0.485). Its weight, tsdf, and color all reset to the pristine
  // zero.
  CHECK(integ
            .integrate(vbg_c, depth_far.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic, &frame)
            .ok());
  CHECK(c_weight[c_front] == 0.0f && c_color[c_front] == 0u);  // color cleared

  // Separate color camera: shift it 10 m off in +X so the same voxel projects
  // out of the color frame. Depth still fuses it; color is skipped (stays
  // zero).
  vr::DepthCameraParams off_cam = cam;
  off_cam.cam_to_world = vr::Mat4f(1.0f);
  off_cam.cam_to_world[3] = vr::Vec4f(10.0f, 0.0f, 0.0f, 1.0f);
  vr::Result<vol::VoxelBlockGrid> og = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, cattrs, 3);
  CHECK(og.ok());
  vol::VoxelBlockGrid vbg_o = std::move(og).value();
  CHECK(vbg_o.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> o_active =
      vbg_o.map().compact_active_blocks();
  CHECK(o_active.ok());
  const std::int32_t op12 = find_ptr(o_active.value(), vr::Vec3i(0, 0, 12));
  CHECK(op12 >= 0);
  const std::size_t o_front =
      static_cast<std::size_t>(op12) + local_index(0, 0, 1, bs);
  const tsdf::ColorFrame off_frame{color_img.data(), color_cam_of(off_cam)};
  CHECK(integ
            .integrate(vbg_o, depth.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Classic, &off_frame)
            .ok());
  const auto* o_color = static_cast<const std::uint32_t*>(
      vbg_o.attribute("color").value().buffer->mapped());
  const auto* o_weight = static_cast<const float*>(
      vbg_o.attribute("weight").value().buffer->mapped());
  CHECK(o_weight[o_front] > 0.0f);  // depth still fuses the voxel
  CHECK(o_color[o_front] == 0u);    // but color is out of frame -> skipped

  // Regression: a voxel's first color observation must ASSIGN (keyed on
  // color_attr == 0), not blend against the black initial attribute, even when
  // depth weight has already accumulated. Warm up a voxel with two depth-only
  // frames, then fuse a color frame: it must take the full sampled RGB. (Keying
  // the assign on the depth weight instead darkened it to ~half here.)
  vr::Result<vol::VoxelBlockGrid> wg = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, cattrs, 3);
  CHECK(wg.ok());
  vol::VoxelBlockGrid vbg_wu = std::move(wg).value();
  CHECK(vbg_wu.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> wu_active =
      vbg_wu.map().compact_active_blocks();
  CHECK(wu_active.ok());
  const std::int32_t wup12 = find_ptr(wu_active.value(), vr::Vec3i(0, 0, 12));
  CHECK(wup12 >= 0);
  const std::size_t wu_front =
      static_cast<std::size_t>(wup12) + local_index(0, 0, 1, bs);
  CHECK(integ.integrate(vbg_wu, depth.data(), cam, 5.0f).ok());  // depth only
  CHECK(integ.integrate(vbg_wu, depth.data(), cam, 5.0f).ok());  // depth only
  const auto* wu_color = static_cast<const std::uint32_t*>(
      vbg_wu.attribute("color").value().buffer->mapped());
  const auto* wu_weight = static_cast<const float*>(
      vbg_wu.attribute("weight").value().buffer->mapped());
  CHECK(wu_weight[wu_front] >
        1.0f);                      // depth weight accumulated across 2 frames
  CHECK(wu_color[wu_front] == 0u);  // ...but no color observed yet
  CHECK(integ
            .integrate(vbg_wu, depth.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Classic, &frame)
            .ok());
  CHECK((wu_color[wu_front] & 0xFFu) == 200u);  // first color assigns the
  CHECK(((wu_color[wu_front] >> 8) & 0xFFu) == 100u);  // full RGB, not a blend
  CHECK(((wu_color[wu_front] >> 16) & 0xFFu) == 50u);  // toward black

  // The colour running mean blends in LINEAR working values, not in the encoded
  // codes (the 2026-08-02 decision). Fuse a second, very different colour into
  // the voxel just assigned above and check the result against the mean
  // computed the right way -- with the wrong way as an explicit foil, because
  // the two differ by ~10 codes here and only a comparison against both proves
  // which one ran. (The gap is widest across high-contrast pairs, which is
  // exactly where a reconstruction gets inspected: silhouettes and depth
  // discontinuities.)
  //
  // The check is weight-independent by construction, which matters because the
  // kernel's w_obs (1/z^2 at this voxel) is not something the host recomputes:
  // `linear_to_srgb` is *concave* on [0, 1], so by Jensen's inequality
  // encode(mean of the linear values) >= mean of the encoded codes, for ANY
  // convex weighting. Blending in linear therefore always lands on a strictly
  // brighter code than blending the codes did -- the same inequality that makes
  // 0.0 and 1.0 fuse to 0.214 instead of 0.5. Asserting the direction (plus a
  // magnitude floor, so it cannot pass vacuously) pins the fix exactly:
  // reverting the kernel to blend encoded values makes `got == wrong` and this
  // fails, while no choice of weights can rescue it.
  {
    const std::uint32_t prev_packed = wu_color[wu_front];
    const float w_before = wu_weight[wu_front];
    constexpr std::uint32_t kDark = 0xFF141414u;  // code 20, far below prev
    std::vector<std::uint32_t> dark_pixels(color_img.size(), kDark);
    tsdf::ColorFrame dark_frame{};
    dark_frame.pixels = dark_pixels.data();
    dark_frame.cam = frame.cam;
    CHECK(integ
              .integrate(vbg_wu, depth.data(), cam, 5.0f,
                         tsdf::IntegrationMode::Classic, &dark_frame)
              .ok());
    const std::uint32_t got = wu_color[wu_front];

    // Both weights are known here rather than recovered from the answer (which
    // would make the comparison circular and unfailable): `w_before` is the
    // stored SDF weight the kernel reads as w_old, and the observation weight
    // is 1/Zc^2 at this voxel -- world z = 97 * 0.005 = 0.485 m under the
    // identity pose, in front of the surface so no behind-dropoff applies. It
    // is the same weight the SDF assertions at the top of this test use.
    const float w_obs = 1.0f / (0.485f * 0.485f);
    const vr::Vec3f prev_lin = vr::unpack_srgb_to_linear(prev_packed);
    const vr::Vec3f obs_lin = vr::unpack_srgb_to_linear(kDark);
    const vr::Vec3f want_lin =
        (prev_lin * w_before + obs_lin * w_obs) / (w_before + w_obs);
    const std::uint32_t want = vr::pack_linear_to_srgb(want_lin);

    // What the kernel produces: the linear mean, to within float32 rounding.
    // This is the assertion that fails if the blend reverts to encoded values.
    for (int i = 0; i < 3; ++i) {
      const auto got_c = static_cast<int>((got >> (8 * i)) & 0xFFu);
      const auto want_c = static_cast<int>((want >> (8 * i)) & 0xFFu);
      CHECK(std::abs(got_c - want_c) <= 1);
    }

    // And the foil, stated on RED because that is where the two disagree.
    // Jensen's inequality puts the code blend strictly darker (encode is
    // concave), but by how much depends entirely on the contrast: red spans
    // 200 -> 20 and the two answers sit ~35 codes apart, while blue spans only
    // 50 -> 20 and they land 3 apart. That is not a quirk of the fixture -- it
    // is the documented shape of this bug, small between similar samples and
    // largest across high-contrast pairs, which is why it concentrates on
    // silhouettes and depth discontinuities and why it survived unnoticed.
    const auto got_r = static_cast<int>(got & 0xFFu);
    const auto wrong_r =
        static_cast<int>((static_cast<float>(prev_packed & 0xFFu) * w_before +
                          static_cast<float>(kDark & 0xFFu) * w_obs) /
                             (w_before + w_obs) +
                         0.5f);
    CHECK(got_r > wrong_r + 20);
    CHECK((got >> 24) == 0xFFu);  // alpha still forced, sentinel still exact
  }

  // A colour frame that is not in the canonical encoded form is REFUSED rather
  // than fused through the wrong curve: the kernel decodes with exactly one
  // transfer function, and "convert once at the sensor boundary" is a contract
  // (sensor::to_canonical does it). A silently-misinterpreted curve is the
  // quiet error the whole decision exists to prevent, so it must be an error.
  {
    tsdf::ColorFrame p3_frame = frame;
    p3_frame.encoding = {vr::ColorEncoding::Transfer::Srgb,
                         vr::ColorEncoding::Primaries::DisplayP3};
    CHECK(!integ
               .integrate(vbg_wu, depth.data(), cam, 5.0f,
                          tsdf::IntegrationMode::Classic, &p3_frame)
               .ok());
    // The default declaration is canonical, so every existing caller is
    // unaffected -- which is what makes this an added guard, not a migration.
    CHECK(vr::is_canonical(tsdf::ColorFrame{}.encoding));
  }

  // Regression: dynamic mode clears stale color whenever the grid carries a
  // `color` attribute -- even on a depth-only (color == nullptr) frame -- so a
  // receded surface leaves no color ghost. (Gating the clear on a color frame
  // being supplied stranded the old color on a depth-only recede.)
  vr::Result<vol::VoxelBlockGrid> zg = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), grid, cattrs, 3);
  CHECK(zg.ok());
  vol::VoxelBlockGrid vbg_dz = std::move(zg).value();
  CHECK(vbg_dz.map()
            .allocate(blocks.data(), static_cast<std::uint32_t>(blocks.size()))
            .value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> dz_active =
      vbg_dz.map().compact_active_blocks();
  CHECK(dz_active.ok());
  const std::int32_t dzp12 = find_ptr(dz_active.value(), vr::Vec3i(0, 0, 12));
  CHECK(dzp12 >= 0);
  const std::size_t dz_front =
      static_cast<std::size_t>(dzp12) + local_index(0, 0, 1, bs);
  CHECK(integ
            .integrate(vbg_dz, depth.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Classic, &frame)
            .ok());  // fuse depth + color
  const auto* dz_color = static_cast<const std::uint32_t*>(
      vbg_dz.attribute("color").value().buffer->mapped());
  const auto* dz_weight = static_cast<const float*>(
      vbg_dz.attribute("weight").value().buffer->mapped());
  CHECK(dz_weight[dz_front] > 0.0f && dz_color[dz_front] != 0u);  // fused
  CHECK(integ
            .integrate(vbg_dz, depth_far.data(), cam, 5.0f,
                       tsdf::IntegrationMode::Dynamic)
            .ok());  // depth-only recede
  CHECK(dz_weight[dz_front] == 0.0f &&
        dz_color[dz_front] == 0u);  // geometry + color ghost both cleared

  std::printf(
      "recon tsdf integrate test passed: classic fusion of a 0.5 m plane (%zu "
      "voxels), %zu cross-checked under a rotated pose, dynamic cleared a "
      "stale "
      "voxel, bilinear blended a step + fell back, color fused via a separate "
      "camera, first-obs assigned after a depth-only warmup, dynamic cleared "
      "color depth-only\n",
      touched, cross_checked);
  return 0;
}
