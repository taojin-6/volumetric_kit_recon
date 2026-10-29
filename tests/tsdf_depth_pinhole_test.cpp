// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// tsdf::to_depth_pinhole, the one place fusion's camera becomes the voxel
// layer's: every field of a DepthCameraParams lands in the same field of
// volume::DepthPinhole, NaN and infinity included, and the two types share one
// layout, so the bytes the depth kernel reads are the bytes it read before.
// Host-only, so it always runs.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::grid::volume;
namespace tsdf = volumetric_kit::recon::tsdf;

static_assert(sizeof(vol::DepthPinhole) == sizeof(vr::DepthCameraParams));
static_assert(offsetof(vol::DepthPinhole, fx) ==
              offsetof(vr::DepthCameraParams, fx));
static_assert(offsetof(vol::DepthPinhole, fy) ==
              offsetof(vr::DepthCameraParams, fy));
static_assert(offsetof(vol::DepthPinhole, cx) ==
              offsetof(vr::DepthCameraParams, cx));
static_assert(offsetof(vol::DepthPinhole, cy) ==
              offsetof(vr::DepthCameraParams, cy));
static_assert(offsetof(vol::DepthPinhole, min_depth) ==
              offsetof(vr::DepthCameraParams, min_depth));
static_assert(offsetof(vol::DepthPinhole, max_depth) ==
              offsetof(vr::DepthCameraParams, max_depth));
static_assert(offsetof(vol::DepthPinhole, width) ==
              offsetof(vr::DepthCameraParams, width));
static_assert(offsetof(vol::DepthPinhole, height) ==
              offsetof(vr::DepthCameraParams, height));
static_assert(offsetof(vol::DepthPinhole, cam_to_world) ==
              offsetof(vr::DepthCameraParams, cam_to_world));

namespace {

// A camera whose every field, and every pose entry, holds a value no other
// does, so a field copied into its neighbour shows.
vr::DepthCameraParams distinct_camera() {
  vr::DepthCameraParams c{};
  c.fx = 101.5f;
  c.fy = 102.25f;
  c.cx = 103.125f;
  c.cy = 104.0625f;
  c.min_depth = 0.375f;
  c.max_depth = 9.5f;
  c.width = 640;
  c.height = 576;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      c.cam_to_world[col][row] = 10.0f + static_cast<float>(4 * col + row);
    }
  }
  return c;
}

bool same_fields(const vol::DepthPinhole& p, const vr::DepthCameraParams& c) {
  return p.fx == c.fx && p.fy == c.fy && p.cx == c.cx && p.cy == c.cy &&
         p.min_depth == c.min_depth && p.max_depth == c.max_depth &&
         p.width == c.width && p.height == c.height &&
         p.cam_to_world == c.cam_to_world;
}

}  // namespace

int main() {
  const vr::DepthCameraParams camera = distinct_camera();
  const vol::DepthPinhole pinhole = tsdf::to_depth_pinhole(camera);
  CHECK(same_fields(pinhole, camera));
  CHECK(std::memcmp(&pinhole, &camera, sizeof(pinhole)) == 0);

  // Copied, not checked: a camera the kernel would refuse samples of still
  // arrives as it was, bit for bit.
  vr::DepthCameraParams odd = camera;
  odd.fx = std::numeric_limits<float>::quiet_NaN();
  odd.max_depth = std::numeric_limits<float>::infinity();
  odd.min_depth = -0.0f;
  odd.width = 0;
  odd.cam_to_world[3][0] = -std::numeric_limits<float>::infinity();
  const vol::DepthPinhole copied = tsdf::to_depth_pinhole(odd);
  CHECK(std::memcmp(&copied, &odd, sizeof(copied)) == 0);
  CHECK(std::isnan(copied.fx) && std::signbit(copied.min_depth));

  std::puts("tsdf depth pinhole: every field copied, the same bytes");
  return 0;
}
