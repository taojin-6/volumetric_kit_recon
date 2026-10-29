// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The sensor package's GpuFramePrep output, the frame contract, fused through
// the device-input overloads (sensor_gpu_frame_prep tests the pass itself):
//   - a lensed depth camera beside a larger colour one fuses, colour and all,
//     into a band over many blocks;
//   - a slanted plane's fused field crosses zero within a tenth of a voxel of
//     the plane.
// Runs on the real driver; exits 0 (skip) where no device is present.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <vector>

#include "grid_readback.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/device_requirements.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace camera = volumetric_kit::sensor::camera;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;

namespace {

constexpr std::uint32_t kWidth = 320;
constexpr std::uint32_t kHeight = 240;
constexpr float kScale = 0.001f;  // metres per unit

camera::CameraModel pinhole() {
  camera::CameraModel c;
  c.size = {kWidth, kHeight};
  c.intrinsics = {260.0f, 258.0f, 161.3f, 118.7f};
  return c;
}

// A strong barrel lens with a little tangential and rational distortion.
camera::CameraModel lensed() {
  camera::CameraModel c = pinhole();
  c.distortion = {-0.28f, 0.09f, 0.0012f, -0.0009f, -0.01f, 0.05f, 0.0f, 0.0f};
  return c;
}

sensor::RgbdFrame frame_of(const std::vector<std::uint16_t>& depth,
                           const camera::CameraModel& depth_cam) {
  sensor::RgbdFrame f;
  f.depth = depth.data();
  f.metres_per_unit = kScale;
  f.depth_camera = depth_cam;
  f.min_depth = 0.1f;
  f.max_depth = 10.0f;
  f.timestamp_ns = 1234;
  return f;
}

// Allocate `depth`'s band, retrying rounds that only lost bucket-lock races:
// adjacent pixels dilate into one block, and a round can hand back such
// failures over a map far from full.
int allocate(vol::VoxelBlockGrid& grid, const vkc::Buffer& depth,
             const sensor::DepthCameraParams& camera) {
  for (int round = 0; round < 5; ++round) {
    vol::AllocFailures why;
    auto failed = grid.map().allocate_from_depth(
        depth, tsdf::to_depth_pinhole(camera), &why);
    CHECK(failed.ok() && !why.capacity_limited());
    if (failed.value() == 0) return 0;
  }
  std::fprintf(stderr, "FAIL: allocation kept losing lock races\n");
  return 1;
}

// The pass's output straight into the device-input fusion overloads.
int test_fuses(vkc::Device& device, vkc::Allocator& allocator,
               sensor::GpuFramePrep& prep) {
  // A small depth camera, 80x60, beside the full colour one: the two sizes
  // differ, as a sensor's do. A tilted surface, 0.7-1.2 m, spreads the band
  // over many blocks. (A 320x240 flat wall here once ran past NVIDIA's 7 s
  // watchdog in CI, Xid 109, contending for the hash map's bucket locks while
  // they were host-visible; PR #81 moved them into device memory.)
  camera::CameraModel depth_cam = lensed();
  camera::PinholeIntrinsics& k = depth_cam.intrinsics;
  k.fx /= 4.0;
  k.fy /= 4.0;
  k.cx = (k.cx + 0.5) / 4.0 - 0.5;
  k.cy = (k.cy + 0.5) / 4.0 - 0.5;
  depth_cam.size = {kWidth / 4, kHeight / 4};
  const std::uint32_t dw = depth_cam.size.width, dh = depth_cam.size.height;
  std::vector<std::uint16_t> raw(std::size_t{dw} * dh);
  for (std::uint32_t v = 0; v < dh; ++v) {
    for (std::uint32_t u = 0; u < dw; ++u) {
      raw[std::size_t{v} * dw + u] =
          static_cast<std::uint16_t>(700 + 5 * u + 3 * v);
    }
  }
  // One R'G'B' word for the whole colour picture, from the host.
  const std::vector<std::uint32_t> words(std::size_t{kWidth} * kHeight,
                                         0xFF2080C0u);
  sensor::RgbdFrame f = frame_of(raw, depth_cam);
  f.color_packed = words.data();
  f.color_camera = pinhole();
  auto out = prep.prepare(f);
  CHECK(out.ok());

  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.01f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 2048;
  grid.num_blocks = 2048 * 8;
  grid.max_chain = 128;
  const vol::AttributeSpec attrs[] = {
      {vol::kTsdfAttribute, sizeof(float)},
      {vol::kWeightAttribute, sizeof(float)},
      {vol::kColorAttribute, sizeof(std::uint32_t)}};
  auto vbg = vol::VoxelBlockGrid::create(device, allocator, grid, attrs, 3);
  auto integrator = tsdf::TsdfIntegrator::create(device, allocator);
  CHECK(vbg.ok() && integrator.ok());
  CHECK(allocate(vbg.value(), *out->depth, out->depth_camera) == 0);
  tsdf::ColorFrame color{};
  color.buffer = out->color.get();
  color.cam = out->color_camera;
  color.encoding = out->color_encoding;
  color.coverage_in_alpha = true;
  CHECK(integrator
            ->integrate(vbg.value(), *out->depth, out->depth_camera, 5.0f,
                        tsdf::IntegrationMode::Classic, &color)
            .ok());
  auto active = vbg->map().compact_active_blocks();
  CHECK(active.ok() && active->size() > 20);
  return 0;
}

// The pass's output fuses where the surface is. A plane tilted across both
// image axes, imaged at every pixel centre of a pinhole camera with an
// off-centre principal point, prepared, allocated and fused: along every voxel
// column the camera sees, the fused field crosses zero within a tenth of a
// voxel of the plane. Sampling half a pixel off the pass's integer centres
// moves the crossing by the plane's depth change over half a pixel, 1.5 to
// 7 mm here.
int test_fuses_at_the_surface(vkc::Device& device, vkc::Allocator& allocator,
                              sensor::GpuFramePrep& prep) {
  camera::CameraModel cam = pinhole();
  camera::PinholeIntrinsics& k = cam.intrinsics;
  k.fx /= 2.0;
  k.fy /= 2.0;
  k.cx = (k.cx + 0.5) / 2.0 - 0.5;
  k.cy = (k.cy + 0.5) / 2.0 - 0.5;
  cam.size = {kWidth / 2, kHeight / 2};
  const std::uint32_t w = cam.size.width, h = cam.size.height;
  // z = z0 + a x + b y in the camera's frame, which is the world's.
  constexpr double kZ0 = 1.0, kA = 0.5, kB = 0.3;
  constexpr float kUnit = 1e-4f;  // a tenth of a millimetre a unit
  std::vector<std::uint16_t> raw(std::size_t{w} * h);
  for (std::uint32_t v = 0; v < h; ++v) {
    for (std::uint32_t u = 0; u < w; ++u) {
      const double x = (u - k.cx) / k.fx, y = (v - k.cy) / k.fy;
      const double z = kZ0 / (1.0 - kA * x - kB * y);
      raw[std::size_t{v} * w + u] =
          static_cast<std::uint16_t>(std::lround(z / kUnit));
    }
  }
  sensor::RgbdFrame f = frame_of(raw, cam);
  f.metres_per_unit = kUnit;
  auto out = prep.prepare(f);
  CHECK(out.ok());
  const sensor::DepthCameraParams& dc = out->depth_camera;

  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.01f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 2048;
  grid.num_blocks = 2048 * 8;
  grid.max_chain = 128;
  const vol::AttributeSpec attrs[] = {{vol::kTsdfAttribute, sizeof(float)},
                                      {vol::kWeightAttribute, sizeof(float)}};
  auto vbg = vol::VoxelBlockGrid::create(device, allocator, grid, attrs, 2);
  auto integrator = tsdf::TsdfIntegrator::create(device, allocator);
  CHECK(vbg.ok() && integrator.ok());
  CHECK(allocate(vbg.value(), *out->depth, dc) == 0);
  CHECK(integrator->integrate(vbg.value(), *out->depth, dc).ok());

  auto active = vbg->map().compact_active_blocks();
  auto tsdf_values = vs_test::read_attribute<float>(
      device, allocator, vbg.value(), vol::kTsdfAttribute);
  auto weights = vs_test::read_attribute<float>(device, allocator, vbg.value(),
                                                vol::kWeightAttribute);
  CHECK(active.ok() && tsdf_values.ok() && weights.ok());
  // Every fused voxel's value, by its global coordinate.
  std::map<std::array<int, 3>, float> fused;
  const int bs = grid.block_size;
  for (const vol::BlockIndex& b : active.value()) {
    for (int i = 0; i < grid.voxels_per_block; ++i) {
      const std::size_t at =
          static_cast<std::size_t>(b.ptr) + static_cast<std::size_t>(i);
      if (weights.value()[at] > 0.0f) {
        fused[{b.coord.x * bs + i % bs, b.coord.y * bs + (i / bs) % bs,
               b.coord.z * bs + i / (bs * bs)}] = tsdf_values.value()[at];
      }
    }
  }
  // A voxel that projects where all four bilinear taps are inside the image.
  const auto interior = [&](int x, int y, int z) {
    const float u =
        dc.fx * static_cast<float>(x) / static_cast<float>(z) + dc.cx;
    const float v =
        dc.fy * static_cast<float>(y) / static_cast<float>(z) + dc.cy;
    return u >= 0.0f && u < w - 1.0f && v >= 0.0f && v < h - 1.0f;
  };
  std::size_t columns = 0;
  double worst = 0.0;
  for (const auto& [voxel, front] : fused) {
    const auto behind = fused.find({voxel[0], voxel[1], voxel[2] + 1});
    if (behind == fused.end() || !(front > 0.0f && behind->second <= 0.0f) ||
        !interior(voxel[0], voxel[1], voxel[2]) ||
        !interior(voxel[0], voxel[1], voxel[2] + 1)) {
      continue;
    }
    const double s = grid.voxel_size;
    const double crossing =
        s * (voxel[2] + front / (front - static_cast<double>(behind->second)));
    const double plane = kZ0 + kA * s * voxel[0] + kB * s * voxel[1];
    worst = std::max(worst, std::fabs(crossing - plane));
    ++columns;
  }
  std::printf("  slanted plane: %zu columns, worst crossing %.3f mm\n", columns,
              worst * 1e3);
  CHECK(columns > 5000);
  CHECK(worst <= 0.1 * grid.voxel_size);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  auto prep = sensor::GpuFramePrep::create(gpu.device, gpu.allocator);
  if (!prep) std::fprintf(stderr, "%s\n", prep.status().message().c_str());
  CHECK(prep.ok());
  if (test_fuses(gpu.device, gpu.allocator, prep.value()) != 0) return 1;
  if (test_fuses_at_the_surface(gpu.device, gpu.allocator, prep.value()) != 0) {
    return 1;
  }
  std::puts("tsdf_prepared_frame: OK");
  return 0;
}

}  // namespace

int main() {
  // The device both packages' kernels run on, as a fusing program's is.
  const vkc::Result<vkc::DeviceRequirements> reqs =
      vkc::merge(vr::device_requirements(), sensor::device_requirements());
  CHECK(reqs.ok());
  return vkc::test::run_on_gpu(reqs.value(), gpu_main);
}
