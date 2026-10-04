// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// AdaptiveGrid on a still camera facing a flat wall with a 4 cm sphere in
// front of it. The wall's coarsest blocks fit the depth exactly, so they stay
// coarse; the sphere's do not, so they refine, and the finer level owns
// blocks only near it. With the sphere gone (Dynamic), the refined blocks
// coarsen and the finer level's blocks are removed. Invalid configurations
// and frames are refused, and the grid moves as an RAII owner should. Run
// under the Khronos layer where installed; its errors fail the test.
// Exits 0 (skip) where no device is present.

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string_view>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/adaptive_grid.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

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

std::atomic<int> g_errors{0};  // validation errors; any thread may report

constexpr std::uint32_t kWidth = 320;
constexpr std::uint32_t kHeight = 240;
constexpr float kWall = 1.0f;                  // the wall's depth
const vr::Vec3f kSphere(0.08f, 0.08f, 0.88f);  // inside one coarse block
constexpr float kRadius = 0.04f;

vr::DepthCameraParams camera() {
  vr::DepthCameraParams cam{};
  cam.fx = 400.0f;
  cam.fy = 400.0f;
  cam.cx = 159.5f;
  cam.cy = 119.5f;
  cam.min_depth = 0.1f;
  cam.max_depth = 5.0f;
  cam.width = kWidth;
  cam.height = kHeight;
  cam.cam_to_world = vr::Mat4f(1.0f);  // at the origin, looking down +z
  return cam;
}

// The depth the camera sees: the wall, and the sphere when present.
std::vector<float> depth_image(bool sphere) {
  const vr::DepthCameraParams cam = camera();
  std::vector<float> depth(kWidth * kHeight, kWall);
  if (!sphere) return depth;
  for (std::uint32_t v = 0; v < kHeight; ++v) {
    for (std::uint32_t u = 0; u < kWidth; ++u) {
      const vr::Vec3f ray =
          glm::normalize(vr::Vec3f((float(u) + 0.5f - cam.cx) / cam.fx,
                                   (float(v) + 0.5f - cam.cy) / cam.fy, 1.0f));
      const float b = glm::dot(ray, kSphere);
      const float disc = b * b - glm::dot(kSphere, kSphere) + kRadius * kRadius;
      if (disc <= 0.0f) continue;
      depth[v * kWidth + u] = (b - std::sqrt(disc)) * ray.z;
    }
  }
  return depth;
}

tsdf::AdaptiveGridConfig config() {
  tsdf::AdaptiveGridConfig c;
  c.num_buckets = 256;
  c.mode = tsdf::IntegrationMode::Dynamic;
  c.refine_offset = 0.0005f;
  c.pixel_stride = 2;
  c.check_every = 2;
  c.calm_checks = 2;
  return c;
}

vkc::Status fuse(tsdf::AdaptiveGrid& grid, const std::vector<float>& depth,
                 int sets) {
  const std::vector<tsdf::FrameInput> frames{
      {{vkc::StorageInput(depth.data()), camera()}, nullptr}};
  for (int i = 0; i < sets; ++i) VKC_TRY(grid.fuse(frames));
  return {};
}

// The centre of a block of `level`, metres.
vr::Vec3f centre(const vol::BlockIndex& b, float voxel) {
  return (vr::Vec3f(b.coord) + 0.5f) * 8.0f * voxel;
}

int test_refines_near_detail(vkc::Device& dev, vkc::Allocator& alloc) {
  auto made = tsdf::AdaptiveGrid::create(dev, alloc, config());
  CHECK(made.ok());
  tsdf::AdaptiveGrid grid = std::move(made).value();
  CHECK(grid.level_count() == 3);
  CHECK(fuse(grid, depth_image(true), 20).ok());

  const tsdf::AdaptiveLevelStats coarse = grid.stats(0);
  std::printf("  refined %u of %u coarse blocks, floor %.4f mm, median %.4f\n",
              coarse.refined, coarse.blocks, 1000.0 * grid.sensor_floor(),
              1000.0 * coarse.median_offset);
  CHECK(coarse.refined > 0);
  CHECK(coarse.refined < coarse.blocks / 2);
  CHECK(grid.sensor_floor() < 0.0002f);

  // The finer level owns blocks, all of them near the sphere.
  auto fine = grid.owned_blocks(1);
  CHECK(fine.ok());
  CHECK(!fine.value().empty());
  for (const vol::BlockIndex& b : fine.value()) {
    CHECK(glm::length(centre(b, 0.01f) - kSphere) < 0.25f);
  }
  // The coarsest level still owns the wall far from it.
  auto wall = grid.owned_blocks(0);
  CHECK(wall.ok());
  bool far_wall = false;
  for (const vol::BlockIndex& b : wall.value()) {
    far_wall = far_wall || glm::length(centre(b, 0.02f) - kSphere) > 0.3f;
  }
  CHECK(far_wall);
  CHECK(wall.value().size() < coarse.blocks);
  CHECK(!grid.owned_blocks(3).ok());

  // The sphere leaves: its blocks coarsen and the finer blocks go.
  CHECK(fuse(grid, depth_image(false), 40).ok());
  std::printf("  after removal: refined %u, finer blocks %u / %u\n",
              grid.stats(0).refined, grid.stats(1).blocks,
              grid.stats(2).blocks);
  CHECK(grid.stats(0).refined == 0);
  auto fine_left = grid.level(1).map().compact_active_blocks();
  CHECK(fine_left.ok() && fine_left.value().empty());
  return 0;
}

int test_refusals(vkc::Device& dev, vkc::Allocator& alloc) {
  const auto refused = [&](tsdf::AdaptiveGridConfig c) {
    auto g = tsdf::AdaptiveGrid::create(dev, alloc, c);
    return !g.ok() && g.status().domain() == vkc::Status::Code::InvalidArgument;
  };
  tsdf::AdaptiveGridConfig c = config();
  c.levels = 1;
  CHECK(refused(c));
  c = config();
  c.levels = 5;
  CHECK(refused(c));
  c = config();
  c.voxel_size = 0.0f;
  CHECK(refused(c));
  c = config();
  c.refine_offset = -1.0f;
  CHECK(refused(c));
  c = config();
  c.refine_offset = NAN;
  CHECK(refused(c));
  c = config();
  c.check_every = 0;
  CHECK(refused(c));

  auto made = tsdf::AdaptiveGrid::create(dev, alloc, config());
  CHECK(made.ok());
  tsdf::AdaptiveGrid grid = std::move(made).value();
  CHECK(grid.fuse({}).ok());  // an empty set fuses nothing
  vr::DepthCameraParams empty = camera();
  empty.width = 0;
  const std::vector<float> depth = depth_image(false);
  CHECK(grid.fuse({{{vkc::StorageInput(depth.data()), empty}, nullptr}})
            .domain() == vkc::Status::Code::InvalidArgument);
  CHECK(grid.fuse({{{vkc::StorageInput(static_cast<const void*>(nullptr)),
                     camera()},
                    nullptr}})
            .domain() == vkc::Status::Code::InvalidArgument);
  auto blocks = grid.level(0).map().compact_active_blocks();
  CHECK(blocks.ok() && blocks.value().empty());  // nothing was fused
  return 0;
}

int test_moves(vkc::Device& dev, vkc::Allocator& alloc) {
  auto a = tsdf::AdaptiveGrid::create(dev, alloc, config());
  auto b = tsdf::AdaptiveGrid::create(dev, alloc, config());
  CHECK(a.ok() && b.ok());
  tsdf::AdaptiveGrid first = std::move(a).value();
  CHECK(fuse(first, depth_image(true), 1).ok());

  tsdf::AdaptiveGrid moved(std::move(first));
  CHECK(moved.valid() && moved.level_count() == 3);
  CHECK(!first.valid());  // NOLINT(bugprone-use-after-move)
  CHECK(first.level_count() == 0 && first.sensor_floor() == 0.0f);
  CHECK(first.stats(0).blocks == 0);
  CHECK(first.fuse({}).domain() == vkc::Status::Code::InvalidArgument);
  CHECK(!first.owned_blocks(0).ok());

  tsdf::AdaptiveGrid other = std::move(b).value();
  other = std::move(moved);                // over a live grid
  CHECK(other.valid() && !moved.valid());  // NOLINT(bugprone-use-after-move)
  CHECK(fuse(other, depth_image(true), 1).ok());

  tsdf::AdaptiveGrid* self = &other;
  other = std::move(*self);
  CHECK(other.valid() && other.level_count() == 3);
  return 0;
}

}  // namespace

int main() {
  // Installed before the instance, so the layer's output reaches the counter.
  vkc::set_log_handler(
      [](vkc::LogLevel level, std::string_view, std::string_view message) {
        if (level == vkc::LogLevel::Error) {
          ++g_errors;
          std::fprintf(stderr, "[vulkan error] %.*s\n",
                       static_cast<int>(message.size()), message.data());
        }
      });
  vkc::InstanceConfig instance_config;
  instance_config.enable_validation = true;  // a no-op without the layer
  vkc::Result<vkc::Instance> instance = vkc::Instance::create(instance_config);
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance (%s); skipping\n",
                 instance.status().message().c_str());
    return 0;
  }
  vkc::Result<vkc::PhysicalDeviceInfo> gpu =
      instance.value().select_physical_device(vr::device_requirements());
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device (%s); skipping\n",
                 gpu.status().message().c_str());
    return 0;
  }
  vkc::Result<vkc::Device> device = vkc::Device::create(
      instance.value(), gpu.value(), vr::device_requirements());
  CHECK(device.ok());
  vkc::Result<vkc::Allocator> allocator =
      vkc::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());

  // The core's default requirements leave scalarBlockLayout off, which the
  // kernels' buffer ABI needs, so create refuses such a device.
  {
    vkc::Result<vkc::Device> plain =
        vkc::Device::create(instance.value(), gpu.value(), {});
    CHECK(plain.ok());
    vkc::Result<vkc::Allocator> plain_allocator =
        vkc::Allocator::create(instance.value().handle(), plain.value());
    CHECK(plain_allocator.ok());
    CHECK(tsdf::AdaptiveGrid::create(plain.value(), plain_allocator.value(), {})
              .status()
              .domain() == vkc::Status::Code::Unsupported);
  }

  std::printf("refines near detail, coarsens when it leaves\n");
  if (test_refines_near_detail(device.value(), allocator.value()) != 0)
    return 1;
  std::printf("refusals\n");
  if (test_refusals(device.value(), allocator.value()) != 0) return 1;
  std::printf("moves\n");
  if (test_moves(device.value(), allocator.value()) != 0) return 1;
  CHECK(g_errors == 0);
  std::printf("PASS\n");
  return 0;
}
