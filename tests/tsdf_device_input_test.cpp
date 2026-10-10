// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The device-input overloads of VoxelHashMap::allocate_from_depth and
// TsdfIntegrator::integrate against the host ones: one depth and colour frame
// fused into two grids, once from host arrays and once from storage buffers,
// must give the same blocks and the same tsdf, weight and colour in each. A
// colour image that marks its coverage in the high byte fuses nothing where
// it says it has no colour. And the buffers they refuse, an empty grid's
// included. Runs on the real driver; exits 0 (skip) where no device is
// present.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <tuple>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
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

constexpr std::uint32_t kWidth = 160;
constexpr std::uint32_t kHeight = 120;

using Coord = std::tuple<int, int, int>;

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

// Every active block's coordinate and first voxel (`ptr` is a voxel offset).
vkc::Result<std::map<Coord, std::int32_t>> blocks_of(vol::VoxelBlockGrid& g) {
  VKC_ASSIGN(std::vector<vol::BlockIndex> active,
             g.map().compact_active_blocks());
  std::map<Coord, std::int32_t> out;
  for (const vol::BlockIndex& b : active) {
    out[Coord{b.coord.x, b.coord.y, b.coord.z}] = b.ptr;
  }
  return out;
}

// The device the grids live on, for the helpers that read them back.
// The two grids' blocks name the same coordinates and hold the same weight in
// every voxel, and the same tsdf and colour in every observed one (the rest
// were never written). A slot can differ: allocation order is the GPU's.
int check_same(const vr_test::Gpu& ctx, vol::VoxelBlockGrid& a,
               vol::VoxelBlockGrid& b) {
  auto ba = blocks_of(a);
  auto bb = blocks_of(b);
  CHECK(ba.ok() && bb.ok());
  CHECK(!ba.value().empty());
  CHECK(ba.value().size() == bb.value().size());
  const auto view = [&](vol::VoxelBlockGrid& g, const char* name) {
    return vr_test::read_attribute<std::uint32_t>(ctx.device, ctx.allocator, g,
                                                  name)
        .value();
  };
  const std::vector<std::uint32_t> wa = view(a, "weight");
  const std::vector<std::uint32_t> wb = view(b, "weight");
  const std::vector<std::uint32_t> ta = view(a, "tsdf");
  const std::vector<std::uint32_t> tb = view(b, "tsdf");
  const std::vector<std::uint32_t> ca = view(a, "color");
  const std::vector<std::uint32_t> cb = view(b, "color");
  const std::size_t voxels = grid_params().voxels_per_block;
  std::size_t observed = 0;
  for (const auto& [coord, ptr] : ba.value()) {
    const auto other = bb.value().find(coord);
    CHECK(other != bb.value().end());
    for (std::size_t k = 0; k < voxels; ++k) {
      const std::size_t ia = static_cast<std::size_t>(ptr) + k;
      const std::size_t ib = static_cast<std::size_t>(other->second) + k;
      CHECK(wa[ia] == wb[ib]);  // bit patterns: weight 0 is 0
      if (wa[ia] == 0) continue;
      ++observed;
      CHECK(ta[ia] == tb[ib]);
      CHECK(ca[ia] == cb[ib]);
    }
  }
  CHECK(observed > 1000);
  return 0;
}

// Allocate the band, retrying rounds that only lost bucket-lock races, so
// both grids end with the same blocks.
template <typename Depth>
int allocate(vol::VoxelBlockGrid& grid, const Depth& depth,
             const vr::DepthCameraParams& camera) {
  for (int round = 0; round < 5; ++round) {
    vol::AllocFailures why;
    auto failed = grid.map().allocate_from_depth(depth, camera, &why);
    CHECK(failed.ok() && !why.capacity_limited());
    if (failed.value() == 0) return 0;
  }
  std::fprintf(stderr, "FAIL: allocation kept losing lock races\n");
  return 1;
}

// Observed voxels (weight > 0) of `g`: how many took black as their colour,
// and how many took none.
struct ColorCounts {
  std::size_t black = 0;
  std::size_t none = 0;
};
vkc::Result<ColorCounts> color_counts(const vr_test::Gpu& ctx,
                                      vol::VoxelBlockGrid& g) {
  VKC_ASSIGN(const std::vector<vol::BlockIndex> active,
             g.map().compact_active_blocks());
  VKC_ASSIGN(
      const std::vector<float> weight,
      vr_test::read_attribute<float>(ctx.device, ctx.allocator, g, "weight"));
  VKC_ASSIGN(const std::vector<std::uint32_t> color,
             vr_test::read_attribute<std::uint32_t>(ctx.device, ctx.allocator,
                                                    g, "color"));
  ColorCounts out;
  for (const vol::BlockIndex& b : active) {
    for (std::int32_t k = 0; k < grid_params().voxels_per_block; ++k) {
      const auto i = static_cast<std::size_t>(b.ptr + k);
      if (!(weight[i] > 0.0f)) continue;
      if (color[i] == 0xFF000000u) ++out.black;
      if (color[i] == 0u) ++out.none;
    }
  }
  return out;
}

// The left half of the colour image says it has no colour (a zero high byte,
// and black): with coverage_in_alpha nothing fuses from it, and without it
// the black is fused as colour, as it would be from any host image.
int test_coverage(vkc::Device& dev, vkc::Allocator& alloc,
                  tsdf::TsdfIntegrator& integrator,
                  const std::vector<float>& depth,
                  const vr::DepthCameraParams& cam) {
  const vr_test::Gpu ctx{dev, alloc};
  std::vector<std::uint32_t> color(kWidth * kHeight);
  for (std::uint32_t v = 0; v < kHeight; ++v) {
    for (std::uint32_t u = 0; u < kWidth; ++u) {
      color[v * kWidth + u] =
          u < kWidth / 2 ? 0u
                         : (200u | (100u << 8) | (50u << 16) | 0xFF000000u);
    }
  }
  auto color_buf = vr_test::upload_device_buffer(
      dev, alloc, color.data(), color.size() * sizeof(std::uint32_t));
  CHECK(color_buf.ok());
  tsdf::ColorFrame frame{};
  frame.buffer = &color_buf.value();
  frame.cam = vr::ColorCameraParams{
      cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height, cam.cam_to_world};
  ColorCounts counts[2];
  for (int masked = 0; masked < 2; ++masked) {
    auto grid = make_grid(dev, alloc);
    CHECK(grid.ok());
    CHECK(allocate(grid.value(), depth.data(), cam) == 0);
    frame.coverage_in_alpha = masked != 0;
    CHECK(integrator
              .integrate(grid.value(), depth.data(), cam, 5.0f,
                         tsdf::IntegrationMode::Classic, &frame)
              .ok());
    auto c = color_counts(ctx, grid.value());
    CHECK(c.ok());
    counts[masked] = c.value();
  }
  std::printf(
      "  coverage: %zu voxels black and %zu uncoloured unmasked, "
      "%zu and %zu masked\n",
      counts[0].black, counts[0].none, counts[1].black, counts[1].none);
  CHECK(counts[0].black > 1000);
  CHECK(counts[1].black == 0);
  CHECK(counts[1].none >= counts[0].none + counts[0].black);
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
  vkc::Device& dev = device.value();
  vkc::Allocator& alloc = allocator.value();
  const vr_test::Gpu ctx{dev, alloc};

  // A tilted, rippled surface 0.6-0.8 m away with a colour gradient over it,
  // so allocation and fusion both vary across the image.
  vr::DepthCameraParams cam{};
  cam.fx = 140.0f;
  cam.fy = 140.0f;
  cam.cx = 79.5f;
  cam.cy = 59.5f;
  cam.min_depth = 0.1f;
  cam.max_depth = 5.0f;
  cam.width = kWidth;
  cam.height = kHeight;
  cam.cam_to_world = vr::Mat4f(1.0f);
  std::vector<float> depth(kWidth * kHeight);
  std::vector<std::uint32_t> color(kWidth * kHeight);
  for (std::uint32_t v = 0; v < kHeight; ++v) {
    for (std::uint32_t u = 0; u < kWidth; ++u) {
      const std::size_t i = v * kWidth + u;
      depth[i] = 0.6f + 0.2f * static_cast<float>(u) / kWidth +
                 0.01f * std::sin(0.3f * static_cast<float>(v));
      if ((u * 7 + v * 3) % 31 == 0) depth[i] = 0.0f;  // a few holes
      color[i] = (u * 255 / kWidth) | ((v * 255 / kHeight) << 8) | (90u << 16);
    }
  }
  const vr::ColorCameraParams color_cam{
      cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height, cam.cam_to_world};

  auto host_grid = make_grid(dev, alloc);
  auto device_grid = make_grid(dev, alloc);
  CHECK(host_grid.ok() && device_grid.ok());
  auto integrator = tsdf::TsdfIntegrator::create(dev, alloc);
  CHECK(integrator.ok());

  auto depth_buf = vr_test::upload_device_buffer(dev, alloc, depth.data(),
                                                 depth.size() * sizeof(float));
  auto color_buf = vr_test::upload_device_buffer(
      dev, alloc, color.data(), color.size() * sizeof(std::uint32_t));
  CHECK(depth_buf.ok() && color_buf.ok());
  CHECK(depth_buf.value().is_device_local() &&
        color_buf.value().is_device_local());
  CHECK(depth_buf.value().mapped() == nullptr &&
        color_buf.value().mapped() == nullptr);

  // Host arrays into one grid, storage buffers into the other; twice, so the
  // running average runs too.
  tsdf::ColorFrame host_color{color.data(), color_cam, {}};
  tsdf::ColorFrame device_color{};
  device_color.buffer = &color_buf.value();
  device_color.cam = color_cam;
  for (int frame = 0; frame < 2; ++frame) {
    CHECK(allocate(host_grid.value(), depth.data(), cam) == 0);
    CHECK(allocate(device_grid.value(), depth_buf.value(), cam) == 0);
    CHECK(integrator
              ->integrate(host_grid.value(), depth.data(), cam, 5.0f,
                          tsdf::IntegrationMode::Classic, &host_color)
              .ok());
    CHECK(integrator
              ->integrate(device_grid.value(), depth_buf.value(), cam, 5.0f,
                          tsdf::IntegrationMode::Classic, &device_color)
              .ok());
  }
  if (check_same(ctx, host_grid.value(), device_grid.value()) != 0) return 1;

  // A device colour image beside a host depth frame is fine too.
  CHECK(integrator
            ->integrate(device_grid.value(), depth.data(), cam, 5.0f,
                        tsdf::IntegrationMode::Classic, &device_color)
            .ok());
  if (test_coverage(dev, alloc, integrator.value(), depth, cam) != 0) return 1;

  // Refusals: an empty buffer, one smaller than the image, one that is not a
  // storage buffer, and a colour frame naming both images.
  const auto invalid = vkc::Status::Code::InvalidArgument;
  // Borrowing an existing VkBuffer without its memory provenance is refused
  // before allocating blocks or updating any voxel, even on unified memory.
  const vkc::Buffer unknown(depth_buf->handle(), depth_buf->size(),
                            depth_buf->usage(), depth_buf->sharing_mode(),
                            nullptr, {}, std::nullopt);
  CHECK(
      device_grid->map().allocate_from_depth(unknown, cam).status().domain() ==
      invalid);
  CHECK(integrator->integrate(device_grid.value(), unknown, cam).domain() ==
        invalid);
  tsdf::ColorFrame unknown_color = device_color;
  unknown_color.buffer = &unknown;
  CHECK(integrator
            ->integrate(device_grid.value(), depth_buf.value(), cam, 5.0f,
                        tsdf::IntegrationMode::Classic, &unknown_color)
            .domain() == invalid);
  const vkc::Buffer empty;
  CHECK(device_grid->map().allocate_from_depth(empty, cam).status().domain() ==
        invalid);
  CHECK(integrator->integrate(device_grid.value(), empty, cam).domain() ==
        invalid);
  auto small = vr_test::upload_device_buffer(dev, alloc, depth.data(),
                                             sizeof(float) * kWidth);
  CHECK(small.ok());
  CHECK(device_grid->map()
            .allocate_from_depth(small.value(), cam)
            .status()
            .domain() == invalid);
  CHECK(
      integrator->integrate(device_grid.value(), small.value(), cam).domain() ==
      invalid);
  vkc::BufferDesc transfer_only;
  transfer_only.size = depth.size() * sizeof(float);
  transfer_only.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  transfer_only.memory = vkc::MemoryUsage::Staging;
  auto not_storage = alloc.create_buffer(transfer_only);
  CHECK(not_storage.ok());
  CHECK(device_grid->map()
            .allocate_from_depth(not_storage.value(), cam)
            .status()
            .domain() == invalid);
  tsdf::ColorFrame both = device_color;
  both.pixels = color.data();
  CHECK(integrator
            ->integrate(device_grid.value(), depth_buf.value(), cam, 5.0f,
                        tsdf::IntegrationMode::Classic, &both)
            .domain() == invalid);
  tsdf::ColorFrame small_color = device_color;
  small_color.buffer = &small.value();
  CHECK(integrator
            ->integrate(device_grid.value(), depth_buf.value(), cam, 5.0f,
                        tsdf::IntegrationMode::Classic, &small_color)
            .domain() == invalid);

  // An empty grid, where there is nothing to fuse into, refuses the same
  // buffers rather than returning before it looks at them.
  auto empty_grid = make_grid(dev, alloc);
  CHECK(empty_grid.ok());
  CHECK(integrator->integrate(empty_grid.value(), unknown, cam).domain() ==
        invalid);
  CHECK(integrator->integrate(empty_grid.value(), empty, cam).domain() ==
        invalid);
  CHECK(
      integrator->integrate(empty_grid.value(), small.value(), cam).domain() ==
      invalid);
  CHECK(integrator
            ->integrate(empty_grid.value(), depth_buf.value(), cam, 5.0f,
                        tsdf::IntegrationMode::Classic, &small_color)
            .domain() == invalid);
  CHECK(integrator->integrate(empty_grid.value(), depth_buf.value(), cam).ok());

  std::puts("tsdf_device_input: OK");
  return 0;
}
