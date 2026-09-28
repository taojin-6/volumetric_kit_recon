// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The device-input overloads of VoxelHashMap::allocate_from_depth and
// TsdfIntegrator::integrate against the host ones: one depth and colour frame
// fused into two grids, once from host arrays and once from storage buffers,
// must give the same blocks and the same tsdf, weight and colour in each. And
// the buffers they refuse. Runs on the real driver; exits 0 (skip) where no
// device is present.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <tuple>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
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

vr::Result<vol::VoxelBlockGrid> make_grid(vr::Device& device,
                                          vr::Allocator& allocator) {
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)},
                                      {"color", sizeof(std::uint32_t)}};
  return vol::VoxelBlockGrid::create(device, allocator, grid_params(), attrs,
                                     3);
}

// Every active block's coordinate and first voxel (`ptr` is a voxel offset).
vr::Result<std::map<Coord, std::int32_t>> blocks_of(vol::VoxelBlockGrid& g) {
  VR_ASSIGN(std::vector<vol::BlockIndex> active,
            g.map().compact_active_blocks());
  std::map<Coord, std::int32_t> out;
  for (const vol::BlockIndex& b : active) {
    out[Coord{b.coord.x, b.coord.y, b.coord.z}] = b.ptr;
  }
  return out;
}

// The two grids' blocks name the same coordinates and hold the same weight in
// every voxel, and the same tsdf and colour in every observed one (the rest
// were never written). A slot can differ: allocation order is the GPU's.
int check_same(vol::VoxelBlockGrid& a, vol::VoxelBlockGrid& b) {
  auto ba = blocks_of(a);
  auto bb = blocks_of(b);
  CHECK(ba.ok() && bb.ok());
  CHECK(!ba.value().empty());
  CHECK(ba.value().size() == bb.value().size());
  const auto view = [](vol::VoxelBlockGrid& g, const char* name) {
    return static_cast<const std::uint32_t*>(
        g.attribute(name).value().buffer->mapped());
  };
  const std::uint32_t* wa = view(a, "weight");
  const std::uint32_t* wb = view(b, "weight");
  const std::uint32_t* ta = view(a, "tsdf");
  const std::uint32_t* tb = view(b, "tsdf");
  const std::uint32_t* ca = view(a, "color");
  const std::uint32_t* cb = view(b, "color");
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

vr::Result<vr::Buffer> upload(vr::Allocator& allocator, const void* data,
                              std::size_t bytes) {
  return vr::upload_storage_buffer(allocator, data, bytes);
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
      vr::Device::create(instance.value(), gpu.value(), {});
  CHECK(device.ok());
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  vr::Device& dev = device.value();
  vr::Allocator& alloc = allocator.value();

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

  auto depth_buf = upload(alloc, depth.data(), depth.size() * sizeof(float));
  auto color_buf =
      upload(alloc, color.data(), color.size() * sizeof(std::uint32_t));
  CHECK(depth_buf.ok() && color_buf.ok());

  // Host arrays into one grid, storage buffers into the other; twice, so the
  // running average runs too.
  tsdf::ColorFrame host_color{color.data(), color_cam, {}};
  tsdf::ColorFrame device_color{};
  device_color.buffer = &color_buf.value();
  device_color.cam = color_cam;
  for (int frame = 0; frame < 2; ++frame) {
    auto a = host_grid->map().allocate_from_depth(depth.data(), cam);
    auto b = device_grid->map().allocate_from_depth(depth_buf.value(), cam);
    CHECK(a.ok() && a.value() == 0);
    CHECK(b.ok() && b.value() == 0);
    CHECK(integrator
              ->integrate(host_grid.value(), depth.data(), cam, 5.0f,
                          tsdf::IntegrationMode::Classic, &host_color)
              .ok());
    CHECK(integrator
              ->integrate(device_grid.value(), depth_buf.value(), cam, 5.0f,
                          tsdf::IntegrationMode::Classic, &device_color)
              .ok());
  }
  if (check_same(host_grid.value(), device_grid.value()) != 0) return 1;

  // A device colour image beside a host depth frame is fine too.
  CHECK(integrator
            ->integrate(device_grid.value(), depth.data(), cam, 5.0f,
                        tsdf::IntegrationMode::Classic, &device_color)
            .ok());

  // Refusals: an empty buffer, one smaller than the image, one that is not a
  // storage buffer, and a colour frame naming both images.
  const auto invalid = vr::Status::Code::InvalidArgument;
  const vr::Buffer empty;
  CHECK(device_grid->map().allocate_from_depth(empty, cam).status().domain() ==
        invalid);
  CHECK(integrator->integrate(device_grid.value(), empty, cam).domain() ==
        invalid);
  auto small = upload(alloc, depth.data(), sizeof(float) * kWidth);
  CHECK(small.ok());
  CHECK(device_grid->map()
            .allocate_from_depth(small.value(), cam)
            .status()
            .domain() == invalid);
  CHECK(
      integrator->integrate(device_grid.value(), small.value(), cam).domain() ==
      invalid);
  vr::BufferDesc transfer_only;
  transfer_only.size = depth.size() * sizeof(float);
  transfer_only.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  transfer_only.memory = vr::MemoryUsage::HostVisible;
  transfer_only.mapped = true;
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

  std::puts("tsdf_device_input: OK");
  return 0;
}
