// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for VoxelHashMap::resize: allocate a set of blocks, grow the table
// to more buckets, and verify the active set survives the resize and that
// further allocation into the grown table works. Runs on the real driver
// (MoltenVK / NVIDIA); exits 0 (skip) where no device is present.

#include <cstdint>
#include <cstdio>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

using Coord = std::tuple<int, int, int>;

// A 3x3x3 cube of block coords centered at `c`.
std::vector<vol::BlockIndex> cube(int cx, int cy, int cz) {
  std::vector<vol::BlockIndex> out;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        vol::BlockIndex block{};
        block.coord = vr::Vec3i(cx + x, cy + y, cz + z);
        out.push_back(block);
      }
    }
  }
  return out;
}

vr::Result<std::set<Coord>> active_set(vol::VoxelHashMap& map) {
  vr::Result<std::vector<vol::BlockIndex>> active = map.compact_active_blocks();
  if (!active) {
    return active.status();
  }
  std::set<Coord> out;
  for (const vol::BlockIndex& block : active.value()) {
    out.insert({block.coord.x, block.coord.y, block.coord.z});
  }
  return out;
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

  // Start small so the resize is a real growth.
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.005f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 256;
  grid.num_blocks = 256 * 8;
  grid.max_chain = 128;

  vr::Result<vol::VoxelHashMap> map_result =
      vol::VoxelHashMap::create(device.value(), allocator.value(), grid);
  if (!map_result) {
    std::fprintf(stderr, "VoxelHashMap::create failed: %s\n",
                 map_result.status().message().c_str());
    return 1;
  }
  vol::VoxelHashMap map = std::move(map_result).value();

  // Allocate cube A, then grow the table.
  std::vector<vol::BlockIndex> a = cube(0, 0, 0);
  std::set<Coord> want;
  for (const vol::BlockIndex& b : a) {
    want.insert({b.coord.x, b.coord.y, b.coord.z});
  }
  vr::Result<std::uint32_t> alloc_a =
      map.allocate(a.data(), static_cast<std::uint32_t>(a.size()));
  CHECK(alloc_a.ok() && alloc_a.value() == 0);

  CHECK(map.resize(1024).ok());
  CHECK(map.grid().num_buckets == 1024);
  CHECK(map.grid().num_blocks == 1024 * 8);

  // The active set survived the growth.
  vr::Result<std::set<Coord>> after = active_set(map);
  CHECK(after.ok());
  CHECK(after.value() == want);

  // The grown table still allocates: cube B, disjoint from A.
  std::vector<vol::BlockIndex> b = cube(20, 20, 20);
  for (const vol::BlockIndex& block : b) {
    want.insert({block.coord.x, block.coord.y, block.coord.z});
  }
  vr::Result<std::uint32_t> alloc_b =
      map.allocate(b.data(), static_cast<std::uint32_t>(b.size()));
  CHECK(alloc_b.ok() && alloc_b.value() == 0);
  vr::Result<std::set<Coord>> both = active_set(map);
  CHECK(both.ok());
  CHECK(both.value() == want);
  CHECK(both.value().size() == 54);

  // resize refuses a non-growing count.
  CHECK(map.resize(1024).domain() == vr::Status::Code::InvalidArgument);
  CHECK(map.resize(512).domain() == vr::Status::Code::InvalidArgument);

  std::printf(
      "recon volume resize test passed: grew 256 -> 1024 buckets, %zu blocks "
      "survived, further allocation works\n",
      both.value().size());
  return 0;
}
