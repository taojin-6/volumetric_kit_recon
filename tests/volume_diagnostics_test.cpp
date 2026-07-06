// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for VoxelHashMap::diagnostics: allocate a set of blocks that all
// hash to one bucket (forcing a deterministic collision chain), and verify the
// reported active / overflow / chain-length + heap statistics. Runs on the real
// driver (MoltenVK / NVIDIA); exits 0 (skip) where no device is present.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/volume/hash.hpp"
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

bool close(float a, float b) { return std::fabs(a - b) < 1e-5f; }

// Find `n` distinct block coords that hash to `target_bucket` -- so they all
// land in one primary bucket + its collision chain.
std::vector<vol::BlockIndex> coords_in_bucket(int target_bucket,
                                              int num_buckets, int n) {
  std::vector<vol::BlockIndex> out;
  for (int x = 0; x < 64 && static_cast<int>(out.size()) < n; ++x) {
    for (int y = 0; y < 64 && static_cast<int>(out.size()) < n; ++y) {
      for (int z = 0; z < 64 && static_cast<int>(out.size()) < n; ++z) {
        const vr::Vec3i c(x, y, z);
        if (static_cast<int>(vol::hash_bucket(c, num_buckets)) ==
            target_bucket) {
          vol::BlockIndex block{};
          block.coord = c;
          out.push_back(block);
        }
      }
    }
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

  // 64 buckets x 4; a 256-block heap.
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.005f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 4;
  grid.num_buckets = 64;
  grid.num_blocks = 64 * 4;
  grid.max_chain = 128;
  const int total_slots = grid.num_buckets * grid.bucket_size;

  vr::Result<vol::VoxelHashMap> map_result =
      vol::VoxelHashMap::create(device.value(), allocator.value(), grid);
  if (!map_result) {
    std::fprintf(stderr, "VoxelHashMap::create failed: %s\n",
                 map_result.status().message().c_str());
    return 1;
  }
  vol::VoxelHashMap map = std::move(map_result).value();

  // Empty table. (This also reads init's writes right after create(); it passes
  // because the buffers are HOST_COHERENT, so it confirms init ran but does not
  // independently prove the init->host visibility barrier -- that is
  // unobservable on coherent memory and would only bite device-local buffers.)
  vr::Result<vol::HashDiagnostics> empty = map.diagnostics();
  CHECK(empty.ok());
  CHECK(empty.value().active_count == 0);
  CHECK(empty.value().overflow_count == 0);
  CHECK(empty.value().max_chain_length == 0);
  CHECK(empty.value().heap_free_count == grid.num_blocks);
  CHECK(empty.value().total_blocks == grid.num_blocks);
  CHECK(close(empty.value().load_factor, 0.0f));
  CHECK(close(empty.value().heap_utilization, 0.0f));

  // 7 coords in bucket 0: 4 fill the primary bucket, 3 overflow into a chain.
  std::vector<vol::BlockIndex> coords =
      coords_in_bucket(0, grid.num_buckets, 7);
  CHECK(coords.size() == 7);
  vr::Result<std::uint32_t> alloc =
      map.allocate(coords.data(), static_cast<std::uint32_t>(coords.size()));
  CHECK(alloc.ok() && alloc.value() == 0);

  vr::Result<vol::HashDiagnostics> diag = map.diagnostics();
  CHECK(diag.ok());
  const vol::HashDiagnostics& d = diag.value();
  CHECK(d.active_count == 7);
  CHECK(d.heap_free_count == grid.num_blocks - 7);
  CHECK(d.total_blocks == grid.num_blocks);
  // bucket_size == 4 primary slots filled, so the other 3 chain off the anchor.
  CHECK(d.overflow_count == 3);
  CHECK(d.max_chain_length == 3);
  // NOTE: for any grid, validate() forces num_blocks ==
  // num_buckets*bucket_size, so total_slots == num_blocks and (one heap block
  // per active slot) load_factor and heap_utilization are numerically identical
  // here -- these two checks do not independently distinguish the fields.
  CHECK(close(d.load_factor, 7.0f / static_cast<float>(total_slots)));
  CHECK(close(d.heap_utilization, 7.0f / static_cast<float>(grid.num_blocks)));

  std::printf(
      "recon volume diagnostics test passed: active=%d overflow=%d "
      "max_chain=%d heap_free=%d load=%.4f\n",
      d.active_count, d.overflow_count, d.max_chain_length, d.heap_free_count,
      static_cast<double>(d.load_factor));
  return 0;
}
