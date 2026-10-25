// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The table the examples' fusion grid starts at (examples/common/
// fuse_frame.hpp's fusion_buckets): larger for a finer grid, and never past
// what the device's maxStorageBufferRange can bind, which is the grid's own
// growth ceiling. Host-only but for that last check, which skips without a
// device.

#include <cstdint>
#include <cstdio>
#include <limits>

#include "fuse_frame.hpp"
#include "gpu_test.hpp"
#include "test_check.hpp"

namespace vkc = volumetric_kit::core;
using vr_example::fusion_buckets;

namespace {

// The 2^27 bytes Vulkan guarantees, which lavapipe reports, and the most a
// 32-bit limit holds, which NVIDIA, MoltenVK and iOS report.
constexpr VkDeviceSize kGuaranteed = VkDeviceSize(1) << 27;
constexpr VkDeviceSize kFourGiB = 0xFFFFFFFFu;
constexpr std::int32_t kAny = std::numeric_limits<std::int32_t>::max();

int host_checks() {
  // A finer grid starts with a table its first set fits, up to 5 mm's.
  CHECK(fusion_buckets(0.02f, kFourGiB) == 16384);
  CHECK(fusion_buckets(0.01f, kFourGiB) == 16384);
  CHECK(fusion_buckets(0.005f, kFourGiB) == 65536);
  CHECK(fusion_buckets(0.0025f, kFourGiB) == 65536);
  // At 128 MiB, every 4-byte attribute array of 8,192 buckets of eight
  // 512-voxel blocks fills its binding exactly.
  CHECK(fusion_buckets(0.02f, kGuaranteed) == 8192);
  CHECK(fusion_buckets(0.01f, kGuaranteed) == 8192);
  CHECK(fusion_buckets(0.005f, kGuaranteed) == 8192);
  CHECK(fusion_buckets(0.0025f, kGuaranteed) == 8192);
  // A range between caps only the start past it, and an exact fit is no cap.
  CHECK(fusion_buckets(0.01f, VkDeviceSize(1) << 29) == 16384);
  CHECK(fusion_buckets(0.005f, VkDeviceSize(1) << 29) == 32768);
  CHECK(fusion_buckets(0.01f, VkDeviceSize(1) << 28) == 16384);
  CHECK(fusion_buckets(0.01f, (VkDeviceSize(1) << 28) - 1) == 16383);
  // A caller's start replaces the default and is capped the same way; a count
  // create refuses reaches it unchanged.
  CHECK(fusion_buckets(0.005f, kFourGiB, 1024) == 1024);
  CHECK(fusion_buckets(0.02f, kGuaranteed, 16384) == 8192);
  CHECK(fusion_buckets(0.02f, kFourGiB, kAny) == 262143);
  CHECK(fusion_buckets(0.02f, kFourGiB, 0) == 0);
  return 0;
}

// The cap is the ceiling a fusion grid on this device grows to.
int gpu_main(vr_test::GpuContext& gpu) {
  auto grid = vr_example::create_fusion_grid(gpu.device, gpu.allocator, 0.02f,
                                             0.08f, 1);
  CHECK(grid.ok() && grid->grid().num_buckets == 1);
  CHECK(grid->max_num_buckets() ==
        fusion_buckets(0.02f, vkc::max_storage_buffer_range(gpu.device), kAny));
  return 0;
}

}  // namespace

int main() {
  CHECK(host_checks() == 0);
  const int rc = vr_test::run_on_gpu(gpu_main);
  if (rc == 0) std::puts("example_fusion_buckets: OK");
  return rc;
}
