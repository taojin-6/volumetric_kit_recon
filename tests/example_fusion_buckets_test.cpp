// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The table the examples' fusion grid starts at (examples/common/
// fuse_frame.hpp's fusion_buckets): larger for a finer grid, and never past
// the grid's own growth ceiling on the device, with a warning when a caller's
// count is lowered. Host-only but for the check against that ceiling, which
// skips without a device.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

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

// The fusion grid's warnings in `log`, apart from any other source's.
std::vector<std::string> grid_warnings(const vkc::test::LogCapture& log) {
  std::vector<std::string> kept;
  for (const std::string& warning : log.warnings()) {
    if (warning.rfind("fusion grid: ", 0) == 0) kept.push_back(warning);
  }
  return kept;
}

// Whether `log` holds `count` of them, the last naming `cap` and `asked`.
bool warned(const vkc::test::LogCapture& log, std::size_t count,
            std::int32_t cap, std::int64_t asked) {
  const std::vector<std::string> warnings = grid_warnings(log);
  return warnings.size() == count &&
         warnings.back().find("starting at " + std::to_string(cap) +
                              " buckets, not " + std::to_string(asked)) !=
             std::string::npos;
}

int host_checks(const vkc::test::LogCapture& log) {
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
  // A caller's start replaces the default; one that fits, or one create
  // refuses, reaches it unchanged. Nothing so far warns: a default is
  // capped by design.
  CHECK(fusion_buckets(0.005f, kFourGiB, 1024) == 1024);
  CHECK(fusion_buckets(0.02f, kFourGiB, 0) == 0);
  CHECK(grid_warnings(log).empty());
  // A caller's start past the cap is capped the same way, and says so.
  CHECK(fusion_buckets(0.02f, kGuaranteed, 16384) == 8192);
  CHECK(warned(log, 1, 8192, 16384));
  CHECK(fusion_buckets(0.02f, kFourGiB, kAny) == 262143);
  CHECK(warned(log, 2, 262143, kAny));
  // Past 4 GiB, which no 32-bit limit reaches, the int32 block pointer
  // VoxelGridParams::validate allows (2^31 - 1 voxels) binds first.
  CHECK(fusion_buckets(0.02f, VkDeviceSize(1) << 34, kAny) == 524287);
  CHECK(warned(log, 3, 524287, kAny));
  return 0;
}

// The cap is the ceiling a fusion grid on this device grows to.
int gpu_main(vr_test::GpuContext& gpu) {
  auto grid = vr_example::create_fusion_grid(gpu.device, gpu.allocator, 0.02f,
                                             0.08f, 1);
  CHECK(grid.ok() && grid->grid().num_buckets == 1);
  CHECK(grid_warnings(gpu.log).empty());
  const std::int32_t cap =
      fusion_buckets(0.02f, vkc::max_storage_buffer_range(gpu.device), kAny);
  CHECK(warned(gpu.log, 1, cap, kAny));
  CHECK(grid->max_num_buckets() == cap);
  return 0;
}

}  // namespace

int main() {
  {
    const vkc::test::LogCapture log;
    CHECK(host_checks(log) == 0);
  }
  const int rc = vr_test::run_on_gpu(gpu_main);
  if (rc == 0) std::puts("example_fusion_buckets: OK");
  return rc;
}
