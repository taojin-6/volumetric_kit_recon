// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The table a fusion grid starts at (tsdf/fusion_grid.hpp's fusion_buckets):
// larger for a finer grid, and never past the grid's own growth ceiling on the
// device, with a warning when a caller's count is lowered. Host-only but for
// the grid on a device: its layout, attributes and ceiling, which skip without
// one.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "gpu_test.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
using tsdf::fusion_buckets;

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
  // A NaN size takes the default start, for create to refuse.
  CHECK(fusion_buckets(std::numeric_limits<float>::quiet_NaN(), kFourGiB) ==
        16384);
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

// The grid: block_grid_params' layout at the caller's start, carrying the
// fuser's three attributes; and the cap is the ceiling it grows to on this
// device.
int gpu_main(vr_test::GpuContext& gpu) {
  auto grid =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, 0.02f, 0.08f, 1);
  CHECK(grid.ok() && grid->grid().num_buckets == 1);
  CHECK(grid_warnings(gpu.log).empty());
  const vol::VoxelGridParams want = vol::block_grid_params(0.02f, 0.08f, 1);
  const vol::VoxelGridParams& got = grid->grid();
  CHECK(got.voxel_size == want.voxel_size && got.trunc_dist == want.trunc_dist);
  CHECK(got.block_size == want.block_size &&
        got.voxels_per_block == want.voxels_per_block);
  CHECK(got.bucket_size == want.bucket_size &&
        got.num_blocks == want.num_blocks && got.max_chain == want.max_chain);
  CHECK(grid->attribute_count() == 3);
  CHECK(grid->attribute("tsdf").ok() &&
        grid->attribute("tsdf")->element_size == sizeof(float));
  CHECK(grid->attribute("weight").ok() &&
        grid->attribute("weight")->element_size == sizeof(float));
  CHECK(grid->attribute("color").ok() &&
        grid->attribute("color")->element_size == sizeof(std::uint32_t));
  const std::int32_t cap =
      fusion_buckets(0.02f, vkc::max_storage_buffer_range(gpu.device), kAny);
  CHECK(warned(gpu.log, 1, cap, kAny));
  CHECK(grid->max_num_buckets() == cap);
  // A NaN voxel is the grid's refusal, reached by defined arithmetic.
  const auto refused =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator,
                               std::numeric_limits<float>::quiet_NaN(), 0.08f);
  CHECK(!refused.ok() &&
        refused.status().domain() == vkc::Status::Code::InvalidArgument);
  return 0;
}

}  // namespace

int main() {
  {
    const vkc::test::LogCapture log;
    CHECK(host_checks(log) == 0);
  }
  const int rc = vr_test::run_on_gpu(gpu_main);
  if (rc == 0) std::puts("tsdf_fusion_grid: OK");
  return rc;
}
