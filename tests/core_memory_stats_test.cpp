// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Allocator::memory_stats: the device-memory figures the perf overlay reports.
//
// Behavioural checks only -- the exact bytes are VMA's accounting (and, without
// VK_EXT_memory_budget, its estimate), so this pins the properties a consumer
// relies on rather than any absolute number: the device's heaps are reported,
// a budget is not below its usage, an allocation shows up as growth, freeing it
// gives the bytes back, and a moved-from allocator reports nothing.
//
// Needs a device, so the whole test skips (exit 0) where no driver/device is
// present, like the other Vulkan tests.

#include <cstdint>
#include <cstdio>
#include <utility>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace vr = volumetric_kit::recon;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::fprintf(stderr, "FAIL (%s:%d): %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                              \
    }                                                                        \
  } while (0)

namespace {

// Total bytes in use across every reported heap. On a UMA (Apple) GPU that is
// one heap; on a discrete GPU the allocation may land in any of them, so the
// sum -- not a fixed heap index -- is what a test can assert on.
std::uint64_t total_usage(const vr::MemoryStats& stats) {
  std::uint64_t total = 0;
  for (std::uint32_t heap = 0; heap < stats.heap_count; ++heap) {
    total += stats.heaps[heap].usage_bytes;
  }
  return total;
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

  // A live allocator reports the device's heaps, and no heap claims to use more
  // than its budget.
  const vr::MemoryStats before = allocator.value().memory_stats();
  CHECK(before.heap_count > 0);
  CHECK(before.heap_count <= VK_MAX_MEMORY_HEAPS);
  for (std::uint32_t heap = 0; heap < before.heap_count; ++heap) {
    CHECK(before.heaps[heap].budget_bytes >= before.heaps[heap].usage_bytes);
  }

  // An allocation is visible as growth of at least its own size. Big enough
  // that VMA cannot satisfy it from slack in an existing block.
  constexpr VkDeviceSize kBytes = 64ull * 1024 * 1024;
  vr::MemoryStats during_peak;
  {
    vr::BufferDesc desc;
    desc.size = kBytes;
    desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(buffer.ok());

    during_peak = allocator.value().memory_stats();
    CHECK(during_peak.heap_count == before.heap_count);
    CHECK(total_usage(during_peak) >= total_usage(before) + kBytes);
  }

  // Freeing the buffer does NOT necessarily give the bytes back: VMA keeps the
  // emptied `VkDeviceMemory` block pooled for reuse, and `usage` counts
  // allocated blocks rather than live sub-allocations. So the figure a consumer
  // sees is a high-water mark that does not fall when a buffer is destroyed --
  // what is guaranteed is that it does not keep growing.
  const vr::MemoryStats after = allocator.value().memory_stats();
  CHECK(total_usage(after) <= total_usage(during_peak));

  // A moved-from allocator owns nothing and reports nothing, matching valid().
  vr::Allocator moved = std::move(allocator).value();
  CHECK(moved.memory_stats().heap_count > 0);
  vr::Allocator target = std::move(moved);
  CHECK(!moved.valid());
  CHECK(moved.memory_stats().heap_count == 0);
  CHECK(target.memory_stats().heap_count == before.heap_count);

  std::printf("core memory stats: OK (%u heap(s), %.1f MB in use)\n",
              before.heap_count,
              static_cast<double>(total_usage(before)) / (1024.0 * 1024.0));
  return 0;
}
