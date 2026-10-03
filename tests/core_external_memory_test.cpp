// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// An exported buffer: on a device that exports memory it comes with a file
// descriptor and works like any other storage buffer; elsewhere it is refused.
// And the memory type such a resource binds: the first that fits, which for
// device-local alone is one the host cannot map wherever the GPU has one.

#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <vector>

#include "buffer_readback.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/external_memory.hpp"
#include "volumetric_kit/recon/core/instance.hpp"

namespace vr = volumetric_kit::recon;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance; skipping\n");
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device; skipping\n");
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value(), gpu.value(), {});
  CHECK(device.ok());
  const vr::Device& dev = device.value();

  VkPhysicalDeviceMemoryProperties memory{};
  vkGetPhysicalDeviceMemoryProperties(dev.physical_device(), &memory);
  const auto flags = [&](std::uint32_t i) {
    return memory.memoryTypes[i].propertyFlags;
  };
  constexpr VkMemoryPropertyFlags kLocal = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
  constexpr VkMemoryPropertyFlags kMapped = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
  const auto local = vr::find_memory_type(dev, ~0u, kLocal);
  CHECK(local && (flags(*local) & kLocal) != 0);
  for (std::uint32_t i = 0; i < *local; ++i) CHECK((flags(i) & kLocal) == 0);
  // The spec's order is what keeps a device-local resource out of the BAR.
  bool unmapped_local = false;
  for (std::uint32_t i = 0; i < memory.memoryTypeCount; ++i) {
    unmapped_local |= (flags(i) & kLocal) != 0 && (flags(i) & kMapped) == 0;
  }
  CHECK(!unmapped_local || (flags(*local) & kMapped) == 0);
  const auto plain = vr::find_memory_type(dev, ~0u, kLocal, kMapped);
  CHECK(plain.has_value() == unmapped_local);
  CHECK(!vr::find_memory_type(dev, 0, 0));
  CHECK(!vr::find_memory_type(dev, 1u << *local, kLocal, kLocal));
  // Types that need features Device::create leaves off are never chosen.
  for (VkMemoryPropertyFlags unusable :
       {VK_MEMORY_PROPERTY_PROTECTED_BIT,
        VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT,
        VK_MEMORY_PROPERTY_DEVICE_COHERENT_BIT_AMD}) {
    CHECK(!vr::find_memory_type(dev, ~0u, unusable));
  }

  if (!dev.exports_memory()) {
    CHECK(vr::create_exported_buffer(dev, 256).status().domain() ==
          vr::Status::Code::Unsupported);
    std::printf(
        "core_external_memory: no export here; refused as unsupported\n");
    return 0;
  }
  CHECK(vr::create_exported_buffer(dev, 0).status().domain() ==
        vr::Status::Code::InvalidArgument);

  constexpr std::size_t kWords = 1024;
  auto made = vr::create_exported_buffer(dev, kWords * sizeof(std::uint32_t));
  CHECK(made.ok());
  vr::ExportedBuffer exported = std::move(made).value();
  CHECK(exported.fd >= 0);
  CHECK(exported.memory_size >= kWords * sizeof(std::uint32_t));
  CHECK(exported.buffer.size() == kWords * sizeof(std::uint32_t));
  CHECK(exported.buffer.is_device_local());
  CHECK(exported.buffer.memory_info().has_value());
  const auto& backing = *exported.buffer.memory_info();
  CHECK(backing.type_index < memory.memoryTypeCount);
  CHECK(backing.properties == flags(backing.type_index));
  CHECK(backing.heap_index == memory.memoryTypes[backing.type_index].heapIndex);
  CHECK(close(exported.fd) == 0);

  // It is an ordinary storage buffer to a batch.
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), dev);
  CHECK(allocator.ok());
  std::vector<std::uint32_t> words(kWords);
  for (std::size_t i = 0; i < kWords; ++i) {
    words[i] = static_cast<std::uint32_t>(i * 2654435761u);
  }
  CHECK(
      vr_test::write_back(dev, allocator.value(), exported.buffer, words).ok());
  auto back = vr_test::read_back<std::uint32_t>(dev, allocator.value(),
                                                exported.buffer, kWords);
  CHECK(back.ok());
  CHECK(back.value() == words);
  std::printf("core_external_memory: OK\n");
  return 0;
}
