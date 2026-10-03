// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Residency is a property of the selected Vulkan memory type, not of whether
// the buffer happens to be mapped. Exercise allocator and raw-adopted buffers
// on every compatible type the device can use, including unified/BAR memory.

#include <cstdint>
#include <cstdio>
#include <utility>

#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace vr = volumetric_kit::recon;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr VkDeviceSize kBytes = 1024 * 1024;
constexpr VkBufferUsageFlags kUsage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                      VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT;

bool matches(const vr::Buffer& buffer,
             const VkPhysicalDeviceMemoryProperties& properties) {
  if (!buffer.memory_info()) return false;
  const auto& info = *buffer.memory_info();
  if (info.type_index >= properties.memoryTypeCount) return false;
  const auto& type = properties.memoryTypes[info.type_index];
  return info.properties == type.propertyFlags &&
         info.heap_index == type.heapIndex &&
         info.heap_index < properties.memoryHeapCount;
}

// Allocate a Vulkan buffer outside VMA, supplying the actual selected memory
// type when adopting it. nullopt means this buffer cannot use that type.
vr::Result<std::optional<vr::Buffer>> raw_buffer(
    const vr::Device& device, std::uint32_t index,
    const VkPhysicalDeviceMemoryProperties& properties) {
  const auto& type = properties.memoryTypes[index];
  // Device::create enables neither protectedMemory nor deviceCoherentMemory.
  // AMD coherent types are still enumerated without the feature, but using
  // them violates VUID-vkAllocateMemory-deviceCoherentMemory-02790. Lazy memory
  // is for transient images, not these buffers.
  if ((type.propertyFlags & (VK_MEMORY_PROPERTY_PROTECTED_BIT |
                             VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT |
                             VK_MEMORY_PROPERTY_DEVICE_COHERENT_BIT_AMD)) != 0)
    return std::optional<vr::Buffer>{};

  VkBufferCreateInfo desc{};
  desc.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  desc.size = kBytes;
  desc.usage = kUsage;
  desc.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VkBuffer handle = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateBuffer(device.handle(), &desc, nullptr, &handle));
  VkMemoryRequirements needs{};
  vkGetBufferMemoryRequirements(device.handle(), handle, &needs);
  if ((needs.memoryTypeBits & (1u << index)) == 0) {
    vkDestroyBuffer(device.handle(), handle, nullptr);
    return std::optional<vr::Buffer>{};
  }
  VkMemoryAllocateInfo alloc{};
  alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc.allocationSize = needs.size;
  alloc.memoryTypeIndex = index;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkResult result = vkAllocateMemory(device.handle(), &alloc, nullptr, &memory);
  if (result == VK_SUCCESS) {
    result = vkBindBufferMemory(device.handle(), handle, memory, 0);
  }
  if (result != VK_SUCCESS) {
    vkDestroyBuffer(device.handle(), handle, nullptr);
    if (memory != VK_NULL_HANDLE)
      vkFreeMemory(device.handle(), memory, nullptr);
    return vr::vk_error(result, "raw test buffer");
  }
  return std::optional<vr::Buffer>(vr::Buffer(
      handle, kBytes, kUsage, VK_SHARING_MODE_EXCLUSIVE, nullptr,
      [vk = device.handle(), handle, memory]() {
        vkDestroyBuffer(vk, handle, nullptr);
        vkFreeMemory(vk, memory, nullptr);
      },
      vr::BufferMemoryInfo{type.propertyFlags, index, type.heapIndex}));
}

}  // namespace

int main() {
  const vr::Buffer empty;
  CHECK(!empty.memory_info() && !empty.is_device_local());
  const vr::Buffer empty_adopted(
      VK_NULL_HANDLE, 0, 0, VK_SHARING_MODE_EXCLUSIVE, nullptr, {},
      vr::BufferMemoryInfo{VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0, 0});
  CHECK(!empty_adopted.memory_info() && !empty_adopted.is_device_local());

  auto instance = vr::Instance::create({});
  if (!instance) {
    std::puts("core_buffer_memory: no Vulkan instance; skipping GPU checks");
    return 0;
  }
  auto physical = instance->select_physical_device();
  if (!physical) {
    std::puts("core_buffer_memory: no compute device; skipping GPU checks");
    return 0;
  }
  auto device = vr::Device::create(instance.value(), physical.value(), {});
  CHECK(device.ok());
  auto allocator = vr::Allocator::create(instance->handle(), device.value());
  CHECK(allocator.ok());
  VkPhysicalDeviceMemoryProperties properties{};
  vkGetPhysicalDeviceMemoryProperties(physical.value(), &properties);
  VkPhysicalDeviceProperties gpu{};
  vkGetPhysicalDeviceProperties(physical.value(), &gpu);
  std::printf("core_buffer_memory: GPU %s\n", gpu.deviceName);
  vr::BufferDesc desc;
  desc.size = kBytes;
  desc.usage = kUsage;
  desc.memory = vr::MemoryUsage::DeviceLocal;
  auto allocated = allocator->create_buffer(desc);
  auto helper = vr::device_storage_buffer(allocator.value(), kBytes);
  CHECK(allocated.ok() && helper.ok());
  for (const vr::Buffer* buffer : {&allocated.value(), &helper.value()}) {
    CHECK(matches(*buffer, properties));
    CHECK(buffer->is_device_local());
    CHECK(buffer->mapped() == nullptr);
    CHECK(vr::StorageInput(*buffer).check("resident", kBytes).ok());
    const auto& memory = *buffer->memory_info();
    std::printf("  device allocation: type=%u heap=%u flags=0x%x\n",
                memory.type_index, memory.heap_index, memory.properties);
  }

  // Exercise the AMD skip even on drivers without coherent memory types.
  // Keep a known-compatible index: without the guard, raw_buffer allocates a
  // real buffer and this check fails instead of silently skipping the case.
  auto coherent_properties = properties;
  const auto local_type = allocated->memory_info()->type_index;
  coherent_properties.memoryTypes[local_type].propertyFlags |=
      VK_MEMORY_PROPERTY_DEVICE_COHERENT_BIT_AMD;
  auto coherent = raw_buffer(device.value(), local_type, coherent_properties);
  CHECK(coherent.ok() && !coherent.value());

  desc.size = 0;
  CHECK(!allocator->create_buffer(desc));
  CHECK(!vr::device_storage_buffer(allocator.value(), 0));

  // An adoption with no memory provenance must not silently turn into a
  // device input, even when the actual allocation happens to be local.
  vr::Buffer unknown(helper->handle(), kBytes, kUsage,
                     VK_SHARING_MODE_EXCLUSIVE, nullptr, {}, std::nullopt);
  CHECK(!unknown.memory_info() && !unknown.is_device_local());
  const auto invalid = vr::Status::Code::InvalidArgument;
  CHECK(vr::StorageInput(unknown).check("unknown", kBytes).domain() == invalid);
  vr::CommandBatch batch(device.value(), allocator.value());
  vr::Buffer scratch;
  CHECK(vr::StorageInput(unknown)
            .buffer(batch, allocator.value(), kBytes, scratch)
            .status()
            .domain() == invalid);

  bool saw_nonlocal = false;
  bool saw_host_local = false;
  for (std::uint32_t i = 0; i < properties.memoryTypeCount; ++i) {
    const auto flags = properties.memoryTypes[i].propertyFlags;
    auto raw = raw_buffer(device.value(), i, properties);
    CHECK(raw.ok());
    if (!raw.value()) continue;
    const vr::Buffer& buffer = *raw.value();
    CHECK(matches(buffer, properties));
    const bool local = (flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0;
    std::printf("  raw allocation: type=%u heap=%u flags=0x%x accepted=%d\n", i,
                buffer.memory_info()->heap_index, flags, local);
    CHECK(buffer.is_device_local() == local);
    CHECK(vr::StorageInput(buffer).check("raw", kBytes).ok() == local);
    saw_nonlocal |= !local;
    saw_host_local |=
        local && (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0;
    if (local) {
      // A host-visible local type is accepted through an actual batch path;
      // no host mapping is necessary for residency or for upload/readback.
      const std::uint32_t word = 0x1234abcd;
      std::uint32_t back = 0;
      vr::CommandBatch transfer(device.value(), allocator.value());
      CHECK(transfer.upload(buffer, 0, &word, sizeof(word)).ok());
      CHECK(transfer.readback(buffer, 0, sizeof(back), &back).ok());
      CHECK(transfer.submit().ok());
      CHECK(back == word);
    }
  }
  // Mapped helper metadata also describes the actual selected type, which
  // may be local on UMA/BAR or non-local on a discrete GPU.
  auto host = vr::storage_buffer(allocator.value(), kBytes);
  CHECK(host.ok() && matches(host.value(), properties));
  CHECK(host->mapped() != nullptr);
  CHECK((host->memory_info()->properties &
         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0);
  CHECK(vr::StorageInput(host.value()).check("mapped", kBytes).ok() ==
        host->is_device_local());
  std::printf("core_buffer_memory: OK (nonlocal=%d, host-visible local=%d)\n",
              saw_nonlocal, saw_host_local);
  return 0;
}
