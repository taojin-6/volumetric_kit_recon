// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/allocator.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include <vk_mem_alloc.h>

#include "vk_physical_device.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {

// Holds the one handle that must stay out of allocator.hpp. Freeing in ~Impl
// (not ~Allocator) is what lets Allocator's move ops default correctly: moving
// the unique_ptr transfers ownership, and the emptied source frees nothing.
struct Allocator::Impl {
  VmaAllocator allocator = nullptr;
  ~Impl() {
    if (allocator != nullptr) {
      vmaDestroyAllocator(allocator);
    }
  }
};

namespace {

/// Ceiling on the distinct queue families one buffer can be shared between.
///
/// Two is the case that exists -- one reconstruction family, one renderer
/// family -- and the headroom is for a third consumer rather than for a device
/// with an unusual queue layout, since what is named here is *consumers*, not
/// what the driver exposes. A fixed array keeps `create_buffer` allocation-free
/// on a path that runs per resource.
constexpr std::uint32_t kMaxQueueFamilies = 4;

VmaMemoryUsage to_vma_usage(MemoryUsage memory) {
  switch (memory) {
    case MemoryUsage::DeviceLocal:
      return VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    case MemoryUsage::HostVisible:
      return VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
    case MemoryUsage::Auto:
      break;
  }
  return VMA_MEMORY_USAGE_AUTO;
}

}  // namespace

namespace detail {

std::uint32_t distinct_queue_families(const std::uint32_t* families,
                                      std::uint32_t count, std::uint32_t* out,
                                      std::uint32_t out_capacity) {
  std::uint32_t distinct = 0;
  for (std::uint32_t i = 0; i < count; ++i) {
    bool seen = false;
    for (std::uint32_t j = 0; j < distinct; ++j) {
      seen = seen || out[j] == families[i];
    }
    if (seen) {
      continue;
    }
    if (distinct == out_capacity) {
      return out_capacity + 1;
    }
    out[distinct++] = families[i];
  }
  return distinct;
}

}  // namespace detail

Result<Allocator> Allocator::create(VkInstance instance, const Device& device) {
  if (instance == VK_NULL_HANDLE) {
    return Status::invalid_argument("Allocator::create: instance is null");
  }
  if (device.handle() == VK_NULL_HANDLE ||
      device.physical_device() == VK_NULL_HANDLE) {
    return Status::invalid_argument("Allocator::create: device is not valid");
  }

  // Tell VMA which Vulkan version's features it may use. Negotiate the
  // effective version from the instance and physical device, then cap at 1.1:
  // recon needs no VMA feature past it, and the conservative floor avoids
  // depending on 1.2+ entry points the loader may route differently under
  // MoltenVK. Mirrors volumetric_kit_gfx.
  std::uint32_t instance_version = VK_API_VERSION_1_0;
  if (vkEnumerateInstanceVersion(&instance_version) != VK_SUCCESS) {
    instance_version = VK_API_VERSION_1_0;
  }
  const std::uint32_t effective = std::min(
      instance_version, detail::physical_api_version(device.physical_device()));

  // With VMA_STATIC_VULKAN_FUNCTIONS (see vma_impl.cpp) VMA resolves entry
  // points against the linked loader; feeding the two get-proc-addr seeds keeps
  // it happy across VMA builds without adopting the dynamic-functions path.
  VmaVulkanFunctions functions{};
  functions.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
  functions.vkGetDeviceProcAddr = vkGetDeviceProcAddr;

  VmaAllocatorCreateInfo info{};
  info.instance = instance;
  info.physicalDevice = device.physical_device();
  info.device = device.handle();
  info.vulkanApiVersion =
      effective >= VK_API_VERSION_1_1 ? VK_API_VERSION_1_1 : VK_API_VERSION_1_0;
  info.pVulkanFunctions = &functions;

  auto impl = std::make_unique<Impl>();
  VR_VK_TRY(vmaCreateAllocator(&info, &impl->allocator));

  Allocator result;
  result.impl_ = std::move(impl);
  return result;
}

Allocator::~Allocator() = default;
Allocator::Allocator(Allocator&& other) noexcept = default;
Allocator& Allocator::operator=(Allocator&& other) noexcept = default;

Result<Buffer> Allocator::create_buffer(const BufferDesc& desc) {
  if (impl_ == nullptr) {
    return Status::invalid_argument(
        "Allocator::create_buffer: allocator is moved-from");
  }
  if (desc.size == 0) {
    return Status::invalid_argument("Allocator::create_buffer: size is zero");
  }
  if (desc.usage == 0) {
    return Status::invalid_argument("Allocator::create_buffer: usage is zero");
  }
  if (desc.mapped && desc.memory == MemoryUsage::DeviceLocal) {
    return Status::invalid_argument(
        "Allocator::create_buffer: a device-local buffer cannot be "
        "host-mapped");
  }
  if (desc.memory == MemoryUsage::HostVisible && !desc.mapped) {
    return Status::invalid_argument(
        "Allocator::create_buffer: a host-visible buffer must be mapped (there "
        "is no separate map()); request mapped=true");
  }

  if (desc.queue_families == nullptr && desc.queue_family_count != 0) {
    return Status::invalid_argument(
        "Allocator::create_buffer: queue_family_count is non-zero but "
        "queue_families is null");
  }

  // Distinct families decide the mode, not the count the caller passed: Vulkan
  // requires CONCURRENT to name at least two, and requires them unique, so a
  // caller passing its compute and render families unconditionally would
  // otherwise be malformed on every platform where those are one family.
  std::uint32_t distinct[kMaxQueueFamilies]{};
  const std::uint32_t distinct_count = detail::distinct_queue_families(
      desc.queue_families, desc.queue_family_count, distinct,
      kMaxQueueFamilies);
  if (distinct_count > kMaxQueueFamilies) {
    return Status::invalid_argument(
        "Allocator::create_buffer: more distinct queue families than this "
        "supports");
  }

  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = desc.size;
  buffer_info.usage = desc.usage;
  // One family is exclusive by definition, and zero is the default path.
  if (distinct_count > 1) {
    buffer_info.sharingMode = VK_SHARING_MODE_CONCURRENT;
    buffer_info.queueFamilyIndexCount = distinct_count;
    buffer_info.pQueueFamilyIndices = distinct;
  } else {
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  }

  VmaAllocationCreateInfo alloc_info{};
  alloc_info.usage = to_vma_usage(desc.memory);
  if (desc.mapped) {
    alloc_info.flags |= VMA_ALLOCATION_CREATE_MAPPED_BIT;
    alloc_info.flags |=
        desc.host_access == HostAccess::SequentialWrite
            ? VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
            : VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    // HOST_COHERENT so writes are visible without an explicit
    // vkFlush/invalidate
    // -- the mapped() contract is a plain pointer.
    alloc_info.requiredFlags |= VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  }

  VkBuffer buffer = VK_NULL_HANDLE;
  VmaAllocation allocation = nullptr;
  VmaAllocationInfo out_info{};
  VR_VK_TRY(vmaCreateBuffer(impl_->allocator, &buffer_info, &alloc_info,
                            &buffer, &allocation, &out_info));

  if (desc.mapped && out_info.pMappedData == nullptr) {
    vmaDestroyBuffer(impl_->allocator, buffer, allocation);
    return vk_error(
        VK_ERROR_MEMORY_MAP_FAILED,
        "Allocator::create_buffer: mapping requested but VMA returned no "
        "mapped pointer");
  }

  // Capture the opaque VMA handles in the type-erased deleter so Buffer frees
  // both without VMA appearing in buffer.hpp.
  VmaAllocator allocator = impl_->allocator;
  return Buffer(buffer, desc.size, desc.usage, out_info.pMappedData,
                [allocator, buffer, allocation]() {
                  vmaDestroyBuffer(allocator, buffer, allocation);
                });
}

MemoryStats Allocator::memory_stats() const {
  MemoryStats stats;
  if (impl_ == nullptr) {
    return stats;  // moved-from: no heaps, no figures
  }
  // VMA writes one VmaBudget per heap; the device's heap count comes from the
  // memory properties VMA already caches, so this needs no physical device.
  const VkPhysicalDeviceMemoryProperties* props = nullptr;
  vmaGetMemoryProperties(impl_->allocator, &props);
  if (props == nullptr) {
    return stats;
  }
  VmaBudget budgets[VK_MAX_MEMORY_HEAPS]{};
  vmaGetHeapBudgets(impl_->allocator, budgets);
  stats.heap_count = props->memoryHeapCount;
  for (std::uint32_t heap = 0; heap < stats.heap_count; ++heap) {
    stats.heaps[heap].usage_bytes = budgets[heap].usage;
    stats.heaps[heap].budget_bytes = budgets[heap].budget;
  }
  return stats;
}

}  // namespace volumetric_kit::recon
