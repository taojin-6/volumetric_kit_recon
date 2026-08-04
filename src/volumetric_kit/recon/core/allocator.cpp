// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/allocator.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include <vk_mem_alloc.h>

#include "queue_family_set.hpp"
#include "vk_physical_device.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {

// Holds the handles that must stay out of allocator.hpp. Freeing in ~Impl (not
// ~Allocator) is what lets Allocator's move ops default correctly: moving the
// shared_ptr transfers a reference, and the emptied source frees nothing.
//
// Held by shared_ptr, and every Buffer's deleter keeps a reference: a Buffer
// frees through this allocator, so the allocator has to outlive it. Making that
// structural rather than documented is what closes the case a *documented*
// ordering rule cannot express -- `a = std::move(b)` ends the resource's life
// while the wrapper `a` visibly lives on, so a reader who satisfies "destroy
// Buffers before their Allocator" by keeping the object alive still gets a
// use-after-free. The shared reference costs one atomic per buffer create /
// destroy, against a device allocation on the same path.
struct Allocator::Impl {
  VmaAllocator allocator = nullptr;
  // How many queue families the device reports, so create_buffer can reject an
  // index that names none of them. Read once here because Vulkan offers no way
  // to ask a VmaAllocator (or a VkDevice) for it afterwards.
  std::uint32_t queue_family_count = 0;
  ~Impl() {
    if (allocator != nullptr) {
      vmaDestroyAllocator(allocator);
    }
  }
};

namespace {

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

  auto impl = std::make_shared<Impl>();
  VR_VK_TRY(vmaCreateAllocator(&info, &impl->allocator));
  impl->queue_family_count = static_cast<std::uint32_t>(
      detail::queue_families(device.physical_device()).size());

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
  std::uint32_t distinct[BufferDesc::kMaxQueueFamilies]{};
  const std::uint32_t distinct_count =
      distinct_queue_families(desc.queue_families, desc.queue_family_count,
                              distinct, BufferDesc::kMaxQueueFamilies);
  if (distinct_count > BufferDesc::kMaxQueueFamilies) {
    return Status::invalid_argument(
        "Allocator::create_buffer: more than BufferDesc::kMaxQueueFamilies (4) "
        "distinct queue families");
  }

  // Every index must name a family this device actually has. Checked in both
  // modes -- an out-of-range index is a caller bug either way -- but it is
  // CONCURRENT where it turns into undefined behaviour: naming a nonexistent
  // family violates VUID-VkBufferCreateInfo-sharingMode-01419, and VMA still
  // returns VK_SUCCESS, so with layers off (the shipping configuration, and the
  // only one on iOS) nothing reports it. The shape that hits this is an app
  // hardcoding its compute and render families -- valid on Apple's four, wrong
  // on a single-family driver such as lavapipe, which both Linux CI legs run.
  for (std::uint32_t i = 0; i < distinct_count; ++i) {
    if (distinct[i] >= impl_->queue_family_count) {
      return Status::invalid_argument(
          "Allocator::create_buffer: queue_families names a family index the "
          "device does not have");
    }
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
  // both without VMA appearing in buffer.hpp. The captured *Impl reference* is
  // what keeps the VmaAllocator alive for as long as this Buffer can free
  // through it -- see the note on Impl.
  return Buffer(buffer, desc.size, desc.usage, buffer_info.sharingMode,
                out_info.pMappedData, [impl = impl_, buffer, allocation]() {
                  vmaDestroyBuffer(impl->allocator, buffer, allocation);
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
