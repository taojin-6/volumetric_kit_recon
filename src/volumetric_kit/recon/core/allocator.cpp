// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/allocator.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include <vk_mem_alloc.h>

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
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(device.physical_device(), &props);
  const std::uint32_t effective = std::min(instance_version, props.apiVersion);

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

  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = desc.size;
  buffer_info.usage = desc.usage;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

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
  return Buffer(buffer, desc.size, out_info.pMappedData,
                [allocator, buffer, allocation]() {
                  vmaDestroyBuffer(allocator, buffer, allocation);
                });
}

}  // namespace volumetric_kit::recon
