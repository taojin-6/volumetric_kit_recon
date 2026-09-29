// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/external_memory.hpp"

#include <cstdint>

#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {

Result<ExportedBuffer> create_exported_buffer(const Device& device,
                                              VkDeviceSize bytes) {
  if (!device.exports_memory()) {
    return Status::unsupported(
        "create_exported_buffer: the device does not export memory "
        "(VK_KHR_external_memory_fd)");
  }
  if (bytes == 0) {
    return Status::invalid_argument("create_exported_buffer: size is zero");
  }
  const VkDevice vk = device.handle();
  constexpr VkExternalMemoryHandleTypeFlags kHandle =
      VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
  constexpr VkBufferUsageFlags kUsage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                        VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                        VK_BUFFER_USAGE_TRANSFER_DST_BIT;

  // The extension alone does not promise an exportable storage buffer.
  VkPhysicalDeviceExternalBufferInfo query{};
  query.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_BUFFER_INFO;
  query.usage = kUsage;
  query.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
  VkExternalBufferProperties can{};
  can.sType = VK_STRUCTURE_TYPE_EXTERNAL_BUFFER_PROPERTIES;
  vkGetPhysicalDeviceExternalBufferProperties(device.physical_device(), &query,
                                              &can);
  if ((can.externalMemoryProperties.externalMemoryFeatures &
       VK_EXTERNAL_MEMORY_FEATURE_EXPORTABLE_BIT) == 0) {
    return Status::unsupported(
        "create_exported_buffer: the device cannot export a storage buffer "
        "as a file descriptor");
  }

  VkExternalMemoryBufferCreateInfo external{};
  external.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO;
  external.handleTypes = kHandle;
  VkBufferCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  info.pNext = &external;
  info.size = bytes;
  info.usage = kUsage;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VkBuffer buffer = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateBuffer(vk, &info, nullptr, &buffer));

  VkMemoryRequirements needs{};
  vkGetBufferMemoryRequirements(vk, buffer, &needs);
  VkPhysicalDeviceMemoryProperties memory{};
  vkGetPhysicalDeviceMemoryProperties(device.physical_device(), &memory);
  std::uint32_t type = memory.memoryTypeCount;
  for (std::uint32_t i = 0; i < memory.memoryTypeCount; ++i) {
    if ((needs.memoryTypeBits & (1u << i)) != 0 &&
        (memory.memoryTypes[i].propertyFlags &
         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0) {
      type = i;
      break;
    }
  }
  if (type == memory.memoryTypeCount) {
    vkDestroyBuffer(vk, buffer, nullptr);
    return Status::unsupported(
        "create_exported_buffer: no device-local memory the buffer can use");
  }

  VkExportMemoryAllocateInfo exported{};
  exported.sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO;
  exported.handleTypes = kHandle;
  VkMemoryDedicatedAllocateInfo dedicated{};
  dedicated.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
  dedicated.pNext = &exported;
  dedicated.buffer = buffer;
  VkMemoryAllocateInfo alloc{};
  alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc.pNext = &dedicated;
  alloc.allocationSize = needs.size;
  alloc.memoryTypeIndex = type;
  VkDeviceMemory backing = VK_NULL_HANDLE;
  VkResult result = vkAllocateMemory(vk, &alloc, nullptr, &backing);
  if (result == VK_SUCCESS) result = vkBindBufferMemory(vk, buffer, backing, 0);
  int fd = -1;
  if (result == VK_SUCCESS) result = device.memory_fd(backing, &fd);
  if (result != VK_SUCCESS) {
    vkDestroyBuffer(vk, buffer, nullptr);
    if (backing != VK_NULL_HANDLE) vkFreeMemory(vk, backing, nullptr);
    return vk_error(result, "create_exported_buffer");
  }

  ExportedBuffer out;
  out.buffer = Buffer(buffer, bytes, kUsage, VK_SHARING_MODE_EXCLUSIVE, nullptr,
                      [vk, buffer, backing]() {
                        vkDestroyBuffer(vk, buffer, nullptr);
                        vkFreeMemory(vk, backing, nullptr);
                      });
  out.fd = fd;
  out.memory_size = needs.size;
  return out;
}

}  // namespace volumetric_kit::recon
