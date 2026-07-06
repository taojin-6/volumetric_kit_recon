// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/descriptor.hpp"

#include "volumetric_kit/recon/core/check.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {

Result<DescriptorSetLayout> DescriptorSetLayout::create(
    VkDevice device, const VkDescriptorSetLayoutBinding* bindings,
    std::uint32_t count) {
  // Argument checks run before the device check so a no-device unit test still
  // exercises them meaningfully.
  if (count > 0 && bindings == nullptr) {
    return Status::invalid_argument(
        "DescriptorSetLayout::create: null bindings with a non-zero count");
  }
  if (device == VK_NULL_HANDLE) {
    return Status::invalid_argument(
        "DescriptorSetLayout::create: device is null");
  }

  VkDescriptorSetLayoutCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  info.bindingCount = count;
  info.pBindings = bindings;

  VkDescriptorSetLayout handle = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateDescriptorSetLayout(device, &info, nullptr, &handle));

  DescriptorSetLayout result;
  result.layout_ =
      UniqueHandle<VkDescriptorSetLayout, vkDestroyDescriptorSetLayout>(device,
                                                                        handle);
  return result;
}

Result<DescriptorPool> DescriptorPool::create(VkDevice device,
                                              const VkDescriptorPoolSize* sizes,
                                              std::uint32_t size_count,
                                              std::uint32_t max_sets) {
  if (max_sets == 0) {
    return Status::invalid_argument("DescriptorPool::create: max_sets is zero");
  }
  if (size_count == 0 || sizes == nullptr) {
    return Status::invalid_argument(
        "DescriptorPool::create: pool sizes must be non-empty and non-null");
  }
  if (device == VK_NULL_HANDLE) {
    return Status::invalid_argument("DescriptorPool::create: device is null");
  }

  VkDescriptorPoolCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  info.maxSets = max_sets;
  info.poolSizeCount = size_count;
  info.pPoolSizes = sizes;

  VkDescriptorPool handle = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateDescriptorPool(device, &info, nullptr, &handle));

  DescriptorPool result;
  result.pool_ =
      UniqueHandle<VkDescriptorPool, vkDestroyDescriptorPool>(device, handle);
  return result;
}

Result<DescriptorSet> DescriptorPool::allocate(VkDescriptorSetLayout layout) {
  if (!pool_.valid()) {
    return Status::invalid_argument(
        "DescriptorPool::allocate: pool is moved-from / empty");
  }
  if (layout == VK_NULL_HANDLE) {
    return Status::invalid_argument("DescriptorPool::allocate: layout is null");
  }

  VkDescriptorSetAllocateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  info.descriptorPool = pool_.get();
  info.descriptorSetCount = 1;
  info.pSetLayouts = &layout;

  VkDescriptorSet set = VK_NULL_HANDLE;
  VR_VK_TRY(vkAllocateDescriptorSets(pool_.device(), &info, &set));
  return DescriptorSet(pool_.device(), set);
}

void DescriptorSet::write_storage_buffer(std::uint32_t binding, VkBuffer buffer,
                                         VkDeviceSize offset,
                                         VkDeviceSize range) const {
  VR_CHECK(valid(), "DescriptorSet::write_storage_buffer on an empty set");

  VkDescriptorBufferInfo buffer_info{};
  buffer_info.buffer = buffer;
  buffer_info.offset = offset;
  buffer_info.range = range;

  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write.dstSet = set_;
  write.dstBinding = binding;
  write.dstArrayElement = 0;
  write.descriptorCount = 1;
  write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  write.pBufferInfo = &buffer_info;

  vkUpdateDescriptorSets(device_, 1, &write, 0, nullptr);
}

}  // namespace volumetric_kit::recon
