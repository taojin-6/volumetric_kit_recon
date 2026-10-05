// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A device-local buffer's contents on the host and back, for a test, through a
// CommandBatch.

#include <cstddef>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"

namespace vr_test {

namespace vkc = volumetric_kit::core;

// The device and allocator a test runs on, handed to its helpers. Held by
// reference: main keeps both where it created them.
struct Gpu {
  vkc::Device& device;
  vkc::Allocator& allocator;
};

// A genuinely resident input, uploaded at the test boundary.
inline vkc::Result<vkc::Buffer> upload_device_buffer(const vkc::Device& device,
                                                     vkc::Allocator& allocator,
                                                     const void* data,
                                                     VkDeviceSize bytes) {
  VKC_ASSIGN(vkc::Buffer buffer, vkc::device_storage_buffer(allocator, bytes));
  vkc::CommandBatch batch(device, allocator);
  VKC_TRY(batch.upload(buffer, 0, data, bytes));
  VKC_TRY(batch.submit());
  return buffer;
}

// The first `count` elements of `buffer`, which needs TRANSFER_SRC usage.
template <typename T>
vkc::Result<std::vector<T>> read_back(const vkc::Device& device,
                                      vkc::Allocator& allocator,
                                      const vkc::Buffer& buffer,
                                      std::size_t count) {
  std::vector<T> out(count);
  vkc::CommandBatch batch(device, allocator);
  VKC_TRY(
      batch.readback(buffer, 0, VkDeviceSize(count) * sizeof(T), out.data()));
  VKC_TRY(batch.submit());
  return out;
}

// Write `data` over the start of `buffer`, which needs TRANSFER_DST usage.
template <typename T>
vkc::Status write_back(const vkc::Device& device, vkc::Allocator& allocator,
                       const vkc::Buffer& buffer, const std::vector<T>& data) {
  vkc::CommandBatch batch(device, allocator);
  VKC_TRY(batch.upload(buffer, 0, data.data(),
                       VkDeviceSize(data.size()) * sizeof(T)));
  return batch.submit();
}

}  // namespace vr_test
