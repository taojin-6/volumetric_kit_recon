// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A device-local buffer's contents on the host, for a test: the buffer cannot
// be mapped, so it is copied into a host-visible one first. The source needs
// TRANSFER_SRC usage, which GpuFramePrep's outputs carry for this.

#include <cstddef>
#include <cstring>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace vr_test {

namespace vr = volumetric_kit::recon;

// The first `count` elements of `buffer`.
template <typename T>
vr::Result<std::vector<T>> read_back(const vr::Device& device,
                                     vr::Allocator& allocator,
                                     const vr::Buffer& buffer,
                                     std::size_t count) {
  const VkDeviceSize bytes = VkDeviceSize(count) * sizeof(T);
  vr::BufferDesc desc;
  desc.size = bytes;
  desc.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  desc.memory = vr::MemoryUsage::HostVisible;
  desc.mapped = true;
  VR_ASSIGN(vr::Buffer staging, allocator.create_buffer(desc));
  VR_TRY(device.submit_single_time([&](VkCommandBuffer cmd) {
    VkBufferCopy region{};
    region.size = bytes;
    vkCmdCopyBuffer(cmd, buffer.handle(), staging.handle(), 1, &region);
    VkMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_HOST_BIT, 0, 1, &barrier, 0, nullptr,
                         0, nullptr);
  }));
  std::vector<T> out(count);
  std::memcpy(out.data(), staging.mapped(), static_cast<std::size_t>(bytes));
  return out;
}

}  // namespace vr_test
