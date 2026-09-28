// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A device-local buffer's contents on the host and back, for a test, through a
// CommandBatch.

#include <cstddef>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/command_batch.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace vr_test {

namespace vr = volumetric_kit::recon;

// The device and allocator a test runs on, handed to its helpers. Held by
// reference: main keeps both where it created them.
struct Gpu {
  vr::Device& device;
  vr::Allocator& allocator;
};

// The first `count` elements of `buffer`, which needs TRANSFER_SRC usage.
template <typename T>
vr::Result<std::vector<T>> read_back(const vr::Device& device,
                                     vr::Allocator& allocator,
                                     const vr::Buffer& buffer,
                                     std::size_t count) {
  std::vector<T> out(count);
  vr::CommandBatch batch(device, allocator);
  VR_TRY(
      batch.readback(buffer, 0, VkDeviceSize(count) * sizeof(T), out.data()));
  VR_TRY(batch.submit());
  return out;
}

// Write `data` over the start of `buffer`, which needs TRANSFER_DST usage.
template <typename T>
vr::Status write_back(const vr::Device& device, vr::Allocator& allocator,
                      const vr::Buffer& buffer, const std::vector<T>& data) {
  vr::CommandBatch batch(device, allocator);
  VR_TRY(batch.upload(buffer, 0, data.data(),
                      VkDeviceSize(data.size()) * sizeof(T)));
  return batch.submit();
}

}  // namespace vr_test
