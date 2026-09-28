// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A device-local buffer's contents on the host, for a test, staged through a
// CommandBatch. The source needs TRANSFER_SRC usage, which GpuFramePrep's
// outputs carry for this.

#include <cstddef>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/command_batch.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace vr_test {

namespace vr = volumetric_kit::recon;

// The first `count` elements of `buffer`.
template <typename T>
vr::Result<std::vector<T>> read_back(const vr::Device& device,
                                     vr::Allocator& allocator,
                                     const vr::Buffer& buffer,
                                     std::size_t count) {
  VR_ASSIGN(vr::StagingArena staging, vr::StagingArena::create(allocator));
  std::vector<T> out(count);
  vr::CommandBatch batch(device, staging);
  VR_TRY(
      batch.readback(buffer, 0, VkDeviceSize(count) * sizeof(T), out.data()));
  VR_TRY(batch.submit());
  return out;
}

}  // namespace vr_test
