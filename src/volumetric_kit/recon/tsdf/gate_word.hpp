// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A gate word, as TsdfIntegrator::record_integrate and
// ChangeDetector::record_detect take one: checked once, here, for both.
// Internal to recon_tsdf.

#include <cstdint>
#include <string>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"

namespace volumetric_kit::recon::tsdf::detail {

// The word's index in its buffer, once `gate` is a complete, aligned word in
// a known device-local storage buffer whose whole size a binding may cover:
// a pass binds the whole buffer, which also keeps the index below a kernel's
// no-gate sentinel (0xFFFFFFFF). `what` names the caller in the message.
inline core::Result<std::uint32_t> gate_word(
    const volumetric_kit::grid::volume::DeviceCount& gate,
    const std::string& what, VkDeviceSize max_storage_buffer_range) {
  constexpr VkDeviceSize word_bytes = sizeof(std::uint32_t);
  if (gate.buffer == nullptr || gate.offset % word_bytes != 0 ||
      gate.buffer->size() < word_bytes ||
      gate.offset > gate.buffer->size() - word_bytes) {
    return core::Status::invalid_argument(
        what + ": the gate's word is missing, misaligned or past its buffer");
  }
  const std::string name = what + ": gate";
  VKC_TRY(core::StorageInput(*gate.buffer)
              .check(name.c_str(), gate.offset + word_bytes));
  VKC_TRY(core::check_storage_buffer_range(name.c_str(), gate.buffer->size(),
                                           max_storage_buffer_range));
  return static_cast<std::uint32_t>(gate.offset / word_bytes);
}

}  // namespace volumetric_kit::recon::tsdf::detail
