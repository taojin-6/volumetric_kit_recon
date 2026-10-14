// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A host frame's entries where the device holds a frame's, for the codec's
// device tests: as the device frame reader leaves them, and as the device
// frame writer reads the transform's.

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "bitstream.hpp"
#include "buffer_readback.hpp"
#include "codec_frames.hpp"
#include "dct_transform.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"

namespace codec_frames {

// The buffers by pointer, so a moved Resident's blocks still point at them.
struct Resident {
  d::ResidentBlocks blocks;
  std::unique_ptr<vkc::Buffer> list, masks, coefficients, modes;
};

// @p entries by stream position, two coefficients a word and an odd K's pad
// half @p pad; the first `modes.size()` a P-frame's reference entries, with
// @p modes, and the new blocks' @p coords in a list.
inline vkc::Result<Resident> resident(
    const vr_test::Gpu& gpu, const d::DctBlocks& entries,
    const std::vector<d::BlockMode>& modes = {},
    const std::vector<vr::Vec3i>& coords = {}, std::uint16_t pad = 0) {
  const auto upload =
      [&gpu](const void* bytes,
             VkDeviceSize size) -> vkc::Result<std::unique_ptr<vkc::Buffer>> {
    VKC_ASSIGN(vkc::Buffer buffer, vr_test::upload_device_buffer(
                                       gpu.device, gpu.allocator, bytes, size));
    return std::make_unique<vkc::Buffer>(std::move(buffer));
  };
  Resident r;
  r.blocks.count = static_cast<std::uint32_t>(entries.masks.size() /
                                              codec::kMaskWordsPerBlock);
  r.blocks.reference_count = static_cast<std::uint32_t>(modes.size());
  r.blocks.coefficient_count = entries.params.coefficient_count;
  if (r.blocks.count != 0) {
    const std::vector<std::uint32_t> packed = pack(entries, pad);
    VKC_ASSIGN(r.masks, upload(entries.masks.data(),
                               entries.masks.size() * sizeof(std::uint32_t)));
    VKC_ASSIGN(r.coefficients,
               upload(packed.data(), packed.size() * sizeof(std::uint32_t)));
    r.blocks.masks = r.masks.get();
    r.blocks.coefficients = r.coefficients.get();
  }
  if (!coords.empty()) {
    std::vector<vr::volume::BlockIndex> list(coords.size());
    for (std::size_t q = 0; q < coords.size(); ++q) list[q].coord = coords[q];
    VKC_ASSIGN(r.list, upload(list.data(),
                              list.size() * sizeof(vr::volume::BlockIndex)));
    r.blocks.list = r.list.get();
  }
  if (!modes.empty()) {
    std::vector<std::uint32_t> words(modes.size());
    for (std::size_t e = 0; e < modes.size(); ++e) {
      words[e] = static_cast<std::uint32_t>(modes[e]);
    }
    VKC_ASSIGN(r.modes,
               upload(words.data(), words.size() * sizeof(std::uint32_t)));
    r.blocks.modes = r.modes.get();
  }
  return r;
}

}  // namespace codec_frames
