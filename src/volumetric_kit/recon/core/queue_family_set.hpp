// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file queue_family_set.hpp
/// @brief Internal (non-installed) reduction of a queue-family list to its
///        distinct entries -- the rule @ref volumetric_kit::recon::BufferDesc
///        "BufferDesc::queue_families" describes, on its own so a host test can
///        pin it.
///
/// This header stays under `src/` and is **not** installed: its location is the
/// access control, so the function needs no `detail` namespace to say it is not
/// public API, and it commits nothing to the shared library's exported ABI. The
/// test reaches it the same way, by adding this directory to its include path.
/// Mirrors `vk_physical_device.hpp`, the tier's other internal header.

#include <cstdint>

namespace volumetric_kit::recon {

/// @brief Reduce @p families to its distinct entries, in first-seen order.
///
/// Pulled out of `Allocator::create_buffer` because of its failure mode: get
/// the reduction wrong and Vulkan rejects the buffer only when a validation
/// layer happens to be installed *and* something turns that diagnostic into a
/// failure, and otherwise builds a silently wrong sharing mode -- the same
/// "correct on the machine that ran it" hazard the camera and colour
/// conventions are kept as pure arithmetic to avoid.
///
/// @param families      Entries to reduce; may be null only when @p count is 0.
/// @param count         Number of entries in @p families.
/// @param out           Receives the distinct entries; may be null only when
///                      @p out_capacity is 0.
/// @param out_capacity  How many entries @p out holds.
/// @return The distinct count, or `out_capacity + 1` when more distinct
///         families were found than @p out can hold -- a value the caller can
///         only treat as an error, since the reduction is then incomplete (it
///         stops at the first entry past capacity and does not report the true
///         total).
inline std::uint32_t distinct_queue_families(const std::uint32_t* families,
                                             std::uint32_t count,
                                             std::uint32_t* out,
                                             std::uint32_t out_capacity) {
  std::uint32_t distinct = 0;
  for (std::uint32_t i = 0; i < count; ++i) {
    bool seen = false;
    for (std::uint32_t j = 0; j < distinct; ++j) {
      if (out[j] == families[i]) {
        seen = true;
        break;
      }
    }
    if (seen) {
      continue;
    }
    if (distinct == out_capacity) {
      return out_capacity + 1;
    }
    out[distinct++] = families[i];
  }
  return distinct;
}

}  // namespace volumetric_kit::recon
