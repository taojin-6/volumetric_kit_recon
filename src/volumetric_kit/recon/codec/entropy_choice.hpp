// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file entropy_choice.hpp
/// @brief Where a frame's rANS coding runs (@ref EntropyCoding): the policy
///        the `Encoder` and the `Decoder` share.
///
/// Internal (under src/, never installed). Each caller also keeps its device
/// coder's build failure, so `kAuto` tries to build it only once.

#include <cstdint>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/codec/codec_params.hpp"

namespace volumetric_kit::recon::codec::detail {

/// @return Whether a frame of @p segments is coded on the device: always for
///         `kDevice`, from @p min_segments for `kAuto`, never for `kHost`.
inline bool codes_on_device(EntropyCoding coding, std::uint64_t segments,
                            std::uint32_t min_segments) {
  return coding == EntropyCoding::kDevice ||
         (coding == EntropyCoding::kAuto && segments >= min_segments);
}

/// @return Whether the host codes a frame the device refused with @p s:
///         under `kAuto`, one the device cannot hold (past
///         `maxStorageBufferRange` or a segment limit, or past free memory).
///         Any other failure is the device's, and is reported.
inline bool retry_on_host(EntropyCoding coding, const core::Status& s) {
  return coding == EntropyCoding::kAuto &&
         (s.domain() == core::Status::Code::InvalidArgument ||
          s.domain() == core::Status::Code::OutOfMemory);
}

}  // namespace volumetric_kit::recon::codec::detail
