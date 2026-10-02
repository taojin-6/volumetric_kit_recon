// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file dct_blocks.hpp
/// @brief What the block transform produces and consumes, on its own so the
///        host-only frame format (@ref bitstream.hpp) can name it without
///        reaching the transform's Vulkan side.
///
/// Internal (under src/, never installed), like the transform and the frame.

#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/codec/codec_params.hpp"

namespace volumetric_kit::recon::codec::detail {

/// @brief One block list's transform: what @ref DctTransform::forward produces
///        and @ref DctTransform::inverse consumes.
///
/// Carries the params and the `trunc_dist` the coefficients were made with, so
/// the inverse checks them against its grid rather than trusting the caller to
/// pass matching ones. The steps are fractions of `trunc_dist`, so decoding
/// into a grid with another band would rescale every SDF and report success.
struct DctBlocks {
  /// The coefficient count and steps the coefficients were quantized with.
  CodecParams params;
  /// The grid's `trunc_dist`, which the SDF was divided by. The inverse
  /// compares it exactly: a frame carries it bit for bit.
  float trunc_dist = 0.0f;
  /// `count * params.coefficient_count` values, by list position: coefficient
  /// `j` of entry `i` is `coefficients[i * coefficient_count + j]`. Each is
  /// within ±@ref kMaxQuantizedMagnitude, so 16 bits hold it, which halves the
  /// transform's largest buffers against 32.
  std::vector<std::int16_t> coefficients;
  /// `count * kMaskWordsPerBlock` words: entry `i`'s start at
  /// `masks[i * kMaskWordsPerBlock]`, and its voxel `v` (see @ref voxel_index)
  /// is bit `v % kMaskWordBits` of word `v / kMaskWordBits`.
  std::vector<std::uint32_t> masks;
};

}  // namespace volumetric_kit::recon::codec::detail
