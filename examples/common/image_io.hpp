// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/image_io.hpp
/// @brief Minimal RGB / depth image decode for the dataset examples, backed by
///        the vendored single-header stb_image.

#include <cstdint>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/result.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief Decode a color image (JPEG/PNG) to packed RGB, one `uint32` per
/// pixel.
///
/// The byte order matches the `tsdf`/`mesh` color attribute: R in the low byte,
/// then G, then B (`R | G<<8 | B<<16`), alpha unused. Row-major, top-left
/// origin.
/// @param path        Image file path.
/// @param expected_w  Expected width; a mismatch is an error (the intrinsics
///                    would be wrong for it).
/// @param expected_h  Expected height.
/// @return `expected_w * expected_h` packed pixels, or a non-OK @ref
///         vr::Status if the file cannot be decoded or its size differs.
vr::Result<std::vector<std::uint32_t>> load_color_packed(
    const std::string& path, std::uint32_t expected_w,
    std::uint32_t expected_h);

/// @brief Decode a 16-bit grayscale depth PNG to float metres.
///
/// The raw `uint16` sample is divided by @p depth_scale (metres = raw /
/// depth_scale, e.g. 6553.5 for Replica) -- matching the float-metres depth the
/// `volume`/`tsdf` tiers expect. A zero raw sample (no return) stays 0.0 and is
/// rejected downstream by the near/far range.
/// @param path         Depth PNG path (16-bit single channel).
/// @param expected_w   Expected width.
/// @param expected_h   Expected height.
/// @param depth_scale  Units-per-metre divisor (> 0).
/// @return `expected_w * expected_h` depths in metres, or a non-OK @ref
///         vr::Status on a decode / size / channel-depth mismatch.
vr::Result<std::vector<float>> load_depth_metres(const std::string& path,
                                                 std::uint32_t expected_w,
                                                 std::uint32_t expected_h,
                                                 float depth_scale);

}  // namespace vr_example
