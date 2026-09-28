// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/raw_frame.hpp
/// @brief A frame as the cameras captured it, before undistortion and colour
///        conversion: what a driver hands to `sensor/utils`'s GPU pass.
///
/// A @ref CapturedFrame is already pinhole and R'G'B'; this is the step before
/// it, with each camera's lens, raw depth units and the decoded Y'CbCr planes.
/// The two cameras keep their own intrinsics and poses: nothing registers one
/// to the other, and `tsdf` fuses them as they are.
///
/// Like the capture contract it reaches `core` alone and no Vulkan, so a
/// driver produces it without compiling against a GPU API.

#include <cstddef>
#include <cstdint>

#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/sensor/lens.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief An 8-bit Y'CbCr 4:2:0 picture in three planes, and the matrix and
///        range it was coded with.
///
/// Chroma is sited as H.265 and MPEG-2 place it by default: horizontally with
/// the even luma columns, vertically between two luma rows.
struct YuvImage {
  /// Y, then Cb and Cr at half size (rounded up). Borrowed.
  const std::uint8_t* plane[3] = {};
  std::size_t stride[3] = {};  ///< Bytes per row of each plane.
  std::uint32_t width = 0;     ///< Luma width (pixels).
  std::uint32_t height = 0;    ///< Luma height (pixels).
  /// The matrix's red and blue weights: BT.601 is 0.299 and 0.114, BT.709
  /// 0.2126 and 0.0722.
  float kr = 0.299f;
  float kb = 0.114f;       ///< See @ref kr.
  bool full_range = true;  ///< Y in 0..255 rather than 16..235.
};

/// @brief One RGB-D frame as the cameras captured it. A non-owning view: the
///        driver's buffers hold until its next call.
struct RawFrame {
  /// Row-major depth in the sensor's units, `depth_camera.width * height`
  /// samples; 0 is "no return".
  const std::uint16_t* depth = nullptr;
  float metres_per_unit = 0.001f;  ///< What one depth unit is, in metres.
  LensCamera depth_camera{};       ///< The depth camera, lens included.
  Mat4f depth_cam_to_world = Mat4f(1.0f);  ///< The depth camera's pose.
  /// Nearer samples are dropped (metres). Set it: the pass needs
  /// `0 < min_depth < max_depth`, since 0 is "no return", and refuses these
  /// zeros rather than fuse nothing.
  float min_depth = 0.0f;
  float max_depth = 0.0f;  ///< Farther samples are dropped (metres).

  /// The colour picture; `color.plane[0]` null when the frame has none. Its
  /// size is @ref color_camera's.
  YuvImage color{};
  LensCamera color_camera{};  ///< The colour camera, lens included.
  Mat4f color_cam_to_world = Mat4f(1.0f);  ///< The colour camera's pose.
  /// What the R'G'B' the matrix gives is encoded as; the pass converts only
  /// from an encoding @ref is_canonical accepts.
  ColorEncoding color_encoding{};

  std::uint64_t timestamp_ns = 0;  ///< As @ref CapturedFrame::timestamp_ns.

  /// @return `true` if this frame carries colour.
  bool has_color() const noexcept { return color.plane[0] != nullptr; }
};

}  // namespace volumetric_kit::recon::sensor
