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
/// driver produces it without compiling against a GPU API: a picture a
/// decoder left on the device is named by `core`'s `Buffer`, declared here.

#include <cstddef>
#include <cstdint>
#include <memory>

#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/sensor/lens.hpp"

namespace volumetric_kit::recon {
class Buffer;
}  // namespace volumetric_kit::recon

namespace volumetric_kit::recon::sensor {

/// @brief How a picture's chroma is laid out.
enum class YuvLayout : std::uint8_t {
  I420,  ///< Cb and Cr each in a plane of its own.
  Nv12,  ///< Cb and Cr interleaved in one plane, Cb first.
};

/// @brief An 8-bit Y'CbCr 4:2:0 picture, on the host or already on the
///        device, and the matrix and range it was coded with.
///
/// Chroma is sited as H.265 and MPEG-2 place it by default: horizontally with
/// the even luma columns, vertically between two luma rows. The planes are Y,
/// then Cb and Cr at half size (rounded up) for I420, or Y then CbCr for
/// NV12, whose chroma rows hold both samples of each pair. They are either
/// host memory in @ref plane or, where a decoder left its picture on the GPU,
/// ranges of @ref device; never both.
struct YuvImage {
  YuvLayout layout = YuvLayout::I420;  ///< How the chroma is laid out.
  /// The planes on the host; `plane[2]` is unused for NV12. Borrowed.
  const std::uint8_t* plane[3] = {};
  std::size_t stride[3] = {};  ///< Bytes per row of each plane.
  /// Or the planes on the device: a storage buffer on the device the frame
  /// is prepared on, holding plane `p` at byte @ref offset `[p]`. The frame
  /// holds it, so the decoder cannot reuse it while the frame is prepared.
  std::shared_ptr<const Buffer> device;
  std::uint64_t offset[3] = {};  ///< Each plane's byte offset in @ref device.
  std::uint32_t width = 0;       ///< Luma width (pixels).
  std::uint32_t height = 0;      ///< Luma height (pixels).
  /// The matrix's red and blue weights: BT.601 is 0.299 and 0.114, BT.709
  /// 0.2126 and 0.0722.
  float kr = 0.299f;
  float kb = 0.114f;       ///< See @ref kr.
  bool full_range = true;  ///< Y in 0..255 rather than 16..235.
};

/// @brief One RGB-D frame as the cameras captured it. A view: the driver's
///        host buffers hold until its next call, and a device picture is
///        held by the frame.
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

  /// The colour picture; with neither `color.plane[0]` nor `color.device`
  /// set, the frame has none. Its size is @ref color_camera's.
  YuvImage color{};
  LensCamera color_camera{};  ///< The colour camera, lens included.
  Mat4f color_cam_to_world = Mat4f(1.0f);  ///< The colour camera's pose.
  /// What the R'G'B' the matrix gives is encoded as; the pass converts only
  /// from an encoding @ref is_canonical accepts.
  ColorEncoding color_encoding{};

  std::uint64_t timestamp_ns = 0;  ///< As @ref CapturedFrame::timestamp_ns.

  /// @return `true` if this frame carries colour.
  bool has_color() const noexcept {
    return color.plane[0] != nullptr || color.device != nullptr;
  }
};

}  // namespace volumetric_kit::recon::sensor
