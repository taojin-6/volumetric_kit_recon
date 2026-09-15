// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/rgbd_frame.hpp
/// @brief A posed RGB-D frame with storage of its own: what a reader decodes
///        into, and what a consumer keeps. `sensor::CapturedFrame` is the
///        borrowed view of one.
///
/// The contract hands out a view whose pixels the device may recycle on the
/// next `poll()` -- any next poll, the empty one that ends a replay included
/// -- and says to copy anything that must outlive it. This is the type that
/// copy lands in, and the type `ReplicaCapture` decodes its frames into, so
/// the two viewers keep a keyframe the same way (`fuse_render` the frame it
/// textures its PNG with, `fuse_viewer` the newest fused frame for its final
/// texture pass) and neither reaches into a capture's buffers after the poll
/// that freed them. Lossless against the contract: every field of the view
/// is here, the encoding declaration and the timestamp included, so a path
/// written over the view works unchanged over the frame.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief A posed RGB-D frame that owns its pixels.
struct RgbdFrame {
  /// Depth in metres, `depth_camera.width * height`, row-major.
  std::vector<float> depth;
  /// Packed RGB (`R | G<<8 | B<<16`), `color_camera.width * height`; empty
  /// when the frame carries no colour.
  std::vector<std::uint32_t> color;
  /// Depth intrinsics, size, range and camera->world pose.
  vr::DepthCameraParams depth_camera{};
  /// Colour intrinsics, size and pose. Meaningful only with @ref color.
  vr::ColorCameraParams color_camera{};
  /// What @ref color is encoded as; defaults to canonical.
  vr::ColorEncoding color_encoding{};
  /// Device timestamp in nanoseconds; 0 when the source reports none.
  std::uint64_t timestamp_ns = 0;

  /// @brief Copy @p frame in: its depth, its colour if it carries any, and
  ///        every field of the header. The buffers are sized from the cameras
  ///        the view is stamped with, as the contract defines their extent.
  ///        Capacity is reused, so a consumer that retains every frame pays a
  ///        copy (~0.1 ms for a 1200x680 RGB-D frame at -O2), not an
  ///        allocation, per frame.
  void assign(const vr::sensor::CapturedFrame& frame) {
    const std::size_t depth_px =
        static_cast<std::size_t>(frame.depth_camera.width) *
        frame.depth_camera.height;
    depth.assign(frame.depth, frame.depth + depth_px);
    if (frame.has_color()) {
      const std::size_t color_px =
          static_cast<std::size_t>(frame.color_camera.width) *
          frame.color_camera.height;
      color.assign(frame.color, frame.color + color_px);
    } else {
      color.clear();
    }
    depth_camera = frame.depth_camera;
    color_camera = frame.color_camera;
    color_encoding = frame.color_encoding;
    timestamp_ns = frame.timestamp_ns;
  }

  /// @brief Forget the frame and release nothing: the capacity stays for the
  ///        next @ref assign.
  void clear() noexcept {
    depth.clear();
    color.clear();
    depth_camera = vr::DepthCameraParams{};
    color_camera = vr::ColorCameraParams{};
    color_encoding = vr::ColorEncoding{};
    timestamp_ns = 0;
  }

  /// @return `true` while this holds no frame (a frame always has depth).
  bool empty() const noexcept { return depth.empty(); }

  /// @return The contract's view of this frame, over this object's storage:
  ///         valid until the next @ref assign, @ref clear, move or
  ///         destruction. Built on each call rather than cached, so the
  ///         defaulted copy and move operations stay correct -- a stored
  ///         pointer would dangle the moment this object moved.
  vr::sensor::CapturedFrame view() const noexcept {
    vr::sensor::CapturedFrame frame{};
    frame.depth = depth.empty() ? nullptr : depth.data();
    frame.color = color.empty() ? nullptr : color.data();
    frame.depth_camera = depth_camera;
    frame.color_camera = color_camera;
    frame.color_encoding = color_encoding;
    frame.timestamp_ns = timestamp_ns;
    return frame;
  }
};

}  // namespace vr_example
