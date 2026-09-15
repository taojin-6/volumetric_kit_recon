// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/owned_frame.hpp
/// @brief A `sensor::CapturedFrame` with storage of its own -- the copy a
///        consumer keeps of a frame it needs past the capture's next poll.
///
/// The contract hands out a view whose pixels the device may recycle on the
/// next `poll()` -- any next poll, the empty one that ends a replay included
/// -- and says to copy anything that must outlive it. This is that copy, so
/// the two viewers keep a keyframe the same way (`fuse_render` the frame it
/// textures its PNG with, `fuse_viewer` the newest fused frame for its final
/// texture pass) and neither reaches into a capture's buffers after the poll
/// that freed them. Lossless against the contract: every field crosses,
/// including the encoding declaration and the timestamp, so a path written
/// over the view works unchanged over the copy.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/sensor/camera_capture.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief A posed RGB-D frame that owns its pixels.
class OwnedFrame {
 public:
  /// @brief Copy @p frame -- its depth, its colour if it carries any, and every
  ///        field of the header -- into this object's storage.
  ///
  /// The buffers are sized from the cameras the frame is stamped with, as the
  /// contract defines their extent. Capacity is reused, so a consumer that
  /// retains every frame pays a copy (~0.1 ms for a 1200x680 RGB-D frame at
  /// -O2), not an allocation, per frame.
  void assign(const vr::sensor::CapturedFrame& frame) {
    const std::size_t depth_px =
        static_cast<std::size_t>(frame.depth_camera.width) *
        frame.depth_camera.height;
    depth_.assign(frame.depth, frame.depth + depth_px);
    if (frame.has_color()) {
      const std::size_t color_px =
          static_cast<std::size_t>(frame.color_camera.width) *
          frame.color_camera.height;
      color_.assign(frame.color, frame.color + color_px);
    } else {
      color_.clear();
    }
    header_ = frame;
    has_frame_ = true;
  }

  /// @brief Forget the frame and release nothing: the capacity stays for the
  ///        next @ref assign.
  void clear() noexcept {
    has_frame_ = false;
    header_ = vr::sensor::CapturedFrame{};
  }

  /// @return `true` between an @ref assign and a @ref clear.
  bool has_frame() const noexcept { return has_frame_; }

  /// @return The frame as the contract type, over this object's storage:
  ///         valid until the next @ref assign, @ref clear, move or
  ///         destruction. Resolved on each call rather than cached, so the
  ///         defaulted copy and move operations stay correct -- a stored
  ///         pointer would dangle the moment this object moved.
  vr::sensor::CapturedFrame view() const noexcept {
    vr::sensor::CapturedFrame frame = header_;
    frame.depth = has_frame_ ? depth_.data() : nullptr;
    frame.color = has_frame_ && !color_.empty() ? color_.data() : nullptr;
    return frame;
  }

 private:
  std::vector<float> depth_;
  std::vector<std::uint32_t> color_;
  // The frame as assigned, pointers included; view() re-points them at the
  // vectors above, so they are never read from here.
  vr::sensor::CapturedFrame header_{};
  bool has_frame_ = false;
};

}  // namespace vr_example
