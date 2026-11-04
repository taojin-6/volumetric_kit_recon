// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The tier's one frame check, which the Tracker runs at a frame's own record
// and a fitter at every record that holds it: a frame the Tracker took is one
// no fitter refuses later. Internal.

#include <cmath>
#include <string>

#include "se3.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace volumetric_kit::recon::track::detail {

// Why `f` cannot be tracked, or empty: it needs depth of at least its camera's
// size that a batch can copy (sensor's frame contract), a known depth noise, a
// rigid pose, and a depth camera's intrinsics and range that make sense.
inline std::string frame_refusal(const sensor::DeviceFrame& f) {
  const sensor::DepthCameraParams& c = f.depth_camera;
  if (!f.depth || !f.depth->valid()) return "no depth";
  if (f.depth->size() < VkDeviceSize(c.width) * c.height * 4 ||
      (f.depth->usage() & VK_BUFFER_USAGE_TRANSFER_SRC_BIT) == 0) {
    return "its depth is smaller than its camera, or not copyable (no "
           "TRANSFER_SRC)";
  }
  if (!f.depth_noise.known()) return "no depth noise";
  if (!rigid(c.cam_to_world)) return "its pose is not rigid";
  if (!(c.fx > 0.0f && c.fy > 0.0f && std::isfinite(c.fx) &&
        std::isfinite(c.fy) && std::isfinite(c.cx) && std::isfinite(c.cy) &&
        c.min_depth >= 0.0f && c.max_depth > c.min_depth &&
        std::isfinite(c.max_depth))) {
    return "its camera's intrinsics or range are invalid";
  }
  return {};
}

}  // namespace volumetric_kit::recon::track::detail
