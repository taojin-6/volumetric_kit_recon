// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A frame's depth camera as the integrator fuses it and the fuser allocates
// it: a filler's depth range ends at its FillRule::max_depth_m. Internal to
// recon_tsdf.

#include <algorithm>

#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/sensor/frame/camera_params.hpp"

namespace volumetric_kit::recon::tsdf::detail {

inline sensor::DepthCameraParams fused_camera(const FrameInput& frame) {
  sensor::DepthCameraParams cam = frame.camera;
  if (frame.role == FrameRole::Filler) {
    cam.max_depth = std::min(cam.max_depth, frame.fill.max_depth_m);
  }
  return cam;
}

}  // namespace volumetric_kit::recon::tsdf::detail
