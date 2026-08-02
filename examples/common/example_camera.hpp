// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/example_camera.hpp
/// @brief Adapt the dataset's shared @ref CameraModel into the recon camera
///        parameter structs the fuse examples drive the pipeline with.

#include <glm/glm.hpp>

#include "dataset.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr_example {

namespace vol = volumetric_kit::recon::volume;

/// @brief Build a depth camera (intrinsics + pose + fusion depth range) for one
///        posed frame. The near plane is the examples' fixed 0.1 m; @p
///        max_depth is the CLI's far-depth gate.
/// @param cam        Shared intrinsics + image dimensions.
/// @param pose       Camera->world for this frame.
/// @param max_depth  Far-depth gate (metres).
inline vol::DepthCameraParams make_depth_camera(const CameraModel& cam,
                                                const glm::mat4& pose,
                                                float max_depth) {
  vol::DepthCameraParams d{};
  d.fx = cam.fx;
  d.fy = cam.fy;
  d.cx = cam.cx;
  d.cy = cam.cy;
  d.min_depth = 0.1f;
  d.max_depth = max_depth;
  d.width = cam.width;
  d.height = cam.height;
  d.cam_to_world = pose;
  return d;
}

}  // namespace vr_example
