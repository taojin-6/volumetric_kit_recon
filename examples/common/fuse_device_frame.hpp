// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_device_frame.hpp
/// @brief @ref vr_example::fuse_frame for a frame `sensor::GpuFramePrep` left
///        on the device.
///
/// Apart from `fuse_frame.hpp` so that only an example running the GPU pass
/// includes `sensor/utils`, and only it links `recon_sensor_utils`.

#include <optional>
#include <vector>

#include "fuse_frame.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace vr_example {

/// @brief @ref fuse_frame for a frame already on the device, as
///        `sensor::GpuFramePrep` hands it out: nothing is uploaded, and depth
///        and colour are fused with their own cameras, the colour's coverage
///        read off its high byte.
/// @param grid        The volume to fuse into.
/// @param integrator  The integrator.
/// @param frame       The prepared frame.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @param mode        `Classic` (the default) keeps free space ahead of a
///                    surface; `Dynamic` clears it, so a surface that moves
///                    away leaves no ghost.
/// @return OK, or the first error of the two steps.
inline vr::Status fuse_frame(
    vr::volume::VoxelBlockGrid& grid, vr::tsdf::TsdfIntegrator& integrator,
    const vr::sensor::DeviceFrame& frame, float max_weight,
    vr::StageMetrics* metrics,
    vr::tsdf::IntegrationMode mode = vr::tsdf::IntegrationMode::Classic) {
  VR_TRY(allocate_band(grid, *frame.depth, frame.depth_camera, metrics));
  vr::tsdf::ColorFrame color{};
  color.buffer = frame.color.get();
  color.cam = frame.color_camera;
  color.encoding = frame.color_encoding;
  color.coverage_in_alpha = true;
  return integrator.integrate(grid, *frame.depth, frame.depth_camera,
                              max_weight, mode,
                              frame.has_color() ? &color : nullptr, metrics);
}

/// @brief @ref fuse_frame for a set of frames, as `sensor::prepare_set` hands
///        them out: every frame's band allocated in one call, then every
///        frame fused in one, so a set costs a few submits rather than a few a
///        frame. An empty entry is skipped.
/// @return OK, or the first error of the two steps.
inline vr::Status fuse_set(
    vr::volume::VoxelBlockGrid& grid, vr::tsdf::TsdfIntegrator& integrator,
    const std::vector<std::optional<vr::sensor::DeviceFrame>>& frames,
    float max_weight, vr::StageMetrics* metrics,
    vr::tsdf::IntegrationMode mode = vr::tsdf::IntegrationMode::Classic) {
  std::vector<vr::volume::DepthInput> depths;
  std::vector<vr::tsdf::ColorFrame> colors;
  std::vector<vr::tsdf::FrameInput> inputs;
  colors.reserve(frames.size());  // the inputs point into it
  for (const std::optional<vr::sensor::DeviceFrame>& frame : frames) {
    if (!frame) continue;
    depths.push_back({vr::StorageInput(*frame->depth), frame->depth_camera});
    const vr::tsdf::ColorFrame* color = nullptr;
    if (frame->has_color()) {
      vr::tsdf::ColorFrame c{};
      c.buffer = frame->color.get();
      c.cam = frame->color_camera;
      c.encoding = frame->color_encoding;
      c.coverage_in_alpha = true;
      colors.push_back(c);
      color = &colors.back();
    }
    inputs.push_back(
        {vr::StorageInput(*frame->depth), frame->depth_camera, color});
  }
  VR_TRY(allocate_band(grid, depths, metrics));
  return integrator.integrate(grid, inputs, max_weight, mode, metrics);
}

}  // namespace vr_example
