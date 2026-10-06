// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_device_frame.hpp
/// @brief @ref vr_example::fuse_frame for the frames `sensor::GpuFramePrep`
///        left on the device: @ref vr_example::fuse_set.
///
/// Apart from `fuse_frame.hpp` so that only an example running the GPU pass
/// includes `sensor/utils`, and only it links `recon_sensor_utils`.

#include <optional>
#include <vector>

#include "fuse_frame.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace vr_example {

namespace vkc = volumetric_kit::core;

/// @brief How a prepared frame's colour is fused: through its own camera, the
///        coverage read off its high byte. Meaningful only when the frame
///        @ref vr::sensor::DeviceFrame::has_color.
inline vr::tsdf::ColorFrame device_color(const vr::sensor::DeviceFrame& frame) {
  vr::tsdf::ColorFrame color{};
  color.buffer = frame.color.get();
  color.cam = frame.color_camera;
  color.encoding = frame.color_encoding;
  color.coverage_in_alpha = true;
  return color;
}

/// @brief @ref fuse_frame for a set of frames already on the device, as
///        `sensor::GpuFramePrep::prepare_batch` hands them out: nothing is
///        uploaded, and depth and colour are fused with their own cameras.
///        Every frame's band is allocated in one call, then every frame fused
///        in one, so a set costs a few submits rather than a few a frame. An
///        empty entry is skipped.
/// @param grid        The volume to fuse into.
/// @param integrator  The integrator.
/// @param frames      The prepared frames.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @param mode        `Classic` (the default) keeps free space ahead of a
///                    surface; `Dynamic` clears it, so a surface that moves
///                    away leaves no ghost.
/// @return OK, or the first error of the two steps.
inline vkc::Status fuse_set(
    vr::volume::VoxelBlockGrid& grid, vr::tsdf::TsdfIntegrator& integrator,
    const std::vector<std::optional<vr::sensor::DeviceFrame>>& frames,
    float max_weight, vkc::StageMetrics* metrics,
    vr::tsdf::IntegrationMode mode = vr::tsdf::IntegrationMode::Classic) {
  std::vector<vr::tsdf::ColorFrame> colors;
  std::vector<vr::tsdf::FrameInput> inputs;
  colors.reserve(frames.size());  // the inputs point into it
  for (const std::optional<vr::sensor::DeviceFrame>& frame : frames) {
    if (!frame) continue;
    const vr::tsdf::ColorFrame* color = nullptr;
    if (frame->has_color()) {
      colors.push_back(device_color(*frame));
      color = &colors.back();
    }
    inputs.push_back(
        {{vkc::StorageInput(*frame->depth), frame->depth_camera}, color});
  }
  // Each input's depth half, sliced off.
  const std::vector<vr::volume::DepthInput> depths(inputs.begin(),
                                                   inputs.end());
  VKC_TRY(allocate_band(grid, depths, metrics));
  return integrator.integrate(grid, inputs, max_weight, mode, metrics);
}

}  // namespace vr_example
