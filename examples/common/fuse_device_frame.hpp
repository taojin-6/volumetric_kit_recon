// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_device_frame.hpp
/// @brief @ref vr_example::fuse_frame for a frame `sensor::GpuFramePrep` left
///        on the device.
///
/// Apart from `fuse_frame.hpp` so that only an example running the GPU pass
/// includes `sensor/utils`, and only it links `recon_sensor_utils`.

#include "fuse_frame.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"

namespace vr_example {

/// @brief @ref fuse_frame for a frame already on the device, as
///        `sensor::GpuFramePrep` hands it out: nothing is uploaded, and depth
///        and colour are fused with their own cameras, the colour's coverage
///        read off its high byte.
/// @param grid        The volume to fuse into.
/// @param integrator  The integrator; `IntegrationMode::Classic`.
/// @param frame       The prepared frame.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @return OK, or the first error of the two steps.
inline vr::Status fuse_frame(vr::volume::VoxelBlockGrid& grid,
                             vr::tsdf::TsdfIntegrator& integrator,
                             const vr::sensor::DeviceFrame& frame,
                             float max_weight, vr::StageMetrics* metrics) {
  VR_TRY(allocate_band(grid, *frame.depth, frame.depth_camera, metrics));
  vr::tsdf::ColorFrame color{};
  color.buffer = frame.color.get();
  color.cam = frame.color_camera;
  color.encoding = frame.color_encoding;
  color.coverage_in_alpha = true;
  return integrator.integrate(grid, *frame.depth, frame.depth_camera,
                              max_weight, vr::tsdf::IntegrationMode::Classic,
                              frame.has_color() ? &color : nullptr, metrics);
}

}  // namespace vr_example
