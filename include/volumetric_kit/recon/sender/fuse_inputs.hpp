// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sender/fuse_inputs.hpp
/// @brief The hand-off from frames prepared on the device
///        (the sensor package's `sensor::DeviceFrame`) to the fuser's
///        inputs.

#include <optional>
#include <vector>

#include "volumetric_kit/recon/sender/export.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace volumetric_kit::recon::sender {

/// @brief A set's prepared frames as the fuser takes them: each frame's
///        input, its colour pointing into @ref colors. Moved, never copied,
///        so the pointers hold.
struct FuseInputs {
  /// The colour of each frame that has it, in the frames' order.
  std::vector<tsdf::ColorFrame> colors;
  /// The frames, as @ref tsdf::Fuser::fuse and @ref tsdf::Fuser::record_fuse
  /// take them.
  std::vector<tsdf::FrameInput> frames;
};

/// @brief The fuser's inputs for @p frames, as a producer such as
///        `sensor::GpuFramePrep` hands them out: nothing is uploaded, and depth
///        and colour are fused through their own cameras, a colour pixel's
///        coverage read off its high byte (@ref
///        tsdf::ColorFrame::coverage_in_alpha). An empty entry is skipped.
/// @param frames  The prepared frames; they must outlive the inputs, which
///                borrow their buffers.
VR_SENDER_API FuseInputs
fuse_inputs(const std::vector<std::optional<sensor::DeviceFrame>>& frames);

}  // namespace volumetric_kit::recon::sender
