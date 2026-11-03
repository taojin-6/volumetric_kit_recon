// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/fuse_inputs.hpp
/// @brief The hand-off from frames prepared on the device
///        (the sensor package's `sensor::DeviceFrame`) to the fuser's
///        inputs.

#include <optional>
#include <vector>

#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief A set's prepared frames as the fuser takes them: each frame's
///        input, its colour pointing into @ref colors. Moved, never copied,
///        so the pointers hold.
struct FuseInputs {
  /// The colour of each frame that has it, in the frames' order.
  std::vector<ColorFrame> colors;
  /// The frames, as @ref Fuser::fuse and @ref Fuser::record_fuse take them.
  std::vector<FrameInput> frames;
};

/// @brief The fuser's inputs for @p frames, as a producer such as
///        `sensor::GpuFramePrep` hands them out: nothing is uploaded, and depth
///        and colour are fused through their own cameras, a colour pixel's
///        coverage read off its high byte
///        (@ref ColorFrame::coverage_in_alpha). An empty entry is skipped;
///        each frame's @ref FrameInput::slot is its index in @p frames, the
///        camera's place in the set, and its @ref FrameInput::noise the
///        frame's `depth_noise`, as its producer stated it (unknown stays
///        unknown, which Dynamic refuses).
/// @param frames  The prepared frames; they must outlive the inputs, which
///                borrow their buffers.
VR_TSDF_API FuseInputs
fuse_inputs(const std::vector<std::optional<sensor::DeviceFrame>>& frames);

}  // namespace volumetric_kit::recon::tsdf
