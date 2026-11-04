// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sender/fuse_inputs.hpp
/// @brief The hand-off from frames prepared on the device
///        (the sensor package's `sensor::DeviceFrame`) to the fuser's
///        inputs.

#include <cstdint>
#include <optional>
#include <vector>

#include "volumetric_kit/recon/sender/export.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace volumetric_kit::recon::sender {

/// @brief A frame that fills where a set's fixed cameras neither see nor
///        hold weight (@ref tsdf::FrameRole::Filler): a phone's, placed by
///        its tracker.
struct Filler {
  /// At its placed pose, both cameras. Its colour, if it has one, fuses as a
  /// fixed frame's does.
  sensor::DeviceFrame frame;
  /// How it was posed: @ref tsdf::FillTier::Anchored only by a fit to the
  /// rig alone.
  tsdf::FillTier tier = tsdf::FillTier::Unanchored;
  /// Its sensor among the mobile ones (@ref tsdf::mobile_source), below
  /// @ref tsdf::kMaxSourcesPerClass.
  std::uint32_t index = 0;
};

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
///        tsdf::ColorFrame::coverage_in_alpha). An empty entry is skipped;
///        each frame's @ref tsdf::FrameInput::slot is its index in
///        @p frames, the camera's place in the set, and its
///        @ref tsdf::FrameInput::noise the frame's `depth_noise`, as its
///        producer stated it (unknown stays unknown, which Dynamic refuses).
/// @param frames  The prepared frames; they must outlive the inputs, which
///                borrow their buffers.
VR_SENDER_API FuseInputs
fuse_inputs(const std::vector<std::optional<sensor::DeviceFrame>>& frames);

/// @brief The fuser's inputs for @p frames, as the overload above gives
///        them, then one for each of @p fillers after them, in order: its
///        frame's depth and colour as a fixed frame's,
///        @ref tsdf::FrameRole::Filler, its tier, its index as the slot,
///        its frame's noise and the default @ref tsdf::FillRule.
/// @param frames   The prepared fixed frames; they and @p fillers must
///                 outlive the inputs, which borrow their buffers.
/// @param fillers  The fillers, fused after every fixed frame in this order.
VR_SENDER_API FuseInputs
fuse_inputs(const std::vector<std::optional<sensor::DeviceFrame>>& frames,
            const std::vector<Filler>& fillers);

}  // namespace volumetric_kit::recon::sender
