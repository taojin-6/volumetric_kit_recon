// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sender/fuse_inputs.hpp"

#include "volumetric_kit/core/vulkan/compute_util.hpp"

namespace volumetric_kit::recon::sender {

namespace {

// How a prepared frame's colour is fused: through its own camera, the
// coverage read off its high byte. Meaningful only when the frame has colour.
tsdf::ColorFrame device_color(const sensor::DeviceFrame& frame) {
  tsdf::ColorFrame color{};
  color.buffer = frame.color.get();
  color.cam = frame.color_camera;
  color.encoding = frame.color_encoding;
  color.coverage_in_alpha = true;
  return color;
}

}  // namespace

FuseInputs fuse_inputs(
    const std::vector<std::optional<sensor::DeviceFrame>>& frames) {
  FuseInputs in;
  in.colors.reserve(frames.size());  // the inputs point into it
  for (const std::optional<sensor::DeviceFrame>& frame : frames) {
    if (!frame) continue;
    const tsdf::ColorFrame* color = nullptr;
    if (frame->has_color()) {
      in.colors.push_back(device_color(*frame));
      color = &in.colors.back();
    }
    in.frames.push_back(
        {{core::StorageInput(*frame->depth), frame->depth_camera}, color});
  }
  return in;
}

}  // namespace volumetric_kit::recon::sender
