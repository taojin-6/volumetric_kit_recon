// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Fusion's frames as the voxel layer's allocation takes them, for the tests
// that allocate a set themselves.

#include <vector>

#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

namespace vr_test {

// The frames' depth, each with its camera through to_depth_pinhole.
inline std::vector<volumetric_kit::grid::volume::DepthInput> depth_inputs(
    const std::vector<volumetric_kit::recon::tsdf::FrameInput>& frames) {
  std::vector<volumetric_kit::grid::volume::DepthInput> out;
  out.reserve(frames.size());
  for (const volumetric_kit::recon::tsdf::FrameInput& f : frames) {
    out.push_back(
        {f.depth, volumetric_kit::recon::tsdf::to_depth_pinhole(f.camera)});
  }
  return out;
}

}  // namespace vr_test
