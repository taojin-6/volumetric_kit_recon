// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_realsense: live reconstruction from one Intel RealSense D400 camera
// (the D455) with the camera's own depth, computed on the camera from its
// infrared pair with the projector on (sensor::RealSenseSensor). The camera
// is read as a sensor::SensorArray of one, its YUYV colour converted and
// undistorted on the GPU, through the live examples' run
// (vr_example::run_live_fusion, which fuse_orbbec shares and which documents
// the run).
//
//   fuse_realsense --config config/examples/fuse_realsense.json
//   fuse_realsense --help
//
// source.serial names the camera (empty: the only one attached); the colour
// and depth sizes and source.fps are its modes, 1280x800 and 1280x720 at
// 30 fps by default; fusion.min_depth_m and max_depth_m its depth gate. The
// colour runs at auto exposure, on the host's clock as the depth does. The
// camera's framesets of its first second or so pair depth with stale colour,
// and the driver refuses them: the camera's counters at the end count them
// failed.

#include <cstdio>

#include "config.hpp"
#include "live_fusion.hpp"
#include "realsense_camera.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vkc = volumetric_kit::core;

int run_main(const vgrid_example::config::Config& opt) {
  const vkc::Status status = vr_example::run_live_fusion(
      opt, "fuse_realsense", vr_example::open_realsense_array,
      vr_example::sensor_counters);
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_realsense failed: %s\n",
                 status.message().c_str());
    return 1;
  }
  return 0;
}

int main(int argc, char** argv) {
  return vgrid_example::configured_main(
      argc, argv, vgrid_example::config::Application::FuseRealsense,
      "fuse_realsense", run_main);
}
