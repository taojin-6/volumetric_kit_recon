// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_realsense: live reconstruction from one Intel RealSense D400 camera
// (the D455) with the camera's own depth, computed on the camera from its
// infrared pair with the projector on (sensor::RealSenseSensor). The run is
// fuse_orbbec's (vr_example::run_live_fusion): the camera read as a
// sensor::SensorArray of one, each frame prepared on the GPU -- its YUYV
// colour converted and undistorted there, its depth undistorted -- and fused
// into a sparse TSDF volume through sender::SetSender, then after run.frames
// frames a marching-cubes mesh written to a binary PLY, a table per frame at
// the end (vr_example::SetLog).
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
// failed. Ctrl+C (or a SIGTERM) ends the run after the frame in hand, as
// run.frames would; the camera is released and the PLY written.

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
