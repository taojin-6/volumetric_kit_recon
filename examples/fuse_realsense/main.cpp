// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_realsense: live reconstruction from one Intel RealSense D400 camera
// (the D455). Its depth is the scenario's source.depth:
//
//   camera  the camera's own, computed on the camera from its infrared pair
//           with the projector on (sensor::RealSenseSensor); any build with
//           the RealSense driver.
//   ffs     Fast-FoundationStereo on its infrared pair with the projector off
//           (sensor::RealSenseStereo under sensor::StereoDepthSensor): each
//           pair rectified and matched on the GPU, on the sensor's own
//           thread. Builds with sensor_ffs only (VS_WITH_FFS: CUDA and
//           TensorRT, an NVIDIA GPU); any other refuses the scenario before
//           anything opens.
//
// Either is read as a sensor::SensorArray of one, its YUYV colour converted
// and undistorted on the GPU, through the live examples' run
// (vr_example::run_live_fusion, which fuse_orbbec shares and which documents
// the run).
//
//   fuse_realsense --config config/examples/fuse_realsense.json
//   fuse_realsense --config config/examples/fuse_realsense_ffs.json
//   fuse_realsense --help
//
// source.serial names the camera (empty: the only one attached); the colour
// size and source.fps are its modes, 1280x800 at 30 fps by default; the depth
// size is the camera's depth mode (1280x720) or FFS's rectified size
// (960x544), which its model (source.ffs_model) must have; fusion.min_depth_m
// and max_depth_m gate either depth. The colour runs at auto exposure, on the
// host's clock as the depth does. The camera's framesets of its first second
// or so pair depth with stale colour, and the driver refuses them: the
// camera's counters at the end count them failed.

#include <cstdio>

#include "config.hpp"
#include "live_fusion.hpp"
#include "realsense_camera.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vkc = volumetric_kit::core;

int run_main(const vgrid_example::config::Config& opt) {
  // A depth this build cannot open is refused as the scenario is.
  if (const vkc::Status depth = vr_example::check_realsense_depth(opt);
      !depth.ok()) {
    std::fprintf(stderr, "fuse_realsense: %s\n", depth.message().c_str());
    return 2;
  }
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
