// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_decklink: live reconstruction from two SDI cameras on a Blackmagic
// DeckLink card, read as one stereo camera (sensor::DecklinkStereo), its
// depth from Fast-FoundationStereo (sensor::StereoDepthSensor): each frame's
// two 10-bit pictures rectified and matched on the GPU, on the sensor's own
// thread. Builds with sensor's DeckLink driver and FFS depth only
// (VS_WITH_DECKLINK: the DeckLink SDK's headers; VS_WITH_FFS: CUDA and
// TensorRT, an NVIDIA GPU); any other build refuses the scenario before
// anything opens.
//
// The depth is read as a sensor::SensorArray of one, posed by the scenario's
// calibration file, its colour the left camera's picture, converted and
// undistorted on the GPU, through the live examples' run
// (vr_example::run_live_fusion, which fuse_orbbec shares and which documents
// the run).
//
//   fuse_decklink --config config/examples/fuse_decklink.json
//   fuse_decklink --help
//
// source.cameras names each camera (its key in the calibration file) and the
// card connector its SDI output feeds; source.pairs names the left and right
// one. The colour size and source.fps are the cameras' mode, 3840x2160 at 30
// fps by default; the depth size is FFS's rectified size (960x544), which its
// model (source.ffs_model) must have; fusion.min_depth_m and max_depth_m gate
// the depth. The cameras free-run: each frame's left and right pictures are
// at most source.max_pair_skew_us apart, half a frame by default, so a still
// scene gives good depth and motion up to that much mismatch.

#include <cstdio>
#include <utility>

#include "config.hpp"
#include "decklink_camera.hpp"
#include "live_fusion.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vkc = volumetric_kit::core;

int run_main(const vgrid_example::config::Config& opt) {
  // A build without the driver or FFS refuses the scenario as invalid.
  if (const vkc::Status build = vr_example::check_decklink_build();
      !build.ok()) {
    std::fprintf(stderr, "fuse_decklink: %s\n", build.message().c_str());
    return 2;
  }
  // The run takes the array alone: every frame states its own depth noise.
  const auto open = [](const vgrid_example::config::Config& o,
                       vkc::Device& device, vkc::Allocator& allocator)
      -> vkc::Result<volumetric_kit::sensor::SensorArray> {
    VKC_ASSIGN(vr_example::DecklinkArray opened,
               vr_example::open_decklink_array(o, device, allocator));
    return std::move(opened.array);
  };
  // The driver stamps each frame on the host's clock.
  const vkc::Status status = vr_example::run_live_fusion(
      opt, "fuse_decklink", open, /*host_clock=*/true,
      vr_example::sensor_counters);
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_decklink failed: %s\n",
                 status.message().c_str());
    return 1;
  }
  return 0;
}

int main(int argc, char** argv) {
  return vgrid_example::configured_main(
      argc, argv, vgrid_example::config::Application::FuseDecklink,
      "fuse_decklink", run_main);
}
