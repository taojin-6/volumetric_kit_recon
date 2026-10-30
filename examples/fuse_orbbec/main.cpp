// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec: live reconstruction from an Orbbec (Femto Mega) camera or a
// synced rig of them (sensor::OrbbecSensor), read as a sensor::SensorArray
// that groups a rig's frames by trigger, through the live examples' run
// (vr_example::run_live_fusion, which fuse_realsense shares and which
// documents the run: fusion, coding and serving, the stream operation, the
// diagnostics, Ctrl+C). config/README.md lists the scenario's keys.
//
//   fuse_orbbec --config config/examples/fuse_orbbec.json
//   fuse_orbbec --help
//
// Depth and colour are fused with their own cameras, the colour decoded onto
// the GPU: H.265 at 3840x2160 and 25 fps unless the scenario names another
// mode (source.color_codec, the dimensions and fps), since MJPEG needs about
// nine times the bandwidth at 4K. source.sync_config
// fuses every camera of a sync configuration (femto_mega_sync.json) as one rig
// on the host's clock, refusing cameras whose sync settings differ from it
// unless source.apply_sync writes it to them. source.calibration poses each
// camera, offsets its depth and sets every camera's colour exposure
// (color_controls) from a calibration file (camera/array_calibration.hpp);
// without it every camera sits at the origin with its depth as read, at auto
// exposure. source.host_clock puts one camera on the
// host's clock (OrbbecSensor::Options::sync_clock_to_host), which its first
// frame's age checks; a rig is always on it. Each camera's counters at the
// end count its `lost` frames too (vr_example::orbbec_counters).

#include <cstdio>

#include "config.hpp"
#include "fuse_orbbec_sets.hpp"
#include "info_log.hpp"
#include "live_fusion.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vkc = volumetric_kit::core;

int run_main(const vgrid_example::config::Config& opt) {
  vgrid_example::print_info_log();  // output.serve's viewers joining and
                                    // leaving
  const vkc::Status status = vr_example::run_live_fusion(
      opt, "fuse_orbbec", vr_example::open_orbbec_array,
      vr_example::orbbec_on_host_clock(opt), vr_example::orbbec_counters);
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_orbbec failed: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}

int main(int argc, char** argv) {
  return vgrid_example::configured_main(
      argc, argv, vgrid_example::config::Application::FuseOrbbec, "fuse_orbbec",
      run_main);
}
