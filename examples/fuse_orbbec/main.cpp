// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec: live reconstruction from an Orbbec (Femto Mega) camera or a
// synced rig of them. Reads its cameras (sensor::OrbbecSensor) as a
// sensor::SensorArray, which groups a rig's frames by trigger and records
// each set's prep into the set's one batch, fuses it there into a sparse TSDF
// volume (tsdf::Fuser, through sender::SetSender), and after run.frames
// frames extracts a marching-cubes mesh and writes it to a binary PLY. It
// prints a table per set at the end (vr_example::SetLog).
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
// frame's age checks; a rig is always on it. operation=stream only streams:
// each set is polled, counted and released, with no GPU preparation, fusion or
// mesh, so the run measures the cameras' stream and decode alone.
// diagnostics.readback reads each set's luma back to the host through
// sensor::PictureReadback, as a separate stage. diagnostics.dump_sets writes
// the first sets' camera JPEGs, and with the readback their luma, into
// diagnostics.dump_directory, which the run makes with the first set
// (vr_example::SetDump). The run reports each camera's counters at the end,
// `lost` among them, and a rig's skew of each secondary to the primary. It
// gives up after ten seconds without a frame rather than waiting, still
// reporting them.
//
// output.stream, output.csv or output.serve codes each fused set as one frame
// of a codec stream (codec_stream::CodecStream) at the set's time,
// the host's clock for a rig, and reports the stream, even when the run fails
// part-way; codec.sweep then prints the codec's rate-distortion table on the
// last grid. Only output.csv or codec.sweep decodes each frame too, for the
// decode's rows and the sweep's grid. output.serve serves each frame live, as
// it is recorded, on that port on every interface (networking::StreamServer),
// to a player that connects, and prints each viewer joining and leaving.
// fusion.dynamic, true by default, resets where a camera sees change, so
// the stream and the PLY follow a changing scene, and every fusion.free_after
// sets the grid frees the blocks such a surface left, as rig_viewer's does.
//
// Ctrl+C (or a SIGTERM) ends the run. Before the cameras stream, the run
// returns with nothing recorded: the empty stream is removed and the cameras
// are released. After, it ends after the set in hand, as run.frames would:
// the stream is finished and its viewers closed, the cameras are released
// and the PLY is written. A second signal of the same kind ends the process.
//
// The run is the live examples' (vr_example::run_live_fusion, which
// fuse_realsense shares); this program opens the Orbbec cameras.

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
      vr_example::orbbec_counters);
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
