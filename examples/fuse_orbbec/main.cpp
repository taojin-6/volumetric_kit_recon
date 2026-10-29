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
// and a rig's skew of each secondary to the primary. It gives up after ten
// seconds without a frame rather than waiting, still reporting them.
//
// output.stream, output.csv or output.serve codes each fused set as one frame
// of a codec stream (codec_stream::CodecStream) at the set's time,
// the host's clock for a rig, and reports the stream, even when the run fails
// part-way; codec.sweep then prints the codec's rate-distortion table on the
// last grid. Only output.csv or codec.sweep decodes each frame too, for the
// decode's rows and the sweep's grid. output.serve serves each frame live, as
// it is recorded, on that port on every interface (networking::StreamServer),
// to a player that connects, and prints each viewer joining and leaving.
// fusion.dynamic, true by default, clears a surface the cameras see past, so
// the stream and the PLY follow a changing scene, and every fusion.free_after
// sets the grid frees the blocks such a surface left, as rig_viewer's does.
//
// Ctrl+C (or a SIGTERM) ends the run. Before the cameras stream, the run
// returns with nothing recorded: the empty stream is removed and the cameras
// are released. After, it ends after the set in hand, as run.frames would:
// the stream is finished and its viewers closed, the cameras are released
// and the PLY is written. A second signal of the same kind ends the process.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "codec_config.hpp"
#include "codec_report.hpp"
#include "codec_sweep.hpp"
#include "config.hpp"
#include "fuse_device.hpp"
#include "fuse_orbbec_sets.hpp"
#include "fuse_sender.hpp"
#include "info_log.hpp"
#include "interrupt.hpp"
#include "set_dump.hpp"
#include "set_log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/grid/codec/encoder.hpp"
#include "volumetric_kit/grid/codec_stream/codec_stream.hpp"
#include "volumetric_kit/grid/eval/mesh_distance.hpp"
#include "volumetric_kit/grid/mesh/marching_cubes.hpp"
#include "volumetric_kit/grid/mesh/mesh.hpp"
#include "volumetric_kit/grid/mesh_io/ply_writer.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/sensor/array/rig_sets.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/prep/picture_readback.hpp"

namespace vr = volumetric_kit::recon;

namespace vgrid = volumetric_kit::grid;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::grid::mesh;
namespace sensor = volumetric_kit::sensor;
namespace eval = volumetric_kit::grid::eval;
namespace codec_stream = volumetric_kit::grid::codec_stream;
namespace sender = volumetric_kit::recon::sender;

namespace {

using Options = vgrid_example::config::Config;

double seconds_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

vkc::Status run(const Options& opt) {
  // First, so a Ctrl+C while the run sets up still unwinds what it opened.
  vgrid_example::catch_interrupts();
  // Checked before anything opens: the dump's directory, which the run makes
  // with the first set it writes, and any codec step the codec refuses.
  vr_example::SetDump dump(opt.diagnostics.dump_directory,
                           static_cast<std::size_t>(opt.diagnostics.dump_sets));
  VKC_TRY(dump.check());
  VKC_ASSIGN(const vgrid::codec::EncoderConfig config,
             vgrid_example::encoder_config(opt.codec));

  // --- GPU, first: the cameras' pictures are decoded onto it ---
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(const vkc::DeviceRequirements reqs,
             vr_example::fuse_device_requirements());
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(reqs));
  VKC_ASSIGN(vkc::Device device, vkc::Device::create(instance, gpu, reqs));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  // --- Codec, with output.stream, output.csv or output.serve: before the
  // cameras, so a port in use, a rerun onto an existing stream, or a PLY that
  // is its file or the CSV's under another name, is refused before any camera
  // opens. A stream that codes no set is removed again when the run ends ---
  std::optional<codec_stream::CodecStream> stream;
  std::optional<vgrid_example::CodecReport> report;
  if (!opt.output.stream.empty() || !opt.output.csv.empty() ||
      opt.output.serve != 0) {
    const codec_stream::CodecOutputs outputs =
        vgrid_example::set_codec_outputs(opt);
    VKC_ASSIGN(
        codec_stream::CodecStream made,
        codec_stream::CodecStream::create(
            device, allocator, config,
            vgrid_example::codec_session("fuse_orbbec", {opt.source.fps, 1}),
            outputs));
    stream.emplace(std::move(made));
    VKC_ASSIGN(vgrid_example::CodecReport rows,
               vgrid_example::CodecReport::create(
                   opt.output.csv, outputs.stream, outputs.decode,
                   config.intra_period));
    report.emplace(std::move(rows));
    VKC_TRY(vgrid_example::check_apart(opt.output.path,
                                       {outputs.stream, opt.output.csv}));
    std::printf("codec: %s\n", vgrid_example::codec_settings(
                                   config, opt.codec.quantization_table)
                                   .c_str());
    if (const vgrid::networking::StreamServer* server = stream->server()) {
      std::printf("serving on port %u\n", unsigned{server->port()});
    }
  }

  // The cameras, read as an array that poses each frame and, when fusing,
  // prepares each set on the device in one batch. An optional so the run can
  // release the cameras before the extract: stop() ends the streams but keeps
  // the cameras held for a restart. After the device, so every picture on it
  // goes first. A Ctrl+C before here skips their discovery, which takes
  // seconds and does not see one.
  if (vgrid_example::interrupted()) {
    std::printf("interrupted before the cameras opened\n");
    return {};
  }
  std::printf(
      "opening the cameras; a Ctrl+C takes effect once they are open\n");
  std::optional<sensor::SensorArray> array;
  VKC_ASSIGN(array, vr_example::open_orbbec_array(opt, device, allocator));

  // --- Volume, unless operation=stream ---
  std::optional<vol::VoxelBlockGrid> volume;
  std::optional<tsdf::Fuser> fuser;
  std::optional<mesh::MarchingCubes> extractor;
  if (opt.operation != vgrid_example::config::Operation::Stream) {
    VKC_ASSIGN(
        vol::VoxelBlockGrid grid,
        tsdf::create_fusion_grid(device, allocator, opt.fusion.voxel_size_m,
                                 opt.fusion.truncation_m));
    volume.emplace(std::move(grid));
    VKC_ASSIGN(tsdf::Fuser f, tsdf::Fuser::create(device, allocator));
    fuser.emplace(std::move(f));
    VKC_ASSIGN(extractor, mesh::MarchingCubes::create(device, allocator, {}));
  }

  // --- Luma readback, with diagnostics.readback ---
  std::unique_ptr<sensor::PictureReadback> readback;
  if (opt.diagnostics.readback) {
    VKC_ASSIGN(readback, sensor::PictureReadback::create(device, allocator));
  }

  std::vector<std::string> ids;  // each camera's, naming its dump files
  for (std::size_t c = 0; c < array->size(); ++c) {
    ids.push_back(array->sensor(c).info().id);
  }

  // --- Stream, until run.frames or Ctrl+C ---
  // A Ctrl+C since the check above ends the run here, before any set.
  if (vgrid_example::interrupted()) {
    std::printf("interrupted before streaming\n");
    return {};
  }
  VKC_TRY(array->start());
  const char* verb = (opt.operation == vgrid_example::config::Operation::Stream)
                         ? "streamed"
                         : "fused";
  sensor::RigSets sets(*array, static_cast<std::size_t>(opt.run.frames));
  vr_example::SetLog log([&device] { return device.counters(); });
  // Every set coded, as it fuses: prep, fusion, free pass and the frame's
  // front half in one submit, its rANS half in a second.
  std::optional<sender::SetSender> out;
  if (volume) {
    VKC_ASSIGN(out, sender::SetSender::create(
                        device, *fuser, *volume, stream ? &*stream : nullptr,
                        vr_example::sender_config(opt, 1)));
  }
  // A set's trigger is on the system clock only where every camera stamps on
  // the host's, as a rig's always do.
  const bool host_clock = vr_example::on_host_clock(*array);
  const auto t_start = std::chrono::steady_clock::now();
  const vkc::Status streamed = [&]() -> vkc::Status {
    std::size_t reported = 0;  // the count last reported; a set may pass 100
    while (!sets.done()) {
      vr_example::SetLog::Set& s = log.begin();
      VKC_ASSIGN(sensor::FrameSet set, sets.next(&s.rows));
      vr_example::print_first_frame_age(*array, sets, set);
      s.arrived(set.arrival_ns, host_clock ? set.timestamp_ns : 0);
      // A set's stages run here, in turn, on its frames; operation=stream runs
      // none past the poll, any readback and any dump, and the set is
      // released as it came.
      std::vector<std::optional<sensor::PictureView>> luma;
      if (readback) {
        VKC_ASSIGN(luma, readback->read(set.frames,
                                        sensor::PictureReadback::Planes::Luma,
                                        &s.rows));
      }
      VKC_TRY(dump.write(set, ids, readback ? &luma : nullptr));
      if (out) {
        // One row for the prep: the cameras prepare together, into the
        // set's front batch, whose commands read the set's pictures. A set
        // stamped before the last frame recorded, as a clock stepping back
        // stamps it, is coded at that frame's time rather than refused. The
        // set is published once its frame is written and served, before any
        // decode back, or after its fusion when it is not coded.
        vkc::CommandBatch front(device, allocator);
        front.time(out->timer());
        VKC_ASSIGN(const sensor::DeviceFrameSet prepared,
                   array->record_process(front, set, &s.rows));
        const std::uint64_t pts_ns =
            std::max(prepared.timestamp_ns, stream ? stream->last_pts_ns() : 0);
        VKC_ASSIGN(const sender::SetSender::Sent sent,
                   out->send(front, prepared.frames,
                             std::make_shared<sensor::FrameSet>(std::move(set)),
                             sets.sets() - 1, pts_ns, &s.rows,
                             [&s] { s.published(); }));
        VKC_TRY(vr_example::report_sent(sent, sets.sets() - 1, "fuse_orbbec", s,
                                        report ? &*report : nullptr));
      } else {
        s.published();  // operation=stream: polled and released only
      }
      if (sets.frames() / 100 > reported / 100) {
        reported = sets.frames();
        std::printf("  %s %zu frames (%.1f fps)\n", verb, reported,
                    static_cast<double>(reported) / seconds_since(t_start));
      }
      // After the set in hand, so a run that ends well holds one.
      if (vgrid_example::interrupted()) {
        std::printf("  interrupted after %zu frames\n", sets.frames());
        break;
      }
    }
    return {};
  }();
  const double secs = seconds_since(t_start);
  const double fps =
      static_cast<double>(sets.frames()) / (secs > 0.0 ? secs : 1.0);
  // Before any error: a run that stalls still reports what it counted, and
  // finishes the stream it recorded and reports the frames it coded.
  vr_example::print_rig_stats(*array, &sets);
  std::fputs(dump.report().c_str(), stdout);
  vkc::Status finished;
  if (stream && stream->coded() > 0) {
    finished = stream->finish();
    // A coded frame per source set; one frame alone has no bitrate.
    if (!report->rows().empty()) {
      report->print(opt.source.fps, report->rows().size() > 1 ? 1 : 0);
    }
  }
  VKC_TRY(streamed);
  VKC_TRY(finished);
  array.reset();

  std::fputs(log.table("per set").c_str(), stdout);
  if (!volume) {
    std::printf("done: streamed %zu frames in %.1fs (%.1f fps)\n",
                sets.frames(), secs, fps);
    return {};
  }

  // --- Mesh -> PLY ---
  VKC_ASSIGN(mesh::Mesh final_mesh, extractor->extract_host(*volume, 0.0f));
  VKC_TRY(vgrid::mesh_io::write_ply(opt.output.path, final_mesh));
  std::printf(
      "done: fused %zu frames in %.1fs (%.1f fps), mesh %zu vertices / %zu "
      "triangles -> %s\n",
      sets.frames(), secs, fps, final_mesh.vertices.size(),
      final_mesh.triangle_count(), opt.output.path.c_str());
  if (stream && opt.codec.sweep) {
    const eval::CompareOptions compare = vgrid_example::codec_compare_options(
        opt.fusion.voxel_size_m, opt.fusion.truncation_m);
    VKC_ASSIGN(const eval::ReferenceMesh reference,
               eval::ReferenceMesh::create(final_mesh, compare));
    VKC_TRY(vgrid_example::run_codec_sweep(device, allocator, *volume,
                                           reference, *extractor, config,
                                           *stream->player()));
  }
  return {};
}

}  // namespace

int run_main(const Options& opt) {
  vgrid_example::print_info_log();  // output.serve's viewers joining and
                                    // leaving
  const vkc::Status status = run(opt);
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
