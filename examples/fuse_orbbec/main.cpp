// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec: live reconstruction from an Orbbec (Femto Mega) camera or a
// synced rig of them. Reads its cameras (sensor::OrbbecSensor) as a
// sensor::SensorArray, which groups a rig's frames by trigger and prepares each
// set on the GPU in one batch, fuses it into a sparse TSDF volume
// (tsdf::Fuser), and after run.frames frames extracts a
// marching-cubes mesh and writes it to a binary PLY.
//
//   fuse_orbbec --config config/examples/fuse_orbbec.json
//   fuse_orbbec --help
//
// Depth and colour are fused with their own cameras, the colour decoded onto
// the GPU: H.265 unless source.color_codec=mjpeg says otherwise, since MJPEG
// needs about nine times the bandwidth at 4K. The source dimensions and fps
// pick the colour mode, 4K H.265 running at 25 fps at most. source.sync_config
// fuses every camera of a sync configuration (femto_mega_sync.json) as one rig
// on the host's clock, refusing cameras whose sync settings differ from it
// unless source.apply_sync writes it to them. source.calibration poses each
// camera from a calibration file (camera/array_calibration.hpp); without it
// every camera sits at the origin. source.host_clock puts one camera on the
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

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "config.hpp"
#include "fuse_frame.hpp"
#include "fuse_orbbec_sets.hpp"
#include "set_dump.hpp"
#include "stage_table.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/io/ply_writer.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/utils/picture_readback.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

using Options = vr_example::config::Config;

double seconds_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

vkc::Status run(const Options& opt) {
  // The dump's directory, checked before anything opens and made with the
  // first set it writes.
  vr_example::SetDump dump(opt.diagnostics.dump_directory,
                           static_cast<std::size_t>(opt.diagnostics.dump_sets));
  VKC_TRY(dump.check());

  // --- GPU, first: the cameras' pictures are decoded onto it ---
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(vr::device_requirements()));
  VKC_ASSIGN(vkc::Device device,
             vkc::Device::create(instance, gpu, vr::device_requirements()));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  // The cameras, read as an array that poses each frame and, when fusing,
  // prepares each set on the device in one batch. An optional so the run can
  // release the cameras before the extract: stop() ends the streams but keeps
  // the cameras held for a restart. After the device, so every picture on it
  // goes first.
  std::optional<sensor::SensorArray> array;
  VKC_ASSIGN(array, vr_example::open_orbbec_array(opt, device, allocator));

  // --- Volume, unless operation=stream ---
  std::optional<vol::VoxelBlockGrid> volume;
  std::optional<tsdf::Fuser> fuser;
  std::optional<mesh::MarchingCubes> extractor;
  if (opt.operation != vr_example::config::Operation::Stream) {
    VKC_ASSIGN(vol::VoxelBlockGrid grid,
               vr_example::create_fusion_grid(device, allocator,
                                              opt.fusion.voxel_size_m,
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

  // --- Stream ---
  VKC_TRY(array->start());
  const char* verb = (opt.operation == vr_example::config::Operation::Stream)
                         ? "streamed"
                         : "fused";
  vr_example::RigSets sets(*array, static_cast<std::size_t>(opt.run.frames));
  vkc::StageMetrics stage_totals;
  const auto t_start = std::chrono::steady_clock::now();
  const vkc::Status streamed = [&]() -> vkc::Status {
    std::size_t reported = 0;  // the count last reported; a set may pass 100
    while (!sets.done()) {
      VKC_ASSIGN(sensor::FrameSet set, sets.next(&stage_totals));
      // A set's stages run here, in turn, on its frames; operation=stream runs
      // none past the poll, any readback and any dump, and the set is
      // released as it came.
      std::vector<std::optional<sensor::PictureView>> luma;
      if (readback) {
        VKC_ASSIGN(luma, readback->read(set.frames,
                                        sensor::PictureReadback::Planes::Luma,
                                        &stage_totals));
      }
      VKC_TRY(dump.write(set, ids, readback ? &luma : nullptr));
      if (volume) {
        // One row for the batch: the cameras prepare together.
        VKC_ASSIGN(const sensor::DeviceFrameSet prepared,
                   array->process(set, &stage_totals));
        VKC_TRY(vr_example::fuse_set(
            *fuser, *volume, prepared.frames, opt.fusion.max_weight,
            tsdf::IntegrationMode::Classic, &stage_totals));
      }
      if (sets.frames() / 100 > reported / 100) {
        reported = sets.frames();
        std::printf("  %s %zu frames (%.1f fps)\n", verb, reported,
                    static_cast<double>(reported) / seconds_since(t_start));
      }
    }
    return {};
  }();
  const double secs = seconds_since(t_start);
  const double fps =
      static_cast<double>(sets.frames()) / (secs > 0.0 ? secs : 1.0);
  // Before any error: a run that stalls still reports what it counted.
  sets.print_stats();
  std::fputs(dump.report().c_str(), stdout);
  VKC_TRY(streamed);
  array.reset();

  if (!volume) {
    vr_example::print_stage_rows(
        "stages per set, mean over " + std::to_string(sets.sets()) + " sets",
        stage_totals, sets.sets());
    std::printf("done: streamed %zu frames in %.1fs (%.1f fps)\n",
                sets.frames(), secs, fps);
    return {};
  }
  vr_example::print_stage_rows("stages per fused frame, mean over " +
                                   std::to_string(sets.frames()) + " frames",
                               stage_totals, sets.frames());

  // --- Mesh -> PLY ---
  VKC_ASSIGN(mesh::Mesh final_mesh, extractor->extract_host(*volume, 0.0f));
  VKC_TRY(vr::io::write_ply(opt.output.path, final_mesh));
  std::printf(
      "done: fused %zu frames in %.1fs (%.1f fps), mesh %zu vertices / %zu "
      "triangles -> %s\n",
      sets.frames(), secs, fps, final_mesh.vertices.size(),
      final_mesh.triangle_count(), opt.output.path.c_str());
  return {};
}

}  // namespace

int run_main(const Options& opt) {
  const vkc::Status status = run(opt);
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_orbbec failed: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}

int main(int argc, char** argv) {
  return vr_example::configured_main(
      argc, argv, vr_example::config::Application::FuseOrbbec, "fuse_orbbec",
      run_main);
}
