// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_replica: the end-to-end reconstruction example. Polls a posed RGB-D
// sequence in the Replica-SLAM layout (nvblox's fuse_replica dataset) through
// the sensor tier's IRgbdSensor interface, prepares each frame on the GPU
// (sensor::GpuFramePrep) as a camera's, fuses it into a sparse TSDF volume
// (tsdf::Fuser: grow the map ahead of need, allocate the truncation band,
// then integrate depth + colour),
// periodically extracts a marching-cubes mesh on the device, and writes the
// final coloured mesh to a binary PLY for inspection. This is the headless
// spine; the live-viewer variant renders the growing mesh each frame through
// the volumetric_kit_gfx sibling.
//
//   fuse_replica --config config/examples/fuse_replica.json
//   fuse_replica --help
//
// source.path is a Replica scene folder (contains results/ and traj.txt); the
// intrinsics default to its ../cam_params.json. Unknown settings are rejected
// by core before runtime resources are opened.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <thread>

#include "config.hpp"
#include "fuse_frame.hpp"
#include "remesh_report.hpp"
#include "replica_config.hpp"
#include "replica_sensor.hpp"
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
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// Validated scenario with reconstruction-friendly defaults for Replica.
using Options = vr_example::config::Config;

vkc::Status run(const Options& opt) {
  // --- Device bring-up (headless: no surface needed) ---
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(vr::device_requirements()));
  VKC_ASSIGN(vkc::Device device,
             vkc::Device::create(instance, gpu, vr::device_requirements()));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  // --- Capture ---
  // The sequence arrives through the sensor interface: frame selection and
  // the depth gate are the sensor's options, the cameras and pose ride on
  // each frame, and the loop below never learns it is reading a disk. A live
  // source replaces this one construction.
  VKC_ASSIGN(vr_example::ReplicaSensor replica, vr_example::open_replica(opt));
  const vr::camera::CameraModel& cam = *replica.info().color;
  std::printf(
      "capture: %zu frames to play, %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n",
      replica.frame_count(), cam.size.width, cam.size.height, cam.intrinsics.fx,
      cam.intrinsics.fy, cam.intrinsics.cx, cam.intrinsics.cy);

  // --- Volume + pipeline ---
  VKC_ASSIGN(vol::VoxelBlockGrid volume,
             vr_example::create_fusion_grid(
                 device, allocator, opt.fusion.voxel_size_m,
                 opt.fusion.truncation_m, opt.fusion.buckets));
  VKC_ASSIGN(tsdf::Fuser fuser, tsdf::Fuser::create(device, allocator));
  VKC_ASSIGN(sensor::GpuFramePrep prep,
             sensor::GpuFramePrep::create(device, allocator));
  VKC_ASSIGN(mesh::MarchingCubes extractor,
             mesh::MarchingCubes::create(device, allocator, [&] {
               mesh::MarchingCubesConfig c;
               c.share_vertices = opt.fusion.share_vertices;
               return c;
             }()));

  // Decoded up front when asked, *outside* the timed region below: streaming
  // spends ~75% of the loop in JPEG/PNG decode, so preloading is what makes
  // the reported fps a measure of fusion rather than of the reader.
  if (opt.source.preload) {
    VKC_TRY(vr_example::preload_frames(replica).status());
  }

  // From here on the source is the interface, not the dataset.
  sensor::IRgbdSensor& capture = replica;
  VKC_TRY(capture.start());

  const auto t_start = std::chrono::steady_clock::now();
  std::size_t fused = 0;
  vr_example::ExtractTotals remeshes;
  vr_example::DirtySurvey dirty(volume);
  // Host and device spans per stage, summed across every fused frame: rows
  // accumulate by name, so the loop adds straight into this.
  vkc::StageMetrics stage_totals;
  for (;;) {
    // An empty poll is "nothing this tick", which a replay and an idle live
    // device report alike; only the source knows whether that is the end. A
    // decode failure on a frame that is present is a real error and stops the
    // run.
    VKC_ASSIGN(const std::optional<sensor::RgbdFrame> polled, capture.poll());
    if (!polled) {
      if (capture.exhausted()) {
        break;
      }
      // A live sensor polled faster than it runs: yield and ask again.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    VKC_ASSIGN(const sensor::DeviceFrame frame,
               prep.prepare(*polled, &stage_totals));
    VKC_TRY(vr_example::fuse_set(
        fuser, volume, {frame}, opt.fusion.max_weight,
        vr_example::integration_mode(opt.fusion.dynamic), &stage_totals));
    ++fused;

    if (opt.fusion.dirty_every > 0 &&
        fused % static_cast<std::size_t>(opt.fusion.dirty_every) == 0) {
      VKC_TRY(dirty.sample(volume));
    }
    if (opt.fusion.mesh_every > 0 &&
        fused % static_cast<std::size_t>(opt.fusion.mesh_every) == 0) {
      // On the device, as a live consumer meshes: nothing is read back in the
      // timed loop, and the host extract below is the PLY's alone. The
      // DeviceMesh borrows the extractor's buffers and is dropped here -- at
      // the default slot_count of 1 the next extract invalidates it, which is
      // exactly what a benchmark wants and what a real consumer must not do.
      mesh::ExtractTimings rt{};
      VKC_ASSIGN(const mesh::DeviceMesh dm,
                 extractor.extract_device(volume, 0.0f, &rt));
      remeshes.add(rt);
      if (fused % 100 == 0) {
        std::printf("  fused %zu frames, %u triangles so far\n", fused,
                    dm.triangle_count);
      }
    }
  }

  const std::int32_t voxels_per_block = volume.grid().voxels_per_block;
  remeshes.print("extract_device", voxels_per_block);
  // Per stage, host against device, averaged over the fused frames. A stage
  // whose device share is small is not a slow kernel and will not be fixed by
  // a faster one. The instrument is not free at this scale: one timed submit
  // costs ~0.13 ms more than an untimed one on MoltenVK (the first
  // vkCmdWriteTimestamp in a command buffer, measured -- see DECISIONS.md),
  // which moves the host column of a sub-millisecond stage.
  vr_example::print_stage_rows(
      "stages per fused frame, mean over " + std::to_string(fused) + " frames",
      stage_totals, fused);
  dirty.print(opt.fusion.dirty_every);

  // --- Final mesh -> PLY ---
  // Measured, so a sweep over fusion.voxel_size_m can be diffed: whole-volume
  // meshing, the on-device neighbour probe and a possible arena refit all hide
  // inside one call, and only the split says which one a slow extract is.
  mesh::ExtractTimings t{};
  VKC_ASSIGN(mesh::Mesh final_mesh, extractor.extract_host(volume, 0.0f, &t));
  vr_example::print_extract(t, voxels_per_block);
  VKC_TRY(vr::io::write_ply(opt.output.path, final_mesh));
  const auto t_end = std::chrono::steady_clock::now();
  const double secs = std::chrono::duration<double>(t_end - t_start).count();
  std::printf(
      "done: fused %zu frames in %.1fs (%.1f fps), final mesh %zu vertices / "
      "%zu triangles -> %s\n",
      fused, secs, fused / (secs > 0.0 ? secs : 1.0),
      final_mesh.vertices.size(), final_mesh.triangle_count(),
      opt.output.path.c_str());
  return {};
}

}  // namespace

int run_main(const Options& opt) {
  const vkc::Status status = run(opt);
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_replica failed: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}

int main(int argc, char** argv) {
  return vr_example::configured_main(
      argc, argv, vr_example::config::Application::FuseReplica, "fuse_replica",
      run_main);
}
