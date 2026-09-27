// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec: live reconstruction from an Orbbec (Femto Mega) camera. Opens
// the camera through sensor::OrbbecCapture, fuses each frame it hands out into
// a sparse TSDF volume -- the same per-frame fuse the dataset example runs
// (examples/common/fuse_frame.hpp) -- and after --frames frames extracts a
// marching-cubes mesh and writes it to a binary PLY.
//
//   fuse_orbbec [--serial SN] [--frames 300] [-o fuse_orbbec.ply]
//               [--voxel 0.02] [--trunc m] [--min-depth m] [--max-depth m]
//               [--max-weight 20]
//
// The camera sits at the world origin (identity pose), so the volume is what
// one fixed view sees, averaged over the run. The run gives up after ten
// seconds without a frame -- a sync secondary with no primary, say -- rather
// than waiting forever.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <thread>

#include "fuse_frame.hpp"
#include "ply_writer.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// How long the loop waits for a frame before calling the camera silent. A
// streaming camera delivers one every ~33 ms; a secondary with no primary, or
// a camera that dropped off the network, delivers none.
constexpr auto kSilenceLimit = std::chrono::seconds(10);

struct Options {
  std::string serial;  // empty: the only camera found
  int frames = 300;    // a camera never runs out, so the run needs an end
  std::string out = "fuse_orbbec.ply";
  float voxel = 0.02f;  // metres
  float trunc = 0.0f;   // truncation distance (metres); 0 => 4 * voxel
  // The depth gate; unset keeps the driver's own defaults.
  std::optional<float> min_depth;
  std::optional<float> max_depth;
  float max_weight = 20.0f;
};

vr::Result<Options> parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const char* v = (i + 1 < argc) ? argv[i + 1] : nullptr;
    auto take = [&]() -> const char* {
      if (v != nullptr) ++i;
      return v;
    };
    auto take_float = [&](float& dst) -> bool {
      const char* s = take();
      if (s == nullptr) return false;
      dst = std::strtof(s, nullptr);
      return true;
    };
    if (a == "--serial") {
      const char* s = take();
      if (s == nullptr)
        return vr::Status::invalid_argument("--serial needs SN");
      opt.serial = s;
    } else if (a == "--frames") {
      const char* s = take();
      if (s == nullptr) return vr::Status::invalid_argument("--frames needs N");
      opt.frames = std::atoi(s);
    } else if (a == "-o" || a == "--out") {
      const char* s = take();
      if (s == nullptr) return vr::Status::invalid_argument("-o needs a path");
      opt.out = s;
    } else if (a == "--voxel") {
      if (!take_float(opt.voxel))
        return vr::Status::invalid_argument("--voxel");
    } else if (a == "--trunc") {
      if (!take_float(opt.trunc))
        return vr::Status::invalid_argument("--trunc");
    } else if (a == "--min-depth") {
      float d = 0.0f;
      if (!take_float(d)) return vr::Status::invalid_argument("--min-depth");
      opt.min_depth = d;
    } else if (a == "--max-depth") {
      float d = 0.0f;
      if (!take_float(d)) return vr::Status::invalid_argument("--max-depth");
      opt.max_depth = d;
    } else if (a == "--max-weight") {
      if (!take_float(opt.max_weight)) {
        return vr::Status::invalid_argument("--max-weight");
      }
    } else {
      return vr::Status::invalid_argument(
          "unknown argument: " + a +
          "\nusage: fuse_orbbec [--serial SN] [--frames N] [-o out.ply] "
          "[--voxel m] [--trunc m] [--min-depth m] [--max-depth m] "
          "[--max-weight w]");
    }
  }
  // strtof takes "nan"/"inf" without complaint, and a NaN knob passes every
  // bound below by comparing false; refuse non-finite values first.
  for (const float knob : {opt.voxel, opt.trunc, opt.max_weight}) {
    if (!std::isfinite(knob)) {
      return vr::Status::invalid_argument(
          "--voxel, --trunc and --max-weight must be finite");
    }
  }
  if (!(opt.voxel > 0.0f)) {
    return vr::Status::invalid_argument("--voxel must be > 0");
  }
  if (opt.trunc <= 0.0f) opt.trunc = 4.0f * opt.voxel;
  if (!(opt.max_weight > 0.0f)) {
    return vr::Status::invalid_argument("--max-weight must be > 0");
  }
  if (opt.frames < 1) {
    return vr::Status::invalid_argument("--frames must be >= 1");
  }
  return opt;
}

vr::Status run(const Options& opt) {
  // --- Camera ---
  // Opened before the GPU so a missing camera fails fast. The depth gate is
  // validated by the driver, which names both values when it refuses one.
  sensor::OrbbecCapture::Options capture_options;
  capture_options.serial = opt.serial;
  if (opt.min_depth) capture_options.min_depth = *opt.min_depth;
  if (opt.max_depth) capture_options.max_depth = *opt.max_depth;
  // An optional so the run can release the camera before the extract: stop()
  // ends the streams but keeps the camera held for a restart.
  std::optional<sensor::OrbbecCapture> camera;
  VR_ASSIGN(camera, sensor::OrbbecCapture::open(capture_options));
  const sensor::OrbbecDeviceInfo info = camera->device_info();
  const vr::ColorCameraParams cam = camera->color_camera();
  std::printf(
      "camera: %s %s, firmware %s, %s %s, sync %s\n"
      "  %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f (depth registered to "
      "colour), depth gate [%.2f, %.2f] m\n",
      info.name.c_str(), info.serial.c_str(), info.firmware_version.c_str(),
      info.connection_type.c_str(), info.ip_address.c_str(),
      sensor::to_string(info.sync_mode), cam.width, cam.height, cam.fx, cam.fy,
      cam.cx, cam.cy, capture_options.min_depth, capture_options.max_depth);
  const bool secondary = sensor::waits_for_primary(info.sync_mode);
  if (secondary) {
    std::printf(
        "  note: a sync %s delivers frames only on another camera's signal\n",
        sensor::to_string(info.sync_mode));
  }

  // --- GPU + volume ---
  VR_ASSIGN(vr::Instance instance, vr::Instance::create({}));
  VR_ASSIGN(VkPhysicalDevice gpu, instance.select_physical_device());
  VR_ASSIGN(vr::Device device, vr::Device::create(instance, gpu, {}));
  VR_ASSIGN(vr::Allocator allocator,
            vr::Allocator::create(instance.handle(), device));

  VR_ASSIGN(
      vol::VoxelBlockGrid volume,
      vr_example::create_fusion_grid(device, allocator, opt.voxel, opt.trunc));
  VR_ASSIGN(tsdf::TsdfIntegrator integrator,
            tsdf::TsdfIntegrator::create(device, allocator, {}));
  VR_ASSIGN(mesh::MarchingCubes extractor,
            mesh::MarchingCubes::create(device, allocator, {}));

  // --- Fuse ---
  sensor::ICameraCapture& capture = *camera;
  VR_TRY(capture.start());
  vr::StageMetrics stage_totals;
  int fused = 0;
  const auto t_start = std::chrono::steady_clock::now();
  auto last_frame = t_start;
  while (fused < opt.frames) {
    VR_ASSIGN(const std::optional<sensor::CapturedFrame> polled,
              capture.poll());
    if (!polled) {
      // A live camera polled faster than it runs. Silence past the limit is a
      // camera that is not coming, and saying which kind beats a hang.
      if (std::chrono::steady_clock::now() - last_frame > kSilenceLimit) {
        return vr::Status::io_error(
            "no frame from camera " + info.serial + " in " +
            std::to_string(kSilenceLimit.count()) + " s" +
            (secondary ? std::string(" -- it is a sync ") +
                             sensor::to_string(info.sync_mode) +
                             ", which captures only on another camera's "
                             "signal; start the camera that drives it"
                       : std::string()));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    last_frame = std::chrono::steady_clock::now();
    VR_TRY(vr_example::fuse_frame(volume, integrator, *polled, opt.max_weight,
                                  &stage_totals));
    ++fused;
    if (fused % 100 == 0) {
      const double secs =
          std::chrono::duration<double>(last_frame - t_start).count();
      std::printf("  fused %d frames (%.1f fps)\n", fused, fused / secs);
    }
  }
  const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start)
          .count();
  const sensor::OrbbecCaptureStats st = camera->stats();
  camera.reset();

  std::printf(
      "camera: %llu pairs received, %llu fused, %llu replaced before a poll "
      "took them, %llu skipped as unprocessable\n",
      static_cast<unsigned long long>(st.received),
      static_cast<unsigned long long>(st.delivered),
      static_cast<unsigned long long>(st.dropped),
      static_cast<unsigned long long>(st.failed));
  std::printf("stages    per fused frame, mean over %d frames\n", fused);
  for (const vr::StageRow& row : stage_totals.rows()) {
    if (row.has_gpu) {
      std::printf("  %-9s host %7.3f ms   device %7.3f ms\n", row.name,
                  row.cpu_ms / fused, row.gpu_ms / fused);
    } else {
      std::printf("  %-9s host %7.3f ms   device       -\n", row.name,
                  row.cpu_ms / fused);
    }
  }

  // --- Mesh -> PLY ---
  VR_ASSIGN(mesh::Mesh final_mesh, extractor.extract_host(volume, 0.0f));
  VR_TRY(vr_example::write_ply(opt.out, final_mesh));
  std::printf(
      "done: fused %d frames in %.1fs (%.1f fps), mesh %zu vertices / %zu "
      "triangles -> %s\n",
      fused, secs, fused / (secs > 0.0 ? secs : 1.0),
      final_mesh.vertices.size(), final_mesh.triangle_count(), opt.out.c_str());
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  vr::Result<Options> opt = parse_args(argc, argv);
  if (!opt) {
    std::fprintf(stderr, "%s\n", opt.status().message().c_str());
    return 2;
  }
  const vr::Status status = run(opt.value());
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_orbbec failed: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}
