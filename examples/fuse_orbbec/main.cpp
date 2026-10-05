// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec: live reconstruction from an Orbbec (Femto Mega) camera or a
// synced rig of them. Opens sensor::OrbbecCapture or sensor::OrbbecRig, fuses
// each frame it hands out into a sparse TSDF volume -- the same per-frame fuse
// the dataset example runs (examples/common/fuse_frame.hpp) -- and after
// --frames frames extracts a marching-cubes mesh and writes it to a binary PLY.
//
//   fuse_orbbec [--serial SN | --rig sync.json [--apply-sync]]
//               [--calibration calib.json] [--frames 300] [-o fuse_orbbec.ply]
//               [--hevc | --mjpeg] [--gpu] [--color 1280x720] [--fps 30]
//               [--voxel 0.02] [--trunc m] [--min-depth m] [--max-depth m]
//               [--max-weight 20]
//
// --hevc streams colour as H.265 rather than MJPEG (a build with
// VR_WITH_FFMPEG); --color and --fps pick the colour mode, 4K H.265 running
// at 25 fps at most. --gpu (a build with VR_WITH_FFMPEG too) undistorts and
// converts on the GPU (sensor::GpuFramePrep) instead of on the host, fusing
// depth and colour with their own cameras, the colour decoded onto the GPU;
// it streams H.265 unless --mjpeg says otherwise, since MJPEG needs about
// nine times the bandwidth. With --rig, each set's cameras are prepared at
// once, one thread per camera (sensor::prepare_set), and fused one after
// another.
// --rig fuses every camera of a sync configuration (femto_mega_sync.json) as
// one rig, refusing cameras that differ from it unless --apply-sync writes it
// to them. --calibration poses each camera from a calibration file
// (sensor/rig_calibration.hpp); without it every camera sits at the origin.
// The run gives up after ten seconds without a frame rather than waiting.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fuse_device_frame.hpp"
#include "fuse_frame.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/io/ply_writer.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rig_calibration.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
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
  std::string serial;       // one camera; empty: the only one found
  std::string rig;          // a sync configuration: fuse its cameras as a rig
  std::string calibration;  // poses by serial; empty: all at the origin
  bool apply_sync = false;  // write the sync configuration where it differs
  // H.265 colour rather than MJPEG; unset, H.265 for --gpu alone.
  std::optional<bool> hevc;
  bool gpu = false;               // raw frames, prepared on the GPU
  std::uint32_t color_width = 0;  // 0 keeps the driver's default mode
  std::uint32_t color_height = 0;
  std::uint32_t fps = 0;
  int frames = 300;  // a camera never runs out, so the run needs an end
  std::string out = "fuse_orbbec.ply";
  float voxel = 0.02f;  // metres
  float trunc = 0.0f;   // truncation distance (metres); 0 => 4 * voxel
  // The depth gate; unset keeps the driver's own defaults.
  std::optional<float> min_depth;
  std::optional<float> max_depth;
  float max_weight = 20.0f;
};

vkc::Result<Options> parse_args(int argc, char** argv) {
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
        return vkc::Status::invalid_argument("--serial needs SN");
      opt.serial = s;
    } else if (a == "--rig" || a == "--calibration") {
      const char* s = take();
      if (s == nullptr)
        return vkc::Status::invalid_argument(a + " needs a path");
      (a == "--rig" ? opt.rig : opt.calibration) = s;
    } else if (a == "--apply-sync") {
      opt.apply_sync = true;
    } else if (a == "--hevc" || a == "--mjpeg") {
      const bool hevc = a == "--hevc";
      if (opt.hevc && *opt.hevc != hevc) {
        return vkc::Status::invalid_argument("--hevc or --mjpeg, not both");
      }
      opt.hevc = hevc;
    } else if (a == "--gpu") {
      opt.gpu = true;
    } else if (a == "--color") {
      const char* s = take();
      unsigned w = 0, h = 0;
      if (s == nullptr || std::sscanf(s, "%ux%u", &w, &h) != 2 || w == 0 ||
          h == 0) {
        return vkc::Status::invalid_argument(
            "--color needs WxH, e.g. 1920x1080");
      }
      opt.color_width = w;
      opt.color_height = h;
    } else if (a == "--fps") {
      const char* s = take();
      if (s == nullptr || std::atoi(s) < 1) {
        return vkc::Status::invalid_argument("--fps needs N >= 1");
      }
      opt.fps = static_cast<std::uint32_t>(std::atoi(s));
    } else if (a == "--frames") {
      const char* s = take();
      if (s == nullptr)
        return vkc::Status::invalid_argument("--frames needs N");
      opt.frames = std::atoi(s);
    } else if (a == "-o" || a == "--out") {
      const char* s = take();
      if (s == nullptr) return vkc::Status::invalid_argument("-o needs a path");
      opt.out = s;
    } else if (a == "--voxel") {
      if (!take_float(opt.voxel))
        return vkc::Status::invalid_argument("--voxel");
    } else if (a == "--trunc") {
      if (!take_float(opt.trunc))
        return vkc::Status::invalid_argument("--trunc");
    } else if (a == "--min-depth") {
      float d = 0.0f;
      if (!take_float(d)) return vkc::Status::invalid_argument("--min-depth");
      opt.min_depth = d;
    } else if (a == "--max-depth") {
      float d = 0.0f;
      if (!take_float(d)) return vkc::Status::invalid_argument("--max-depth");
      opt.max_depth = d;
    } else if (a == "--max-weight") {
      if (!take_float(opt.max_weight)) {
        return vkc::Status::invalid_argument("--max-weight");
      }
    } else {
      return vkc::Status::invalid_argument(
          "unknown argument: " + a +
          "\nusage: fuse_orbbec [--serial SN | --rig sync.json [--apply-sync]] "
          "[--calibration calib.json] [--frames N] [--hevc | --mjpeg] "
          "[--gpu] [--color WxH] [--fps N] "
          "[-o out.ply] [--voxel m] [--trunc m] [--min-depth m] "
          "[--max-depth m] [--max-weight w]");
    }
  }
  // strtof takes "nan"/"inf" without complaint, and a NaN knob passes every
  // bound below by comparing false; refuse non-finite values first.
  for (const float knob : {opt.voxel, opt.trunc, opt.max_weight}) {
    if (!std::isfinite(knob)) {
      return vkc::Status::invalid_argument(
          "--voxel, --trunc and --max-weight must be finite");
    }
  }
  if (!(opt.voxel > 0.0f)) {
    return vkc::Status::invalid_argument("--voxel must be > 0");
  }
  if (opt.trunc <= 0.0f) opt.trunc = 4.0f * opt.voxel;
  if (!(opt.max_weight > 0.0f)) {
    return vkc::Status::invalid_argument("--max-weight must be > 0");
  }
  if (opt.frames < 1) {
    return vkc::Status::invalid_argument("--frames must be >= 1");
  }
  return opt;
}

// One camera or a rig, behind the capture contract the fuse loop reads.
struct Source {
  std::optional<sensor::OrbbecCapture> camera;
  std::optional<sensor::OrbbecRig> rig;
  bool waits_for_primary = false;  // one camera, and a sync secondary
  std::string name;                // for the silence message

  sensor::ICameraCapture& capture() {
    return camera ? static_cast<sensor::ICameraCapture&>(*camera) : *rig;
  }
  void print_stats() const {
    const auto line = [](const char* who,
                         const sensor::OrbbecCaptureStats& st) {
      std::printf(
          "  %s: %llu pairs received, %llu fused, %llu dropped, %llu "
          "unprocessable, %llu lost to the colour decoder\n",
          who, static_cast<unsigned long long>(st.received),
          static_cast<unsigned long long>(st.delivered),
          static_cast<unsigned long long>(st.dropped),
          static_cast<unsigned long long>(st.failed),
          static_cast<unsigned long long>(st.lost));
      // Colour meant to stay on the GPU that came to the host instead
      // (OrbbecCaptureStats::host_pictures); never on the host path.
      if (st.host_pictures != 0) {
        std::printf("  %s: %llu frames with colour on the host\n", who,
                    static_cast<unsigned long long>(st.host_pictures));
      }
    };
    if (camera) {
      line(camera->device_info().serial.c_str(), camera->stats());
      return;
    }
    const sensor::OrbbecRigStats st = rig->stats();
    std::printf("rig: %llu sets, %llu missing a camera\n",
                static_cast<unsigned long long>(st.sets),
                static_cast<unsigned long long>(st.incomplete));
    for (std::size_t i = 0; i < st.cameras.size(); ++i) {
      line(rig->device_info(i).serial.c_str(), st.cameras[i]);
    }
  }
};

void print_camera(const sensor::OrbbecDeviceInfo& info,
                  const vr::ColorCameraParams& cam) {
  std::printf(
      "camera: %s %s, firmware %s, %s %s, sync %s\n"
      "  %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f (depth registered to "
      "colour), at (%.3f, %.3f, %.3f) m\n",
      info.name.c_str(), info.serial.c_str(), info.firmware_version.c_str(),
      info.connection_type.c_str(), info.ip_address.c_str(),
      sensor::to_string(info.sync_mode), cam.width, cam.height, cam.fx, cam.fy,
      cam.cx, cam.cy, cam.cam_to_world[3].x, cam.cam_to_world[3].y,
      cam.cam_to_world[3].z);
}

// The colour stream the command line asked for, over the driver's defaults.
// Raw frames' colour is decoded onto `device`, where the hardware leaves it,
// in buffers made through `allocator`.
void apply_streams(const Options& opt, const vkc::Device& device,
                   vkc::Allocator& allocator,
                   sensor::OrbbecStreamOptions& streams) {
  if (opt.hevc.value_or(opt.gpu)) {
    streams.color_codec = sensor::OrbbecColorCodec::Hevc;
  }
  streams.raw = opt.gpu;
  if (opt.gpu) {
    streams.device = &device;
    streams.allocator = &allocator;
  }
  if (opt.color_width != 0) {
    streams.color_width = opt.color_width;
    streams.color_height = opt.color_height;
  }
  if (opt.fps != 0) streams.fps = opt.fps;
}

// Opened on the GPU the frames are decoded and fused on. The depth gate is
// validated by the driver, which names both values when it refuses one.
vkc::Result<Source> open_source(const Options& opt, const vkc::Device& device,
                                vkc::Allocator& allocator) {
  if (!opt.rig.empty() && !opt.serial.empty()) {
    return vkc::Status::invalid_argument(
        "--rig and --serial exclude each other");
  }
  if (opt.apply_sync && opt.rig.empty()) {
    return vkc::Status::invalid_argument("--apply-sync needs --rig");
  }
  std::vector<sensor::RigCameraCalibration> calibration;
  if (!opt.calibration.empty()) {
    VKC_ASSIGN(calibration, sensor::read_rig_calibration(opt.calibration));
  }
  Source source;
  if (!opt.rig.empty()) {
    sensor::OrbbecRig::Options rig_options;
    VKC_ASSIGN(rig_options.sync, sensor::read_orbbec_sync_config(opt.rig));
    rig_options.calibration = calibration;
    rig_options.apply_sync_config = opt.apply_sync;
    if (opt.min_depth) rig_options.min_depth = *opt.min_depth;
    if (opt.max_depth) rig_options.max_depth = *opt.max_depth;
    apply_streams(opt, device, allocator, rig_options);
    VKC_ASSIGN(source.rig, sensor::OrbbecRig::open(rig_options));
    for (std::size_t i = 0; i < source.rig->camera_count(); ++i) {
      print_camera(source.rig->device_info(i), source.rig->color_camera(i));
    }
    source.name = "the rig";
    return source;
  }
  sensor::OrbbecCapture::Options capture_options;
  capture_options.serial = opt.serial;
  if (!calibration.empty()) {
    // One camera, posed from the file: the named one, or the file's only.
    const auto it = std::find_if(
        calibration.begin(), calibration.end(), [&](const auto& c) {
          return opt.serial.empty() ? calibration.size() == 1
                                    : c.serial == opt.serial;
        });
    if (it == calibration.end()) {
      return vkc::Status::not_found(
          opt.calibration + (opt.serial.empty()
                                 ? std::string(" poses several cameras; name "
                                               "one with --serial")
                                 : " has no camera " + opt.serial));
    }
    capture_options.serial = it->serial;
    capture_options.cam_to_world = it->cam_to_world;
  }
  if (opt.min_depth) capture_options.min_depth = *opt.min_depth;
  if (opt.max_depth) capture_options.max_depth = *opt.max_depth;
  apply_streams(opt, device, allocator, capture_options);
  VKC_ASSIGN(source.camera, sensor::OrbbecCapture::open(capture_options));
  const sensor::OrbbecDeviceInfo& info = source.camera->device_info();
  print_camera(info, source.camera->color_camera());
  source.waits_for_primary = sensor::waits_for_primary(info.sync_mode);
  if (source.waits_for_primary) {
    std::printf(
        "  note: a sync %s delivers frames only on another camera's signal\n",
        sensor::to_string(info.sync_mode));
  }
  source.name = "camera " + info.serial;
  return source;
}

vkc::Status run(const Options& opt) {
  // --- GPU, first: the cameras' pictures are decoded onto it ---
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(vr::device_requirements()));
  VKC_ASSIGN(vkc::Device device,
             vkc::Device::create(instance, gpu, vr::device_requirements()));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  // An optional so the run can release the cameras before the extract:
  // stop() ends the streams but keeps the cameras held for a restart. After
  // the device, so every picture on it goes first.
  std::optional<Source> source;
  VKC_ASSIGN(source, open_source(opt, device, allocator));

  // --- Volume ---

  VKC_ASSIGN(
      vol::VoxelBlockGrid volume,
      vr_example::create_fusion_grid(device, allocator, opt.voxel, opt.trunc));
  VKC_ASSIGN(tsdf::TsdfIntegrator integrator,
             tsdf::TsdfIntegrator::create(device, allocator));
  VKC_ASSIGN(mesh::MarchingCubes extractor,
             mesh::MarchingCubes::create(device, allocator, {}));
  // The source says which frames it hands out: raw ones (--gpu) are prepared
  // on the device first, and a raw rig's whole sets at once, a pass a camera.
  sensor::ICameraCapture& capture = source->capture();
  const bool raw_frames = capture.raw_frames();
  const bool raw_sets = raw_frames && source->rig;
  std::vector<sensor::GpuFramePrep> preps;
  std::size_t passes = 0;
  if (raw_frames) passes = raw_sets ? source->rig->camera_count() : 1;
  for (std::size_t c = 0; c < passes; ++c) {
    VKC_ASSIGN(sensor::GpuFramePrep one,
               sensor::GpuFramePrep::create(device, allocator));
    preps.push_back(std::move(one));
  }

  // --- Fuse ---
  VKC_TRY(capture.start());
  vkc::StageMetrics stage_totals;
  int fused = 0;
  int reported = 0;  // the count last reported; a set may step past 100
  const auto t_start = std::chrono::steady_clock::now();
  auto last_frame = t_start;
  while (fused < opt.frames) {
    // The poll's host time counts only when it hands out a frame: an empty
    // poll is the loop waiting, not the driver working. On the host path it
    // is the undistortion, registration and conversion; with --gpu, next to
    // nothing, the work moving to the "frame prep" row.
    const auto t_poll = std::chrono::steady_clock::now();
    bool got = false;
    std::optional<sensor::CapturedFrame> polled;
    std::optional<sensor::RawFrame> raw;
    std::optional<sensor::OrbbecRigRawSet> set;
    if (raw_sets) {
      VKC_ASSIGN(set, source->rig->poll_raw_set());
      got = set && set->count() > 0;
    } else if (raw_frames) {
      VKC_ASSIGN(raw, capture.poll_raw());
      got = raw.has_value();
    } else {
      VKC_ASSIGN(polled, capture.poll());
      got = polled.has_value();
    }
    if (got) {
      stage_totals.add_cpu("poll",
                           std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t_poll)
                               .count());
    }
    if (!got) {
      // A live camera polled faster than it runs. Silence past the limit is a
      // camera that is not coming, and saying which kind beats a hang.
      if (std::chrono::steady_clock::now() - last_frame > kSilenceLimit) {
        return vkc::Status::io_error(
            "no frame from " + source->name + " in " +
            std::to_string(kSilenceLimit.count()) + " s" +
            (source->waits_for_primary
                 ? std::string(" -- it is a sync secondary, which captures "
                               "only on another camera's signal; start the "
                               "camera that drives it")
                 : std::string()));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    last_frame = std::chrono::steady_clock::now();
    if (set) {
      // Timed as one row: the cameras run at once, so their sum would
      // overstate what the set costs.
      const auto t_prep = std::chrono::steady_clock::now();
      VKC_ASSIGN(const std::vector<std::optional<sensor::DeviceFrame>> frames,
                 sensor::prepare_set(preps, set->frames));
      stage_totals.add_cpu("frame prep",
                           std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t_prep)
                               .count());
      // The set's frames up to --frames, fused together.
      std::vector<std::optional<sensor::DeviceFrame>> take;
      for (const std::optional<sensor::DeviceFrame>& frame : frames) {
        if (!frame || fused + static_cast<int>(take.size()) == opt.frames) {
          continue;
        }
        take.push_back(frame);
      }
      VKC_TRY(vr_example::fuse_set(volume, integrator, take, opt.max_weight,
                                   &stage_totals));
      fused += static_cast<int>(take.size());
    } else if (raw) {
      VKC_ASSIGN(const sensor::DeviceFrame frame,
                 preps.front().prepare(*raw, &stage_totals));
      VKC_TRY(vr_example::fuse_frame(volume, integrator, frame, opt.max_weight,
                                     &stage_totals));
      ++fused;
    } else {
      VKC_TRY(vr_example::fuse_frame(volume, integrator, *polled,
                                     opt.max_weight, &stage_totals));
      ++fused;
    }
    if (fused / 100 > reported / 100) {
      reported = fused;
      const double secs =
          std::chrono::duration<double>(last_frame - t_start).count();
      std::printf("  fused %d frames (%.1f fps)\n", fused, fused / secs);
    }
  }
  const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start)
          .count();
  source->print_stats();
  source.reset();

  std::printf("stages    per fused frame, mean over %d frames\n", fused);
  for (const vkc::StageRow& row : stage_totals.rows()) {
    if (row.has_gpu) {
      std::printf("  %-9s host %7.3f ms   device %7.3f ms\n", row.name,
                  row.cpu_ms / fused, row.gpu_ms / fused);
    } else {
      std::printf("  %-9s host %7.3f ms   device       -\n", row.name,
                  row.cpu_ms / fused);
    }
  }

  // --- Mesh -> PLY ---
  VKC_ASSIGN(mesh::Mesh final_mesh, extractor.extract_host(volume, 0.0f));
  VKC_TRY(vr::io::write_ply(opt.out, final_mesh));
  std::printf(
      "done: fused %d frames in %.1fs (%.1f fps), mesh %zu vertices / %zu "
      "triangles -> %s\n",
      fused, secs, fused / (secs > 0.0 ? secs : 1.0),
      final_mesh.vertices.size(), final_mesh.triangle_count(), opt.out.c_str());
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  vkc::Result<Options> opt = parse_args(argc, argv);
  if (!opt) {
    std::fprintf(stderr, "%s\n", opt.status().message().c_str());
    return 2;
  }
  const vkc::Status status = run(opt.value());
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_orbbec failed: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}
