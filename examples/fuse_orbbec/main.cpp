// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec: live reconstruction from an Orbbec (Femto Mega) camera or a
// synced rig of them. Opens sensor::OrbbecSensor (one camera) or
// sensor::OrbbecRig, prepares each frame on the GPU (sensor::GpuFramePrep), a
// rig's set in one batch, fuses it into a sparse TSDF volume
// (examples/common/fuse_frame.hpp), and after --frames frames extracts
// a marching-cubes mesh and writes it to a binary PLY.
//
//   fuse_orbbec [--serial SN | --rig sync.json [--apply-sync]]
//               [--calibration calib.json] [--frames 300] [-o fuse_orbbec.ply]
//               [--hevc | --mjpeg] [--host-clock] [--color 1280x720]
//               [--fps 30]
//               [--voxel 0.02] [--trunc m] [--min-depth m] [--max-depth m]
//               [--max-weight 20]
//
// Depth and colour are fused with their own cameras, the colour decoded onto
// the GPU: H.265 unless --mjpeg says otherwise, since MJPEG needs about nine
// times the bandwidth at 4K. --color and --fps pick the colour mode, 4K H.265
// running at 25 fps at most. --rig fuses every camera of a sync configuration
// (femto_mega_sync.json) as one rig, refusing cameras that differ from it
// unless --apply-sync writes it to them. --calibration poses each camera from
// a calibration file (camera/array_calibration.hpp); without it every camera
// sits at the origin. --host-clock sets one camera's clock to the host's at
// the start (OrbbecSensor::Options::sync_clock_to_host), which is how that is
// checked; a rig syncs its own. The run gives up after ten seconds without a
// frame rather than waiting.

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

#include "fuse_frame.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/io/ply_writer.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;
namespace sensor = volumetric_kit::recon::sensor;
namespace camera = volumetric_kit::recon::camera;

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
  bool hevc = true;         // H.265 colour rather than MJPEG
  bool host_clock = false;  // one camera on the host's clock
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
  bool codec_given = false;
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
      if (codec_given && opt.hevc != hevc) {
        return vkc::Status::invalid_argument("--hevc or --mjpeg, not both");
      }
      opt.hevc = hevc;
      codec_given = true;
    } else if (a == "--host-clock") {
      opt.host_clock = true;
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
          "[--host-clock] [--color WxH] [--fps N] "
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
  if (opt.host_clock && !opt.rig.empty()) {
    return vkc::Status::invalid_argument(
        "--host-clock is for one camera; a rig syncs its own");
  }
  return opt;
}

// One camera or a rig, its frames as the cameras captured them.
// TODO(sensor): read a SensorArray of OrbbecSensors instead, once
// OrbbecSensor checks and writes the sync settings OrbbecRig does.
struct Source {
  std::optional<sensor::OrbbecSensor> sensor;
  std::optional<sensor::OrbbecRig> rig;
  bool waits_for_primary = false;  // one camera, and a sync secondary
  std::string name;                // for the silence message

  std::size_t cameras() const { return rig ? rig->camera_count() : 1; }
  vkc::Status start() { return rig ? rig->start() : sensor->start(); }
  // The frames this poll took, one entry per camera: a rig's set, or the
  // camera's newest frame; none when nothing new has arrived.
  vkc::Result<std::vector<std::optional<sensor::RgbdFrame>>> poll() {
    std::vector<std::optional<sensor::RgbdFrame>> frames;
    if (rig) {
      VKC_ASSIGN(std::optional<sensor::OrbbecRigSet> set, rig->poll_set());
      if (set) frames = std::move(set->frames);
    } else {
      VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, sensor->poll());
      if (frame) frames.push_back(std::move(frame));
    }
    return frames;
  }
  void print_stats() const {
    const auto line = [](const char* who, const sensor::OrbbecStreamStats& st) {
      std::printf(
          "  %s: %llu pairs received, %llu fused, %llu dropped, %llu "
          "unprocessable, %llu lost to the colour decoder\n",
          who, static_cast<unsigned long long>(st.received),
          static_cast<unsigned long long>(st.delivered),
          static_cast<unsigned long long>(st.dropped),
          static_cast<unsigned long long>(st.failed),
          static_cast<unsigned long long>(st.lost));
    };
    if (sensor) {
      line(sensor->device_info().serial.c_str(), sensor->orbbec_stats());
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

// A camera: what it reports, its colour camera's factory model at the opened
// mode, and where it sits.
void print_camera(const sensor::OrbbecDeviceInfo& info,
                  const camera::CameraModel& color, const camera::Mat4d& pose) {
  std::printf(
      "camera: %s %s, firmware %s, %s %s, sync %s\n"
      "  colour %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f, at (%.3f, %.3f, "
      "%.3f) m\n",
      info.name.c_str(), info.serial.c_str(), info.firmware_version.c_str(),
      info.connection_type.c_str(), info.ip_address.c_str(),
      sensor::to_string(info.sync_mode), color.size.width, color.size.height,
      color.intrinsics.fx, color.intrinsics.fy, color.intrinsics.cx,
      color.intrinsics.cy, pose[3].x, pose[3].y, pose[3].z);
}

// The streams the command line asked for, over the driver's defaults, the
// colour decoded onto `device`, where the hardware leaves it, in buffers made
// through `allocator`.
void apply_streams(const Options& opt, const vkc::Device& device,
                   vkc::Allocator& allocator,
                   sensor::OrbbecStreamOptions& streams) {
  streams.color_codec = opt.hevc ? sensor::OrbbecColorCodec::Hevc
                                 : sensor::OrbbecColorCodec::Mjpeg;
  streams.device = &device;
  streams.allocator = &allocator;
  if (opt.color_width != 0) {
    streams.color_width = opt.color_width;
    streams.color_height = opt.color_height;
  }
  if (opt.fps != 0) streams.fps = opt.fps;
  if (opt.min_depth) streams.min_depth = *opt.min_depth;
  if (opt.max_depth) streams.max_depth = *opt.max_depth;
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
  camera::ArrayCalibration calibration;
  if (!opt.calibration.empty()) {
    VKC_ASSIGN(calibration, camera::read_array_calibration(opt.calibration));
  }
  Source source;
  if (!opt.rig.empty()) {
    sensor::OrbbecRig::Options rig_options;
    VKC_ASSIGN(rig_options.sync, sensor::read_orbbec_sync_config(opt.rig));
    rig_options.calibration = calibration;
    rig_options.apply_sync_config = opt.apply_sync;
    apply_streams(opt, device, allocator, rig_options);
    VKC_ASSIGN(source.rig, sensor::OrbbecRig::open(rig_options));
    for (std::size_t i = 0; i < source.rig->camera_count(); ++i) {
      print_camera(source.rig->device_info(i), source.rig->color_camera(i),
                   source.rig->color_to_world(i));
    }
    source.name = "the rig";
    return source;
  }
  sensor::OrbbecSensor::Options sensor_options;
  sensor_options.serial = opt.serial;
  const std::vector<camera::SensorCalibration>& sensors = calibration.sensors;
  if (!sensors.empty()) {
    // One camera, posed from the file: the named one, or the file's only.
    const camera::SensorCalibration* sensor =
        !opt.serial.empty()   ? camera::find_sensor(calibration, opt.serial)
        : sensors.size() == 1 ? &sensors[0]
                              : nullptr;
    if (sensor == nullptr) {
      return vkc::Status::not_found(
          opt.calibration +
          (opt.serial.empty()
               ? std::string(" poses several cameras; name one with --serial")
               : " does not pose camera " + opt.serial));
    }
    sensor_options.serial = sensor->id;
    sensor_options.color_to_world = sensor->color_to_world;
  }
  sensor_options.sync_clock_to_host = opt.host_clock;
  apply_streams(opt, device, allocator, sensor_options);
  VKC_ASSIGN(source.sensor, sensor::OrbbecSensor::open(sensor_options));
  const sensor::OrbbecDeviceInfo& info = source.sensor->device_info();
  const sensor::SensorInfo& sensor_info = source.sensor->info();
  print_camera(info, *sensor_info.color, sensor_options.color_to_world);
  const camera::CameraModel& depth = *sensor_info.depth;
  std::printf("  depth %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n  %s clock\n",
              depth.size.width, depth.size.height, depth.intrinsics.fx,
              depth.intrinsics.fy, depth.intrinsics.cx, depth.intrinsics.cy,
              sensor::to_string(sensor_info.clock));
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
  // Each poll's frames are prepared on the device in one batch, a pass a
  // camera.
  std::vector<sensor::GpuFramePrep> preps;
  for (std::size_t c = 0; c < source->cameras(); ++c) {
    VKC_ASSIGN(sensor::GpuFramePrep one,
               sensor::GpuFramePrep::create(device, allocator));
    preps.push_back(std::move(one));
  }

  // --- Fuse ---
  VKC_TRY(source->start());
  vkc::StageMetrics stage_totals;
  int fused = 0;
  int reported = 0;  // the count last reported; a set may step past 100
  const auto t_start = std::chrono::steady_clock::now();
  auto last_frame = t_start;
  while (fused < opt.frames) {
    // The poll's host time counts only when it hands out a frame: an empty
    // poll is the loop waiting, not the driver working.
    const auto t_poll = std::chrono::steady_clock::now();
    VKC_ASSIGN(const std::vector<std::optional<sensor::RgbdFrame>> polled,
               source->poll());
    const bool got =
        std::any_of(polled.begin(), polled.end(),
                    [](const auto& frame) { return frame.has_value(); });
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
    if (fused == 0 && source->sensor) {
      // The SDK's host clock is the system clock: on it (--host-clock), the
      // frame is milliseconds old; the camera's own is where it was last set.
      const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch());
      std::printf("  first frame %.1f ms before the host's clock\n",
                  (static_cast<double>(host.count()) -
                   static_cast<double>(polled.front()->timestamp_ns)) /
                      1e6);
    }
    // One row for the batch: the cameras prepare together.
    VKC_ASSIGN(
        const std::vector<std::optional<sensor::DeviceFrame>> frames,
        sensor::GpuFramePrep::prepare_batch(preps, polled, &stage_totals));
    // The frames up to --frames, fused together.
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
