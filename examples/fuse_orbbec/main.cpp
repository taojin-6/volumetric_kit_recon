// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec: live reconstruction from an Orbbec (Femto Mega) camera or a
// synced rig of them. Reads its cameras (sensor::OrbbecSensor) as a
// sensor::SensorArray, which groups a rig's frames by trigger and prepares each
// set on the GPU in one batch, fuses it into a sparse TSDF volume
// (tsdf::Fuser), and after --frames frames extracts a
// marching-cubes mesh and writes it to a binary PLY.
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
// (femto_mega_sync.json) as one rig on the host's clock, refusing cameras whose
// sync settings differ from it unless --apply-sync writes it to them.
// --calibration poses each camera from a calibration file
// (camera/array_calibration.hpp); without it every camera sits at the origin.
// --host-clock puts one camera on the host's clock
// (OrbbecSensor::Options::sync_clock_to_host), which its first frame's age
// checks; a rig is always on it. The run reports each camera's counters at the
// end, and a rig's skew of each secondary to the primary. It gives up after
// ten seconds without a frame rather than waiting.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cli.hpp"
#include "fuse_frame.hpp"
#include "fusion_flags.hpp"
#include "orbbec_flags.hpp"
#include "stage_table.hpp"
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
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
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
  std::string serial;  // one camera; empty: the only one found
  vr_example::OrbbecFlags cameras;
  vr_example::FusionFlags fusion{0.02f};
  bool host_clock = false;  // one camera on the host's clock
  int frames = 300;         // a camera never runs out, so the run needs an end
  std::string out = "fuse_orbbec.ply";
};

vkc::Result<Options> parse_args(int argc, char** argv) {
  Options opt;
  vr_example::Cli cli("fuse_orbbec");
  cli.option("--serial", "SN", opt.serial);
  opt.cameras.add_to(cli);
  cli.one_of({"--serial", "--rig"})
      .flag("--host-clock", opt.host_clock)
      .option("--frames", "N", opt.frames, 1)
      .option({"-o", "--out"}, "out.ply", opt.out);
  opt.fusion.add_to(cli);
  cli.check([&opt] {
    if (opt.host_clock && !opt.cameras.rig.empty()) {
      return vkc::Status::invalid_argument(
          "--host-clock is for one camera; a rig is always on the host's "
          "clock");
    }
    return vkc::Status{};
  });
  VKC_TRY(cli.parse(argc, argv));
  return opt;
}

// A camera: what it reports, its cameras' factory models at the opened modes,
// its clock, and where the calibration puts it.
void print_camera(const sensor::OrbbecSensor& opened,
                  const camera::ArrayCalibration& calibration) {
  const sensor::OrbbecDeviceInfo& info = opened.device_info();
  const camera::CameraModel& color = *opened.info().color;
  const camera::CameraModel& depth = *opened.info().depth;
  const camera::SensorCalibration* posed =
      camera::find_sensor(calibration, info.serial);
  const camera::Mat4d pose =
      posed != nullptr ? posed->color_to_world : camera::Mat4d(1.0);
  std::printf(
      "camera: %s %s, firmware %s, %s %s, sync %s, %s clock\n"
      "  colour %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f, at (%.3f, %.3f, "
      "%.3f) m\n"
      "  depth %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n",
      info.name.c_str(), info.serial.c_str(), info.firmware_version.c_str(),
      info.connection_type.c_str(), info.ip_address.c_str(),
      sensor::to_string(info.sync_mode), sensor::to_string(opened.info().clock),
      color.size.width, color.size.height, color.intrinsics.fx,
      color.intrinsics.fy, color.intrinsics.cx, color.intrinsics.cy, pose[3].x,
      pose[3].y, pose[3].z, depth.size.width, depth.size.height,
      depth.intrinsics.fx, depth.intrinsics.fy, depth.intrinsics.cx,
      depth.intrinsics.cy);
}

// The run's cameras, opened on the GPU their frames are decoded and fused on:
// one, or every camera of a sync configuration (open_orbbec_sensors). The
// depth gate is validated by the driver, which names both values when it
// refuses one.
vkc::Result<std::vector<std::unique_ptr<sensor::IRgbdSensor>>> open_cameras(
    const Options& opt, const camera::ArrayCalibration& calibration,
    const vkc::Device& device, vkc::Allocator& allocator) {
  sensor::OrbbecSensor::Options o;
  opt.cameras.apply(opt.fusion, device, allocator, o);
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  const auto t_open = std::chrono::steady_clock::now();
  if (!opt.cameras.rig.empty()) {
    o.apply_sync = opt.cameras.apply_sync;
    VKC_ASSIGN(sensors, sensor::open_orbbec_sensors(opt.cameras.rig, o));
  } else {
    o.serial = opt.serial;
    const std::vector<camera::SensorCalibration>& posed = calibration.sensors;
    if (!posed.empty()) {
      // One camera, posed from the file: the named one, or the file's only.
      const camera::SensorCalibration* sensor =
          !opt.serial.empty() ? camera::find_sensor(calibration, opt.serial)
          : posed.size() == 1 ? &posed[0]
                              : nullptr;
      if (sensor == nullptr) {
        return vkc::Status::not_found(
            opt.cameras.calibration +
            (opt.serial.empty()
                 ? std::string(" poses several cameras; name one with --serial")
                 : " does not pose camera " + opt.serial));
      }
      o.serial = sensor->id;
    }
    o.sync_clock_to_host = opt.host_clock;
    VKC_ASSIGN(sensor::OrbbecSensor opened, sensor::OrbbecSensor::open(o));
    sensors.push_back(
        std::make_unique<sensor::OrbbecSensor>(std::move(opened)));
  }
  for (const std::unique_ptr<sensor::IRgbdSensor>& opened : sensors) {
    // Every camera here is an OrbbecSensor.
    print_camera(static_cast<const sensor::OrbbecSensor&>(*opened),
                 calibration);
  }
  if (opt.cameras.rig.empty() &&
      sensors[0]->info().role == sensor::SyncRole::Secondary) {
    std::printf(
        "  note: a sync secondary delivers frames only on another camera's "
        "signal\n");
  }
  std::printf(
      "opened %zu camera(s) in %.1f s\n", sensors.size(),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_open)
          .count());
  return sensors;
}

// A secondary's frame's time less its set's, the primary's, over the run.
struct Skew {
  double sum_ms = 0.0;
  double worst_ms = 0.0;  // the largest magnitude
  std::uint64_t frames = 0;
};

// Each camera's counters, and for a rig each secondary's skew to the primary.
void print_stats(sensor::SensorArray& array, const std::vector<Skew>& skews) {
  const sensor::SensorArrayStats st = array.stats();
  std::printf("%llu sets, %llu missing a camera, %llu frames unmatched\n",
              static_cast<unsigned long long>(st.sets),
              static_cast<unsigned long long>(st.incomplete),
              static_cast<unsigned long long>(st.unmatched));
  for (std::size_t i = 0; i < st.sensors.size(); ++i) {
    const sensor::SensorStats& s = st.sensors[i];
    std::printf(
        "  %s: %llu received, %llu delivered, %llu dropped, %llu failed",
        array.sensor(i).info().id.c_str(),
        static_cast<unsigned long long>(s.received),
        static_cast<unsigned long long>(s.delivered),
        static_cast<unsigned long long>(s.dropped),
        static_cast<unsigned long long>(s.failed));
    if (skews[i].frames > 0) {
      std::printf("; skew to the primary %+.2f ms mean, %.2f ms worst",
                  skews[i].sum_ms / static_cast<double>(skews[i].frames),
                  skews[i].worst_ms);
    }
    std::printf("\n");
  }
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

  // The cameras, read as an array that poses each frame from the calibration
  // and prepares each set on the device, a pass a camera, in one batch. An
  // optional so the run can release the cameras before the extract: stop()
  // ends the streams but keeps the cameras held for a restart. After the
  // device, so every picture on it goes first.
  sensor::SensorArray::Options array_options;
  if (!opt.cameras.calibration.empty()) {
    VKC_ASSIGN(array_options.calibration,
               camera::read_array_calibration(opt.cameras.calibration));
  }
  array_options.device = &device;
  array_options.allocator = &allocator;
  std::optional<sensor::SensorArray> array;
  {
    VKC_ASSIGN(auto sensors,
               open_cameras(opt, array_options.calibration, device, allocator));
    VKC_ASSIGN(array,
               sensor::SensorArray::open(std::move(sensors), array_options));
  }
  const bool rig = !opt.cameras.rig.empty();
  const std::string name =
      rig ? std::string("the rig") : "camera " + array->sensor(0).info().id;
  const bool secondary =
      !rig && array->sensor(0).info().role == sensor::SyncRole::Secondary;

  // --- Volume ---

  VKC_ASSIGN(vol::VoxelBlockGrid volume,
             vr_example::create_fusion_grid(device, allocator, opt.fusion.voxel,
                                            opt.fusion.trunc));
  VKC_ASSIGN(tsdf::Fuser fuser, tsdf::Fuser::create(device, allocator));
  VKC_ASSIGN(mesh::MarchingCubes extractor,
             mesh::MarchingCubes::create(device, allocator, {}));

  // --- Fuse ---
  VKC_TRY(array->start());
  vkc::StageMetrics stage_totals;
  std::vector<Skew> skews(array->size());
  int fused = 0;
  int reported = 0;  // the count last reported; a set may step past 100
  const auto t_start = std::chrono::steady_clock::now();
  auto last_frame = t_start;
  while (fused < opt.frames) {
    // The poll's host time counts only when it hands out a set: an empty
    // poll is the loop waiting, not the driver working.
    const auto t_poll = std::chrono::steady_clock::now();
    VKC_ASSIGN(const std::optional<sensor::FrameSet> set, array->poll_set());
    if (!set) {
      // A live camera polled faster than it runs. Silence past the limit is a
      // camera that is not coming, and saying which kind beats a hang.
      if (std::chrono::steady_clock::now() - last_frame > kSilenceLimit) {
        return vkc::Status::io_error(
            "no frame from " + name + " in " +
            std::to_string(kSilenceLimit.count()) + " s" +
            (secondary
                 ? std::string(" -- it is a sync secondary, which captures "
                               "only on another camera's signal; start the "
                               "camera that drives it")
                 : std::string()));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    stage_totals.add_cpu("poll", std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - t_poll)
                                     .count());
    last_frame = std::chrono::steady_clock::now();
    if (fused == 0 && !rig) {
      // The SDK's host clock is the system clock: on it (--host-clock), the
      // frame is milliseconds old; the camera's own is where it was last set.
      const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch());
      std::printf("  first frame %.1f ms before the host's clock\n",
                  (static_cast<double>(host.count()) -
                   static_cast<double>(set->timestamp_ns)) /
                      1e6);
    }
    for (std::size_t c = 0; c < set->frames.size(); ++c) {
      if (c == array->primary() || !set->frames[c]) continue;
      const double ms = static_cast<double>(static_cast<std::int64_t>(
                            set->frames[c]->timestamp_ns - set->timestamp_ns)) /
                        1e6;
      skews[c].sum_ms += ms;
      skews[c].worst_ms = std::max(skews[c].worst_ms, std::fabs(ms));
      ++skews[c].frames;
    }
    // One row for the batch: the cameras prepare together.
    VKC_ASSIGN(const sensor::DeviceFrameSet prepared,
               array->process(*set, &stage_totals));
    // The frames up to --frames, fused together.
    std::vector<std::optional<sensor::DeviceFrame>> take;
    for (const std::optional<sensor::DeviceFrame>& frame : prepared.frames) {
      if (!frame || fused + static_cast<int>(take.size()) == opt.frames) {
        continue;
      }
      take.push_back(frame);
    }
    VKC_TRY(vr_example::fuse_set(fuser, volume, take, opt.fusion.max_weight,
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
  print_stats(*array, skews);
  array.reset();

  vr_example::print_stage_rows(
      "stages per fused frame, mean over " + std::to_string(fused) + " frames",
      stage_totals, static_cast<std::size_t>(fused));

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
