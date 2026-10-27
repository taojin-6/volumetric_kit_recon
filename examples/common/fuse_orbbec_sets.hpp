// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_orbbec_sets.hpp
/// @brief fuse_orbbec's cameras: opened and read as a sensor array, and the
///        report of the array's counters and a rig's skew over its sets
///        (`sensor::RigSets`). Header-only, compiled only into the examples
///        built with the Orbbec driver.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "orbbec_config.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/sensor/array/rig_sets.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @return An opened camera's report: what it reports, its cameras' factory
///         models and lens models at the opened modes, its clock, and where
///         @p calibration puts it. Three lines, each ending in a newline.
/// @param info         The camera's `OrbbecSensor::device_info()`.
/// @param sensor       Its `IRgbdSensor::info()`, with both camera models.
/// @param calibration  The run's calibration; a camera it does not pose sits
///                     at the origin.
inline std::string orbbec_camera_report(
    const vr::sensor::OrbbecDeviceInfo& info,
    const vr::sensor::SensorInfo& sensor,
    const vr::camera::ArrayCalibration& calibration) {
  const vr::camera::SensorCalibration* posed =
      vr::camera::find_sensor(calibration, info.serial);
  const vr::camera::Mat4d pose = posed != nullptr && posed->color_to_world
                                     ? *posed->color_to_world
                                     : vr::camera::Mat4d(1.0);
  // " WxH <lens> @ fx=.. fy=.. cx=.. cy=..", a camera's part of its line.
  const auto camera = [](const vr::camera::CameraModel& model,
                         const std::string& lens) {
    char size[32];
    char intrinsics[96];
    std::snprintf(size, sizeof size, " %ux%u ", model.size.width,
                  model.size.height);
    std::snprintf(intrinsics, sizeof intrinsics,
                  " @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f", model.intrinsics.fx,
                  model.intrinsics.fy, model.intrinsics.cx,
                  model.intrinsics.cy);
    return size + lens + intrinsics;
  };
  char at[96];
  std::snprintf(at, sizeof at, ", at (%.3f, %.3f, %.3f) m\n", pose[3].x,
                pose[3].y, pose[3].z);
  return "camera: " + info.name + " " + info.serial + ", firmware " +
         info.firmware_version + ", " + info.connection_type + " " +
         info.ip_address + ", sync " + vr::sensor::to_string(info.sync_mode) +
         ", " + vr::sensor::to_string(sensor.clock) + " clock\n" + "  colour" +
         camera(*sensor.color, info.color_lens_model) + at + "  depth" +
         camera(*sensor.depth, info.depth_lens_model) + "\n";
}

/// @brief Print an opened camera's @ref orbbec_camera_report.
inline void print_orbbec_camera(
    const vr::sensor::OrbbecSensor& opened,
    const vr::camera::ArrayCalibration& calibration) {
  std::fputs(
      orbbec_camera_report(opened.device_info(), opened.info(), calibration)
          .c_str(),
      stdout);
}

/// @brief The camera a single-camera run opens: @p serial, which
///        @p calibration (read from @p path) must pose if it lists any
///        camera, or else the file's only posed camera. A file may list a
///        camera's lenses alone.
/// @return The serial, empty for "the first camera found" when both
///         @p serial and @p calibration are empty; or
///         `Status::Code::NotFound` naming @p path when the file does not
///         pose @p serial, or poses no camera or several and @p serial is
///         empty.
inline vkc::Result<std::string> posed_camera(
    const vr::camera::ArrayCalibration& calibration, const std::string& serial,
    const std::string& path) {
  const std::vector<vr::camera::SensorCalibration>& listed =
      calibration.sensors;
  if (listed.empty()) return serial;
  const auto posed = [](const vr::camera::SensorCalibration& s) {
    return s.color_to_world.has_value();
  };
  const vr::camera::SensorCalibration* sensor = nullptr;
  if (!serial.empty()) {
    sensor = vr::camera::find_sensor(calibration, serial);
  } else if (std::count_if(listed.begin(), listed.end(), posed) == 1) {
    sensor = &*std::find_if(listed.begin(), listed.end(), posed);
  }
  if (sensor == nullptr || !posed(*sensor)) {
    return vkc::Status::not_found(
        path + (serial.empty() ? std::string(" does not pose exactly one "
                                             "camera; name one with "
                                             "source.serial")
                               : " does not pose camera " + serial));
  }
  return sensor->id;
}

/// @brief Open fuse_orbbec's cameras, each printed: one, or every camera of
///        the `source.sync_config` file (`open_orbbec_sensors`), the colour
///        decoded onto
///        @p device. A single camera is @ref posed_camera's. The depth gate
///        is validated by the driver, which names both values when it
///        refuses one.
/// @return The cameras; @ref posed_camera's refusal; or the driver's
///         error.
inline vkc::Result<std::vector<std::unique_ptr<vr::sensor::IRgbdSensor>>>
open_orbbec_cameras(const config::Config& opt,
                    const vr::camera::ArrayCalibration& calibration,
                    const vkc::Device& device, vkc::Allocator& allocator) {
  vr::sensor::OrbbecSensor::Options o;
  apply_orbbec_config(opt, device, allocator, o);
  std::vector<std::unique_ptr<vr::sensor::IRgbdSensor>> sensors;
  const auto t_open = std::chrono::steady_clock::now();
  if (!opt.source.sync_config.empty()) {
    o.apply_sync = opt.source.apply_sync;
    VKC_ASSIGN(sensors,
               vr::sensor::open_orbbec_sensors(opt.source.sync_config, o));
  } else {
    VKC_ASSIGN(o.serial, posed_camera(calibration, opt.source.serial,
                                      opt.source.calibration));
    o.sync_clock_to_host = opt.source.host_clock;
    VKC_ASSIGN(vr::sensor::OrbbecSensor opened,
               vr::sensor::OrbbecSensor::open(o));
    sensors.push_back(
        std::make_unique<vr::sensor::OrbbecSensor>(std::move(opened)));
  }
  for (const std::unique_ptr<vr::sensor::IRgbdSensor>& opened : sensors) {
    // Every camera here is an OrbbecSensor.
    print_orbbec_camera(static_cast<const vr::sensor::OrbbecSensor&>(*opened),
                        calibration);
  }
  if (opt.source.sync_config.empty() &&
      sensors[0]->info().role == vr::sensor::SyncRole::Secondary) {
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

/// @brief fuse_orbbec's cameras (@ref open_orbbec_cameras) read as an array
///        posed by `source.calibration`, preparing each set on @p device --
///        unless the stream operation, whose array builds no GPU pass.
/// @return The array; the calibration file's error, the cameras', or
///         `SensorArray::open`'s.
inline vkc::Result<vr::sensor::SensorArray> open_orbbec_array(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator) {
  vr::sensor::SensorArray::Options options;
  if (!opt.source.calibration.empty()) {
    VKC_ASSIGN(options.calibration,
               vr::camera::read_array_calibration(opt.source.calibration));
  }
  if (opt.operation != config::Operation::Stream) {
    options.device = &device;
    options.allocator = &allocator;
  }
  VKC_ASSIGN(auto sensors,
             open_orbbec_cameras(opt, options.calibration, device, allocator));
  return vr::sensor::SensorArray::open(std::move(sensors), options);
}

/// @return Whether every sensor of @p array stamps its frames on the host's
///         clock, so a set's time is on the system clock: a rig's always
///         are, a single camera's only with `source.host_clock`.
inline bool on_host_clock(vr::sensor::SensorArray& array) {
  for (std::size_t i = 0; i < array.size(); ++i) {
    if (array.sensor(i).info().clock != vr::sensor::ClockDomain::Host) {
      return false;
    }
  }
  return true;
}

/// @brief Print a single camera's first set's age against the host's clock,
///        once @p sets has handed out @p set, its first; nothing for a rig or
///        a later set. The SDK's host clock is the system clock: on it
///        (`source.host_clock`) the frame is milliseconds old; the camera's
///        own is where it was last set.
inline void print_first_frame_age(vr::sensor::SensorArray& array,
                                  const vr::sensor::RigSets& sets,
                                  const vr::sensor::FrameSet& set) {
  if (sets.sets() != 1 || array.size() != 1) return;
  const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  std::printf("  first frame %.1f ms before the host's clock\n",
              (static_cast<double>(host.count()) -
               static_cast<double>(set.timestamp_ns)) /
                  1e6);
}

/// @brief Print @p array's sets, each camera's counters, `lost` included,
///        and for a rig each secondary's mean and worst skew to the primary
///        over @p sets.
/// @param sets  The run's sets; null, for a run that polls the array itself
///              (rig_viewer's), prints no skew.
/// @pre The array's sensors are all `OrbbecSensor`s.
inline void print_rig_stats(vr::sensor::SensorArray& array,
                            const vr::sensor::RigSets* sets) {
  const vr::sensor::SensorArrayStats st = array.stats();
  std::printf("%llu sets, %llu missing a camera, %llu frames unmatched\n",
              static_cast<unsigned long long>(st.sets),
              static_cast<unsigned long long>(st.incomplete),
              static_cast<unsigned long long>(st.unmatched));
  for (std::size_t i = 0; i < array.size(); ++i) {
    const auto& camera =
        static_cast<const vr::sensor::OrbbecSensor&>(array.sensor(i));
    const vr::sensor::OrbbecStreamStats s = camera.orbbec_stats();
    std::printf(
        "  %s: %llu received, %llu delivered, %llu dropped, %llu failed, "
        "%llu lost",
        camera.info().id.c_str(), static_cast<unsigned long long>(s.received),
        static_cast<unsigned long long>(s.delivered),
        static_cast<unsigned long long>(s.dropped),
        static_cast<unsigned long long>(s.failed),
        static_cast<unsigned long long>(s.lost));
    const vr::sensor::RigSets::Skew lag =
        sets != nullptr ? sets->skew(i) : vr::sensor::RigSets::Skew{};
    if (lag.frames > 0) {
      std::printf("; skew to the primary %+.2f ms mean, %.2f ms worst",
                  lag.sum_ms / static_cast<double>(lag.frames), lag.worst_ms);
    }
    std::printf("\n");
  }
}

}  // namespace vr_example
