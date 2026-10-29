// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_orbbec_sets.hpp
/// @brief The Orbbec examples' cameras: opened (fuse_orbbec's and
///        rig_viewer's), read as a sensor array, and each camera's counters,
///        `lost` among them, in the array's report (`live_sets.hpp`).
///        Header-only, compiled only into the examples built with the Orbbec
///        driver.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "live_sets.hpp"
#include "orbbec_config.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/array/rig_sets.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace config = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @return An opened camera's report: what it reports, its cameras' factory
///         models and lens models at the opened modes, its colour exposure as
///         read back, its clock, and where @p calibration puts it. Three
///         lines, each ending in a newline.
/// @param info         The camera's `OrbbecSensor::device_info()`.
/// @param sensor       Its `IRgbdSensor::info()`, with both camera models.
/// @param calibration  The run's calibration; a camera it does not pose sits
///                     at the origin.
inline std::string orbbec_camera_report(
    const sensor::OrbbecDeviceInfo& info, const sensor::SensorInfo& sensor,
    const sensor::camera::ArrayCalibration& calibration) {
  const sensor::camera::SensorCalibration* posed =
      sensor::camera::find_sensor(calibration, info.serial);
  const sensor::camera::Mat4d pose = posed != nullptr && posed->color_to_world
                                         ? *posed->color_to_world
                                         : sensor::camera::Mat4d(1.0);
  const std::string exposure =
      sensor.color_controls
          ? ", exposure " + std::to_string(sensor.color_controls->exposure) +
                " gain " + std::to_string(sensor.color_controls->gain)
          : std::string(", auto exposure");
  char at[96];
  std::snprintf(at, sizeof at, ", at (%.3f, %.3f, %.3f) m\n", pose[3].x,
                pose[3].y, pose[3].z);
  return "camera: " + info.name + " " + info.serial + ", firmware " +
         info.firmware_version + ", " + info.connection_type + " " +
         info.ip_address + ", sync " + sensor::to_string(info.sync_mode) +
         ", " + sensor::to_string(sensor.clock) + " clock\n" + "  colour" +
         camera_summary(*sensor.color, info.color_lens_model) + exposure + at +
         "  depth" + camera_summary(*sensor.depth, info.depth_lens_model) +
         "\n";
}

/// @brief Print an opened camera's @ref orbbec_camera_report.
inline void print_orbbec_camera(
    const sensor::OrbbecSensor& opened,
    const sensor::camera::ArrayCalibration& calibration) {
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
    const sensor::camera::ArrayCalibration& calibration,
    const std::string& serial, const std::string& path) {
  const std::vector<sensor::camera::SensorCalibration>& listed =
      calibration.sensors;
  if (listed.empty()) return serial;
  const auto posed = [](const sensor::camera::SensorCalibration& s) {
    return s.color_to_world.has_value();
  };
  const sensor::camera::SensorCalibration* sensor = nullptr;
  if (!serial.empty()) {
    sensor = sensor::camera::find_sensor(calibration, serial);
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

/// @brief Open a live run's cameras, each printed: one, or every camera of
///        the `source.sync_config` file (`open_orbbec_sensors`), the colour
///        decoded onto @p device, each at @p calibration's colour exposure
///        (`ArrayCalibration::color_controls`; auto exposure without it). A
///        single camera is @ref posed_camera's. The depth gate is validated
///        by the driver, which names both values when it refuses one.
/// @return The cameras; @ref posed_camera's refusal; or the driver's
///         error.
inline vkc::Result<std::vector<std::unique_ptr<sensor::IRgbdSensor>>>
open_orbbec_cameras(const config::Config& opt,
                    const sensor::camera::ArrayCalibration& calibration,
                    const vkc::Device& device, vkc::Allocator& allocator) {
  sensor::OrbbecSensor::Options o;
  apply_orbbec_config(opt, device, allocator, o);
  o.color_controls = calibration.color_controls;
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  const auto t_open = std::chrono::steady_clock::now();
  if (!opt.source.sync_config.empty()) {
    o.apply_sync = opt.source.apply_sync;
    VKC_ASSIGN(sensors, sensor::open_orbbec_sensors(opt.source.sync_config, o));
  } else {
    VKC_ASSIGN(o.serial, posed_camera(calibration, opt.source.serial,
                                      opt.source.calibration));
    o.sync_clock_to_host = opt.source.host_clock;
    VKC_ASSIGN(sensor::OrbbecSensor opened, sensor::OrbbecSensor::open(o));
    sensors.push_back(
        std::make_unique<sensor::OrbbecSensor>(std::move(opened)));
  }
  for (const std::unique_ptr<sensor::IRgbdSensor>& opened : sensors) {
    // Every camera here is an OrbbecSensor.
    print_orbbec_camera(static_cast<const sensor::OrbbecSensor&>(*opened),
                        calibration);
  }
  if (opt.source.sync_config.empty() &&
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

/// @brief fuse_orbbec's cameras (@ref open_orbbec_cameras) read as an array
///        posed by `source.calibration`, preparing each set on @p device --
///        unless the stream operation, whose array builds no GPU pass.
/// @return The array; the calibration file's error, the cameras', or
///         `SensorArray::open`'s.
inline vkc::Result<sensor::SensorArray> open_orbbec_array(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator) {
  sensor::SensorArray::Options options;
  if (!opt.source.calibration.empty()) {
    VKC_ASSIGN(options.calibration,
               sensor::camera::read_array_calibration(opt.source.calibration));
  }
  if (opt.operation != config::Operation::Stream) {
    options.device = &device;
    options.allocator = &allocator;
  }
  VKC_ASSIGN(auto sensors,
             open_orbbec_cameras(opt, options.calibration, device, allocator));
  return sensor::SensorArray::open(std::move(sensors), options);
}

/// @return An Orbbec camera's counters (`OrbbecSensor::orbbec_stats`), for
///         @ref array_report: the interface's, then "<lost> lost".
/// @pre @p camera is an `OrbbecSensor`.
inline std::string orbbec_counters(const sensor::IRgbdSensor& camera) {
  const sensor::OrbbecStreamStats s =
      static_cast<const sensor::OrbbecSensor&>(camera).orbbec_stats();
  char text[192];
  std::snprintf(
      text, sizeof text,
      "%llu received, %llu delivered, %llu dropped, %llu failed, %llu lost",
      static_cast<unsigned long long>(s.received),
      static_cast<unsigned long long>(s.delivered),
      static_cast<unsigned long long>(s.dropped),
      static_cast<unsigned long long>(s.failed),
      static_cast<unsigned long long>(s.lost));
  return text;
}

/// @brief Print @p array's report (@ref array_report) with each camera's
///        @ref orbbec_counters.
/// @param sets  The run's sets; null, for a run that polls the array itself
///              (rig_viewer's), prints no skew.
/// @pre The array's sensors are all `OrbbecSensor`s.
inline void print_rig_stats(sensor::SensorArray& array,
                            const sensor::RigSets* sets) {
  std::fputs(array_report(array, sets, orbbec_counters).c_str(), stdout);
}

}  // namespace vr_example
