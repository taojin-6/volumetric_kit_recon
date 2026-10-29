// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/realsense_camera.hpp
/// @brief fuse_realsense's camera: the scenario's options for the RealSense
///        driver (`sensor::RealSenseSensor`), the camera's report, and the
///        camera opened as a sensor array of one. Header-only, compiled only
///        into the programs built with the RealSense driver.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "config.hpp"
#include "live_sets.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/realsense/realsense_sensor.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace config = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @return The driver's options from a FuseRealsense scenario: the camera
///         (`source.serial`, empty for the only one attached), its modes
///         (the colour and depth sizes and `source.fps`) and its depth gate
///         (`fusion.min_depth_m`, `fusion.max_depth_m`). The camera's own
///         depth, so the projector stays on; colour at auto exposure.
inline sensor::RealSenseSensor::Options realsense_options(
    const config::Config& opt) {
  sensor::RealSenseSensor::Options o;
  o.serial = opt.source.serial;
  o.depth_width = opt.source.depth_width;
  o.depth_height = opt.source.depth_height;
  o.color_width = opt.source.color_width;
  o.color_height = opt.source.color_height;
  o.fps = opt.source.fps;
  o.min_depth = opt.fusion.min_depth_m;
  o.max_depth = opt.fusion.max_depth_m;
  return o;
}

/// @return A lens's name for a camera's line: "pinhole" with no distortion,
///         "brown-conrady" with only its five coefficients, else "rational".
inline const char* lens_name(const sensor::camera::RationalDistortion& d) {
  if (d.k4 != 0.0 || d.k5 != 0.0 || d.k6 != 0.0) return "rational";
  if (d.k1 != 0.0 || d.k2 != 0.0 || d.p1 != 0.0 || d.p2 != 0.0 || d.k3 != 0.0) {
    return "brown-conrady";
  }
  return "pinhole";
}

/// @return An opened camera's report, three lines each ending in a newline:
///         what it reports, its link and clock; its colour camera's model at
///         the opened mode and its exposure as read back; its depth camera's,
///         and how far the colour camera sits from it.
/// @param info    The camera's `RealSenseSensor::device_info()`.
/// @param sensor  Its `IRgbdSensor::info()`, with both camera models.
inline std::string realsense_camera_report(
    const sensor::RealSenseDeviceInfo& info, const sensor::SensorInfo& sensor) {
  const std::string exposure =
      sensor.color_controls
          ? ", exposure " + std::to_string(sensor.color_controls->exposure) +
                " gain " + std::to_string(sensor.color_controls->gain)
          : std::string(", auto exposure");
  const sensor::camera::Mat4d& to_color = sensor.depth_to_color;
  char baseline[96];
  std::snprintf(baseline, sizeof baseline,
                ", colour at (%.1f, %.1f, %.1f) mm\n", to_color[3].x * 1e3,
                to_color[3].y * 1e3, to_color[3].z * 1e3);
  return "camera: " + info.name + " " + info.serial + ", firmware " +
         info.firmware_version + ", USB " + info.usb_type + ", " +
         sensor::to_string(sensor.clock) + " clock\n" + "  colour" +
         camera_summary(*sensor.color, lens_name(sensor.color->distortion)) +
         exposure + "\n" + "  depth" +
         camera_summary(*sensor.depth, lens_name(sensor.depth->distortion)) +
         baseline;
}

/// @brief Open fuse_realsense's camera (@ref realsense_options), print its
///        @ref realsense_camera_report, and read it as an array of one that
///        prepares each frame on @p device: its YUYV colour converted there.
/// @return The array; or the driver's refusal (no camera, several with no
///         `source.serial`, a mode it lacks), or `SensorArray::open`'s.
inline vkc::Result<sensor::SensorArray> open_realsense_array(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator) {
  const auto t_open = std::chrono::steady_clock::now();
  VKC_ASSIGN(sensor::RealSenseSensor camera,
             sensor::RealSenseSensor::open(realsense_options(opt)));
  std::fputs(
      realsense_camera_report(camera.device_info(), camera.info()).c_str(),
      stdout);
  std::printf(
      "opened the camera in %.1f s\n",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_open)
          .count());
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensors.push_back(
      std::make_unique<sensor::RealSenseSensor>(std::move(camera)));
  sensor::SensorArray::Options options;
  options.device = &device;
  options.allocator = &allocator;
  return sensor::SensorArray::open(std::move(sensors), options);
}

}  // namespace vr_example
