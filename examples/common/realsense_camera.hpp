// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/realsense_camera.hpp
/// @brief fuse_realsense's camera and its depth, the scenario's
///        `source.depth`: the camera's own (`sensor::RealSenseSensor`), or
///        FFS on its infrared pair (`sensor::RealSenseStereo` under
///        `sensor::StereoDepthSensor`, where the build has sensor_ffs:
///        VR_EXAMPLE_FFS). The scenario's options for each, the camera's
///        report, and the depth opened as a sensor array of one, so the run
///        is the same for both. Header-only, compiled only into the programs
///        built with the RealSense driver.

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
#include "volumetric_kit/sensor/realsense/realsense_stereo.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#ifdef VR_EXAMPLE_FFS
#include "volumetric_kit/sensor/ffs/stereo_depth_sensor.hpp"
#include "volumetric_kit/sensor/stereo/stereo_rectifier.hpp"
#endif

namespace vr_example {

namespace config = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @return The driver's options for the camera's own depth from a
///         FuseRealsense scenario: the camera (`source.serial`, empty for the
///         only one attached), its modes (the colour and depth sizes and
///         `source.fps`), its depth gate (`fusion.min_depth_m`,
///         `fusion.max_depth_m`) and its projector (`source.emitter`, on
///         unless stated); colour at auto exposure.
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
  o.emitter = opt.source.emitter;
  return o;
}

/// @return The driver's options for FFS depth's infrared pair: the camera,
///         its colour size and `source.fps`, and its projector
///         (`source.emitter`, off unless stated); the infrared cameras at the
///         driver's own mode, which the rectifier resamples to the depth
///         size; colour at auto exposure.
inline sensor::RealSenseStereo::Options realsense_stereo_options(
    const config::Config& opt) {
  sensor::RealSenseStereo::Options o;
  o.serial = opt.source.serial;
  o.color_width = opt.source.color_width;
  o.color_height = opt.source.color_height;
  o.fps = opt.source.fps;
  o.emitter = opt.source.emitter;
  return o;
}

#ifdef VR_EXAMPLE_FFS
/// @return FFS depth's options: the pair rectified to the depth size as luma
///         at alpha 0 (luma carries no coverage, so no border is matched),
///         and the model, its plan cache and the depth gate.
inline sensor::StereoDepthSensor::Options ffs_depth_options(
    const config::Config& opt) {
  sensor::StereoDepthSensor::Options o;
  o.rectify.size = {opt.source.depth_width, opt.source.depth_height};
  o.rectify.alpha = 0.0f;
  o.rectify.output = sensor::StereoRectifier::Output::Luma8;
  o.ffs.model = opt.source.ffs_model;
  o.ffs.plan_cache = opt.source.ffs_plan_cache;
  o.ffs.min_depth = opt.fusion.min_depth_m;
  o.ffs.max_depth = opt.fusion.max_depth_m;
  return o;
}
#endif

/// @return OK when this build can open the scenario's depth;
///         `Status::Code::Unsupported` for FFS in a build without sensor_ffs,
///         saying how to build it. fuse_realsense checks it first, before any
///         GPU or camera opens.
inline vkc::Status check_realsense_depth(const config::Config& opt) {
#ifndef VR_EXAMPLE_FFS
  if (opt.source.depth == config::DepthSource::Ffs) {
    return vkc::Status::unsupported(
        "source.depth ffs needs fuse_realsense built with sensor's FFS "
        "depth, sensor_ffs (VS_WITH_FFS=ON, with VS_WITH_CUDA and TensorRT: "
        "Linux on an NVIDIA GPU); this build has the camera's own depth "
        "only");
  }
#else
  (void)opt;
#endif
  return {};
}

/// @return The depth's line for the run's log, ending in a newline: the
///         camera's own, or FFS at its size with its model, and the
///         projector.
inline std::string realsense_depth_line(const config::Config& opt) {
  const char* projector =
      opt.source.emitter ? ", projector on\n" : ", projector off\n";
  if (opt.source.depth == config::DepthSource::Camera) {
    return std::string("depth: the camera's own") + projector;
  }
  return "depth: FFS on the infrared cameras at " +
         std::to_string(opt.source.depth_width) + "x" +
         std::to_string(opt.source.depth_height) + " (" + opt.source.ffs_model +
         ")" + projector;
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
///         and where the depth camera sits in the colour camera's frame
///         (`SensorInfo::depth_to_color`'s translation).
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
  char at[96];
  std::snprintf(at, sizeof at,
                ", at (%.1f, %.1f, %.1f) mm in the colour camera's frame\n",
                to_color[3].x * 1e3, to_color[3].y * 1e3, to_color[3].z * 1e3);
  return "camera: " + info.name + " " + info.serial + ", firmware " +
         info.firmware_version + ", USB " + info.usb_type + ", " +
         sensor::to_string(sensor.clock) + " clock\n" + "  colour" +
         camera_summary(*sensor.color, lens_name(sensor.color->distortion)) +
         exposure + "\n" + "  depth" +
         camera_summary(*sensor.depth, lens_name(sensor.depth->distortion)) +
         at;
}

/// @brief Open fuse_realsense's depth, print the camera's
///        @ref realsense_camera_report, and read it as an array of one that
///        prepares each frame on @p device. The camera's own depth is a
///        `RealSenseSensor` (@ref realsense_options); FFS depth a
///        `StereoDepthSensor` over the camera's infrared pair
///        (@ref realsense_stereo_options, @ref ffs_depth_options), which
///        rectifies each pair and runs FFS on @p device on a thread of its
///        own, its depth left there for the frame prep. Either way the YUYV
///        colour is converted on the device.
/// @return The array; or the driver's refusal (no camera, several with no
///         `source.serial`, a mode it lacks), FFS's (no model, one of
///         another size, no CUDA on the device; with no cached plan it builds
///         one first, for minutes), @ref check_realsense_depth's, or
///         `SensorArray::open`'s.
inline vkc::Result<sensor::SensorArray> open_realsense_array(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator) {
  VKC_TRY(check_realsense_depth(opt));
  const auto t_open = std::chrono::steady_clock::now();
  const auto seconds = [&t_open] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         t_open)
        .count();
  };
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  std::fputs(realsense_depth_line(opt).c_str(), stdout);
  if (opt.source.depth == config::DepthSource::Camera) {
    VKC_ASSIGN(sensor::RealSenseSensor camera,
               sensor::RealSenseSensor::open(realsense_options(opt)));
    std::fputs(
        realsense_camera_report(camera.device_info(), camera.info()).c_str(),
        stdout);
    sensors.push_back(
        std::make_unique<sensor::RealSenseSensor>(std::move(camera)));
  } else {
#ifdef VR_EXAMPLE_FFS
    VKC_ASSIGN(sensor::RealSenseStereo camera,
               sensor::RealSenseStereo::open(realsense_stereo_options(opt)));
    const sensor::RealSenseDeviceInfo found = camera.device_info();
    std::printf("loading FFS's plan, or building it (minutes, once)\n");
    VKC_ASSIGN(std::unique_ptr<sensor::StereoDepthSensor> depth,
               sensor::StereoDepthSensor::create(
                   device, allocator,
                   std::make_unique<sensor::RealSenseStereo>(std::move(camera)),
                   ffs_depth_options(opt)));
    std::fputs(realsense_camera_report(found, depth->info()).c_str(), stdout);
    sensors.push_back(std::move(depth));
#endif
  }
  std::printf("opened the camera in %.1f s\n", seconds());
  sensor::SensorArray::Options options;
  options.device = &device;
  options.allocator = &allocator;
  return sensor::SensorArray::open(std::move(sensors), options);
}

}  // namespace vr_example
