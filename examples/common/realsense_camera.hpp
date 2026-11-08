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
#include "ffs_depth.hpp"
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
/// The D455 infrared pair's disparity noise under FFS, in pixels: the
/// frame-to-frame sigma measured with the 23-36-37 checkpoint at 1280 x 720,
/// projector off, pooled over 0.33-3 m (0.076, 0.042 and 0.016 px a bin).
/// Two bins are above it: 0.33-1 m, and 3-5 m (0.057 px), inside the default
/// 5 m depth gate. The template's 960 x 544 is not measured; held in pixels
/// there, it is worth about 0.053 px at 1280 x 720, above the pooled value
/// but still below those two bins. With the projector on, the bins are
/// 0.017, 0.028, 0.014 and 0.043 px; the value is stated either way. Another
/// checkpoint is measured for its own.
inline constexpr float kD455FfsDisparitySigma = 0.04f;

/// @return FFS depth's options for the infrared cameras: their views as
///         luma (`Luma8`) and @ref kD455FfsDisparitySigma
///         (@ref ffs_depth_options).
inline sensor::StereoDepthSensor::Options realsense_ffs_options(
    const config::Config& opt) {
  return ffs_depth_options(opt, kD455FfsDisparitySigma,
                           sensor::StereoRectifier::Output::Luma8);
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

/// @return An opened camera's report, three lines each ending in a newline:
///         what it reports, its link and clock; its colour camera's model at
///         the opened mode and its exposure as read back; its depth camera's,
///         and where the depth camera sits in the colour camera's frame
///         (`SensorInfo::depth_to_color`'s translation).
/// @param info    The camera's `RealSenseSensor::device_info()`.
/// @param sensor  Its `IRgbdSensor::info()`, with both camera models.
inline std::string realsense_camera_report(
    const sensor::RealSenseDeviceInfo& info, const sensor::SensorInfo& sensor) {
  const sensor::camera::Mat4d& to_color = sensor.depth_to_color;
  char at[96];
  std::snprintf(at, sizeof at,
                ", at (%.1f, %.1f, %.1f) mm in the colour camera's frame\n",
                to_color[3].x * 1e3, to_color[3].y * 1e3, to_color[3].z * 1e3);
  return "camera: " + info.name + " " + info.serial + ", firmware " +
         info.firmware_version + ", USB " + info.usb_type + ", " +
         sensor::to_string(sensor.clock) + " clock\n" + "  colour" +
         camera_summary(*sensor.color, lens_name(sensor.color->distortion)) +
         exposure_text(sensor) + "\n" + "  depth" +
         camera_summary(*sensor.depth, lens_name(sensor.depth->distortion)) +
         at;
}

/// @brief Open fuse_realsense's depth, print the camera's
///        @ref realsense_camera_report, and read it as an array of one that
///        prepares each frame on @p device. The camera's own depth is a
///        `RealSenseSensor` (@ref realsense_options); FFS depth a
///        `StereoDepthSensor` over the camera's infrared pair
///        (@ref realsense_stereo_options, @ref realsense_ffs_options), which
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
  // fuse_realsense refuses this first, before the GPU opens; checked here too
  // so that the opener never hands SensorArray an empty list.
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
    // TODO: give the per-set table FFS's rows (rectify, network) once
    // sensor's StereoDepthSensor reports its stage times; until then its time
    // is inside "wait: cameras", and the per-set counters, which count the
    // whole device, take in the rectifier's submit and wait (DECISIONS.md).
    VKC_ASSIGN(std::unique_ptr<sensor::StereoDepthSensor> depth,
               sensor::StereoDepthSensor::create(
                   device, allocator,
                   std::make_unique<sensor::RealSenseStereo>(std::move(camera)),
                   realsense_ffs_options(opt)));
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
