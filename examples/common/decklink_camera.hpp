// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/decklink_camera.hpp
/// @brief fuse_decklink's camera: two SDI cameras on a DeckLink card as one
///        stereo camera (`sensor::DecklinkStereo`), its depth from FFS
///        (`sensor::StereoDepthSensor`), read as a sensor array of one,
///        posed by the scenario's calibration file. The scenario's options,
///        the camera's report and its depth noise. Header-only; the camera
///        opens only where the build has sensor_decklink and sensor_ffs
///        (VR_EXAMPLE_DECKLINK, VR_EXAMPLE_FFS), and every other build
///        refuses the scenario before anything opens.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "config.hpp"
#include "ffs_depth.hpp"
#include "live_sets.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/frame/depth_noise.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/stereo/stereo_sensor.hpp"
#ifdef VR_EXAMPLE_FFS
#include "volumetric_kit/sensor/ffs/stereo_depth_sensor.hpp"
#include "volumetric_kit/sensor/stereo/stereo_rectifier.hpp"
#endif
#if defined(VR_EXAMPLE_DECKLINK) && defined(VR_EXAMPLE_FFS)
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/stereo/stereo_calibration.hpp"
#endif
#ifdef VR_EXAMPLE_DECKLINK
#include "volumetric_kit/sensor/decklink/decklink_stereo.hpp"
#endif

namespace vr_example {

namespace config = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// The Blackmagic cameras' disparity noise under FFS, in pixels at
/// 960 x 544. Provisional: the D455's worst measured depth bin (0.076 px,
/// 0.33-1 m, `kD455FfsDisparitySigma`'s study), rounded up.
// TODO: measure it on this stereo camera (a still room, each pixel's
// disparity spread over 300 frames) and state the value here.
inline constexpr float kBlackmagicFfsDisparitySigma = 0.08f;

/// @return OK where this build can open fuse_decklink's camera: sensor's
///         DeckLink driver and its FFS depth both built;
///         `Status::Code::Unsupported` otherwise, naming what is missing and
///         how to build it. fuse_decklink checks it first, before any GPU or
///         camera opens.
inline vkc::Status check_decklink_build() {
  std::vector<std::string> missing;
#ifndef VR_EXAMPLE_DECKLINK
  missing.emplace_back(
      "sensor's DeckLink driver, sensor_decklink (VS_WITH_DECKLINK=ON and "
      "DeckLinkSDK_ROOT: Linux x86-64)");
#endif
#ifndef VR_EXAMPLE_FFS
  missing.emplace_back(
      "sensor's FFS depth, sensor_ffs (VS_WITH_FFS=ON, with VS_WITH_CUDA and "
      "TensorRT: an NVIDIA GPU)");
#endif
  if (missing.empty()) return {};
  std::string parts = missing[0];
  if (missing.size() > 1) parts += " and " + missing[1];
  return vkc::Status::unsupported("the DeckLink stereo camera needs " + parts +
                                  "; this build lacks " +
                                  (missing.size() > 1 ? "both" : "it"));
}

/// @brief The scenario's stereo camera: `source.pairs[0]`'s two cameras,
///        each as `source.cameras` names it (id and input).
struct DecklinkCameras {
  config::LiveCamera left;
  config::LiveCamera right;
};

/// @return `source.pairs[0]`'s left and right cameras of `source.cameras`;
///         the core has checked that both are there.
inline DecklinkCameras decklink_cameras(const config::Config& opt) {
  const auto named = [&opt](const std::string& id) {
    for (const config::LiveCamera& camera : opt.source.cameras) {
      if (camera.id == id) return camera;
    }
    return config::LiveCamera{};
  };
  const config::StereoPairConfig& pair = opt.source.pairs.at(0);
  return {named(pair.left), named(pair.right)};
}

/// @return The distance between the two cameras, metres: the length of
///         `left_to_right`'s translation, which rectification keeps.
inline double stereo_baseline(const sensor::StereoCalibration& pair) {
  const auto& t = pair.left_to_right[3];
  return std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
}

/// @return The depth noise every frame carries, as FFS states it:
///         `DepthNoise::stereo` of @ref kBlackmagicFfsDisparitySigma, the
///         rectified focal length and the baseline. What fuse_decklink's
///         Dynamic fusion reads.
/// @param pair       The stereo camera's calibration.
/// @param rectified  The rectified left camera, FFS's depth camera.
inline sensor::DepthNoise decklink_depth_noise(
    const sensor::StereoCalibration& pair,
    const sensor::camera::CameraModel& rectified) {
  return sensor::DepthNoise::stereo(kBlackmagicFfsDisparitySigma,
                                    rectified.intrinsics.fx,
                                    stereo_baseline(pair));
}

/// @return The opened camera's report, six lines each ending in a newline:
///         the two cameras, their inputs, mode and clock; the colour camera
///         (the left one, lens included); the depth camera (FFS's, the
///         rectified left one); the baseline and how far apart left and
///         right may be; the depth FFS keeps (`fusion.min_depth_m` to
///         `max_depth_m`) and its near limit (`f B /`
///         @ref kFfsMaxDisparity), not yet gated; and the depth noise
///         (@ref decklink_depth_noise) at 2 m and 4 m.
/// @param opt    The scenario.
/// @param pair   The stereo camera's calibration.
/// @param depth  Its depth's `IRgbdSensor::info()`, with both camera models.
inline std::string decklink_camera_report(const config::Config& opt,
                                          const sensor::StereoCalibration& pair,
                                          const sensor::SensorInfo& depth) {
  const DecklinkCameras cameras = decklink_cameras(opt);
  const sensor::camera::CameraModel& rectified = *depth.depth;
  const double baseline = stereo_baseline(pair);
  const sensor::DepthNoise noise = decklink_depth_noise(pair, rectified);
  char mode[96];
  std::snprintf(mode, sizeof mode, ", %ux%u at %u fps, ",
                opt.source.color_width, opt.source.color_height,
                opt.source.fps);
  char numbers[320];
  std::snprintf(
      numbers, sizeof numbers,
      "  baseline %.1f mm, left and right at most %.3f ms apart\n"
      "  depth kept %.2f-%.2f m; FFS's near limit %.2f m, not yet gated\n"
      "  depth noise %.1f mm at 2 m, %.1f mm at 4 m (disparity %.2f px, "
      "provisional)\n",
      baseline * 1e3, opt.source.max_pair_skew_us * 1e-3,
      double{opt.fusion.min_depth_m}, double{opt.fusion.max_depth_m},
      rectified.intrinsics.fx * baseline / double{kFfsMaxDisparity},
      noise.sigma(2.0f) * 1e3, noise.sigma(4.0f) * 1e3,
      double{kBlackmagicFfsDisparitySigma});
  return "camera: DeckLink " + cameras.left.id + " on " + cameras.left.input +
         " (left) and " + cameras.right.id + " on " + cameras.right.input +
         " (right)" + mode + sensor::to_string(depth.clock) + " clock\n" +
         "  colour" +
         camera_summary(*depth.color, lens_name(depth.color->distortion)) +
         "\n  depth" + camera_summary(rectified, "pinhole") +
         ", FFS in the rectified left camera\n" + numbers;
}

#ifdef VR_EXAMPLE_FFS
/// @return FFS depth's options for the DeckLink cameras: their views as
///         R'G'B' (`Rgba8`), as FFS was trained and run on these cameras,
///         and @ref kBlackmagicFfsDisparitySigma (@ref ffs_depth_options).
inline sensor::StereoDepthSensor::Options decklink_ffs_options(
    const config::Config& opt) {
  return ffs_depth_options(opt, kBlackmagicFfsDisparitySigma,
                           sensor::StereoRectifier::Output::Rgba8);
}
#endif

#ifdef VR_EXAMPLE_DECKLINK
/// @return The driver's options: @ref decklink_cameras' connectors, the
///         cameras' mode (`source.color_width`, `color_height`, `fps`), the
///         most left and right may lie apart (`source.max_pair_skew_us`),
///         and @p pair's lenses and pose.
inline sensor::DecklinkStereo::Options decklink_options(
    const config::Config& opt, const sensor::StereoCalibration& pair) {
  const DecklinkCameras cameras = decklink_cameras(opt);
  sensor::DecklinkStereo::Options o;
  o.left = {cameras.left.id, cameras.left.input};
  o.right = {cameras.right.id, cameras.right.input};
  o.calibration = pair;
  o.width = opt.source.color_width;
  o.height = opt.source.color_height;
  o.fps = opt.source.fps;
  o.max_skew_ns = std::uint64_t{opt.source.max_pair_skew_us} * 1000u;
  return o;
}
#endif

/// @brief fuse_decklink's camera, opened.
struct DecklinkArray {
  /// The FFS depth sensor, its colour the left camera's picture, alone in
  /// an array posed by the calibration file, by the left camera's id.
  sensor::SensorArray array;
  /// The depth noise every frame carries (@ref decklink_depth_noise).
  sensor::DepthNoise depth_noise;
};

/// @brief Open fuse_decklink's camera, print its @ref decklink_camera_report,
///        and read it as an array of one that prepares each frame on
///        @p device.
///
/// The calibration file (`source.calibration`) gives the stereo camera
/// (`sensor::stereo_calibration` of `source.pairs[0]`'s two ids); the driver
/// opens the two inputs on @p device (@ref decklink_options), and
/// `StereoDepthSensor` rectifies each frame on @p device as R'G'B' at the
/// depth size and runs FFS on it on a thread of its own
/// (@ref decklink_ffs_options), its
/// depth left there for the frame prep, which converts and undistorts the
/// left camera's 10-bit picture as the frame's colour.
/// @return The array and its depth noise; or @ref check_decklink_build's
///         refusal, the calibration file's (missing, invalid), the stereo
///         camera's (an id it lacks, unposed, lenses of other sizes), the
///         driver's (no card or Desktop Video, a connector without its own
///         input, a camera at another mode), the rectifier's or FFS's (no
///         model, one of another size, no CUDA on the device; with no cached
///         plan it builds one first, for minutes), or `SensorArray::open`'s.
inline vkc::Result<DecklinkArray> open_decklink_array(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator) {
#if !(defined(VR_EXAMPLE_DECKLINK) && defined(VR_EXAMPLE_FFS))
  // fuse_decklink refuses this first, before the GPU opens; refused here too
  // so that no build hands the run an empty array.
  (void)opt;
  (void)device;
  (void)allocator;
  return check_decklink_build();
#else
  const auto t_open = std::chrono::steady_clock::now();
  VKC_ASSIGN(sensor::camera::ArrayCalibration file,
             sensor::camera::read_array_calibration(opt.source.calibration));
  const DecklinkCameras cameras = decklink_cameras(opt);
  VKC_ASSIGN(
      const sensor::StereoCalibration pair,
      sensor::stereo_calibration(file, cameras.left.id, cameras.right.id));
  std::printf("depth: FFS on the DeckLink cameras at %ux%u (%s)\n",
              opt.source.depth_width, opt.source.depth_height,
              opt.source.ffs_model.c_str());
  VKC_ASSIGN(sensor::DecklinkStereo camera,
             sensor::DecklinkStereo::open(decklink_options(opt, pair), device,
                                          allocator));
  std::printf("loading FFS's plan, or building it (minutes, once)\n");
  VKC_ASSIGN(std::unique_ptr<sensor::StereoDepthSensor> depth,
             sensor::StereoDepthSensor::create(
                 device, allocator,
                 std::make_unique<sensor::DecklinkStereo>(std::move(camera)),
                 decklink_ffs_options(opt)));
  std::fputs(decklink_camera_report(opt, pair, depth->info()).c_str(), stdout);
  const sensor::DepthNoise noise =
      decklink_depth_noise(pair, *depth->info().depth);
  std::printf(
      "opened the camera in %.1f s\n",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_open)
          .count());
  std::vector<std::unique_ptr<sensor::IRgbdSensor> > sensors;
  sensors.push_back(std::move(depth));
  sensor::SensorArray::Options options;
  options.calibration = std::move(file);
  options.device = &device;
  options.allocator = &allocator;
  VKC_ASSIGN(sensor::SensorArray array,
             sensor::SensorArray::open(std::move(sensors), options));
  return DecklinkArray{std::move(array), noise};
#endif
}

}  // namespace vr_example
