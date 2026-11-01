// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/ffs_depth.hpp
/// @brief FFS depth's options from a scenario, one copy for every live
///        example whose depth is a stereo camera under sensor's
///        `StereoDepthSensor`: fuse_realsense's infrared cameras and
///        fuse_decklink's SDI cameras. Header-only; the options need
///        sensor_ffs (VR_EXAMPLE_FFS).

#ifdef VR_EXAMPLE_FFS
#include "config.hpp"
#include "volumetric_kit/sensor/ffs/stereo_depth_sensor.hpp"
#include "volumetric_kit/sensor/stereo/stereo_rectifier.hpp"
#endif

namespace vr_example {

/// The exported FFS checkpoint's (23-36-37) disparity search range, in
/// pixels at the rectified size: a surface nearer than `f B / 192` comes
/// back saturated.
inline constexpr float kFfsMaxDisparity = 192.0f;

#ifdef VR_EXAMPLE_FFS
namespace config = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;

/// @return FFS depth's options: the pair rectified to the scenario's depth
///         size at alpha 0 as @p views, the model, its plan cache, the depth
///         gate (`fusion.min_depth_m`, `max_depth_m`) and the near gate at
///         @ref kFfsMaxDisparity (FFS drops depth nearer than 1.05 `f B /`
///         192), and the camera's disparity noise, from which every frame
///         states its depth noise.
/// @param opt              The scenario.
/// @param disparity_sigma  The camera's disparity noise under FFS, in pixels
///                         at the depth size.
/// @param views            `Luma8` for infrared cameras (luma carries no
///                         coverage; alpha 0 leaves no border to match);
///                         `Rgba8` for colour cameras, as FFS was trained.
inline sensor::StereoDepthSensor::Options ffs_depth_options(
    const config::Config& opt, float disparity_sigma,
    sensor::StereoRectifier::Output views) {
  sensor::StereoDepthSensor::Options o;
  o.rectify.size = {opt.source.depth_width, opt.source.depth_height};
  o.rectify.alpha = 0.0f;
  o.rectify.output = views;
  o.ffs.model = opt.source.ffs_model;
  o.ffs.plan_cache = opt.source.ffs_plan_cache;
  o.ffs.min_depth = opt.fusion.min_depth_m;
  o.ffs.max_depth = opt.fusion.max_depth_m;
  o.ffs.disparity_sigma = disparity_sigma;
  o.ffs.max_disparity = kFfsMaxDisparity;
  return o;
}
#endif

}  // namespace vr_example
