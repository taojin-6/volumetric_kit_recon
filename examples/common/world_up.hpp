// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/world_up.hpp
/// @brief The world's up by what a rig's calibration says its world is: the
///        one rule a live run's stream states (`live_up`, `fuse_sender.hpp`)
///        and `rig_viewer` turns about. On the calibration alone, so a viewer
///        takes it without the sender's includes.

#include <algorithm>
#include <array>

#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace vr_example {

/// @return The world's up by what @p rig says the world is:
///         - a tag's world: +z, out of the tag's printed face, straight up for
///           a tag laid face up on the floor;
///         - a sensor's world, or one posing no sensor (an empty @p rig, as
///           for no file): -y, that colour camera's image up (x right, y down,
///           z forward), upright when that camera is mounted level and the
///           right way up, not for one turned on its side;
///         - any camera posed in a world @p rig does not name (version 1, or
///           no `world`): all zero, unknown.
inline std::array<float, 3> calibration_up(
    const volumetric_kit::sensor::camera::ArrayCalibration& rig) {
  namespace camera = volumetric_kit::sensor::camera;
  if (rig.world.apriltag) return {0.0f, 0.0f, 1.0f};
  const bool posed = std::any_of(rig.sensors.begin(), rig.sensors.end(),
                                 [](const camera::SensorCalibration& s) {
                                   return s.color_to_world.has_value();
                                 });
  if (posed && rig.world.sensor.empty()) return {};
  return {0.0f, -1.0f, 0.0f};
}

}  // namespace vr_example
