// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/rig_calibration.hpp
/// @brief A fixed rig's calibration file: each camera's pose and lens, as
///        calib writes it and a capture reads it.
///
/// The `device_calibration` section of a JSON document, one entry per serial;
/// other sections, and entries that are not objects, are ignored:
///
/// @code{.json}
/// {"device_calibration": {
///   "CL2A141000N": {
///     "intrinsics": {"fx": 746.7, "fy": 746.5, "cx": 637.7, "cy": 346.4},
///     "distortion": {"k1": 0.08, "k2": -0.1, "p1": 0, "p2": 0, "k3": 0.04,
///                    "k4": 0, "k5": 0, "k6": 0},
///     "optimal_intrinsics": {"fx": 740.2, "fy": 741.0, "cx": 636.9,
///                            "cy": 345.8},
///     "pose": {"rvec": [0, 0, 0], "tvec": [0, 0, 0]}}}}
/// @endcode
///
/// `pose` is the colour camera's OpenCV extrinsic, `x_cam = R(rvec) x_world +
/// tvec`: `rvec` a Rodrigues vector in radians, `tvec` in metres, the world
/// the reference camera's frame. It is the one required field.

#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/export.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief Pinhole intrinsics, in pixels of the image the calibration used.
struct PinholeIntrinsics {
  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
};

/// @brief OpenCV's rational lens model, in its coefficient order.
struct LensDistortion {
  float k1 = 0.0f, k2 = 0.0f, p1 = 0.0f, p2 = 0.0f;
  float k3 = 0.0f, k4 = 0.0f, k5 = 0.0f, k6 = 0.0f;
};

/// @brief One camera of a calibrated rig.
struct RigCameraCalibration {
  std::string serial;  ///< How a capture finds the camera.
  /// Colour camera -> world: the inverse of the file's `pose`, in this repo's
  /// camera axes and column-major, ready for `ColorCameraParams`.
  Mat4f cam_to_world = Mat4f(1.0f);
  std::optional<PinholeIntrinsics> intrinsics;          ///< If the file has it.
  std::optional<LensDistortion> distortion;             ///< If the file has it.
  std::optional<PinholeIntrinsics> optimal_intrinsics;  ///< If the file has it.
};

/// @brief Parse a calibration document.
/// @return One entry per camera, ordered by serial; or
///         @ref Status::Code::InvalidArgument naming what is wrong: not JSON,
///         no `device_calibration` object or no camera in it, a camera without
///         a `pose` of two 3-vectors, or a field that fails
///         @ref validate_rig_calibration.
VR_SENSOR_API Result<std::vector<RigCameraCalibration>> parse_rig_calibration(
    const std::string& json);

/// @brief Read and parse the calibration file at @p path.
/// @return As @ref parse_rig_calibration, messages naming the file; or
///         @ref Status::Code::IoError if it cannot be read.
VR_SENSOR_API Result<std::vector<RigCameraCalibration>> read_rig_calibration(
    const std::string& path);

/// @brief The checks a calibration must pass to be read or written: at least
///        one camera, each serial non-empty, UTF-8 and unique, `cam_to_world`
///        a rigid transform, and any intrinsics finite with positive focal
///        lengths, any distortion finite.
VR_SENSOR_API Status
validate_rig_calibration(const std::vector<RigCameraCalibration>& cameras);

/// @brief Write @p cameras as a calibration document (the
///        `device_calibration` section alone) that @ref read_rig_calibration
///        reads back to within float round-off.
/// @return OK; @ref Status::Code::InvalidArgument for a calibration
///         @ref validate_rig_calibration refuses; or
///         @ref Status::Code::IoError if the file cannot be written.
VR_SENSOR_API Status write_rig_calibration(
    const std::string& path, const std::vector<RigCameraCalibration>& cameras);

}  // namespace volumetric_kit::recon::sensor
