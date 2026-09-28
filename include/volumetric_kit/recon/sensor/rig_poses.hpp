// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/rig_poses.hpp
/// @brief The rig pose file: where each camera of a fixed rig sits, as a
///        calibration writes it and a capture reads it.
///
/// A JSON document -- written by `volumetric_kit_calib`, or by
/// @ref write_rig_poses:
///
/// @code{.json}
/// {
///   "format": "volumetric_kit/rig_poses",
///   "version": 1,
///   "units": "m",
///   "camera_axes": "opencv",
///   "cameras": [
///     {"serial": "CL2A141000N", "sensor": "color",
///      "cam_to_world": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],
///                       [0, 0, 0, 1]]}
///   ]
/// }
/// @endcode
///
/// `cam_to_world` is **row-major** -- each inner array is a row, as numpy's
/// `tolist()` writes a matrix -- and maps the camera's frame into the rig's
/// world. `units`, `camera_axes` and each camera's `sensor` are required
/// declarations, not defaults: a pose in millimetres, in the OpenGL camera
/// convention or of the depth camera reads as a valid rig and reconstructs a
/// wrong one, so the writer says which it wrote. Version 1 accepts metres, the
/// OpenCV axes (+X right, +Y down, +Z forward) and the colour camera -- the one
/// a registered frame is posed by. Keys it does not know are ignored, so a
/// later writer can add fields without breaking this reader.

#include <string>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/export.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief One camera of a rig and where it sits.
struct RigCameraPose {
  /// The camera's serial number -- how a capture finds it.
  std::string serial;
  /// Colour camera -> world: metres, this repo's camera axes, column-major,
  /// ready for `ColorCameraParams::cam_to_world`.
  Mat4f cam_to_world = Mat4f(1.0f);
};

/// @brief Parse a rig pose document.
///
/// @param json  The document's text.
/// @return One pose per camera, in the document's order; or
///         @ref Status::Code::InvalidArgument naming what is wrong: text that
///         is not JSON, a `format` or `version` this reader does not know, a
///         missing or unsupported `units` / `camera_axes` / `sensor`, no
///         cameras, an empty or repeated serial, or a `cam_to_world` that is
///         not a 4x4 rigid transform (a rotation that is not orthonormal to
///         1e-4 or that reflects, or a bottom row other than `[0, 0, 0, 1]`).
VR_SENSOR_API Result<std::vector<RigCameraPose>> parse_rig_poses(
    const std::string& json);

/// @brief Read and parse the rig pose file at @p path.
/// @return As @ref parse_rig_poses, or @ref Status::Code::IoError if the file
///         cannot be read. Messages name the file.
VR_SENSOR_API Result<std::vector<RigCameraPose>> read_rig_poses(
    const std::string& path);

/// @brief Write @p poses as a rig pose document that @ref read_rig_poses
///        reads back exactly.
/// @return OK; @ref Status::Code::InvalidArgument for poses the reader would
///         refuse (checked before anything is written); or
///         @ref Status::Code::IoError if the file cannot be written.
VR_SENSOR_API Status write_rig_poses(const std::string& path,
                                     const std::vector<RigCameraPose>& poses);

}  // namespace volumetric_kit::recon::sensor
