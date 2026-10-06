// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/lens.hpp
/// @brief A camera before undistortion: pinhole intrinsics and the lens that
///        bends them, in OpenCV's rational model.
///
/// Everything downstream of the capture boundary is pinhole
/// (`core/camera_params.hpp`), so a lens appears only here, where a frame is
/// undistorted: by `sensor/utils`'s GPU pass, whose GLSL mirrors
/// @ref distort_normalized, or by a driver's own filter.
///
/// TODO(sensor): the raw frame takes the camera tier's `CameraModel` in the
/// sensor-interface PR, and these two types go.

#include <cstdint>

#include "volumetric_kit/recon/camera/projection.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief OpenCV's rational lens model, in its coefficient order. All zero is
///        a pinhole lens.
struct LensDistortion {
  float k1 = 0.0f, k2 = 0.0f, p1 = 0.0f, p2 = 0.0f;
  float k3 = 0.0f, k4 = 0.0f, k5 = 0.0f, k6 = 0.0f;
};

/// @brief A camera as it captured: its intrinsics, its image size, and the
///        lens between the two.
///
/// Undistorting keeps the intrinsics and drops the lens, so the pinhole camera
/// of the undistorted image has the same `fx`, `fy`, `cx`, `cy` and size.
/// Pixel centres are at integer coordinates, as OpenCV and this repo's
/// cameras place them.
struct LensCamera {
  float fx = 0.0f;           ///< Focal length x (pixels).
  float fy = 0.0f;           ///< Focal length y (pixels).
  float cx = 0.0f;           ///< Principal point x (pixels).
  float cy = 0.0f;           ///< Principal point y (pixels).
  std::uint32_t width = 0;   ///< Image width (pixels).
  std::uint32_t height = 0;  ///< Image height (pixels).
  LensDistortion lens{};     ///< The lens; all zero for a pinhole camera.
};

/// @brief Where the lens moves a point: normalized pinhole coordinates
///        (`x = (u - cx) / fx`) to the normalized coordinates it is imaged at.
///
/// OpenCV's forward model, the camera tier's `distort_rational` in float. The
/// GPU undistortion samples the captured image at this point for each pixel of
/// the undistorted one, which is why only the forward direction is needed.
/// @param d  The lens.
/// @param p  A normalized pinhole point.
/// @return The normalized point @p p is imaged at.
inline Vec2f distort_normalized(const LensDistortion& d, Vec2f p) noexcept {
  const float c[8] = {d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6};
  Vec2f out;
  camera::distort_rational(c, p.x, p.y, &out.x, &out.y);
  return out;
}

}  // namespace volumetric_kit::recon::sensor
