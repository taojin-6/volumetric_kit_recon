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

#include <cstdint>

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
/// OpenCV's forward model: a rational radial term and a tangential one. The
/// GPU undistortion samples the captured image at this point for each pixel of
/// the undistorted one, which is why only the forward direction is needed.
/// @param d  The lens.
/// @param p  A normalized pinhole point.
/// @return The normalized point @p p is imaged at.
inline Vec2f distort_normalized(const LensDistortion& d, Vec2f p) noexcept {
  const float r2 = p.x * p.x + p.y * p.y;
  const float r4 = r2 * r2;
  const float r6 = r4 * r2;
  const float radial = (1.0f + d.k1 * r2 + d.k2 * r4 + d.k3 * r6) /
                       (1.0f + d.k4 * r2 + d.k5 * r4 + d.k6 * r6);
  const float xy = p.x * p.y;
  return Vec2f(
      p.x * radial + 2.0f * d.p1 * xy + d.p2 * (r2 + 2.0f * p.x * p.x),
      p.y * radial + d.p1 * (r2 + 2.0f * p.y * p.y) + 2.0f * d.p2 * xy);
}

}  // namespace volumetric_kit::recon::sensor
