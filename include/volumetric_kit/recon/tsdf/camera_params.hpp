// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/camera_params.hpp
/// @brief The posed pinhole color camera a @ref ColorFrame was captured with.
///
/// The color-camera counterpart of `volume/camera_params.hpp`, split out of
/// `tsdf/tsdf_integrator.hpp` for the same reason: describing a camera should
/// not cost compiling the integrator that consumes one. Host-side POD over GLM,
/// no Vulkan. `tsdf_integrator.hpp` includes this header, so every existing
/// user is unaffected.

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief Pinhole intrinsics + pose + image dimensions of the (separate) color
///        camera a @ref ColorFrame was captured with.
///
/// The color-camera analogue of @ref volume::DepthCameraParams, without the
/// depth-range fields a color image has no use for. Uploaded verbatim to the
/// integrate kernel's color-camera SSBO and read through scalar block layout,
/// so it packs byte-for-byte to the shader's `ColorCameraParams`: the scalars
/// at their natural 4-byte offsets, the `mat4` at offset 24. The
/// `static_assert`s pin that layout (a drift is a compile error, not silent
/// misprojection).
struct ColorCameraParams {
  float fx;              ///< Focal length x (pixels).
  float fy;              ///< Focal length y (pixels).
  float cx;              ///< Principal point x (pixels).
  float cy;              ///< Principal point y (pixels).
  std::uint32_t width;   ///< Color image width (pixels).
  std::uint32_t height;  ///< Color image height (pixels).
  Mat4f cam_to_world;    ///< Camera -> world rigid transform (column-major).
};
static_assert(sizeof(ColorCameraParams) == 88,
              "ColorCameraParams must be 88 bytes");
static_assert(offsetof(ColorCameraParams, width) == 16, "layout drift");
static_assert(offsetof(ColorCameraParams, cam_to_world) == 24, "layout drift");
static_assert(std::is_trivially_copyable_v<ColorCameraParams>,
              "ColorCameraParams must be trivially copyable");
static_assert(std::is_standard_layout_v<ColorCameraParams>,
              "ColorCameraParams must be standard-layout");

}  // namespace volumetric_kit::recon::tsdf
