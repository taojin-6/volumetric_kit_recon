// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file volume/camera_params.hpp
/// @brief The posed pinhole depth camera the fusion entry points take.
///
/// Split out of `volume/voxel_hash_map.hpp` so that *describing* a camera does
/// not cost compiling the map that consumes one. This header is host-side POD
/// over GLM and reaches no Vulkan, where `voxel_hash_map.hpp` pulls the VMA
/// allocator, the RAII buffers and the compute-pipeline wrappers in through
/// `core/vulkan.hpp`.
///
/// The `sensor` tier is what makes the split worth having: its capture contract
/// is implemented *out of tree* (an ARKit driver in `volumetric_kit_ios`, per
/// the 2026-08-02 decision), and a platform driver should not have to
/// preprocess the whole Vulkan surface just to fill in an intrinsics struct.
/// `voxel_hash_map.hpp` includes this header, so every existing user is
/// unaffected.

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::volume {

/// @brief Pinhole depth-camera intrinsics + pose for
///        @ref VoxelHashMap::allocate_from_depth.
///
/// Uploaded verbatim to the depth-allocation kernel, which reads it through
/// scalar block layout -- so this packs byte-for-byte to the shader's
/// `CameraParams`: the scalars at their natural 4-byte offsets, the `mat4` at
/// offset 32. The `static_assert`s below pin that layout (a drift is a compile
/// error, not silent misprojection).
struct DepthCameraParams {
  float fx;              ///< Focal length x (pixels).
  float fy;              ///< Focal length y (pixels).
  float cx;              ///< Principal point x (pixels).
  float cy;              ///< Principal point y (pixels).
  float min_depth;       ///< Reject samples nearer than this (metres).
  float max_depth;       ///< Reject samples farther than this (metres).
  std::uint32_t width;   ///< Depth image width (pixels).
  std::uint32_t height;  ///< Depth image height (pixels).
  Mat4f cam_to_world;    ///< Camera -> world rigid transform (column-major).
};
static_assert(sizeof(DepthCameraParams) == 96,
              "DepthCameraParams must be 96 bytes");
static_assert(offsetof(DepthCameraParams, min_depth) == 16, "layout drift");
static_assert(offsetof(DepthCameraParams, width) == 24, "layout drift");
static_assert(offsetof(DepthCameraParams, cam_to_world) == 32, "layout drift");
static_assert(std::is_trivially_copyable_v<DepthCameraParams>,
              "DepthCameraParams must be trivially copyable");
static_assert(std::is_standard_layout_v<DepthCameraParams>,
              "DepthCameraParams must be standard-layout");

}  // namespace volumetric_kit::recon::volume
