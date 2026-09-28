// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file texture/texture_atlas.hpp
/// @brief Several cameras' images in one texture: the views a mesh is
///        textured from, where each image sits, and the packing.
///
/// Host-side and Vulkan-free. @ref ProjectiveTexturer writes `uv0` into the
/// layout; the renderer samples the packed atlas.

#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/texture/export.hpp"

namespace volumetric_kit::recon::texture {

/// @brief One camera a mesh is textured from.
struct TextureView {
  /// Depth in metres, `cam.width * cam.height`, row-major, registered to the
  /// colour image: the occlusion reference.
  const float* depth = nullptr;
  /// The colour camera, posed. Its image is this view's tile.
  DepthCameraParams cam{};
};

/// @brief Where one view's image sits in the atlas, in pixels.
struct AtlasTile {
  std::uint32_t x = 0;
  std::uint32_t y = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

/// @brief An atlas: its size, and one tile per view in view order.
struct AtlasLayout {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<AtlasTile> tiles;
};

/// @brief Lay the views' images side by side in view order, starting a new
///        row when the next image would pass @p max_extent.
/// @param views       The views, whose cameras give each image's size.
/// @param max_extent  The largest atlas width and height the renderer takes:
///                    its device's `maxImageDimension2D`
///                    (@ref ProjectiveTexturer::max_atlas_extent).
/// @return The layout; @ref Status::Code::InvalidArgument for no views or an
///         empty image, and @ref Status::Code::Unsupported for an image or a
///         stack of rows larger than @p max_extent.
VR_TEXTURE_API Result<AtlasLayout> side_by_side_atlas(
    const std::vector<TextureView>& views, std::uint32_t max_extent);

/// @brief Copy each view's image into its tile of @p atlas.
/// @param images  One image per tile, each `tile.width * tile.height` 32-bit
///                pixels, row-major (the mesh colour's packed RGB, or RGBA8).
/// @param layout  Where each image goes.
/// @param atlas   Resized to `layout.width * layout.height`; pixels no tile
///                covers are zero.
/// @return OK; @ref Status::Code::InvalidArgument for an image count other
///         than the tile count, a null image, or a tile outside the atlas.
VR_TEXTURE_API Status
pack_atlas(const std::vector<const std::uint32_t*>& images,
           const AtlasLayout& layout, std::vector<std::uint32_t>* atlas);

}  // namespace volumetric_kit::recon::texture
