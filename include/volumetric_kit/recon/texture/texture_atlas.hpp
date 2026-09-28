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

/// @brief One camera a mesh is textured from: a depth map, the camera that
///        took it, and the size of the colour image registered to it.
///
/// The depth map may be smaller than the colour image, as on a phone whose
/// 256x192 depth is registered to a 1920x1440 capture. The coordinate a vertex
/// gets is its pixel centre as a fraction of the depth map, which is the same
/// fraction of the colour image when @ref cam is a pixel-centre-preserving
/// rescale of the colour camera (`sensor::depth_from_registered_color`), so
/// the tile is the colour image at its own resolution.
struct TextureView {
  /// Depth in metres, `cam.width * cam.height`, row-major: the occlusion
  /// reference.
  const float* depth = nullptr;
  /// The depth camera, posed, with the depth range a sample must fall in to
  /// count (`min_depth < max_depth`). It projects the vertices and is the
  /// camera the occlusion test reads @ref depth through.
  DepthCameraParams cam{};
  /// The colour image's width and height in pixels, which is this view's
  /// tile. Both zero (the default) means the depth map's size; one zero and
  /// not the other is refused.
  std::uint32_t image_width = 0;
  std::uint32_t image_height = 0;
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
/// @param views       The views, whose @ref TextureView::image_width and
///                    `image_height` (or depth maps) give each tile's size.
/// @param max_extent  The largest atlas width and height the renderer takes:
///                    its device's `maxImageDimension2D`
///                    (@ref ProjectiveTexturer::max_atlas_extent).
/// @return The layout; @ref Status::Code::InvalidArgument for no views, an
///         empty image, or an image or a stack of rows larger than
///         @p max_extent -- the code @ref ProjectiveTexturer::texture gives a
///         layout past the device's extent.
VR_TEXTURE_API Result<AtlasLayout> side_by_side_atlas(
    const std::vector<TextureView>& views, std::uint32_t max_extent);

/// @brief Copy each view's image into its tile of @p atlas.
/// @param images  One image per tile, each `tile.width * tile.height` 32-bit
///                pixels, row-major (the mesh colour's packed RGB, or RGBA8).
/// @param layout  Where each image goes.
/// @param atlas   Resized to `layout.width * layout.height`; pixels no tile
///                covers are zero.
/// @return OK; @ref Status::Code::InvalidArgument for a null @p atlas, an
///         image count other than the tile count, a null image, a tile
///         outside the atlas, or two tiles that overlap.
VR_TEXTURE_API Status
pack_atlas(const std::vector<const std::uint32_t*>& images,
           const AtlasLayout& layout, std::vector<std::uint32_t>* atlas);

}  // namespace volumetric_kit::recon::texture
