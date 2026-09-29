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
#include <optional>
#include <vector>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/texture/export.hpp"

namespace volumetric_kit::recon {
class Buffer;
}  // namespace volumetric_kit::recon

namespace volumetric_kit::recon::texture {

/// @brief One camera a mesh is textured from: a depth map, the camera that
///        took it, and the colour image its tile holds -- either registered to
///        the depth camera or taken by a colour camera of its own.
///
/// **Registered** (no @ref color_camera): the depth map may be smaller than
/// the colour image, as on a phone whose 256x192 depth is registered to a
/// 1920x1440 capture. The coordinate a vertex gets is its pixel centre as a
/// fraction of the depth map, which is the same fraction of the colour image
/// when @ref cam is a pixel-centre-preserving rescale of the colour camera
/// (`sensor::depth_from_registered_color`), so the tile is the colour image at
/// its own resolution.
///
/// **A colour camera of its own** (@ref color_camera set), as a
/// `sensor::GpuFramePrep` frame keeps: the depth camera still decides what is
/// visible, and the vertex is projected into the colour camera for its
/// coordinate, so the tile is that camera's image at its size.
///
/// The depth is a host array (@ref depth) or a storage buffer already on the
/// device (@ref depth_buffer), exactly one of the two. A device frame therefore
/// textures without visiting the host:
/// @code
/// const sensor::DeviceFrame& f = ...;  // from GpuFramePrep::prepare
/// texture::TextureView view;
/// view.cam = f.depth_camera;
/// view.depth_buffer = f.depth.get();
/// view.color_camera = f.color_camera;
/// @endcode
struct TextureView {
  /// Depth in metres, `cam.width * cam.height`, row-major: the occlusion
  /// reference, on the host. Null when @ref depth_buffer holds it instead.
  const float* depth = nullptr;
  /// The depth camera, posed, with the depth range a sample must fall in to
  /// count (`min_depth < max_depth`). It projects the vertices for the
  /// occlusion test, which reads the depth through it.
  DepthCameraParams cam{};
  /// The colour image's width and height in pixels, which is this view's
  /// tile. Both zero (the default) means the depth map's size, or
  /// @ref color_camera's when that is set; one zero and not the other, or a
  /// size @ref color_camera does not have, is refused.
  std::uint32_t image_width = 0;
  std::uint32_t image_height = 0;
  /// The same depth as @ref depth, already on the device: a storage buffer of
  /// at least `cam.width * cam.height` floats that a batch can copy from
  /// (`device_storage_buffer` makes one, as `GpuFramePrep` does). Its writer
  /// must have finished, which a dispatch on the texturer's device guarantees.
  /// Borrowed for the call. Null when @ref depth holds it instead.
  const Buffer* depth_buffer = nullptr;
  /// The camera the tile's image was taken with, when it is not registered to
  /// @ref cam. Unset (the default) means registered, as described above.
  std::optional<ColorCameraParams> color_camera = std::nullopt;
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
///                    `image_height` (or colour cameras, or depth maps) give
///                    each tile's size.
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
