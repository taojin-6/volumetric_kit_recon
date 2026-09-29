// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/texture/texture_atlas.hpp"

#include <algorithm>
#include <cstring>
#include <string>

#include "atlas_checks.hpp"

namespace volumetric_kit::recon::texture {

namespace detail {

Result<ImageSize> view_image_size(const TextureView& view, std::size_t index,
                                  const std::string& who) {
  const bool given = view.image_width != 0 || view.image_height != 0;
  if (given && (view.image_width == 0 || view.image_height == 0)) {
    return Status::invalid_argument(who + "view " + std::to_string(index) +
                                    " gives its image one side and not the "
                                    "other");
  }
  // A colour camera's image IS the tile: its pixels are what the vertices'
  // coordinates address, so a different size would misplace every one.
  const ImageSize own =
      view.color_camera
          ? ImageSize{view.color_camera->width, view.color_camera->height}
          : ImageSize{view.cam.width, view.cam.height};
  if (given && view.color_camera &&
      (view.image_width != own.width || view.image_height != own.height)) {
    return Status::invalid_argument(
        who + "view " + std::to_string(index) + " gives its image as " +
        std::to_string(view.image_width) + "x" +
        std::to_string(view.image_height) + ", but its colour camera's is " +
        std::to_string(own.width) + "x" + std::to_string(own.height));
  }
  const ImageSize size =
      given ? ImageSize{view.image_width, view.image_height} : own;
  if (size.width == 0 || size.height == 0) {
    return Status::invalid_argument(who + "view " + std::to_string(index) +
                                    " has no image");
  }
  return size;
}

Status check_tiles(const AtlasLayout& layout, const std::string& who) {
  const std::vector<AtlasTile>& tiles = layout.tiles;
  for (std::size_t i = 0; i < tiles.size(); ++i) {
    const AtlasTile& a = tiles[i];
    if (std::uint64_t{a.x} + a.width > layout.width ||
        std::uint64_t{a.y} + a.height > layout.height) {
      return Status::invalid_argument(who + "tile " + std::to_string(i) +
                                      " lies outside the atlas");
    }
    // Inside the atlas, so no sum below can wrap. Pairwise: a layout holds a
    // rig's or a keyframe set's views, tens to hundreds.
    for (std::size_t j = 0; j < i; ++j) {
      const AtlasTile& b = tiles[j];
      if (a.x < b.x + b.width && b.x < a.x + a.width && a.y < b.y + b.height &&
          b.y < a.y + a.height) {
        return Status::invalid_argument(who + "tiles " + std::to_string(j) +
                                        " and " + std::to_string(i) +
                                        " overlap");
      }
    }
  }
  return {};
}

}  // namespace detail

Result<AtlasLayout> side_by_side_atlas(const std::vector<TextureView>& views,
                                       std::uint32_t max_extent) {
  const std::string who = "side_by_side_atlas: ";
  if (views.empty()) {
    return Status::invalid_argument(who + "no views");
  }
  // floor(sqrt(n)) rows of ceil(n / rows): four 4K views make 7680 x 4320,
  // not 15360 x 2160, and three stay in one row, which a square grid would
  // leave a quarter empty.
  std::size_t rows = 1;
  while ((rows + 1) * (rows + 1) <= views.size()) ++rows;
  const std::size_t columns = (views.size() + rows - 1) / rows;
  AtlasLayout layout;
  std::size_t in_row = 0;   // tiles in this row so far
  std::uint64_t x = 0;      // where the next tile of this row starts
  std::uint64_t y = 0;      // where this row starts
  std::uint64_t row_h = 0;  // this row's tallest tile
  std::uint64_t width = 0;  // the widest row so far
  for (std::size_t i = 0; i < views.size(); ++i) {
    VR_ASSIGN(const detail::ImageSize size,
              detail::view_image_size(views[i], i, who));
    const std::uint32_t w = size.width;
    const std::uint32_t h = size.height;
    // InvalidArgument, as every device limit this tier checks: the vertex
    // count past one dispatch, a buffer past maxStorageBufferRange, and a
    // layout past maxImageDimension2D in ProjectiveTexturer::texture.
    if (w > max_extent || h > max_extent) {
      return Status::invalid_argument(
          who + "view " + std::to_string(i) + "'s " + std::to_string(w) + "x" +
          std::to_string(h) + " image is larger than the atlas extent " +
          std::to_string(max_extent));
    }
    if (in_row == columns || x + w > max_extent) {  // wrap
      y += row_h;
      x = 0;
      row_h = 0;
      in_row = 0;
    }
    layout.tiles.push_back(
        {static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), w, h});
    x += w;
    ++in_row;
    row_h = std::max<std::uint64_t>(row_h, h);
    width = std::max(width, x);
  }
  const std::uint64_t height = y + row_h;
  if (height > max_extent) {
    return Status::invalid_argument(
        who + std::to_string(views.size()) + " images need an atlas " +
        std::to_string(height) + " tall, past the extent " +
        std::to_string(max_extent));
  }
  layout.width = static_cast<std::uint32_t>(width);
  layout.height = static_cast<std::uint32_t>(height);
  return layout;
}

Status pack_atlas(const std::vector<const std::uint32_t*>& images,
                  const AtlasLayout& layout,
                  std::vector<std::uint32_t>* atlas) {
  // TODO(texture): pack on the GPU into an image gfx samples directly, rather
  // than on the host for gfx to upload; that needs images in core.
  const std::string who = "pack_atlas: ";
  if (atlas == nullptr) {
    return Status::invalid_argument(who + "atlas is null");
  }
  if (images.size() != layout.tiles.size()) {
    return Status::invalid_argument(
        who + std::to_string(images.size()) + " images for " +
        std::to_string(layout.tiles.size()) + " tiles");
  }
  for (std::size_t i = 0; i < images.size(); ++i) {
    if (images[i] == nullptr) {
      return Status::invalid_argument(who + "image " + std::to_string(i) +
                                      " is null");
    }
  }
  VR_TRY(detail::check_tiles(layout, who));
  const std::size_t w = layout.width;
  atlas->assign(w * layout.height, 0u);
  for (std::size_t i = 0; i < images.size(); ++i) {
    const AtlasTile& t = layout.tiles[i];
    for (std::uint32_t row = 0; row < t.height; ++row) {
      std::memcpy(atlas->data() + (t.y + row) * w + t.x,
                  images[i] + static_cast<std::size_t>(row) * t.width,
                  t.width * sizeof(std::uint32_t));
    }
  }
  return {};
}

}  // namespace volumetric_kit::recon::texture
