// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/texture/texture_atlas.hpp"

#include <algorithm>
#include <cstring>
#include <string>

namespace volumetric_kit::recon::texture {

Result<AtlasLayout> side_by_side_atlas(const std::vector<TextureView>& views,
                                       std::uint32_t max_extent) {
  if (views.empty()) {
    return Status::invalid_argument("side_by_side_atlas: no views");
  }
  AtlasLayout layout;
  std::uint64_t x = 0;      // where the next tile of this row starts
  std::uint64_t y = 0;      // where this row starts
  std::uint64_t row_h = 0;  // this row's tallest tile
  std::uint64_t width = 0;  // the widest row so far
  for (std::size_t i = 0; i < views.size(); ++i) {
    const std::uint32_t w = views[i].cam.width;
    const std::uint32_t h = views[i].cam.height;
    if (w == 0 || h == 0) {
      return Status::invalid_argument("side_by_side_atlas: view " +
                                      std::to_string(i) + " has no image");
    }
    if (w > max_extent || h > max_extent) {
      return Status::unsupported("side_by_side_atlas: view " +
                                 std::to_string(i) + "'s " + std::to_string(w) +
                                 "x" + std::to_string(h) +
                                 " image is larger than the atlas extent " +
                                 std::to_string(max_extent));
    }
    if (x + w > max_extent) {  // wrap
      y += row_h;
      x = 0;
      row_h = 0;
    }
    layout.tiles.push_back(
        {static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), w, h});
    x += w;
    row_h = std::max<std::uint64_t>(row_h, h);
    width = std::max(width, x);
  }
  const std::uint64_t height = y + row_h;
  if (height > max_extent) {
    return Status::unsupported(
        "side_by_side_atlas: " + std::to_string(views.size()) +
        " images need an atlas " + std::to_string(height) +
        " tall, past the extent " + std::to_string(max_extent));
  }
  layout.width = static_cast<std::uint32_t>(width);
  layout.height = static_cast<std::uint32_t>(height);
  return layout;
}

Status pack_atlas(const std::vector<const std::uint32_t*>& images,
                  const AtlasLayout& layout,
                  std::vector<std::uint32_t>* atlas) {
  if (images.size() != layout.tiles.size()) {
    return Status::invalid_argument(
        "pack_atlas: " + std::to_string(images.size()) + " images for " +
        std::to_string(layout.tiles.size()) + " tiles");
  }
  for (std::size_t i = 0; i < images.size(); ++i) {
    const AtlasTile& t = layout.tiles[i];
    if (images[i] == nullptr) {
      return Status::invalid_argument("pack_atlas: image " + std::to_string(i) +
                                      " is null");
    }
    if (std::uint64_t{t.x} + t.width > layout.width ||
        std::uint64_t{t.y} + t.height > layout.height) {
      return Status::invalid_argument("pack_atlas: tile " + std::to_string(i) +
                                      " lies outside the atlas");
    }
  }
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
