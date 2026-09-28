// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file atlas_checks.hpp
/// @brief The checks @ref side_by_side_atlas, @ref pack_atlas and
///        @ref ProjectiveTexturer share, so a view's tile size and a layout's
///        soundness mean one thing to all three.
///
/// Internal (under src/, never installed).

#include <cstddef>
#include <cstdint>
#include <string>

#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"

namespace volumetric_kit::recon::texture::detail {

/// @brief A view's image size, which is its tile's.
struct ImageSize {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

/// @brief @p view's @ref TextureView::image_width x `image_height`, or its
///        depth map's size when both are zero.
/// @param who    The caller's name, prefixed to a refusal.
/// @param index  The view's index, for the refusal.
/// @return The size; InvalidArgument when exactly one of the two is zero or
///         the size is empty.
Result<ImageSize> view_image_size(const TextureView& view, std::size_t index,
                                  const std::string& who);

/// @brief Refuse a layout whose tiles do not lie inside it, or overlap: a
///        tile over another would take its pixels in @ref pack_atlas, and a
///        triangle given the lower view would sample the upper one's image.
/// @param who  The caller's name, prefixed to a refusal.
/// @return OK, or InvalidArgument.
Status check_tiles(const AtlasLayout& layout, const std::string& who);

}  // namespace volumetric_kit::recon::texture::detail
