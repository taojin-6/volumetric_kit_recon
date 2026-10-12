// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/set_dump.hpp
/// @brief fuse_orbbec's dump of its first sets (`diagnostics.dump_sets`):
///        each camera's JPEG as it came and, with the readback, the luma the
///        device decoded from it, so the two can be compared off line.
///        Header-only.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/utils/picture_readback.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @return Where @ref dump_set writes camera @p id's file of set @p set:
///         `<directory>/set_NNN_<id><suffix>`, the set's index three digits
///         or more.
inline std::string dump_file(const std::string& directory, std::size_t set,
                             const std::string& id, const char* suffix) {
  char index[32];
  std::snprintf(index, sizeof index, "/set_%03zu_", set);
  return directory + index + id + suffix;
}

/// @return @p view's luma as packed RGBA8 rows, R = G = B = Y and A = 255,
///         as `io::write_png_rgba8` takes them.
inline std::vector<std::uint8_t> luma_rgba(
    const vr::sensor::PictureView& view) {
  std::vector<std::uint8_t> rgba(std::size_t{view.width} * view.height * 4);
  std::uint8_t* out = rgba.data();
  for (std::uint32_t y = 0; y < view.height; ++y) {
    const std::uint8_t* row = view.luma + y * view.stride;
    for (std::uint32_t x = 0; x < view.width; ++x, out += 4) {
      out[0] = out[1] = out[2] = row[x];
      out[3] = 255;
    }
  }
  return rgba;
}

/// @brief Write set @p index's camera JPEGs into @p directory, and, given
///        @p luma, each camera's luma beside its JPEG as an RGBA8 PNG
///        (@ref luma_rgba): `set_NNN_<id>.jpg` and `set_NNN_<id>_luma.png`
///        (@ref dump_file). A camera missing from the set writes nothing;
///        every frame is checked before anything is written.
/// @param directory  An existing directory.
/// @param index      The set's index in the run, which names its files.
/// @param set        The set, each frame's JPEG in its
///                   `RgbdFrame::color_bitstream`.
/// @param ids        Each camera's id, in the array's order.
/// @param luma       Optional: `PictureReadback::read`'s views of @p set.
/// @return OK; `Status::Code::InvalidArgument` for a frame carrying no JPEG
///         (colour that was not MJPEG) or ids or views not one per frame;
///         or the error of `write_file` (`InvalidArgument` for a JPEG already
///         at its path) or `io::write_png_rgba8`.
inline vkc::Status dump_set(
    const std::string& directory, std::size_t index,
    const vr::sensor::FrameSet& set, const std::vector<std::string>& ids,
    const std::vector<std::optional<vr::sensor::PictureView>>* luma) {
  if (ids.size() != set.frames.size() ||
      (luma != nullptr && luma->size() != set.frames.size())) {
    return vkc::Status::invalid_argument(
        "dump_set: ids and views are one per frame of the set");
  }
  for (std::size_t c = 0; c < set.frames.size(); ++c) {
    if (set.frames[c] && set.frames[c]->color_bitstream.codec !=
                             vr::sensor::ColorBitstream::Codec::Jpeg) {
      return vkc::Status::invalid_argument(
          "dump_set: camera " + ids[c] +
          "'s frame carries no JPEG; the dump takes MJPEG colour");
    }
  }
  for (std::size_t c = 0; c < set.frames.size(); ++c) {
    if (!set.frames[c]) continue;
    const vr::sensor::ColorBitstream& jpeg = set.frames[c]->color_bitstream;
    VKC_TRY(vkc::write_file(
        dump_file(directory, index, ids[c], ".jpg"),
        std::string_view(reinterpret_cast<const char*>(jpeg.data), jpeg.size),
        vkc::WriteMode::CreateNew));
    if (luma == nullptr || !(*luma)[c]) continue;
    const vr::sensor::PictureView& view = *(*luma)[c];
    const std::vector<std::uint8_t> rgba = luma_rgba(view);
    VKC_TRY(vr::io::write_png_rgba8(
        dump_file(directory, index, ids[c], "_luma.png"), rgba.data(),
        rgba.size(), view.width, view.height));
  }
  return {};
}

}  // namespace vr_example
