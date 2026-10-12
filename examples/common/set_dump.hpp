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
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
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

/// @brief Check that @ref dump_set can write @p set: ids and views one per
///        frame, and every frame the set holds carrying a JPEG.
/// @return OK; or `Status::Code::InvalidArgument` naming the first camera
///         whose frame carries no JPEG (colour that was not MJPEG), or for
///         ids or views not one per frame.
inline vkc::Status check_set(
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
  return {};
}

/// @brief Write set @p index's camera JPEGs into @p directory, and, given
///        @p luma, each camera's luma beside its JPEG as an RGBA8 PNG
///        (@ref luma_rgba): `set_NNN_<id>.jpg` and `set_NNN_<id>_luma.png`
///        (@ref dump_file). A camera missing from the set writes nothing;
///        the set passes @ref check_set before anything is written.
/// @param directory  An existing directory.
/// @param index      The set's index in the run, which names its files.
/// @param set        The set, each frame's JPEG in its
///                   `RgbdFrame::color_bitstream`.
/// @param ids        Each camera's id, in the array's order.
/// @param luma       Optional: `PictureReadback::read`'s views of @p set.
/// @return OK; @ref check_set's refusal; or the error of `write_file`
///         (`InvalidArgument` for a JPEG already at its path) or
///         `io::write_png_rgba8`.
inline vkc::Status dump_set(
    const std::string& directory, std::size_t index,
    const vr::sensor::FrameSet& set, const std::vector<std::string>& ids,
    const std::vector<std::optional<vr::sensor::PictureView>>* luma) {
  VKC_TRY(check_set(set, ids, luma));
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

/// @brief fuse_orbbec's dump over a run: the first sets, up to a limit, each
///        written by @ref dump_set into a directory made for them.
///
/// The directory is checked before the cameras open (@ref check), and made
/// only with the first set, so a run that fails before it leaves nothing
/// behind to refuse the next run.
class SetDump {
 public:
  /// @param directory  Where the sets go; it must not exist yet, and its
  ///                   parent must.
  /// @param limit      How many sets to write; 0 turns the dump off.
  SetDump(std::string directory, std::size_t limit)
      : directory_(std::move(directory)), limit_(limit) {}

  /// @brief Refuse, before anything is opened, a directory that could not be
  ///        made for the first set: one already there, or a parent that is
  ///        not a directory. Nothing is made.
  /// @return OK, the dump off included; `Status::Code::InvalidArgument` when
  ///         something is at the directory's path; `NotFound` when its
  ///         parent is not a directory; `IoError` when neither can be told.
  vkc::Status check() const {
    if (limit_ == 0) return {};
    namespace fs = std::filesystem;
    fs::path path(directory_);
    if (!path.has_filename() && path.has_parent_path()) {
      path = path.parent_path();  // "a/b/" is "a/b"
    }
    std::error_code error;
    const fs::file_type here = fs::symlink_status(path, error).type();
    if (here == fs::file_type::none) {
      return vkc::Status::io_error("SetDump: cannot tell what is at " +
                                   directory_ + ": " + error.message());
    }
    if (here != fs::file_type::not_found) {
      return vkc::Status::invalid_argument(
          "SetDump: " + directory_ +
          " already exists; the dump writes into a new directory");
    }
    const fs::path parent =
        path.has_parent_path() ? path.parent_path() : fs::path(".");
    if (!fs::is_directory(parent, error)) {
      return vkc::Status::not_found("SetDump: " + directory_ +
                                    "'s parent is not a directory");
    }
    return {};
  }

  /// @brief Write @p set (@ref dump_set) as the next set, while fewer than
  ///        the limit are written; past it, do nothing. The first set is
  ///        checked (@ref check_set) before the directory is made, and the
  ///        directory is removed again if its files fail while it is empty.
  /// @param set   The set.
  /// @param ids   Each camera's id, in the array's order.
  /// @param luma  Optional: `PictureReadback::read`'s views of @p set.
  /// @return OK; or the error of @ref check_set, `create_new_directory` or
  ///         @ref dump_set.
  vkc::Status write(
      const vr::sensor::FrameSet& set, const std::vector<std::string>& ids,
      const std::vector<std::optional<vr::sensor::PictureView>>* luma) {
    if (written_ >= limit_) return {};
    if (written_ == 0) {
      VKC_TRY(check_set(set, ids, luma));
      VKC_TRY(vkc::create_new_directory(directory_));
      const vkc::Status first = dump_set(directory_, 0, set, ids, luma);
      if (!first.ok()) {
        std::error_code ignored;  // removes the directory only when empty
        std::filesystem::remove(directory_, ignored);
        return first;
      }
    } else {
      VKC_TRY(dump_set(directory_, written_, set, ids, luma));
    }
    ++written_;
    return {};
  }

  /// @return The sets written so far.
  std::size_t written() const noexcept { return written_; }

  /// @return The run's closing line: the sets written and where, and a
  ///         warning when the run ended before the limit (`run.frames` counts
  ///         frames, so a rig's run holds fewer sets); empty with the dump
  ///         off.
  std::string report() const {
    if (limit_ == 0) return {};
    char counts[96];
    if (written_ == limit_) {
      std::snprintf(counts, sizeof counts, "dumped %zu sets to ", written_);
      return counts + directory_ + "\n";
    }
    std::snprintf(counts, sizeof counts, "warning: dumped %zu of %zu sets to ",
                  written_, limit_);
    return counts + directory_ +
           ": the run ended first (run.frames counts frames, one per camera)\n";
  }

 private:
  std::string directory_;
  std::size_t limit_;
  std::size_t written_ = 0;
};

}  // namespace vr_example
