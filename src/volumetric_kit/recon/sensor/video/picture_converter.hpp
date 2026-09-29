// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A decoded host frame in the layout the caller asked for, through swscale.
// Codec-neutral. Internal.

#include <optional>

#include "ffmpeg.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/video/decoded_picture.hpp"

namespace volumetric_kit::recon::sensor::video {

/// @return The matrix @p space names; an unspecified one by @p height, as
///         @ref DecodedPicture::matrix describes.
VideoColorMatrix resolve_matrix(AVColorSpace space, int height) noexcept;

/// @return The encoding @p transfer and @p primaries name, as
///         @ref DecodedPicture::encoding describes.
std::optional<ColorEncoding> resolve_encoding(
    AVColorTransferCharacteristic transfer,
    AVColorPrimaries primaries) noexcept;

// The matrix, range and encoding @p frame declares, set on @p picture as
// PictureConverter::convert sets them: @p unlabelled_color stands in for a
// frame that declares no matrix.
void describe_color(
    const AVFrame& frame,
    const std::optional<VideoColorDescription>& unlabelled_color,
    DecodedPicture& picture);

class PictureConverter {
 public:
  /// @param who What its errors are reported as (the decoder's name).
  explicit PictureConverter(const char* who) noexcept : who_(who) {}

  /// @brief Lay @p frame (in host memory) out as @p layout. Planes point into
  ///        @p frame when it is already 8-bit 4:2:0 and @p layout is Yuv420,
  ///        else into this converter; valid until the next call or until
  ///        @p frame changes. `pts` is left to the caller. A frame that
  ///        declares no matrix is converted by @p unlabelled_color, if given.
  /// @return The picture; InvalidArgument for a hardware or unconvertible
  ///         format; IoError if swscale fails.
  Result<DecodedPicture> convert(const AVFrame& frame, VideoPixelLayout layout,
                                 const std::optional<VideoColorDescription>&
                                     unlabelled_color = std::nullopt);

 private:
  // What sws_ converts. Any change rebuilds it: swscale can reuse a freed
  // context's address, so the pointer cannot say whether it was set up.
  struct Setup {
    int width = 0;
    int height = 0;
    AVPixelFormat source = AV_PIX_FMT_NONE;
    AVPixelFormat target = AV_PIX_FMT_NONE;
    VideoColorMatrix matrix = VideoColorMatrix::Bt709;
    bool full_range = false;
    bool same(const Setup& o) const noexcept {
      return width == o.width && height == o.height && source == o.source &&
             target == o.target && matrix == o.matrix &&
             full_range == o.full_range;
    }
  };

  const char* who_;
  SwsContextPtr sws_;
  Setup setup_;
  FramePtr out_;
};

}  // namespace volumetric_kit::recon::sensor::video
