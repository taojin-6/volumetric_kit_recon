// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "picture_converter.hpp"

#include <string>

namespace volumetric_kit::recon::sensor::video {
namespace {

constexpr bool kYuvSetupReportsFailure = LIBSWSCALE_VERSION_MAJOR < 7;

int sws_matrix(VideoColorMatrix matrix) noexcept {
  switch (matrix) {
    case VideoColorMatrix::Bt601:
      return SWS_CS_ITU601;
    case VideoColorMatrix::Bt709:
      return SWS_CS_ITU709;
    case VideoColorMatrix::Bt2020:
      return SWS_CS_BT2020;
    case VideoColorMatrix::Smpte240m:
      return SWS_CS_SMPTE240M;
    case VideoColorMatrix::Fcc:
      return SWS_CS_FCC;
  }
  return SWS_CS_DEFAULT;
}

bool full_range(const AVFrame& frame) noexcept {
  const auto format = static_cast<AVPixelFormat>(frame.format);
  return frame.color_range == AVCOL_RANGE_JPEG ||
         format == AV_PIX_FMT_YUVJ420P || format == AV_PIX_FMT_YUVJ422P ||
         format == AV_PIX_FMT_YUVJ444P;
}

bool is_host_yuv420(AVPixelFormat format) noexcept {
  return format == AV_PIX_FMT_YUV420P || format == AV_PIX_FMT_YUVJ420P;
}

}  // namespace

VideoColorMatrix resolve_matrix(AVColorSpace space, int height) noexcept {
  switch (space) {
    case AVCOL_SPC_BT709:
      return VideoColorMatrix::Bt709;
    case AVCOL_SPC_BT470BG:
    case AVCOL_SPC_SMPTE170M:
      return VideoColorMatrix::Bt601;
    case AVCOL_SPC_BT2020_NCL:
    case AVCOL_SPC_BT2020_CL:
      return VideoColorMatrix::Bt2020;
    case AVCOL_SPC_SMPTE240M:
      return VideoColorMatrix::Smpte240m;
    case AVCOL_SPC_FCC:
      return VideoColorMatrix::Fcc;
    default:
      break;
  }
  return height > 576 ? VideoColorMatrix::Bt709 : VideoColorMatrix::Bt601;
}

std::optional<ColorEncoding> resolve_encoding(
    AVColorTransferCharacteristic transfer,
    AVColorPrimaries primaries) noexcept {
  ColorEncoding encoding;
  switch (transfer) {
    case AVCOL_TRC_UNSPECIFIED:
    case AVCOL_TRC_BT709:
    case AVCOL_TRC_SMPTE170M:  // BT.601's and BT.2020's SDR curve is BT.709's
    case AVCOL_TRC_BT2020_10:
    case AVCOL_TRC_BT2020_12:
      encoding.transfer = ColorEncoding::Transfer::Bt709;
      break;
    case AVCOL_TRC_IEC61966_2_1:
      encoding.transfer = ColorEncoding::Transfer::Srgb;
      break;
    case AVCOL_TRC_LINEAR:
      encoding.transfer = ColorEncoding::Transfer::Linear;
      break;
    case AVCOL_TRC_SMPTE2084:
      encoding.transfer = ColorEncoding::Transfer::Bt2020Pq;
      break;
    default:
      return std::nullopt;
  }
  switch (primaries) {
    case AVCOL_PRI_UNSPECIFIED:
    case AVCOL_PRI_BT709:
      encoding.primaries = ColorEncoding::Primaries::Bt709;
      break;
    case AVCOL_PRI_SMPTE432:
      encoding.primaries = ColorEncoding::Primaries::DisplayP3;
      break;
    case AVCOL_PRI_BT2020:
      encoding.primaries = ColorEncoding::Primaries::Bt2020;
      break;
    default:
      return std::nullopt;
  }
  return encoding;
}

void describe_color(
    const AVFrame& frame,
    const std::optional<VideoColorDescription>& unlabelled_color,
    DecodedPicture& picture) {
  // A stream that declares no matrix reads as limited range whether or not
  // it declares one, so the range is taken with the matrix.
  if (unlabelled_color && frame.colorspace == AVCOL_SPC_UNSPECIFIED) {
    picture.matrix = unlabelled_color->matrix;
    picture.full_range = unlabelled_color->full_range;
  } else {
    // The displayed height: a hardware frame's still counts its top crop.
    picture.matrix =
        resolve_matrix(frame.colorspace, static_cast<int>(picture.height));
    picture.full_range = full_range(frame);
  }
  picture.encoding = resolve_encoding(frame.color_trc, frame.color_primaries);
  switch (frame.chroma_location) {
    case AVCHROMA_LOC_CENTER:
      picture.chroma_location = ChromaLocation::Center;
      break;
    case AVCHROMA_LOC_TOPLEFT:
      picture.chroma_location = ChromaLocation::TopLeft;
      break;
    case AVCHROMA_LOC_TOP:
      picture.chroma_location = ChromaLocation::Top;
      break;
    case AVCHROMA_LOC_BOTTOMLEFT:
      picture.chroma_location = ChromaLocation::BottomLeft;
      break;
    case AVCHROMA_LOC_BOTTOM:
      picture.chroma_location = ChromaLocation::Bottom;
      break;
    default:
      picture.chroma_location = ChromaLocation::Left;
      break;
  }
}

Result<DecodedPicture> PictureConverter::convert(
    const AVFrame& frame, VideoPixelLayout layout,
    const std::optional<VideoColorDescription>& unlabelled_color) {
  const auto format = static_cast<AVPixelFormat>(frame.format);
  const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(format);
  if (desc == nullptr || (desc->flags & AV_PIX_FMT_FLAG_HWACCEL) != 0 ||
      sws_isSupportedInput(format) == 0 || frame.width <= 0 ||
      frame.height <= 0) {
    return Status::invalid_argument(std::string(who_) + ": cannot convert a " +
                                    (desc != nullptr ? desc->name : "unknown") +
                                    " frame");
  }

  DecodedPicture picture;
  picture.width = static_cast<std::uint32_t>(frame.width);
  picture.height = static_cast<std::uint32_t>(frame.height);
  picture.layout = layout;
  describe_color(frame, unlabelled_color, picture);

  const AVFrame* source = &frame;
  if (layout != VideoPixelLayout::Yuv420 || !is_host_yuv420(format)) {
    const bool rgb = layout == VideoPixelLayout::Rgb24;
    // Before swscale 9, packed RGB's one-luma-row path rounds vertical
    // chroma weights to 0 or 1/2 (FFmpeg commit 095f8038fa). Planar RGB
    // uses the general filter, preserving the 1/4 and 3/4 weights too.
    const bool planar_rgb =
        rgb && LIBSWSCALE_VERSION_MAJOR < 9 && desc->log2_chroma_h != 0;
    const AVPixelFormat target = !rgb         ? AV_PIX_FMT_YUV420P
                                 : planar_rgb ? AV_PIX_FMT_GBRP
                                              : AV_PIX_FMT_RGB24;
    if (out_ == nullptr) out_.reset(av_frame_alloc());
    if (out_ == nullptr) return ffmpeg_alloc_error(who_, "an output frame");
    if (out_->width != frame.width || out_->height != frame.height ||
        out_->format != target) {
      av_frame_unref(out_.get());
      out_->width = frame.width;
      out_->height = frame.height;
      out_->format = target;
      const int err = av_frame_get_buffer(out_.get(), 0);
      if (err < 0) return ffmpeg_error(who_, "allocating a picture", err);
    }
    const Setup setup{frame.width,
                      frame.height,
                      format,
                      target,
                      picture.matrix,
                      picture.full_range,
                      picture.chroma_location};
    if (sws_ == nullptr || !setup.same(setup_)) {
      // Full horizontal chroma interpolation and accurate rounding: the
      // default path replicates chroma and rounds coarsely.
      SwsContextPtr next(sws_alloc_context());
      if (next == nullptr)
        return ffmpeg_alloc_error(who_, "a conversion context");
      const auto chroma = chroma_offset(picture.chroma_location);
      const int x = static_cast<int>(256 * chroma[0]);
      const int y = static_cast<int>(256 * chroma[1]);
      // Only subsampled axes have a chroma offset. RGB, and 4:2:2's
      // vertical axis, sample at the luma centres themselves.
      const struct {
        const char* name;
        int value;
      } options[] = {
          {"srcw", frame.width},
          {"srch", frame.height},
          {"dstw", frame.width},
          {"dsth", frame.height},
          {"src_format", format},
          {"dst_format", target},
          {"sws_flags", SWS_BILINEAR | SWS_ACCURATE_RND | SWS_FULL_CHR_H_INT},
          {"src_h_chr_pos", desc->log2_chroma_w != 0 ? x : 0},
          {"src_v_chr_pos", desc->log2_chroma_h != 0 ? y : 0},
          {"dst_h_chr_pos", rgb ? 0 : x},
          {"dst_v_chr_pos", rgb ? 0 : y}};
      for (const auto& option : options) {
        const int err =
            av_opt_set_int(next.get(), option.name, option.value, 0);
        if (err < 0)
          return ffmpeg_error(who_, "setting chroma conversion", err);
      }
      const int initialized = sws_init_context(next.get(), nullptr, nullptr);
      if (initialized < 0)
        return ffmpeg_error(who_, "initializing conversion", initialized);
      // One matrix both ways: swscale reads the source table for a YUV
      // source and the target table for a YUV target (an RGB-coded stream).
      // A YUV target keeps the source's range, which is what the label says;
      // left alone, swscale takes a YUVJ source to limited range.
      const int* coefficients = sws_getCoefficients(sws_matrix(picture.matrix));
      const int range = picture.full_range ? 1 : 0;
      const int target_range = rgb ? 1 : range;
      const int set = sws_setColorspaceDetails(next.get(), coefficients, range,
                                               coefficients, target_range, 0,
                                               1 << 16, 1 << 16);
      // Before swscale 7, a YUV (or grey) source to a YUV target reports -1
      // after taking both ranges: there is no RGB table to build (FFmpeg
      // 4.4's libswscale/utils.c).
      const bool yuv_to_yuv = !rgb && (desc->flags & AV_PIX_FMT_FLAG_RGB) == 0;
      if (set < 0 && !(kYuvSetupReportsFailure && yuv_to_yuv)) {
        return Status::io_error(std::string(who_) +
                                ": swscale refused the colour matrix");
      }
      sws_ = std::move(next);
      setup_ = setup;
    }
    // A negative AVERROR since FFmpeg 5, and 0 from 4.4 on a bad slice.
    const int rows = sws_scale(sws_.get(), frame.data, frame.linesize, 0,
                               frame.height, out_->data, out_->linesize);
    if (rows <= 0) {
      return ffmpeg_error(who_, "converting the picture",
                          rows < 0 ? rows : AVERROR(EINVAL));
    }
    source = out_.get();
    if (planar_rgb) {
      const std::size_t stride = std::size_t{picture.width} * 3;
      rgb_.resize(stride * picture.height);
      for (int y = 0; y < frame.height; ++y) {
        const auto* g = source->data[0] + y * source->linesize[0];
        const auto* b = source->data[1] + y * source->linesize[1];
        const auto* r = source->data[2] + y * source->linesize[2];
        auto* row = rgb_.data() + y * stride;
        for (int x = 0; x < frame.width; ++x) {
          row[3 * x] = r[x];
          row[3 * x + 1] = g[x];
          row[3 * x + 2] = b[x];
        }
      }
      picture.plane[0] = rgb_.data();
      picture.stride[0] = stride;
      return picture;
    }
  }

  const int planes = layout == VideoPixelLayout::Rgb24 ? 1 : 3;
  for (int i = 0; i < planes; ++i) {
    picture.plane[i] = source->data[i];
    picture.stride[i] = static_cast<std::size_t>(source->linesize[i]);
  }
  return picture;
}

}  // namespace volumetric_kit::recon::sensor::video
