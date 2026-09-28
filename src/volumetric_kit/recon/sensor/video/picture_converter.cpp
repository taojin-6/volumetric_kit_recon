// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "picture_converter.hpp"

#include <string>

namespace volumetric_kit::recon::sensor::video {
namespace {

constexpr const char* kWho = "HevcDecoder";

int sws_matrix(VideoColorMatrix matrix) noexcept {
  switch (matrix) {
    case VideoColorMatrix::Bt601:
      return SWS_CS_ITU601;
    case VideoColorMatrix::Bt709:
      return SWS_CS_ITU709;
    case VideoColorMatrix::Bt2020:
      return SWS_CS_BT2020;
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
    case AVCOL_SPC_SMPTE240M:
      return VideoColorMatrix::Bt709;
    case AVCOL_SPC_BT470BG:
    case AVCOL_SPC_SMPTE170M:
    case AVCOL_SPC_FCC:
      return VideoColorMatrix::Bt601;
    case AVCOL_SPC_BT2020_NCL:
    case AVCOL_SPC_BT2020_CL:
      return VideoColorMatrix::Bt2020;
    default:
      break;
  }
  return height > 576 ? VideoColorMatrix::Bt709 : VideoColorMatrix::Bt601;
}

Result<DecodedPicture> PictureConverter::convert(const AVFrame& frame,
                                                 VideoPixelLayout layout) {
  const auto format = static_cast<AVPixelFormat>(frame.format);
  const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(format);
  if (desc == nullptr || (desc->flags & AV_PIX_FMT_FLAG_HWACCEL) != 0 ||
      sws_isSupportedInput(format) == 0 || frame.width <= 0 ||
      frame.height <= 0) {
    return Status::invalid_argument(std::string(kWho) + ": cannot convert a " +
                                    (desc != nullptr ? desc->name : "unknown") +
                                    " frame");
  }

  DecodedPicture picture;
  picture.width = static_cast<std::uint32_t>(frame.width);
  picture.height = static_cast<std::uint32_t>(frame.height);
  picture.layout = layout;
  picture.matrix = resolve_matrix(frame.colorspace, frame.height);
  picture.full_range = full_range(frame);

  const AVFrame* source = &frame;
  if (layout != VideoPixelLayout::Yuv420 || !is_host_yuv420(format)) {
    const AVPixelFormat target = layout == VideoPixelLayout::Rgb24
                                     ? AV_PIX_FMT_RGB24
                                     : AV_PIX_FMT_YUV420P;
    if (out_ == nullptr) out_.reset(av_frame_alloc());
    if (out_ == nullptr) return ffmpeg_alloc_error(kWho, "an output frame");
    if (out_->width != frame.width || out_->height != frame.height ||
        out_->format != target) {
      av_frame_unref(out_.get());
      out_->width = frame.width;
      out_->height = frame.height;
      out_->format = target;
      const int err = av_frame_get_buffer(out_.get(), 0);
      if (err < 0) return ffmpeg_error(kWho, "allocating a picture", err);
    }
    const Setup setup{frame.width, frame.height,   format,
                      target,      picture.matrix, picture.full_range};
    if (sws_ == nullptr || !setup.same(setup_)) {
      // Full horizontal chroma interpolation and accurate rounding: the
      // default path replicates chroma and rounds coarsely.
      sws_.reset(sws_getContext(
          frame.width, frame.height, format, frame.width, frame.height, target,
          SWS_BILINEAR | SWS_ACCURATE_RND | SWS_FULL_CHR_H_INT, nullptr,
          nullptr, nullptr));
      if (sws_ == nullptr) {
        return Status::io_error(std::string(kWho) +
                                ": swscale cannot convert " + desc->name +
                                " to " + av_get_pix_fmt_name(target));
      }
      const int* coefficients = sws_getCoefficients(sws_matrix(picture.matrix));
      if (target == AV_PIX_FMT_RGB24 &&
          sws_setColorspaceDetails(sws_.get(), coefficients,
                                   picture.full_range ? 1 : 0, coefficients, 1,
                                   0, 1 << 16, 1 << 16) < 0) {
        sws_.reset();
        return Status::io_error(std::string(kWho) +
                                ": swscale refused the colour matrix");
      }
      setup_ = setup;
    }
    sws_scale(sws_.get(), frame.data, frame.linesize, 0, frame.height,
              out_->data, out_->linesize);
    source = out_.get();
  }

  const int planes = layout == VideoPixelLayout::Rgb24 ? 1 : 3;
  for (int i = 0; i < planes; ++i) {
    picture.plane[i] = source->data[i];
    picture.stride[i] = static_cast<std::size_t>(source->linesize[i]);
  }
  return picture;
}

}  // namespace volumetric_kit::recon::sensor::video
