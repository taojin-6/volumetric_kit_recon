// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The picture converter on frames built by hand, no codec: the matrix and
// encoding a stream's colour tags resolve to, Yuv420 passed through without a
// copy, NV12 split exactly, RGB against the standards' constants for each
// matrix and range, a converted Yuv420 true to its label, a size change, and
// the frames it refuses.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>

#include "ffmpeg.hpp"
#include "picture_converter.hpp"
#include "yuv_reference.hpp"

namespace sensor = volumetric_kit::recon::sensor;
namespace video = volumetric_kit::recon::sensor::video;
using sensor::VideoColorMatrix;
using sensor::VideoPixelLayout;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// A solid (y, u, v) frame in @p format: planar YUV at any subsampling, or
// nv12. For gbrp, (y, u, v) are (g, b, r).
video::FramePtr solid(AVPixelFormat format, int width, int height, int y, int u,
                      int v) {
  video::FramePtr frame(av_frame_alloc());
  frame->format = format;
  frame->width = width;
  frame->height = height;
  if (av_frame_get_buffer(frame.get(), 0) < 0) std::abort();
  const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get(format);
  const int cw = AV_CEIL_RSHIFT(width, desc->log2_chroma_w);
  const int ch = AV_CEIL_RSHIFT(height, desc->log2_chroma_h);
  for (int row = 0; row < height; ++row) {
    std::memset(frame->data[0] + row * frame->linesize[0], y, width);
  }
  for (int row = 0; row < ch; ++row) {
    if (format == AV_PIX_FMT_NV12) {
      std::uint8_t* line = frame->data[1] + row * frame->linesize[1];
      for (int x = 0; x < cw; ++x) {
        line[2 * x] = static_cast<std::uint8_t>(u);
        line[2 * x + 1] = static_cast<std::uint8_t>(v);
      }
    } else {
      std::memset(frame->data[1] + row * frame->linesize[1], u, cw);
      std::memset(frame->data[2] + row * frame->linesize[2], v, cw);
    }
  }
  return frame;
}

int test_resolve_matrix() {
  CHECK(video::resolve_matrix(AVCOL_SPC_BT709, 144) == VideoColorMatrix::Bt709);
  CHECK(video::resolve_matrix(AVCOL_SPC_SMPTE170M, 2160) ==
        VideoColorMatrix::Bt601);
  CHECK(video::resolve_matrix(AVCOL_SPC_BT470BG, 720) ==
        VideoColorMatrix::Bt601);
  CHECK(video::resolve_matrix(AVCOL_SPC_BT2020_NCL, 720) ==
        VideoColorMatrix::Bt2020);
  CHECK(video::resolve_matrix(AVCOL_SPC_UNSPECIFIED, 576) ==
        VideoColorMatrix::Bt601);
  CHECK(video::resolve_matrix(AVCOL_SPC_UNSPECIFIED, 720) ==
        VideoColorMatrix::Bt709);
  CHECK(video::resolve_matrix(AVCOL_SPC_SMPTE240M, 1080) ==
        VideoColorMatrix::Smpte240m);
  CHECK(video::resolve_matrix(AVCOL_SPC_FCC, 480) == VideoColorMatrix::Fcc);
  CHECK(video::resolve_matrix(AVCOL_SPC_RGB, 720) == VideoColorMatrix::Bt709);
  return 0;
}

int test_resolve_encoding() {
  using Transfer = volumetric_kit::recon::ColorEncoding::Transfer;
  using Primaries = volumetric_kit::recon::ColorEncoding::Primaries;
  const auto is = [](std::optional<volumetric_kit::recon::ColorEncoding> e,
                     Transfer t, Primaries p) {
    return e.has_value() && e->transfer == t && e->primaries == p;
  };
  CHECK(is(video::resolve_encoding(AVCOL_TRC_BT709, AVCOL_PRI_BT709),
           Transfer::Bt709, Primaries::Bt709));
  CHECK(
      is(video::resolve_encoding(AVCOL_TRC_UNSPECIFIED, AVCOL_PRI_UNSPECIFIED),
         Transfer::Bt709, Primaries::Bt709));
  CHECK(is(video::resolve_encoding(AVCOL_TRC_IEC61966_2_1, AVCOL_PRI_SMPTE432),
           Transfer::Srgb, Primaries::DisplayP3));
  CHECK(is(video::resolve_encoding(AVCOL_TRC_SMPTE2084, AVCOL_PRI_BT2020),
           Transfer::Bt2020Pq, Primaries::Bt2020));
  CHECK(is(video::resolve_encoding(AVCOL_TRC_BT2020_10, AVCOL_PRI_BT2020),
           Transfer::Bt709, Primaries::Bt2020));
  CHECK(is(video::resolve_encoding(AVCOL_TRC_LINEAR, AVCOL_PRI_BT709),
           Transfer::Linear, Primaries::Bt709));
  // Nothing true to declare: HLG, and BT.601's own primaries.
  CHECK(!video::resolve_encoding(AVCOL_TRC_ARIB_STD_B67, AVCOL_PRI_BT2020));
  CHECK(!video::resolve_encoding(AVCOL_TRC_SMPTE170M, AVCOL_PRI_SMPTE170M));

  video::PictureConverter converter("test");
  auto frame = solid(AV_PIX_FMT_YUV420P, 64, 32, 100, 90, 160);
  frame->color_trc = AVCOL_TRC_SMPTE2084;
  frame->color_primaries = AVCOL_PRI_BT2020;
  auto picture = converter.convert(*frame, VideoPixelLayout::Yuv420);
  CHECK(picture.ok());
  CHECK(is(picture->encoding, Transfer::Bt2020Pq, Primaries::Bt2020));
  return 0;
}

// One converter follows changes to the chroma tag, including when swscale
// resamples or converts to RGB. The same continuous ramps give the same RGB
// at even and odd luma pixels, regardless of their chroma sample locations.
int test_chroma_locations() {
  using Location = sensor::ChromaLocation;
  struct Case {
    AVChromaLocation tag;
    Location location;
    int dx;
    int dy;
  };
  const Case cases[] = {{AVCHROMA_LOC_UNSPECIFIED, Location::Left, 0, 5},
                        {AVCHROMA_LOC_CENTER, Location::Center, 5, 5},
                        {AVCHROMA_LOC_TOPLEFT, Location::TopLeft, 0, 0},
                        {AVCHROMA_LOC_TOP, Location::Top, 5, 0},
                        {AVCHROMA_LOC_BOTTOMLEFT, Location::BottomLeft, 0, 10},
                        {AVCHROMA_LOC_BOTTOM, Location::Bottom, 5, 10},
                        {AVCHROMA_LOC_LEFT, Location::Left, 0, 5}};
  video::PictureConverter converter("test");
  for (const auto format :
       {AV_PIX_FMT_YUV420P, AV_PIX_FMT_NV12, AV_PIX_FMT_YUV422P}) {
    for (const auto layout :
         {VideoPixelLayout::Yuv420, VideoPixelLayout::Rgb24}) {
      for (const auto& c : cases) {
        auto frame = solid(format, 16, 16, 128, 128, 128);
        frame->colorspace = AVCOL_SPC_SMPTE170M;
        frame->color_range = AVCOL_RANGE_JPEG;
        frame->chroma_location = c.tag;
        const bool subsampled_y = format != AV_PIX_FMT_YUV422P;
        for (int y = 0; y < (subsampled_y ? 8 : 16); ++y) {
          for (int x = 0; x < 8; ++x) {
            const auto u = static_cast<std::uint8_t>(48 + 20 * x + c.dx);
            const auto v = static_cast<std::uint8_t>(
                48 + (subsampled_y ? 20 * y + c.dy : 10 * y));
            frame->data[1][y * frame->linesize[1] +
                           (format == AV_PIX_FMT_NV12 ? 2 * x : x)] = u;
            if (format == AV_PIX_FMT_NV12) {
              frame->data[1][y * frame->linesize[1] + 2 * x + 1] = v;
            } else {
              frame->data[2][y * frame->linesize[2] + x] = v;
            }
          }
        }
        auto picture = converter.convert(*frame, layout);
        CHECK(picture.ok());
        CHECK(picture->chroma_location == c.location);
        if (layout == VideoPixelLayout::Rgb24) {
          for (int y : {7, 8}) {
            for (int x : {7, 8}) {
              const auto* pixel =
                  picture->plane[0] + y * picture->stride[0] + 3 * x;
              const auto want =
                  yuv_reference::rgb(128, 48 + 10 * x, 48 + 10 * y,
                                     picture->matrix, picture->full_range);
              for (int channel = 0; channel < 3; ++channel) {
                if (std::abs(pixel[channel] - want[channel]) > 2) {
                  std::fprintf(stderr,
                               "%s chroma %d at (%d, %d) channel %d: %d, "
                               "want %d\n",
                               av_get_pix_fmt_name(format), c.tag, x, y,
                               channel, pixel[channel], want[channel]);
                  CHECK(false);
                }
              }
            }
          }
        } else {
          CHECK(std::abs(picture->plane[1][4 * picture->stride[1] + 4] -
                         (128 + c.dx)) <= 1);
          CHECK(std::abs(picture->plane[2][4 * picture->stride[2] + 4] -
                         (128 + c.dy)) <= 1);
        }
      }
    }
  }
  return 0;
}

int test_yuv420_passes_through() {
  video::PictureConverter converter("test");
  const auto frame = solid(AV_PIX_FMT_YUV420P, 64, 32, 100, 90, 160);
  auto picture = converter.convert(*frame, VideoPixelLayout::Yuv420);
  CHECK(picture.ok());
  CHECK(picture->width == 64 && picture->height == 32);
  for (int i = 0; i < 3; ++i) {
    CHECK(picture->plane[i] == frame->data[i]);
    CHECK(picture->stride[i] == static_cast<std::size_t>(frame->linesize[i]));
  }
  return 0;
}

int test_nv12_splits_exactly() {
  video::PictureConverter converter("test");
  const auto frame = solid(AV_PIX_FMT_NV12, 64, 32, 100, 90, 160);
  auto picture = converter.convert(*frame, VideoPixelLayout::Yuv420);
  CHECK(picture.ok());
  for (int row = 0; row < 16; ++row) {
    for (int x = 0; x < 32; ++x) {
      CHECK(picture->plane[1][row * picture->stride[1] + x] == 90);
      CHECK(picture->plane[2][row * picture->stride[2] + x] == 160);
    }
  }
  CHECK(picture->plane[0][31 * picture->stride[0] + 63] == 100);
  return 0;
}

int test_rgb_follows_matrix_and_range() {
  struct Case {
    AVColorSpace space;
    AVColorRange range;
    VideoColorMatrix matrix;
  };
  const Case cases[] = {
      {AVCOL_SPC_BT709, AVCOL_RANGE_MPEG, VideoColorMatrix::Bt709},
      {AVCOL_SPC_SMPTE170M, AVCOL_RANGE_MPEG, VideoColorMatrix::Bt601},
      {AVCOL_SPC_BT2020_NCL, AVCOL_RANGE_MPEG, VideoColorMatrix::Bt2020},
      {AVCOL_SPC_BT709, AVCOL_RANGE_JPEG, VideoColorMatrix::Bt709},
      {AVCOL_SPC_SMPTE240M, AVCOL_RANGE_MPEG, VideoColorMatrix::Smpte240m},
      {AVCOL_SPC_FCC, AVCOL_RANGE_MPEG, VideoColorMatrix::Fcc},
  };
  // The last is saturated enough that SMPTE 240M and BT.709 differ by 5.
  const int colours[][3] = {
      {100, 90, 160}, {180, 150, 100}, {60, 170, 120}, {80, 208, 208}};
  // One converter throughout: its colour setup must follow each frame.
  video::PictureConverter converter("test");
  for (const Case& c : cases) {
    for (const auto& yuv : colours) {
      auto frame = solid(AV_PIX_FMT_YUV420P, 64, 32, yuv[0], yuv[1], yuv[2]);
      frame->colorspace = c.space;
      frame->color_range = c.range;
      auto picture = converter.convert(*frame, VideoPixelLayout::Rgb24);
      CHECK(picture.ok());
      CHECK(picture->matrix == c.matrix);
      CHECK(picture->full_range == (c.range == AVCOL_RANGE_JPEG));
      const auto want = yuv_reference::rgb(yuv[0], yuv[1], yuv[2], c.matrix,
                                           c.range == AVCOL_RANGE_JPEG);
      const std::uint8_t* px = picture->plane[0] + 16 * picture->stride[0];
      for (int k = 0; k < 3; ++k) {
        if (std::abs(px[3 * 32 + k] - want[k]) > 2) {
          std::fprintf(stderr, "channel %d: %d, want %d\n", k, px[3 * 32 + k],
                       want[k]);
          CHECK(false);
        }
      }
    }
  }
  return 0;
}

// A full-range YUVJ source converted to Yuv420 stays full range, as its label
// says (swscale alone takes a J format to limited range).
int test_yuv420_keeps_range() {
  video::PictureConverter converter("test");
  for (const AVPixelFormat format :
       {AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUV422P}) {
    auto frame = solid(format, 64, 32, 255, 128, 128);
    if (format == AV_PIX_FMT_YUV422P) frame->color_range = AVCOL_RANGE_JPEG;
    auto picture = converter.convert(*frame, VideoPixelLayout::Yuv420);
    CHECK(picture.ok());
    CHECK(picture->full_range);
    CHECK(picture->plane[0][16 * picture->stride[0] + 32] == 255);
  }
  auto limited = solid(AV_PIX_FMT_YUV422P, 64, 32, 235, 128, 128);
  auto picture = converter.convert(*limited, VideoPixelLayout::Yuv420);
  CHECK(picture.ok());
  CHECK(!picture->full_range);
  CHECK(picture->plane[0][16 * picture->stride[0] + 32] == 235);
  return 0;
}

// An RGB-coded (GBR) picture converted to Yuv420 by the matrix it is
// labelled with: converting the planes back by that label gives the colour.
int test_rgb_source_to_yuv420() {
  const int rgb[3] = {40, 60, 220};
  for (const int height : {480, 720}) {
    video::PictureConverter to_yuv("test");
    auto gbr = solid(AV_PIX_FMT_GBRP, 64, height, rgb[1], rgb[2], rgb[0]);
    gbr->colorspace = AVCOL_SPC_RGB;
    auto yuv = to_yuv.convert(*gbr, VideoPixelLayout::Yuv420);
    CHECK(yuv.ok());
    CHECK(yuv->matrix ==
          (height > 576 ? VideoColorMatrix::Bt709 : VideoColorMatrix::Bt601));
    const std::size_t row = static_cast<std::size_t>(height / 2);
    const int y = yuv->plane[0][row * yuv->stride[0] + 32];
    const int u = yuv->plane[1][row / 2 * yuv->stride[1] + 16];
    const int v = yuv->plane[2][row / 2 * yuv->stride[2] + 16];
    const auto back = yuv_reference::rgb(y, u, v, yuv->matrix, yuv->full_range);
    for (int k = 0; k < 3; ++k) {
      if (std::abs(back[k] - rgb[k]) > 3) {
        std::fprintf(stderr, "height %d channel %d: %d, want %d\n", height, k,
                     back[k], rgb[k]);
        CHECK(false);
      }
    }
  }
  return 0;
}

// The unlabelled colour stands in for a frame that declares no matrix --
// its limited range too, which is how FFmpeg reads a stream that declares
// neither -- for both the conversion and what the picture reports. A frame
// that declares one keeps it.
int test_unlabelled_color() {
  video::PictureConverter converter("test");
  auto frame = solid(AV_PIX_FMT_YUV420P, 64, 32, 100, 90, 160);
  frame->colorspace = AVCOL_SPC_UNSPECIFIED;
  frame->color_range = AVCOL_RANGE_MPEG;
  const sensor::VideoColorDescription femto{VideoColorMatrix::Bt601, true};
  auto rgb = converter.convert(*frame, VideoPixelLayout::Rgb24, femto);
  CHECK(rgb.ok());
  CHECK(rgb->matrix == VideoColorMatrix::Bt601 && rgb->full_range);
  const auto want =
      yuv_reference::rgb(100, 90, 160, VideoColorMatrix::Bt601, true);
  const std::uint8_t* px = rgb->plane[0] + 16 * rgb->stride[0] + 3 * 32;
  for (int k = 0; k < 3; ++k) CHECK(std::abs(px[k] - want[k]) <= 2);
  auto yuv = converter.convert(*frame, VideoPixelLayout::Yuv420, femto);
  CHECK(yuv.ok());
  CHECK(yuv->matrix == VideoColorMatrix::Bt601 && yuv->full_range);
  CHECK(yuv->plane[0] == frame->data[0]);  // the label changes, not the bytes

  frame->colorspace = AVCOL_SPC_BT709;
  auto labelled = converter.convert(*frame, VideoPixelLayout::Rgb24, femto);
  CHECK(labelled.ok());
  CHECK(labelled->matrix == VideoColorMatrix::Bt709 && !labelled->full_range);
  const auto as_labelled =
      yuv_reference::rgb(100, 90, 160, VideoColorMatrix::Bt709, false);
  px = labelled->plane[0] + 16 * labelled->stride[0] + 3 * 32;
  for (int k = 0; k < 3; ++k) CHECK(std::abs(px[k] - as_labelled[k]) <= 2);
  return 0;
}

int test_size_change() {
  video::PictureConverter converter("test");
  const auto large = solid(AV_PIX_FMT_YUV420P, 64, 32, 100, 90, 160);
  const auto small = solid(AV_PIX_FMT_YUV420P, 32, 16, 50, 128, 128);
  CHECK(converter.convert(*large, VideoPixelLayout::Rgb24).ok());
  auto picture = converter.convert(*small, VideoPixelLayout::Rgb24);
  CHECK(picture.ok());
  CHECK(picture->width == 32 && picture->height == 16);
  const auto want =
      yuv_reference::rgb(50, 128, 128, VideoColorMatrix::Bt601, false);
  CHECK(std::abs(picture->plane[0][15 * picture->stride[0] + 3 * 31] -
                 want[0]) <= 2);
  return 0;
}

int test_refusals() {
  video::PictureConverter converter("test");
  auto frame = solid(AV_PIX_FMT_YUV420P, 64, 32, 100, 90, 160);
  frame->format = AV_PIX_FMT_VIDEOTOOLBOX;  // a hardware surface
  CHECK(converter.convert(*frame, VideoPixelLayout::Rgb24).status().domain() ==
        volumetric_kit::core::Status::Code::InvalidArgument);
  frame->format = AV_PIX_FMT_YUV420P;
  frame->width = 0;
  CHECK(converter.convert(*frame, VideoPixelLayout::Rgb24).status().domain() ==
        volumetric_kit::core::Status::Code::InvalidArgument);
  return 0;
}

}  // namespace

int main() {
  if (test_resolve_matrix() != 0) return 1;
  if (test_resolve_encoding() != 0) return 1;
  if (test_chroma_locations() != 0) return 1;
  if (test_yuv420_passes_through() != 0) return 1;
  if (test_nv12_splits_exactly() != 0) return 1;
  if (test_rgb_follows_matrix_and_range() != 0) return 1;
  if (test_yuv420_keeps_range() != 0) return 1;
  if (test_rgb_source_to_yuv420() != 0) return 1;
  if (test_unlabelled_color() != 0) return 1;
  if (test_size_change() != 0) return 1;
  if (test_refusals() != 0) return 1;
  std::puts("sensor_video_converter: OK");
  return 0;
}
