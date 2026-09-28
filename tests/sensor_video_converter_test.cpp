// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The picture converter on frames built by hand, no codec: the matrix a
// stream's colour space resolves to, Yuv420 passed through without a copy,
// NV12 split exactly, RGB against the standards' constants for each matrix
// and range, a size change, and the frames it refuses.

#include <cstdio>
#include <cstdlib>
#include <cstring>

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

// A solid (y, u, v) frame in @p format: yuv420p or nv12.
video::FramePtr solid(AVPixelFormat format, int width, int height, int y, int u,
                      int v) {
  video::FramePtr frame(av_frame_alloc());
  frame->format = format;
  frame->width = width;
  frame->height = height;
  if (av_frame_get_buffer(frame.get(), 0) < 0) std::abort();
  for (int row = 0; row < height; ++row) {
    std::memset(frame->data[0] + row * frame->linesize[0], y, width);
  }
  for (int row = 0; row < height / 2; ++row) {
    if (format == AV_PIX_FMT_NV12) {
      std::uint8_t* line = frame->data[1] + row * frame->linesize[1];
      for (int x = 0; x < width / 2; ++x) {
        line[2 * x] = static_cast<std::uint8_t>(u);
        line[2 * x + 1] = static_cast<std::uint8_t>(v);
      }
    } else {
      std::memset(frame->data[1] + row * frame->linesize[1], u, width / 2);
      std::memset(frame->data[2] + row * frame->linesize[2], v, width / 2);
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
  return 0;
}

int test_yuv420_passes_through() {
  video::PictureConverter converter;
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
  video::PictureConverter converter;
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
  };
  const int colours[][3] = {{100, 90, 160}, {180, 150, 100}, {60, 170, 120}};
  // One converter throughout: its colour setup must follow each frame.
  video::PictureConverter converter;
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

int test_size_change() {
  video::PictureConverter converter;
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
  video::PictureConverter converter;
  auto frame = solid(AV_PIX_FMT_YUV420P, 64, 32, 100, 90, 160);
  frame->format = AV_PIX_FMT_VIDEOTOOLBOX;  // a hardware surface
  CHECK(converter.convert(*frame, VideoPixelLayout::Rgb24).status().domain() ==
        volumetric_kit::recon::Status::Code::InvalidArgument);
  frame->format = AV_PIX_FMT_YUV420P;
  frame->width = 0;
  CHECK(converter.convert(*frame, VideoPixelLayout::Rgb24).status().domain() ==
        volumetric_kit::recon::Status::Code::InvalidArgument);
  return 0;
}

}  // namespace

int main() {
  if (test_resolve_matrix() != 0) return 1;
  if (test_yuv420_passes_through() != 0) return 1;
  if (test_nv12_splits_exactly() != 0) return 1;
  if (test_rgb_follows_matrix_and_range() != 0) return 1;
  if (test_size_change() != 0) return 1;
  if (test_refusals() != 0) return 1;
  std::puts("sensor_video_converter: OK");
  return 0;
}
