// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The hardware back ends, no decoding: the platform's order (NVIDIA ahead of
// an integrated GPU on Linux), FFmpeg's device for each, which ones the
// platform can be asked whether they decode, and which can crop.

#include <cstdio>
#include <vector>

#include "hw_backend.hpp"

namespace sensor = volumetric_kit::recon::sensor;
namespace video = volumetric_kit::recon::sensor::video;
using sensor::VideoDecodeBackend;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

int test_platform_order() {
  const std::vector<VideoDecodeBackend> order =
      video::platform_hardware_order();
#if defined(__APPLE__)
  CHECK(order ==
        std::vector<VideoDecodeBackend>{VideoDecodeBackend::VideoToolbox});
#elif defined(_WIN32)
  CHECK(order == (std::vector<VideoDecodeBackend>{
                     VideoDecodeBackend::Cuda, VideoDecodeBackend::D3d11va}));
#else
  // NVIDIA first, Intel's and AMD's VAAPI last.
  CHECK(order == (std::vector<VideoDecodeBackend>{VideoDecodeBackend::Cuda,
                                                  VideoDecodeBackend::Vulkan,
                                                  VideoDecodeBackend::Vaapi}));
#endif
  return 0;
}

int test_device_types() {
  CHECK(video::device_type(VideoDecodeBackend::Auto) == AV_HWDEVICE_TYPE_NONE);
  CHECK(video::device_type(VideoDecodeBackend::Software) ==
        AV_HWDEVICE_TYPE_NONE);
  CHECK(video::device_type(VideoDecodeBackend::VideoToolbox) ==
        AV_HWDEVICE_TYPE_VIDEOTOOLBOX);
  CHECK(video::device_type(VideoDecodeBackend::Cuda) == AV_HWDEVICE_TYPE_CUDA);
  CHECK(video::device_type(VideoDecodeBackend::Vulkan) ==
        AV_HWDEVICE_TYPE_VULKAN);
  CHECK(video::device_type(VideoDecodeBackend::Vaapi) ==
        AV_HWDEVICE_TYPE_VAAPI);
  CHECK(video::device_type(VideoDecodeBackend::D3d11va) ==
        AV_HWDEVICE_TYPE_D3D11VA);
  return 0;
}

// Only VideoToolbox can be asked, and only on Apple; the rest are left to the
// probe clip. Whether the answer is yes depends on the Mac, so only that there
// is one is checked (CI's VR_TEST_HEVC_BACKEND checks the yes).
int test_hardware_decodes() {
  for (const VideoDecodeBackend b :
       {VideoDecodeBackend::Cuda, VideoDecodeBackend::Vulkan,
        VideoDecodeBackend::Vaapi, VideoDecodeBackend::D3d11va}) {
    CHECK(!video::hardware_decodes(b, AV_CODEC_ID_HEVC).has_value());
  }
  const auto vt_hevc = video::hardware_decodes(VideoDecodeBackend::VideoToolbox,
                                               AV_CODEC_ID_HEVC);
#if defined(__APPLE__)
  CHECK(vt_hevc.has_value());
  std::printf("  videotoolbox decodes hevc: %s\n", *vt_hevc ? "yes" : "no");
  CHECK(video::hardware_decodes(VideoDecodeBackend::VideoToolbox,
                                AV_CODEC_ID_H264)
            .has_value());
#else
  CHECK(!vt_hevc.has_value());
#endif
  CHECK(!video::hardware_decodes(VideoDecodeBackend::VideoToolbox,
                                 AV_CODEC_ID_MJPEG)
             .has_value());
  return 0;
}

int test_crops() {
  CHECK(!video::crops_left_and_top(VideoDecodeBackend::VideoToolbox));
  CHECK(video::crops_left_and_top(VideoDecodeBackend::Cuda));
  CHECK(video::crops_left_and_top(VideoDecodeBackend::Software));
  return 0;
}

}  // namespace

int main() {
  if (test_platform_order() != 0) return 1;
  if (test_device_types() != 0) return 1;
  if (test_hardware_decodes() != 0) return 1;
  if (test_crops() != 0) return 1;
  std::puts("sensor_video_backend: OK");
  return 0;
}
