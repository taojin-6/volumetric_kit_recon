// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/video/video_backend.hpp"

namespace volumetric_kit::recon::sensor {

const char* to_string(VideoDecodeBackend backend) noexcept {
  switch (backend) {
    case VideoDecodeBackend::Auto:
      return "auto";
    case VideoDecodeBackend::Software:
      return "software";
    case VideoDecodeBackend::VideoToolbox:
      return "videotoolbox";
    case VideoDecodeBackend::Cuda:
      return "cuda";
    case VideoDecodeBackend::Vulkan:
      return "vulkan";
    case VideoDecodeBackend::Vaapi:
      return "vaapi";
    case VideoDecodeBackend::D3d11va:
      return "d3d11va";
  }
  return "unknown";
}

}  // namespace volumetric_kit::recon::sensor
