// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/video/video_backend.hpp
/// @brief Where a video decoder runs.

#include "volumetric_kit/recon/sensor/video/export.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief FFmpeg's hardware back ends, or its software decoder.
enum class VideoDecodeBackend {
  Auto,          ///< The platform's first back end that decodes the stream.
  Software,      ///< FFmpeg's CPU decoder.
  VideoToolbox,  ///< Apple.
  Cuda,          ///< NVIDIA (NVDEC).
  Vaapi,         ///< Intel and AMD on Linux.
  D3d11va,       ///< Windows.
};

/// @return A stable lowercase name for @p backend (`"cuda"`, ...).
VR_SENSOR_VIDEO_API const char* to_string(VideoDecodeBackend backend) noexcept;

}  // namespace volumetric_kit::recon::sensor
