// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/orbbec_config.hpp
/// @brief fuse_orbbec's scenario: what it refuses, and its validated camera
///        settings with the runtime decoder resources attached.

#include "config.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_stream.hpp"

namespace vr_example {

namespace config = volumetric_kit::core::config;

/// @return OK unless @p settings names the phone, `source.mobile` or
///         `diagnostics.track_log` (which requires it), which fuse_orbbec
///         does not run yet: `Status::Code::Unsupported`, naming both keys.
///         fuse_orbbec checks it first, before any GPU or camera opens, so
///         the rig never fuses alone in a run that asked for the phone.
inline volumetric_kit::core::Status check_orbbec_phone(
    const config::Config& settings) {
  // TODO: run the phone's session beside the rig, then take these keys.
  if (!settings.source.mobile && settings.diagnostics.track_log.empty()) {
    return {};
  }
  return volumetric_kit::core::Status::unsupported(
      "source.mobile (the phone's session) and diagnostics.track_log are not "
      "run by this fuse_orbbec yet, which fuses the rig's cameras alone; "
      "remove both to fuse the rig");
}

/// @return Runtime stream settings copied from core's validated configuration.
inline volumetric_kit::sensor::OrbbecStreamOptions orbbec_stream_options(
    const config::Config& settings) {
  volumetric_kit::sensor::OrbbecStreamOptions streams;
  namespace sensor = volumetric_kit::sensor;
  const auto& source = settings.source;
  streams.color_codec = source.color_codec == config::ColorCodec::Hevc
                            ? sensor::OrbbecColorCodec::Hevc
                            : sensor::OrbbecColorCodec::Mjpeg;
  streams.color_width = source.color_width;
  streams.color_height = source.color_height;
  streams.depth_width = source.depth_width;
  streams.depth_height = source.depth_height;
  streams.fps = source.fps;
  streams.min_depth = settings.fusion.min_depth_m;
  streams.max_depth = settings.fusion.max_depth_m;
  return streams;
}

/// @brief Attach decoder resources owned by the running application.
/// @param settings A configuration validated for this application by core.
/// @param device Decoder device, kept alive by the application.
/// @param allocator Decoder allocator, kept alive by the application.
/// @param streams Destination runtime stream options.
inline void apply_orbbec_config(
    const config::Config& settings, const volumetric_kit::core::Device& device,
    volumetric_kit::core::Allocator& allocator,
    volumetric_kit::sensor::OrbbecStreamOptions& streams) {
  streams = orbbec_stream_options(settings);
  streams.device = &device;
  streams.allocator = &allocator;
}

}  // namespace vr_example
