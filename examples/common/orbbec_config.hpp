// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/orbbec_config.hpp
/// @brief Attach runtime decoder resources to validated camera settings.

#include "config.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_stream.hpp"

namespace vr_example {

namespace config = volumetric_kit::core::config;

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
