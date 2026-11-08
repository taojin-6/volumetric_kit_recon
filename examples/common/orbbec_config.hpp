// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/orbbec_config.hpp
/// @brief fuse_orbbec's scenario as the Orbbec driver's stream settings.

#include "config.hpp"
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

}  // namespace vr_example
