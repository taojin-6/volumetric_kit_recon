// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/orbbec_flags.hpp
/// @brief The flags both Orbbec examples take -- the rig (`--rig`,
///        `--calibration`, `--apply-sync`) and its streams (`--hevc |
///        --mjpeg`, `--color`, `--fps`) -- and the stream options they set.
///        Compiled only into the examples built with the Orbbec driver.

#include <cstdint>
#include <cstdio>
#include <string>

#include "cli.hpp"
#include "fusion_flags.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_stream.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief An Orbbec example's cameras and streams, as its command line chose
///        them.
struct OrbbecFlags {
  std::string rig;          ///< A sync configuration: its cameras as a rig.
  std::string calibration;  ///< Poses by serial; empty: all at the origin.
  bool apply_sync = false;  ///< Write the sync configuration where it differs.
  /// H.265 colour rather than MJPEG, which takes about nine times the
  /// bandwidth (185 against 21 Mbit/s a camera at 4K).
  bool hevc = true;
  std::uint32_t color_width = 0;  ///< 0 keeps the driver's default mode.
  std::uint32_t color_height = 0;
  std::uint32_t fps = 0;  ///< 0 keeps the driver's default.

  /// @brief Declare the six flags on @p cli, and the rule that
  ///        `--apply-sync` needs `--rig`. This object must outlive the parse.
  void add_to(Cli& cli) {
    cli.option("--rig", "sync.json", rig)
        .option("--calibration", "calib.json", calibration)
        .flag("--apply-sync", apply_sync)
        .choice<bool>({{"--hevc", true}, {"--mjpeg", false}}, hevc)
        .on("--color", "WxH",
            [this](const std::string& flag, const char* value) {
              unsigned width = 0, height = 0;
              int used = 0;
              if (std::sscanf(value, "%ux%u%n", &width, &height, &used) != 2 ||
                  value[used] != '\0' || width == 0 || height == 0) {
                return vkc::Status::invalid_argument(
                    flag + " needs WxH, e.g. 1920x1080");
              }
              color_width = width;
              color_height = height;
              return vkc::Status{};
            })
        .option("--fps", "N", fps, 1)
        .check([this] {
          return apply_sync && rig.empty()
                     ? vkc::Status::invalid_argument("--apply-sync needs --rig")
                     : vkc::Status{};
        });
  }

  /// @brief The streams asked for, over the driver's defaults, with
  ///        @p fusion's depth gate, the colour decoded onto @p device in
  ///        buffers made through @p allocator.
  void apply(const FusionFlags& fusion, const vkc::Device& device,
             vkc::Allocator& allocator,
             vr::sensor::OrbbecStreamOptions& streams) const {
    streams.color_codec = hevc ? vr::sensor::OrbbecColorCodec::Hevc
                               : vr::sensor::OrbbecColorCodec::Mjpeg;
    streams.device = &device;
    streams.allocator = &allocator;
    if (color_width != 0) {
      streams.color_width = color_width;
      streams.color_height = color_height;
    }
    if (fps != 0) streams.fps = fps;
    fusion.apply_depth_gate(streams);
  }
};

}  // namespace vr_example
