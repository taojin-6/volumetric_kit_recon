// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/replica_config.hpp
/// @brief Map the core's resolved dataset settings to the example sensor.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>

#include "config.hpp"
#include "replica_sensor.hpp"

namespace vr_example {

/// @return Sensor options copied from a validated dataset configuration.
inline ReplicaSensor::Options replica_options(const config::Config& settings) {
  ReplicaSensor::Options options;
  options.frame_limit = static_cast<std::size_t>(settings.run.frames);
  options.frame_stride = static_cast<std::size_t>(settings.fusion.stride);
  options.min_depth = settings.fusion.min_depth_m;
  options.max_depth = settings.fusion.max_depth_m;
  return options;
}

/// @brief Open the dataset named by already validated settings.
/// @return The source, or its asset I/O error.
inline volumetric_kit::core::Result<ReplicaSensor> open_replica(
    const config::Config& settings) {
  return ReplicaSensor::open(settings.source.path,
                             settings.source.camera_parameters,
                             replica_options(settings));
}

/// @brief Decode @p sensor's frames up front, reporting the projected memory
///        before spending it (preloading has no cap of its own, so a
///        long sequence can ask for gigabytes) and what it held after.
/// @param cancel  As @ref ReplicaSensor::preload.
/// @return @ref ReplicaSensor::preload's.
inline vkc::Result<std::size_t> preload_frames(
    ReplicaSensor& sensor, const std::atomic<bool>* cancel = nullptr) {
  constexpr double kMiB = 1024.0 * 1024.0;
  std::printf("preloading %.0f MB...\n",
              static_cast<double>(sensor.preload_bytes_projected()) / kMiB);
  const auto start = std::chrono::steady_clock::now();
  VKC_ASSIGN(const std::size_t frames, sensor.preload(cancel));
  std::printf(
      "preloaded %zu frames (%.0f MB) in %.1fs\n", frames,
      static_cast<double>(sensor.preloaded_bytes()) / kMiB,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count());
  return frames;
}

}  // namespace vr_example
