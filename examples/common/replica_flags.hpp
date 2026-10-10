// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/replica_flags.hpp
/// @brief The flags every Replica example takes -- `<scene_dir>`,
///        `--cam-params`, `--max-frames` and `--preload` -- the sensor they
///        open, and the preload's report.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <string>

#include "cli.hpp"
#include "fusion_flags.hpp"
#include "replica_sensor.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vr_example {

/// @brief A Replica example's sequence, as its command line chose it.
struct ReplicaFlags {
  /// @param default_max_frames  The example's own frame cap.
  explicit ReplicaFlags(int default_max_frames)
      : max_frames(default_max_frames) {}

  std::string scene_dir;  ///< Holds `results/` and `traj.txt`.
  /// The intrinsics; empty is `<scene_dir>/../cam_params.json`.
  std::string cam_params;
  int max_frames;  ///< One past the highest frame index to play.
  /// Decode every frame up front (@ref preload_frames), so the loop measures
  /// compute rather than the JPEG/PNG decoder.
  bool preload = false;

  /// @brief Declare `<scene_dir>` and the three flags on @p cli. This object
  ///        must outlive the parse.
  void add_to(Cli& cli) {
    cli.positional("scene_dir", scene_dir)
        .option("--cam-params", "path", cam_params)
        .option("--max-frames", "N", max_frames, 1)
        .flag("--preload", preload);
  }

  /// @return @ref cam_params, or its default beside the scene.
  std::string cam_params_path() const {
    return cam_params.empty() ? scene_dir + "/../cam_params.json" : cam_params;
  }

  /// @brief Open the sequence with @p fusion's depth gate.
  /// @param stride  Play every N-th frame.
  vkc::Result<ReplicaSensor> open(const FusionFlags& fusion,
                                  std::size_t stride = 1) const {
    ReplicaSensor::Options options;
    options.frame_limit = static_cast<std::size_t>(max_frames);
    options.frame_stride = stride;
    fusion.apply_depth_gate(options);
    return ReplicaSensor::open(scene_dir, cam_params_path(), options);
  }
};

/// @brief `--preload`: decode @p sensor's frames up front, saying what that
///        will hold before spending it (the flag has no cap of its own, so a
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
