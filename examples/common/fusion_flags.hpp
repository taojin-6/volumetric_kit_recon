// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fusion_flags.hpp
/// @brief The fusion flags every example that fuses a sensor's frames takes,
///        with one validation: `--voxel`, `--trunc`, `--min-depth`,
///        `--max-depth` and `--max-weight`, and the rule that refuses them
///        in a mode that fuses nothing.

#include <optional>
#include <string>

#include "cli.hpp"
#include "grid_layout.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vr_example {

/// @brief A fusing example's volume and depth gate, as its command line set
///        them.
struct FusionFlags {
  /// @param default_voxel  The example's own default voxel edge (metres).
  explicit FusionFlags(float default_voxel) : voxel(default_voxel) {}

  float voxel;  ///< Voxel edge (metres).
  /// Truncation distance (metres); 0 or below is @ref default_trunc of
  /// @ref voxel, set by the check @ref add_to declares.
  float trunc = 0.0f;
  /// The depth gate (metres), each end the source's own default when unset.
  std::optional<float> min_depth;
  std::optional<float> max_depth;  ///< See @ref min_depth.
  float max_weight = 20.0f;        ///< The running-average cap.

  /// The flags' names, in the order @ref add_to declares them: @ref voxel,
  /// @ref trunc, @ref min_depth, @ref max_depth, @ref max_weight.
  static constexpr const char* kNames[] = {"--voxel", "--trunc", "--min-depth",
                                           "--max-depth", "--max-weight"};

  /// @brief Declare the five flags on @p cli, and the check that validates
  ///        them and defaults @ref trunc. This object must outlive the parse.
  void add_to(Cli& cli) {
    cli.option(kNames[0], "m", voxel)
        .option(kNames[1], "m", trunc)
        .option(kNames[2], "m", min_depth)
        .option(kNames[3], "m", max_depth)
        .option(kNames[4], "w", max_weight)
        .check([this] { return finish(); });
  }

  /// @brief Refuse the five flags on a command line that gives @p name, a
  ///        mode that fuses nothing (@ref Cli::excludes).
  /// @pre @ref add_to and @p name were declared on @p cli.
  static void excluded_by(Cli& cli, const char* name) {
    for (const char* flag : kNames) cli.excludes(name, {flag});
  }

  /// @brief Validate and default @ref trunc; @ref add_to's check. Numbers are
  ///        already finite (@ref parse_number), and @ref check_voxel keeps
  ///        the default band so. A depth end given alone is checked against
  ///        the other's default by the source, which names both values when
  ///        it refuses.
  vkc::Status finish() {
    VKC_TRY(check_voxel(voxel));
    if (trunc <= 0.0f) trunc = default_trunc(voxel);
    if ((min_depth && !(*min_depth > 0.0f)) ||
        (max_depth && !(*max_depth > 0.0f))) {
      return vkc::Status::invalid_argument(
          "--min-depth and --max-depth must be > 0");
    }
    if (min_depth && max_depth && !(*min_depth < *max_depth)) {
      return vkc::Status::invalid_argument(
          "--min-depth must be below --max-depth");
    }
    if (!(max_weight > 0.0f)) {
      return vkc::Status::invalid_argument("--max-weight must be > 0");
    }
    return {};
  }

  /// @brief Set the depth gate's given ends on a source's options (anything
  ///        with `min_depth` and `max_depth`), leaving the others its own.
  template <typename SourceOptions>
  void apply_depth_gate(SourceOptions& options) const {
    if (min_depth) options.min_depth = *min_depth;
    if (max_depth) options.max_depth = *max_depth;
  }
};

}  // namespace vr_example
