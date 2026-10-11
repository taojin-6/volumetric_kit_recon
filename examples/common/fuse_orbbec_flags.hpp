// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_orbbec_flags.hpp
/// @brief fuse_orbbec's command line: one camera or a rig and its streams
///        (@ref vr_example::OrbbecFlags), the run's length, and the fusion
///        `--no-fuse` leaves out. Compiled only into the examples built with
///        the Orbbec driver.

#include <string>

#include "cli.hpp"
#include "fusion_flags.hpp"
#include "orbbec_flags.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vr_example {

/// @brief fuse_orbbec's options, as its command line set them.
struct FuseOrbbecFlags {
  std::string serial;         ///< One camera; empty: the only one found.
  OrbbecFlags cameras;        ///< The rig, its poses and its streams.
  FusionFlags fusion{0.02f};  ///< The volume and the depth gate.
  bool host_clock = false;    ///< One camera on the host's clock.
  /// The frames the run takes: a camera never runs out, so a run needs an
  /// end.
  int frames = 300;
  std::string out = "fuse_orbbec.ply";  ///< Where the mesh is written.
  /// Stream only: each set is polled, counted and released, with no GPU
  /// preparation, fusion or mesh, so a run measures the cameras' stream and
  /// decode alone.
  bool no_fuse = false;

  /// @brief Declare fuse_orbbec's flags on @p cli, with their rules:
  ///        `--serial` or `--rig`, `--host-clock` only for one camera, and
  ///        `--no-fuse` without the fusion flags or `-o`. This object must
  ///        outlive the parse.
  void add_to(Cli& cli) {
    cli.option("--serial", "SN", serial);
    cameras.add_to(cli);
    cli.one_of({"--serial", "--rig"})
        .flag("--host-clock", host_clock)
        .option("--frames", "N", frames, 1)
        .flag("--no-fuse", no_fuse)
        .option({"-o", "--out"}, "out.ply", out);
    fusion.add_to(cli);
    FusionFlags::excluded_by(cli, "--no-fuse");
    cli.excludes("--no-fuse", {"-o"}).check([this] {
      if (host_clock && !cameras.rig.empty()) {
        return vkc::Status::invalid_argument(
            "--host-clock is for one camera; a rig is always on the host's "
            "clock");
      }
      return vkc::Status{};
    });
  }
};

}  // namespace vr_example
