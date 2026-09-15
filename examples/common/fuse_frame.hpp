// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_frame.hpp
/// @brief One frame's fusion, the way every example does it: allocate the
///        truncation band (growing the map when it overflows), then integrate
///        depth and -- when the frame carries it -- colour.
///
/// Header-only and compiled only into the executables that fuse: it includes
/// the `tsdf` tier, which `vr_example_common` deliberately does not link.
/// Three copies of this loop had already drifted apart in what they printed,
/// timed and guarded, and the encoding hand-off below is the kind of line a
/// fourth copy drops -- with no error, since a `ColorFrame` left defaulted
/// *declares* canonical rather than saying nothing.

#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief Allocate the truncation band for @p frame into @p grid, growing the
///        map (preserving the per-voxel data already fused) if it overflows.
///
/// Grows only for a *capacity* limit. Depth allocation is the most contended
/// entry point in the map -- adjacent pixels dilate into the same block, and
/// the kernel's bucket spin-lock gives up after a bounded number of retries --
/// so a round can hand back a residue of pure lock failures over a table that
/// is nowhere near full. Doubling on that is expensive and unbounded: at the
/// examples' defaults each attribute array goes 768 MiB -> 1536 MiB, and
/// `resize` builds the grown buffers beside the old ones, so the transient
/// peak is ~2.3 GiB -- for pressure that does not exist. Such a round is
/// reported and retried instead; the next dispatch sees less contention
/// because the blocks that did land are now present.
///
/// The grow is its own `"resize"` row rather than folded into `"allocate"` or
/// left untimed: it is by far the most expensive thing an overflowing frame
/// does, and charging it to the allocate row would sink that stage's device
/// share on exactly the frames where the host cost is not the kernel at all.
/// The tier fills `"allocate"` itself, every round under the one name, so
/// there is no scope around the loop here.
///
/// @param grid     The volume to allocate into.
/// @param frame    The posed depth frame; its depth camera drives the
///                 unprojection and the range gate.
/// @param metrics  Optional stage rows (`"allocate"`, `"resize"`); null
///                 measures nothing.
/// @return OK once every surface block is allocated; @ref
///         vr::Status::Code::OutOfMemory if the map cannot grow further or
///         kept overflowing after five rounds; or the tier's own error.
inline vr::Status allocate_band(vr::volume::VoxelBlockGrid& grid,
                                const vr::sensor::CapturedFrame& frame,
                                vr::StageMetrics* metrics) {
  constexpr int kRounds = 5;
  for (int round = 0; round < kRounds; ++round) {
    vr::volume::AllocFailures failures;
    VR_ASSIGN(const std::uint32_t failed,
              grid.map().allocate_from_depth(frame.depth, frame.depth_camera,
                                             &failures, metrics));
    if (failed == 0) {
      return {};
    }
    if (!failures.capacity_limited()) {
      std::printf(
          "  %u allocations lost bucket-lock races (no capacity limit) -> "
          "retrying without growing\n",
          failed);
      continue;
    }
    // Double in int64 and bail before the block index (bucket_size * buckets)
    // would overflow int32, so a growth that can no longer fit reports
    // cleanly instead of tripping the signed-overflow UB.
    const std::int64_t grown =
        static_cast<std::int64_t>(grid.grid().num_buckets) * 2;
    if (grown * grid.grid().bucket_size >
        std::numeric_limits<std::int32_t>::max()) {
      return vr::Status::out_of_memory(
          "allocate_band: map cannot grow further without overflowing the "
          "block index");
    }
    // Report the occupancy alongside the reason: it is a 4-byte read of the
    // heap counter (not the O(total slots) diagnostics scan), and it is what
    // says whether this grow was inevitable or premature. A capture-scale
    // consumer should poll it and grow on a threshold instead of waiting for
    // the failure -- linear probing degrades sharply past ~0.7, so growing at
    // the cliff means every insert before it ran at its slowest.
    const vr::Result<float> load = grid.map().load_factor();
    std::printf(
        "  map overflow at %.3f load (%u fails: %u chain, %u heap, %u table) "
        "-> resize to %lld buckets\n",
        load.ok() ? load.value() : -1.0f, failed, failures.chain, failures.heap,
        failures.table, static_cast<long long>(grown));
    {
      vr::StageScope resize_span(metrics, "resize");
      VR_TRY(grid.resize(static_cast<std::int32_t>(grown)));
    }
  }
  return vr::Status::out_of_memory(
      "allocate_band: allocation kept overflowing after " +
      std::to_string(kRounds) + " rounds");
}

/// @brief Fuse one captured frame: @ref allocate_band, then integrate its
///        depth -- and its colour, if it has any -- into @p grid.
///
/// The frame's encoding declaration is carried across rather than left
/// defaulted: `ColorFrame::encoding` defaults to canonical, so leaving it out
/// does not mean "unspecified" -- it *declares* canonical, and a source that
/// forgot to convert would be fused through the wrong curve instead of
/// refused. A frame without colour is fused depth-only; passing an empty
/// colour frame would be refused by the tier.
///
/// @param grid        The volume to fuse into.
/// @param integrator  The integrator; `IntegrationMode::Classic`.
/// @param frame       The posed frame, as the capture handed it out.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @return OK, or the first error of the two steps.
inline vr::Status fuse_frame(vr::volume::VoxelBlockGrid& grid,
                             vr::tsdf::TsdfIntegrator& integrator,
                             const vr::sensor::CapturedFrame& frame,
                             float max_weight, vr::StageMetrics* metrics) {
  VR_TRY(allocate_band(grid, frame, metrics));
  const vr::tsdf::ColorFrame color{frame.color, frame.color_camera,
                                   frame.color_encoding};
  return integrator.integrate(grid, frame.depth, frame.depth_camera, max_weight,
                              vr::tsdf::IntegrationMode::Classic,
                              frame.has_color() ? &color : nullptr, metrics);
}

}  // namespace vr_example
