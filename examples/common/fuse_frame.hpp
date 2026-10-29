// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_frame.hpp
/// @brief A set of frames prepared on the GPU fused through the library's
///        `tsdf::Fuser`, into the grid `tsdf::create_fusion_grid` makes.
///
/// Header-only and compiled only into the executables that fuse: it includes
/// the `sender` tier, which `vr_example_common` deliberately does not link.
/// What is left here is the blocking calls the non-sending examples fuse
/// with, the fuser's inputs taken from `sender::fuse_inputs`, and the lines
/// the examples print about what a set did to the grid.

#include <cstdint>
#include <cstdio>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/sender/fuse_inputs.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @brief Print what a set did to the grid, when it did anything worth a line:
///        grew, was refused a grow, or left blocks out.
inline void print_fuse_report(const vr::tsdf::FuseReport& report) {
  if (report.grows > 0) {
    std::printf("  map grew %d time(s): %d -> %d buckets, %.3f load\n",
                report.grows, report.from_buckets, report.to_buckets,
                double(report.load_factor));
  }
  if (!report.growth_error.ok()) {
    std::printf("  map grow failed: %s\n",
                report.growth_error.message().c_str());
  }
  if (report.dropped > 0) {
    const vr::volume::AllocFailures& f = report.failures;
    std::printf(
        "  %u allocation requests left out (%u lock, %u chain, %u heap, %u "
        "table) after "
        "%d grow(s)\n",
        report.dropped, f.lock, f.chain, f.heap, f.table, report.grows);
  }
}

/// @return The integration mode `fusion.dynamic` selects: `Dynamic` when
///         @p dynamic, else `Classic` (@ref fuse_set).
inline vr::tsdf::IntegrationMode integration_mode(bool dynamic) {
  return dynamic ? vr::tsdf::IntegrationMode::Dynamic
                 : vr::tsdf::IntegrationMode::Classic;
}

/// @brief Fuse a set of frames already on the device, as
///        `sensor::GpuFramePrep` hands them out, through @p fuser
///        (`sender::fuse_inputs`): one frame is `{frame}`. What the set did
///        to the grid is printed (@ref print_fuse_report).
/// @param fuser       The fuser.
/// @param grid        The volume to fuse into.
/// @param frames      The prepared frames.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param mode        `Classic` keeps free space ahead of a surface;
///                    `Dynamic` clears it, so a surface that moves away
///                    leaves no ghost.
/// @param metrics     Optional stage rows; null measures nothing.
/// @return OK, or `Fuser::fuse`'s error.
inline vkc::Status fuse_set(
    vr::tsdf::Fuser& fuser, vr::volume::VoxelBlockGrid& grid,
    const std::vector<std::optional<sensor::DeviceFrame>>& frames,
    float max_weight, vr::tsdf::IntegrationMode mode,
    vkc::StageMetrics* metrics) {
  const vr::sender::FuseInputs in = vr::sender::fuse_inputs(frames);
  VKC_ASSIGN(const vr::tsdf::FuseReport report,
             fuser.fuse(grid, in.frames, max_weight, mode, metrics));
  print_fuse_report(report);
  return {};
}

/// @return Whether `fusion.free_after`'s pass is due after @p sets fused
///         sets: every @p free_after of them; never at 0.
inline bool free_due(std::uint64_t sets, int free_after) {
  return free_after > 0 && sets % static_cast<std::uint64_t>(free_after) == 0;
}

/// @brief `fusion.free_after`'s pass, run after each fused set: every
///        @p free_after sets, free the blocks of @p grid that neither an
///        allocation nor an observed voxel has stamped for more than
///        @p free_after ticks of its clock, one a fused set
///        (`VoxelBlockGrid::free_stale_blocks`), such as those a surface
///        left under Dynamic integration. 0 never frees.
///
/// Housekeeping: a failed pass is printed to stderr as
/// "<program>: free blocks: <why>", and the caller fuses on.
/// @param grid        The volume the sets fuse into.
/// @param sets        The sets fused so far, this one included.
/// @param free_after  `fusion.free_after`, nonnegative.
/// @param program     The name the failure is printed under.
/// @param metrics     Optional stage rows: the pass's `"block stamps"`.
inline void free_stale_blocks_every(vr::volume::VoxelBlockGrid& grid,
                                    std::uint64_t sets, int free_after,
                                    const char* program,
                                    vkc::StageMetrics* metrics) {
  if (!free_due(sets, free_after)) return;
  const vkc::Result<std::uint32_t> freed =
      grid.free_stale_blocks(static_cast<std::uint32_t>(free_after), metrics);
  if (!freed) {
    std::fprintf(stderr, "%s: free blocks: %s\n", program,
                 freed.status().message().c_str());
  }
}

/// @brief Prepare and fuse one frame, retaining it as the next keyframe only
///        after both steps succeed.
///
/// Keeps the previous keyframe's buffers alive throughout the attempt, so a
/// preparation or fusion failure leaves its pixels and cameras available for
/// texturing the final mesh. The prep allocates separate outputs while the
/// previous frame holds its own; this does not roll back changes to the grid.
/// @param fuser       The fuser.
/// @param grid        The volume to fuse into.
/// @param prep        The frame preparation pass.
/// @param frame       The captured frame to prepare and fuse.
/// @param keyframe    The newest successfully fused frame; unchanged on error.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param mode        As @ref fuse_set's.
/// @param metrics     Optional stage rows; null measures nothing.
/// @return OK with @p keyframe replaced, or the preparation or fusion error.
inline vkc::Status fuse_keyframe(
    vr::tsdf::Fuser& fuser, vr::volume::VoxelBlockGrid& grid,
    sensor::GpuFramePrep& prep, const sensor::RgbdFrame& frame,
    std::optional<sensor::DeviceFrame>& keyframe, float max_weight,
    vr::tsdf::IntegrationMode mode, vkc::StageMetrics* metrics) {
  VKC_ASSIGN(sensor::DeviceFrame prepared, prep.prepare(frame, metrics));
  VKC_TRY(fuse_set(fuser, grid, {prepared}, max_weight, mode, metrics));
  keyframe = std::move(prepared);
  return {};
}

}  // namespace vr_example
