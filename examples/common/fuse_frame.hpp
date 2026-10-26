// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_frame.hpp
/// @brief A set of frames prepared on the GPU fused through the library's
///        `tsdf::Fuser`, into the grid `tsdf::create_fusion_grid` makes.
///
/// Header-only and compiled only into the executables that fuse: it includes
/// the `tsdf` tier and `sensor/utils`, which `vr_example_common` deliberately
/// does not link. What is left here is the hand-off from a
/// `sensor::DeviceFrame` to the fuser's inputs, which the tier cannot see,
/// and the lines the examples print about what a set did to the grid.

#include <cstdint>
#include <cstdio>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @brief How a prepared frame's colour is fused: through its own camera, the
///        coverage read off its high byte. Meaningful only when the frame
///        @ref vr::sensor::DeviceFrame::has_color.
inline vr::tsdf::ColorFrame device_color(const vr::sensor::DeviceFrame& frame) {
  vr::tsdf::ColorFrame color{};
  color.buffer = frame.color.get();
  color.cam = frame.color_camera;
  color.encoding = frame.color_encoding;
  color.coverage_in_alpha = true;
  return color;
}

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

/// @brief A set's prepared frames as the fuser takes them: each frame's input,
///        pointing at its colour in @ref colors. Moved, never copied, so the
///        pointers hold.
struct FuseInputs {
  std::vector<vr::tsdf::ColorFrame> colors;
  std::vector<vr::tsdf::FrameInput> frames;
};

/// @brief The fuser's inputs for @p frames, as `sensor::GpuFramePrep` hands
///        them out: nothing is uploaded, and depth and colour are fused with
///        their own cameras (@ref device_color). An empty entry is skipped.
inline FuseInputs fuse_inputs(
    const std::vector<std::optional<vr::sensor::DeviceFrame>>& frames) {
  FuseInputs in;
  in.colors.reserve(frames.size());  // the inputs point into it
  for (const std::optional<vr::sensor::DeviceFrame>& frame : frames) {
    if (!frame) continue;
    const vr::tsdf::ColorFrame* color = nullptr;
    if (frame->has_color()) {
      in.colors.push_back(device_color(*frame));
      color = &in.colors.back();
    }
    in.frames.push_back(
        {{vkc::StorageInput(*frame->depth), frame->depth_camera}, color});
  }
  return in;
}

/// @brief Fuse a set of frames already on the device, as
///        `sensor::GpuFramePrep` hands them out, through @p fuser
///        (@ref fuse_inputs): one frame is `{frame}`. What the set did to
///        the grid is printed (@ref print_fuse_report).
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
    const std::vector<std::optional<vr::sensor::DeviceFrame>>& frames,
    float max_weight, vr::tsdf::IntegrationMode mode,
    vkc::StageMetrics* metrics) {
  const FuseInputs in = fuse_inputs(frames);
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
    vr::sensor::GpuFramePrep& prep, const vr::sensor::RgbdFrame& frame,
    std::optional<vr::sensor::DeviceFrame>& keyframe, float max_weight,
    vr::tsdf::IntegrationMode mode, vkc::StageMetrics* metrics) {
  VKC_ASSIGN(vr::sensor::DeviceFrame prepared, prep.prepare(frame, metrics));
  VKC_TRY(fuse_set(fuser, grid, {prepared}, max_weight, mode, metrics));
  keyframe = std::move(prepared);
  return {};
}

}  // namespace vr_example
