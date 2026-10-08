// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_frame.hpp
/// @brief The volume every example fuses into, and a set of frames prepared
///        on the GPU fused into it through the library's `tsdf::Fuser`.
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

#include "grid_layout.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @brief The volume every example fuses into: 8x8x8-voxel blocks hashed into
///        buckets of eight, carrying the three attributes @ref fuse_set
///        writes -- `tsdf`, `weight` and `color`.
///
/// The layout is @ref example_grid_params, which every example shares; the
/// resolution, the band and the table's starting size are theirs to choose.
/// The fuser grows the table, so @p num_buckets sets where it starts, not
/// what it holds.
///
/// @param device       The recon device.
/// @param allocator    Its allocator.
/// @param voxel_size   Voxel edge (metres).
/// @param trunc_dist   Truncation distance (metres).
/// @param num_buckets  Initial hash-bucket count; `8 * num_buckets` must fit
///                     an `int32_t`.
/// @return The grid, or @ref vr::volume::VoxelBlockGrid::create's error.
inline vkc::Result<vr::volume::VoxelBlockGrid> create_fusion_grid(
    vkc::Device& device, vkc::Allocator& allocator, float voxel_size,
    float trunc_dist, std::int32_t num_buckets = 16384) {
  const vr::volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                             {"weight", sizeof(float)},
                                             {"color", sizeof(std::uint32_t)}};
  return vr::volume::VoxelBlockGrid::create(
      device, allocator,
      example_grid_params(voxel_size, trunc_dist, num_buckets), attrs, 3);
}

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
  using vr::volume::GrowthOutcome;
  for (const vr::volume::GrowthEvent* event : {&report.ahead, &report.grow}) {
    const char* why = event == &report.ahead ? "ahead" : "for a capacity limit";
    switch (event->outcome) {
      case GrowthOutcome::Grew:
        std::printf("  map grew %s at %.3f load: %d -> %d buckets\n", why,
                    double(event->load_factor), event->from_buckets,
                    event->to_buckets);
        break;
      case GrowthOutcome::DeclinedForMemory:
        std::printf(
            "  map grow to %d buckets declined: needs %.0f MiB, %.0f MiB "
            "free\n",
            event->to_buckets, double(event->needed_bytes) / (1 << 20),
            double(event->headroom_bytes) / (1 << 20));
        break;
      case GrowthOutcome::ResizeFailed:
        std::printf("  map grow to %d buckets failed: %s\n", event->to_buckets,
                    event->error.message().c_str());
        break;
      default:
        break;
    }
  }
  if (report.allocation_refused) {
    std::printf("  map at %.3f load: no new blocks this set\n",
                double(report.load_factor));
  }
  if (report.dropped > 0) {
    const vr::volume::AllocFailures& f = report.failures;
    std::printf(
        "  %u blocks left out (%u lock, %u chain, %u heap, %u table) after "
        "%d grow(s)\n",
        report.dropped, f.lock, f.chain, f.heap, f.table, report.grows);
  }
}

/// @brief Fuse a set of frames already on the device, as
///        `sensor::GpuFramePrep` hands them out, through @p fuser: nothing is
///        uploaded, and depth and colour are fused with their own cameras. An
///        empty entry is skipped; one frame is `{frame}`. What the set did to
///        the grid is printed (@ref print_fuse_report).
/// @param fuser       The fuser.
/// @param grid        The volume to fuse into.
/// @param frames      The prepared frames.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @param mode        `Classic` (the default) keeps free space ahead of a
///                    surface; `Dynamic` clears it, so a surface that moves
///                    away leaves no ghost.
/// @return OK, or `Fuser::fuse`'s error.
inline vkc::Status fuse_set(
    vr::tsdf::Fuser& fuser, vr::volume::VoxelBlockGrid& grid,
    const std::vector<std::optional<vr::sensor::DeviceFrame>>& frames,
    float max_weight, vkc::StageMetrics* metrics,
    vr::tsdf::IntegrationMode mode = vr::tsdf::IntegrationMode::Classic) {
  std::vector<vr::tsdf::ColorFrame> colors;
  std::vector<vr::tsdf::FrameInput> inputs;
  colors.reserve(frames.size());  // the inputs point into it
  for (const std::optional<vr::sensor::DeviceFrame>& frame : frames) {
    if (!frame) continue;
    const vr::tsdf::ColorFrame* color = nullptr;
    if (frame->has_color()) {
      colors.push_back(device_color(*frame));
      color = &colors.back();
    }
    inputs.push_back(
        {{vkc::StorageInput(*frame->depth), frame->depth_camera}, color});
  }
  VKC_ASSIGN(const vr::tsdf::FuseReport report,
             fuser.fuse(grid, inputs, max_weight, mode, metrics));
  print_fuse_report(report);
  return {};
}

/// @brief Prepare and fuse one frame, retaining it as the next keyframe only
///        after both steps succeed.
///
/// Keeps the previous keyframe's buffers alive throughout the attempt, so a
/// preparation or fusion failure leaves its pixels and cameras available for
/// texturing the final mesh. The prep allocates separate outputs while the
/// previous frame holds its own; this does not roll back changes to the grid.
/// @param fuser       The fuser; `IntegrationMode::Classic`.
/// @param grid        The volume to fuse into.
/// @param prep        The frame preparation pass.
/// @param frame       The captured frame to prepare and fuse.
/// @param keyframe    The newest successfully fused frame; unchanged on error.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @return OK with @p keyframe replaced, or the preparation or fusion error.
inline vkc::Status fuse_keyframe(
    vr::tsdf::Fuser& fuser, vr::volume::VoxelBlockGrid& grid,
    vr::sensor::GpuFramePrep& prep, const vr::sensor::RgbdFrame& frame,
    std::optional<vr::sensor::DeviceFrame>& keyframe, float max_weight,
    vkc::StageMetrics* metrics) {
  VKC_ASSIGN(vr::sensor::DeviceFrame prepared, prep.prepare(frame, metrics));
  VKC_TRY(fuse_set(fuser, grid, {prepared}, max_weight, metrics));
  keyframe = std::move(prepared);
  return {};
}

}  // namespace vr_example
