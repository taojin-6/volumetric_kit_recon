// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_frame.hpp
/// @brief Fusion the way every example does it: the volume it fuses into,
///        and a set of frames prepared on the GPU fused into it -- the
///        truncation band allocated (the map grown when it overflows), then
///        depth and, where a frame carries it, colour integrated.
///
/// Header-only and compiled only into the executables that fuse: it includes
/// the `tsdf` tier and `sensor/utils`, which `vr_example_common` deliberately
/// does not link. Three copies of this loop had already drifted apart in what
/// they printed, timed and guarded, and the encoding hand-off in
/// @ref vr_example::device_color is the kind of line a fourth copy drops --
/// with no error, since a `ColorFrame` left defaulted *declares* canonical
/// rather than saying nothing.

#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "grid_layout.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @brief The volume every example fuses into: 8x8x8-voxel blocks hashed into
///        buckets of eight, carrying the three attributes @ref fuse_set
///        writes -- `tsdf`, `weight` and `color`.
///
/// The layout is @ref example_grid_params, which every example shares; the
/// resolution, the band and the table's starting size are theirs to choose.
/// The table grows on overflow (@ref allocate_band), so @p num_buckets sets
/// where it starts, not what it holds.
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

/// @brief Allocate the truncation band for @p frames into @p grid, each round
///        one submit for them all, growing the map (preserving the per-voxel
///        data already fused) if it overflows.
///
/// Grows only for a *capacity* limit. A frame of mostly new blocks -- the
/// first, or a fast pan -- makes many of them at once, and the kernel's bucket
/// spin-lock gives up after a bounded number of retries, so a round can hand
/// back a residue of pure lock failures over a table that is nowhere near
/// full. Doubling on that is expensive and unbounded: at the
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
/// @param frames   Each camera's depth on the device, with the camera that
///                 drives its unprojection and range gate.
/// @param metrics  Optional stage rows (`"allocate"`, `"resize"`); null
///                 measures nothing.
/// @return OK once every surface block is allocated; @ref
///         vkc::Status::Code::OutOfMemory if the map cannot grow further or
///         kept overflowing after five rounds; or the tier's own error.
inline vkc::Status allocate_band(
    vr::volume::VoxelBlockGrid& grid,
    const std::vector<vr::volume::DepthInput>& frames,
    vkc::StageMetrics* metrics) {
  constexpr int kRounds = 5;
  for (int round = 0; round < kRounds; ++round) {
    vr::volume::AllocFailures failures;
    VKC_ASSIGN(const std::uint32_t failed,
               grid.map().allocate_from_depth(frames, &failures, metrics));
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
      return vkc::Status::out_of_memory(
          "allocate_band: map cannot grow further without overflowing the "
          "block index");
    }
    // Report the occupancy alongside the reason: it is a host copy of the
    // heap counter (not the O(total slots) diagnostics scan), and it is what
    // says whether this grow was inevitable or premature. A capture-scale
    // consumer should poll it and grow on a threshold instead of waiting for
    // the failure -- linear probing degrades sharply past ~0.7, so growing at
    // the cliff means every insert before it ran at its slowest.
    const vkc::Result<float> load = grid.map().load_factor();
    std::printf(
        "  map overflow at %.3f load (%u fails: %u chain, %u heap, %u table) "
        "-> resize to %lld buckets\n",
        load.ok() ? load.value() : -1.0f, failed, failures.chain, failures.heap,
        failures.table, static_cast<long long>(grown));
    {
      vkc::StageScope resize_span(metrics, "resize");
      VKC_TRY(grid.resize(static_cast<std::int32_t>(grown)));
    }
  }
  return vkc::Status::out_of_memory(
      "allocate_band: allocation kept overflowing after " +
      std::to_string(kRounds) + " rounds");
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

/// @brief Fuse a set of frames already on the device, as
///        `sensor::GpuFramePrep` hands them out: nothing is uploaded, and
///        depth and colour are fused with their own cameras. Every frame's
///        band is allocated in one call (@ref allocate_band), then every frame
///        fused in one, so a set costs a few submits rather than a few a
///        frame. An empty entry is skipped; one frame is `{frame}`.
/// @param grid        The volume to fuse into.
/// @param integrator  The integrator.
/// @param frames      The prepared frames.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @param mode        `Classic` (the default) keeps free space ahead of a
///                    surface; `Dynamic` clears it, so a surface that moves
///                    away leaves no ghost.
/// @return OK, or the first error of the two steps.
inline vkc::Status fuse_set(
    vr::volume::VoxelBlockGrid& grid, vr::tsdf::TsdfIntegrator& integrator,
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
  // Each input's depth half, sliced off.
  const std::vector<vr::volume::DepthInput> depths(inputs.begin(),
                                                   inputs.end());
  VKC_TRY(allocate_band(grid, depths, metrics));
  return integrator.integrate(grid, inputs, max_weight, mode, metrics);
}

/// @brief Prepare and fuse one frame, retaining it as the next keyframe only
///        after both steps succeed.
///
/// Keeps the previous keyframe's buffers alive throughout the attempt, so a
/// preparation or fusion failure leaves its pixels and cameras available for
/// texturing the final mesh. The prep allocates separate outputs while the
/// previous frame holds its own; this does not roll back changes to the grid.
/// @param grid        The volume to fuse into.
/// @param integrator  The integrator; `IntegrationMode::Classic`.
/// @param prep        The frame preparation pass.
/// @param frame       The captured frame to prepare and fuse.
/// @param keyframe    The newest successfully fused frame; unchanged on error.
/// @param max_weight  The running-average cap (`TsdfIntegrator::integrate`).
/// @param metrics     Optional stage rows; null measures nothing.
/// @return OK with @p keyframe replaced, or the preparation or fusion error.
inline vkc::Status fuse_keyframe(
    vr::volume::VoxelBlockGrid& grid, vr::tsdf::TsdfIntegrator& integrator,
    vr::sensor::GpuFramePrep& prep, const vr::sensor::RgbdFrame& frame,
    std::optional<vr::sensor::DeviceFrame>& keyframe, float max_weight,
    vkc::StageMetrics* metrics) {
  VKC_ASSIGN(vr::sensor::DeviceFrame prepared, prep.prepare(frame, metrics));
  VKC_TRY(fuse_set(grid, integrator, {prepared}, max_weight, metrics));
  keyframe = std::move(prepared);
  return {};
}

}  // namespace vr_example
