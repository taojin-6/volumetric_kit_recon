// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/stage_metrics.hpp
/// @brief The interop seam for timings: recon's @ref vr::StageMetrics rows
///        mapped into the renderer's @ref vg::FrameMetrics shape.
///
/// This file used to *implement* the collector, because recon had none. It now
/// implements only the mapping, which is the whole of what a neutral app owes
/// this seam.
///
/// The two row types are structurally identical and neither is the other's:
/// recon cannot include from a sibling it does not depend on, and a shared
/// package for four fields was weighed and judged too thin. So each library
/// declares in its own vocabulary and the one place that knows both -- here --
/// converts with a loop. Same shape as `DeviceRequirements` at the device seam
/// and `MarchingCubesConfig::extra_vertex_usage` at the buffer seam; neither
/// release couples to the other.
///
/// **recon's spans are wall clock unless a `GpuTimer` ran.** Every recon
/// dispatch goes through `Device::submit_single_time`, which blocks on a fence,
/// so a host span around one covers record, submit, the stall and device
/// execution together. `StageRow::has_gpu` is what says whether the device half
/// was measured separately, and it maps straight onto the renderer's field of
/// the same name -- so the overlay's GPU column is blank for a stage recon
/// timed only on the host, and populated for one it timed with a
/// `vr::GpuTimer`. Neither side has to know which is which.

#include <vector>

#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"

namespace fuse_viewer {

namespace vg = volumetric_kit::gfx;
namespace vr = volumetric_kit::recon;

/// @brief Map recon's stage rows into the renderer's section rows.
///
/// A field-for-field copy; the two structs agree on all four. Labels cross as
/// pointers on both sides, so the strings must outlive every read of the result
/// -- true of the string literals every recon tier reports with, and **not**
/// true of a `vr::GpuTimer`'s labels, which the timer owns and frees on its
/// next `reset()`. Consume those before reusing the timer.
///
/// @param metrics  The rows a fuse iteration collected.
/// @return The same rows in the renderer's shape.
inline std::vector<vg::FrameMetrics::Section> to_sections(
    const vr::StageMetrics& metrics) {
  std::vector<vg::FrameMetrics::Section> sections;
  sections.reserve(metrics.rows().size());
  for (const vr::StageRow& row : metrics.rows()) {
    sections.push_back({row.name, row.cpu_ms, row.gpu_ms, row.has_gpu});
  }
  return sections;
}

}  // namespace fuse_viewer
