// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_panels.hpp
/// @brief The Dear ImGui rows both viewers report recon's side in -- the
///        map's occupancy and recon's device memory -- and `fuse_viewer`'s
///        Reconstruction panel. Each builds into the ImGui frame the caller is
///        driving (it calls neither NewFrame nor Render), as gfx's
///        `draw_metrics_panel` does.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <imgui.h>

#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace fuse_viewer {

namespace vkc = volumetric_kit::core;
namespace rmesh = volumetric_kit::recon::mesh;
namespace vol = volumetric_kit::recon::volume;

/// @return @p bytes in MiB (1024^2), the unit gfx's metrics panel prints.
inline double to_mebibytes(std::uint64_t bytes) {
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

/// @brief A filled bar carrying its own ceiling, which turns colour at
///        @p warn_at of @p capacity.
///
/// A threshold a reader has to know about is one they will miss: a scan can
/// sit deep in the band where allocation has stopped with every other figure
/// looking healthy, as one did on an iPad for half a scan behind an
/// `errors 0` banner. The colour changes only when the reader is meant to
/// act.
inline void gauge(const char* label, double value, double capacity,
                  double warn_at, const char* overlay) {
  const double fraction =
      capacity > 0.0 ? std::min(value / capacity, 1.0) : 0.0;
  const bool hot = fraction >= warn_at;
  if (hot) {
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                          ImVec4(0.72f, 0.39f, 0.18f, 1.0f));
  }
  ImGui::Text("%s", label);
  ImGui::SameLine(110.0f);
  ImGui::ProgressBar(static_cast<float>(fraction), ImVec2(-1.0f, 0.0f),
                     overlay);
  if (hot) {
    ImGui::PopStyleColor();
  }
}

/// @brief The map's occupancy against its block heap, the figure that says a
///        scan has stopped taking in new geometry, warning at the map's own
///        grow threshold (`VoxelHashMap::kGrowThreshold`).
/// @param load_factor  `VoxelHashMap::load_factor`, a host copy of the heap
///                     counter cheap enough to read every frame; negative
///                     when that read failed, drawn full and `unavailable`:
///                     an unknown occupancy is not a low one.
/// @param blocks       The heap's size.
inline void draw_map_gauge(float load_factor, std::int32_t blocks) {
  const double warn = vol::VoxelHashMap::kGrowThreshold;
  char overlay[80];
  if (load_factor < 0.0f) {
    std::snprintf(overlay, sizeof(overlay), "unavailable, %d blocks", blocks);
    gauge("map", 1.0, 1.0, warn, overlay);
  } else {
    std::snprintf(overlay, sizeof(overlay), "%.1f%% of %d blocks%s",
                  100.0 * load_factor, blocks,
                  load_factor >= warn ? "  -- grow now" : "");
    gauge("map", load_factor, 1.0, warn, overlay);
  }
}

/// @brief recon's device memory: its own allocator's share of each heap
///        (`reserved_bytes`) against the heap's budget. On a shared device
///        each library allocates separately, so this is recon's footprint,
///        not the process total.
inline void draw_recon_memory(const vkc::MemoryStats& memory) {
  for (std::uint32_t heap = 0; heap < memory.heap_count; ++heap) {
    const vkc::HeapStats& stats = memory.heaps[heap];
    if (stats.reserved_bytes == 0) continue;
    char overlay[64];
    std::snprintf(overlay, sizeof(overlay), "%.0f / %.0f MiB",
                  to_mebibytes(stats.reserved_bytes),
                  to_mebibytes(stats.budget_bytes));
    char label[32];
    std::snprintf(label, sizeof(label), "recon heap %u", heap);
    gauge(label, static_cast<double>(stats.reserved_bytes),
          static_cast<double>(stats.budget_bytes), 0.9, overlay);
  }
}

/// What `fuse_viewer`'s reconstruction side is holding, shown beside the
/// renderer's frame metrics, whose one memory pair is gfx's allocator's.
struct ReconstructionPanel {
  std::size_t fused_frames = 0;
  std::size_t total_frames = 0;
  std::size_t vertices = 0;
  std::size_t triangles = 0;
  std::uint64_t mesh_version = 0;
  std::int32_t map_buckets = 0;
  std::int32_t map_blocks = 0;
  float map_load_factor = 0.0f;  ///< Negative: the read failed.
  double fuse_ms = 0.0;
  std::uint64_t preloaded_bytes = 0;
  vkc::MemoryStats recon_memory;
  rmesh::ExtractTimings extract;
};

/// @brief `fuse_viewer`'s Reconstruction panel.
inline void draw_reconstruction_panel(const ReconstructionPanel& panel) {
  if (!ImGui::Begin("Reconstruction")) {
    ImGui::End();
    return;
  }
  ImGui::Text("fused    %zu / %zu frames", panel.fused_frames,
              panel.total_frames);
  ImGui::Text("fuse     %.2f ms/frame", panel.fuse_ms);
  ImGui::Separator();
  ImGui::Text("mesh v%llu",
              static_cast<unsigned long long>(panel.mesh_version));
  ImGui::Text("  %zu vertices / %zu triangles", panel.vertices,
              panel.triangles);
  // The two output buffers' occupancy, each against its own capacity:
  // MarchingCubesConfig::share_vertices decouples them, and whichever fills
  // first forces the refit. Past 0.9 the next growth step is one -- the
  // arenas are grow-only, so a full one costs a second dispatch, the count
  // below going to 2, which the ..dispatch row cannot show as it sums both.
  // The MiB is the whole ring, every slot.
  {
    char overlay[64];
    std::snprintf(overlay, sizeof(overlay), "%u / %u verts",
                  panel.extract.emitted_vertices,
                  panel.extract.vertex_capacity);
    gauge("  arena", panel.extract.emitted_vertices,
          panel.extract.vertex_capacity, 0.9, overlay);
    std::snprintf(overlay, sizeof(overlay), "%u / %u tris",
                  panel.extract.emitted_triangles,
                  panel.extract.triangle_capacity);
    gauge("  indices", panel.extract.emitted_triangles,
          panel.extract.triangle_capacity, 0.9, overlay);
  }
  ImGui::Text("  output %.0f MiB (ring), %u dispatch%s",
              to_mebibytes(panel.extract.arena_bytes), panel.extract.dispatches,
              panel.extract.dispatches == 1 ? "" : "es");
  draw_map_gauge(panel.map_load_factor, panel.map_blocks);
  // num_blocks is bucket_size * num_buckets, so the bucket count is what a
  // resize doubles.
  ImGui::Text("  %d buckets", panel.map_buckets);
  ImGui::Separator();
  draw_recon_memory(panel.recon_memory);
  if (panel.preloaded_bytes != 0) {
    ImGui::Text("frame cache   %.0f MiB (host)",
                to_mebibytes(panel.preloaded_bytes));
  }
  ImGui::End();
}

}  // namespace fuse_viewer
