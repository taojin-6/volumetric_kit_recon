// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/rig_report.hpp
/// @brief What `rig_viewer` reports: its Rig panel, the line it prints every
///        120 frames and at the end, the atlas copies' device time and the
///        rates it prints at the end, and `render.texture_stats`, the share of
///        the mesh each camera textured.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include <imgui.h>

#include "stage_table.hpp"
#include "viewer_panels.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"

namespace fuse_viewer {

namespace rsensor = volumetric_kit::recon::sensor;
namespace rtex = volumetric_kit::recon::texture;

/// Why a mesh went out without an atlas. Such a mesh draws in fused vertex
/// colour until the next remesh, so a run that often publishes one flickers
/// between the cameras' texture and the fused colour; the Rig panel counts
/// each.
enum Untextured : std::size_t {
  kEmptyMesh,      ///< The extract had no triangles, so nothing to texture.
  kNoColour,       ///< No camera of the set carried colour gfx can copy.
  kTextureOff,     ///< Texturing switched off in the View panel.
  kTextureFailed,  ///< The texture pass refused or failed (said on stderr).
  kUntexturedReasons,
};
inline constexpr const char* kUntexturedNames[kUntexturedReasons] = {
    "empty extract", "no colour", "texturing off", "texture failed"};

/// What the rig side reports beside the renderer's metrics, sampled on the
/// fuse thread (which owns the array) and copied out by the render thread
/// every frame, so it holds only what changes.
struct RigPanel {
  std::uint64_t sets_fused = 0;
  std::uint64_t frames_fused = 0;
  std::size_t cameras_textured = 0;  ///< In the newest textured set.
  std::size_t cameras_held = 0;      ///< Of those, from a held frame.
  /// Over the run: views the texture pass was given from a held frame, and
  /// remeshes it was given fewer cameras than the rig has -- each one a
  /// flicker, where the missing cameras' triangles fell to fused colour. Both
  /// count views given, not triangles taken: a held view whose depth no
  /// longer agrees with the mesh takes none, which render.texture_stats shows.
  std::uint64_t held_views = 0;
  std::uint64_t short_remeshes = 0;  ///< See @ref held_views.
  std::array<std::uint64_t, kUntexturedReasons> untextured{};
  rsensor::SensorArrayStats stats;
  std::size_t vertices = 0;
  std::size_t triangles = 0;
  std::uint64_t mesh_version = 0;
  float map_load_factor = 0.0f;  ///< Negative: the read failed.
  std::int32_t map_blocks = 0;
  double fuse_ms = 0.0;    ///< The newest set's prep, allocate and integrate.
  double remesh_ms = 0.0;  ///< The newest remesh's extract and texture.
  vkc::MemoryStats recon_memory;
  bool silent = false;  ///< No set within the fuse thread's silence limit.
  /// Meshes the window committed per second, over the last second: how
  /// often what is drawn changes, the viewer's live rate. Filled on the
  /// render thread, as the mesh counts are.
  double mesh_rate = 0.0;
};

/// @brief The Rig panel.
/// @param serials      Each camera's, in the array's order.
/// @param silence_s    The fuse thread's silence limit, for the warning.
inline void draw_rig_panel(const RigPanel& panel,
                           const std::vector<std::string>& serials,
                           long long silence_s) {
  if (!ImGui::Begin("Rig")) {
    ImGui::End();
    return;
  }
  ImGui::Text("sets     %llu fused (%llu frames), %llu missing a camera",
              static_cast<unsigned long long>(panel.sets_fused),
              static_cast<unsigned long long>(panel.frames_fused),
              static_cast<unsigned long long>(panel.stats.incomplete));
  ImGui::Text("frames   %llu in no set",
              static_cast<unsigned long long>(panel.stats.unmatched));
  if (panel.silent) {
    ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.2f, 1.0f),
                       "no set from the rig in %lld s", silence_s);
  }
  ImGui::Text("fuse     %.2f ms/set", panel.fuse_ms);
  ImGui::Text("remesh   %.2f ms, the newest", panel.remesh_ms);
  ImGui::Text("mesh     %.1f updates/s", panel.mesh_rate);
  ImGui::Text("textured from %zu camera%s, %zu held", panel.cameras_textured,
              panel.cameras_textured == 1 ? "" : "s", panel.cameras_held);
  ImGui::Text("  so far  %llu held views, %llu remeshes short of a camera",
              static_cast<unsigned long long>(panel.held_views),
              static_cast<unsigned long long>(panel.short_remeshes));
  for (std::size_t r = 0; r < kUntexturedReasons; ++r) {
    if (panel.untextured[r] == 0) continue;
    ImGui::Text("  untextured meshes, %s: %llu", kUntexturedNames[r],
                static_cast<unsigned long long>(panel.untextured[r]));
  }
  ImGui::Separator();
  for (std::size_t i = 0; i < panel.stats.sensors.size(); ++i) {
    const rsensor::SensorStats& st = panel.stats.sensors[i];
    ImGui::Text("%s  %llu in, %llu dropped, %llu failed",
                i < serials.size() ? serials[i].c_str() : "?",
                static_cast<unsigned long long>(st.received),
                static_cast<unsigned long long>(st.dropped),
                static_cast<unsigned long long>(st.failed));
  }
  ImGui::Separator();
  ImGui::Text("mesh v%llu  %zu vertices / %zu triangles",
              static_cast<unsigned long long>(panel.mesh_version),
              panel.vertices, panel.triangles);
  draw_map_gauge(panel.map_load_factor, panel.map_blocks);
  draw_recon_memory(panel.recon_memory);
  ImGui::End();
}

/// @brief The line printed every 120 drawn frames: the fusion and remesh
///        cost, the mesh, how it was textured, and the atlas copies.
inline void print_rig_progress(int drawn, const RigPanel& panel,
                               std::uint64_t atlas_copies) {
  std::printf(
      "frame %d: %llu sets fused (%.1f ms/set, remesh %.1f ms), mesh "
      "v%llu (%zu triangles, %.1f updates/s), textured from %zu "
      "camera%s (%zu held; %llu held views, %llu remeshes short so far), "
      "%llu atlas copies",
      drawn, static_cast<unsigned long long>(panel.sets_fused), panel.fuse_ms,
      panel.remesh_ms, static_cast<unsigned long long>(panel.mesh_version),
      panel.triangles, panel.mesh_rate, panel.cameras_textured,
      panel.cameras_textured == 1 ? "" : "s", panel.cameras_held,
      static_cast<unsigned long long>(panel.held_views),
      static_cast<unsigned long long>(panel.short_remeshes),
      static_cast<unsigned long long>(atlas_copies));
  for (std::size_t r = 0; r < kUntexturedReasons; ++r) {
    if (panel.untextured[r] != 0) {
      std::printf(", %llu untextured (%s)",
                  static_cast<unsigned long long>(panel.untextured[r]),
                  kUntexturedNames[r]);
    }
  }
  std::printf("\n");
}

/// @brief The line printed when the window closes.
/// @param sets_per_s    The sets fused a second (@ref CountRate).
/// @param meshes_per_s  The meshes drawn a second while the rig fused
///                      (@ref CountRate).
inline void print_rig_summary(int drawn, const RigPanel& panel,
                              std::uint64_t atlas_copies, double sets_per_s,
                              double meshes_per_s) {
  std::printf(
      "rig_viewer: drew %d frames; %llu sets (%llu frames) fused, %.2f "
      "sets/s; mesh v%llu with %zu triangles, %.2f updates/s; %llu atlas "
      "copies\n",
      drawn, static_cast<unsigned long long>(panel.sets_fused),
      static_cast<unsigned long long>(panel.frames_fused), sets_per_s,
      static_cast<unsigned long long>(panel.mesh_version), panel.triangles,
      meshes_per_s, static_cast<unsigned long long>(atlas_copies));
}

/// @brief A running count's rate from the first time it is seen above zero
///        to the last time it rose: neither the wait before its first event
///        nor an idle stretch after its last dilutes it.
class CountRate {
 public:
  using Clock = std::chrono::steady_clock;

  /// @brief The count is @p count at @p now; a count that has not risen
  ///        since the last call changes nothing.
  void at(std::uint64_t count, Clock::time_point now) {
    if (count == 0) return;
    if (first_count_ == 0) {
      first_count_ = count;
      first_ = now;
    }
    if (count > last_count_) {
      last_count_ = count;
      last_ = now;
    }
  }

  /// @return The rise a second from the first count to the last; 0 until
  ///         it has risen.
  double per_second() const {
    const double seconds =
        std::chrono::duration<double>(last_ - first_).count();
    return seconds > 0.0
               ? static_cast<double>(last_count_ - first_count_) / seconds
               : 0.0;
  }

 private:
  std::uint64_t first_count_ = 0;
  std::uint64_t last_count_ = 0;
  Clock::time_point first_{};
  Clock::time_point last_{};
};

/// The gfx profiler scope rig_viewer records its atlas copy under, and
/// @ref AtlasCopyTimes reads back.
inline constexpr const char* kAtlasCopyRow = "atlas copy";

/// @brief The device time of the atlas copies the window made, the newest
///        `keep`, read from the gfx profiler's snapshots.
///
/// A snapshot lags its frame: the profiler publishes a frame's figures when
/// its in-flight slot comes round again (`Profiler::begin_frame`), and the
/// scope runs every frame, whether it copies or not. So each slot remembers
/// whether the frame it carried copied, and only that frame's row counts.
///
/// @code
/// fuse_viewer::AtlasCopyTimes copies(frames_in_flight, keep);
/// // each frame, once the frame loop has begun it:
/// copies.begun(frame.slot, profiler.metrics().sections);
/// // ... a copy, or none, under profiler.gpu_scope(cmd, kAtlasCopyRow) ...
/// if (copied && submitted) copies.copied(frame.slot);
/// // at the end:
/// std::fputs(copies.report().c_str(), stdout);
/// @endcode
class AtlasCopyTimes {
 public:
  /// @param frames_in_flight  The frame loop's slots.
  /// @param keep              The copies kept, the newest; 0 keeps every
  ///                          copy.
  AtlasCopyTimes(std::uint32_t frames_in_flight, std::size_t keep)
      : copied_(frames_in_flight, false), keep_(keep) {}

  /// @brief A frame has begun in @p slot: @p published, the profiler's
  ///        snapshot of the frame the slot carried before, counts when that
  ///        frame copied, once.
  void begun(std::uint32_t slot, const std::vector<vkc::StageRow>& published) {
    if (slot >= copied_.size() || !copied_[slot]) return;
    copied_[slot] = false;
    for (const vkc::StageRow& row : published) {
      if (std::strcmp(row.name, kAtlasCopyRow) != 0) continue;
      // gfx's profiler says a measured row through has_gpu alone.
      if (row.device == vkc::DeviceTime::Measured || row.has_gpu) {
        if (keep_ != 0 && device_ms_.size() == keep_) {
          device_ms_.pop_front();
          dropped_ = true;
        }
        device_ms_.push_back(row.gpu_ms);
      } else {
        ++untimed_;
      }
      return;
    }
  }

  /// @brief The frame in @p slot copied an atlas, and was submitted.
  void copied(std::uint32_t slot) {
    if (slot < copied_.size()) copied_[slot] = true;
  }

  /// @return One line: the copies' device milliseconds, mean / p50 / p95 /
  ///         max over those timed, or n/a when none was; and the copies the
  ///         queue did not time.
  std::string report() const {
    char line[256];
    if (device_ms_.empty()) {
      std::snprintf(line, sizeof line,
                    "atlas copy, device ms: n/a; %llu untimed\n",
                    static_cast<unsigned long long>(untimed_));
      return line;
    }
    std::snprintf(
        line, sizeof line,
        "atlas copy, device ms (mean / p50 / p95 / max) over %s%zu timed: "
        "%s; %llu untimed\n",
        dropped_ ? "the newest " : "", device_ms_.size(),
        vr_example::format_stats(
            std::vector<double>(device_ms_.begin(), device_ms_.end()))
            .c_str(),
        static_cast<unsigned long long>(untimed_));
    return line;
  }

 private:
  std::vector<bool> copied_;  // per slot: its frame copied
  std::size_t keep_ = 0;
  std::deque<double> device_ms_;
  std::uint64_t untimed_ = 0;
  bool dropped_ = false;
};

/// The triangles each camera textured, and those none did.
struct TextureShares {
  std::size_t triangles = 0;
  std::size_t untextured = 0;
  std::vector<std::size_t> per_camera;  ///< In the layout's tile order.
};

/// @brief Count, per camera, the triangles of an unshared mesh it textured:
///        those whose uv0 (a triangle's three share one) fall in its tile.
///        A negative uv0 is textured by none.
inline TextureShares count_texture_shares(
    const std::vector<rmesh::Vertex>& vertices,
    const rtex::AtlasLayout& layout) {
  TextureShares out;
  out.per_camera.assign(layout.tiles.size(), 0);
  out.triangles = vertices.size() / 3;
  for (std::size_t t = 0; t < out.triangles; ++t) {
    const auto& uv = vertices[3 * t].uv0;
    if (!(uv.x >= 0.0f)) {
      ++out.untextured;
      continue;
    }
    const float px = uv.x * static_cast<float>(layout.width);
    const float py = uv.y * static_cast<float>(layout.height);
    for (std::size_t c = 0; c < layout.tiles.size(); ++c) {
      const rtex::AtlasTile& tile = layout.tiles[c];
      if (px >= tile.x && px < tile.x + tile.width && py >= tile.y &&
          py < tile.y + tile.height) {
        ++out.per_camera[c];
        break;
      }
    }
  }
  return out;
}

/// @brief Print @p shares as percentages of the mesh, marking the cameras
///        textured from a held frame.
/// @param held       Per camera, whether its view was held.
/// @param occlusion  The texture pass's threshold (metres).
/// @param dynamic    Whether fusion clears free space.
/// @param read_ms    What reading the mesh back cost.
inline void print_texture_shares(const TextureShares& shares,
                                 const std::vector<bool>& held, float occlusion,
                                 bool dynamic, double read_ms) {
  const double n =
      shares.triangles > 0 ? static_cast<double>(shares.triangles) : 1.0;
  std::printf("texture stats: %zu triangles, %.1f%% untextured;",
              shares.triangles,
              100.0 * static_cast<double>(shares.untextured) / n);
  for (std::size_t c = 0; c < shares.per_camera.size(); ++c) {
    std::printf(" camera %zu %.1f%%%s", c,
                100.0 * static_cast<double>(shares.per_camera[c]) / n,
                c < held.size() && held[c] ? " (held)" : "");
  }
  std::printf(" (occlusion %.1f cm, %s; read back in %.0f ms)\n",
              100.0 * occlusion, dynamic ? "dynamic" : "static", read_ms);
}

}  // namespace fuse_viewer
