// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_metrics.hpp
/// @brief The live viewers' host-side figures: a count's rate over the run
///        and over the last second, the atlas copies' device time read from
///        gfx's profiler snapshots, the share of a mesh each camera textured,
///        the frame each camera textures a remesh from, and the end table's
///        title.
///
/// It includes no gfx or ImGui header, and must not: tests/ builds it on
/// every leg, without the viewer, so the figures `rig_viewer` prints are
/// checked where gfx is not fetched.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "stage_table.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/grid/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace fuse_viewer {

namespace vkc = volumetric_kit::core;
namespace rmesh = volumetric_kit::grid::mesh;
namespace rsensor = volumetric_kit::sensor;
namespace rtex = volumetric_kit::recon::texture;

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

/// @brief A running count's rate over the newest window of a second or
///        more: its rise over the window, divided by the window's length,
///        held until the next window closes.
///
/// `rig_viewer` reads the mesh generation it draws through it each frame,
/// for the Rig panel's updates a second.
class WindowRate {
 public:
  using Clock = std::chrono::steady_clock;

  /// @brief The first window opens at @p start, at a count of 0.
  explicit WindowRate(Clock::time_point start) : start_(start) {}

  /// @brief The count is @p count at @p now. A second or more after the
  ///        window opened, the window closes with the rate over it, and
  ///        the next opens at @p now and @p count.
  /// @return The rate over the newest closed window; 0 until one closes.
  double at(std::uint64_t count, Clock::time_point now) {
    const double seconds = std::chrono::duration<double>(now - start_).count();
    if (seconds >= 1.0) {
      rate_ = static_cast<double>(count - count_) / seconds;
      count_ = count;
      start_ = now;
    }
    return rate_;
  }

 private:
  Clock::time_point start_;
  std::uint64_t count_ = 0;
  double rate_ = 0.0;
};

/// The gfx profiler scope the live viewers record their atlas copy under,
/// and @ref AtlasCopyTimes reads back.
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
        vgrid_example::format_stats(
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
/// @param dynamic    Whether fusion is Dynamic: it resets where a camera
///                   sees change.
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

/// A camera's newest frame with colour, and the set it came in (sets count
/// from 1).
struct NewestFrame {
  rsensor::DeviceFrame frame;
  std::uint64_t set = 0;
};

/// The frame a remesh textures one camera from, and whether it is held: from
/// an earlier set.
struct TextureSource {
  const rsensor::DeviceFrame* frame = nullptr;  ///< Null: none.
  bool held = false;
};

/// @brief Per camera, the frame a remesh of set @p set textures it from: its
///        newest, when that is this set's, or an earlier one taken at most
///        @p hold_ns before @p set_ns on the host's clock
///        (`fusion.hold_ms`); else none.
/// @param newest   Per camera, its newest frame with colour, if any.
/// @param set      The set being remeshed.
/// @param set_ns   Its timestamp.
/// @param hold_ns  How old a held frame may be; 0 holds only a frame taken
///                 at @p set_ns.
/// @return One per entry of @p newest, pointing into it.
inline std::vector<TextureSource> texture_sources(
    const std::vector<std::optional<NewestFrame>>& newest, std::uint64_t set,
    std::uint64_t set_ns, std::uint64_t hold_ns) {
  std::vector<TextureSource> out(newest.size());
  for (std::size_t c = 0; c < newest.size(); ++c) {
    if (!newest[c]) continue;
    const bool held = newest[c]->set != set;
    const std::uint64_t taken = newest[c]->frame.timestamp_ns;
    if (!held || (set_ns >= taken && set_ns - taken <= hold_ns)) {
      out[c] = {&newest[c]->frame, held};
    }
  }
  return out;
}

/// @return The title of a live viewer's per-set table: `per set`, or, when
///         the log dropped its oldest sets, `per set (the newest K of N)`.
/// @param kept     The sets the table covers.
/// @param dropped  The older sets it let go.
inline std::string per_set_title(std::size_t kept, std::uint64_t dropped) {
  return dropped == 0 ? std::string("per set")
                      : "per set (the newest " + std::to_string(kept) + " of " +
                            std::to_string(kept + dropped) + ")";
}

}  // namespace fuse_viewer
