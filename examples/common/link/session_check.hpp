// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/link/session_check.hpp
/// @brief What `tools/session_check` checks of a session's frames as it
///        reads them: the frame contract (`testing::RgbdContract`); each
///        frame's pattern index read on the device
///        (`testing::PatternReader`), its picture's against its depth's, as
///        a pattern source (`test_pattern`, `pattern_depth`) carries one
///        index in both; its times; and, at the end, the sensor's counters,
///        lost spans and link counts.
///
/// @code
/// VKC_ASSIGN(auto check, link::SessionCheck::create(device, allocator,
///                                                   phone->info()));
/// VKC_TRY(phone->start());
/// while (!phone->exhausted()) {
///   VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, phone->poll());
///   if (frame) VKC_TRY(check->take(*frame));
/// }
/// const link::SessionReport report = check->report(*phone);
/// std::fputs(link::format(report).c_str(), stdout);  // ends with the breach
/// @endcode
///
/// @warning Not thread-safe: the thread that polls the frames uses it.

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "stage_table.hpp"  // grid's nearest_rank
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/testing/pattern_reader.hpp"
#include "volumetric_kit/sensor/testing/rgbd_contract.hpp"

namespace vr_example::link {

/// @brief A span of lost frames this long or more starts a new window in the
///        tracking lane (owner, 2026-11-03): a step of the map hidden in the
///        gap then never joins two windows.
inline constexpr std::uint64_t kLossWindowSplitNs = 100'000'000;

/// @brief What a @ref SessionCheck found.
struct SessionReport {
  sensor::SensorInfo info;    ///< The sensor's, as it opened.
  sensor::SensorStats stats;  ///< Its counters at the end.
  std::uint64_t frames = 0;   ///< Frames checked.
  /// Frames whose picture names its depth's index: its own picture.
  std::uint64_t own_picture = 0;
  /// Frames whose picture names another index than its depth's.
  std::uint64_t wrong = 0;
  /// Frames whose picture names no index, with a colour camera.
  std::uint64_t no_picture_index = 0;
  std::uint64_t no_depth_index = 0;  ///< Frames whose depth names none.
  std::uint64_t depth_spans = 0;     ///< `skipped()`: spans,
  std::uint64_t depth_lost = 0;      ///< and the records in them.
  std::uint64_t color_spans = 0;     ///< `skipped_color()`: spans,
  std::uint64_t color_lost = 0;      ///< and the records in them.
  /// Depth spans with a time on each side @ref kLossWindowSplitNs or more
  /// apart: the lane's window splits.
  std::uint64_t window_splits = 0;
  RgbdStreamSensor::LinkStats link;  ///< All 0 for a file.
  std::uint64_t first_ns = 0;        ///< The first frame's capture time.
  std::uint64_t last_ns = 0;         ///< The last frame's.
  double seconds = 0.0;  ///< Wall time from the first frame taken to the last.
  /// Arrival minus capture in ms, p50, p95 and max (nearest rank): live, on
  /// the host's clock, so across hosts it holds their clocks' offset too.
  std::optional<std::array<double, 3>> latency_ms;
  /// The frame contract's first breach (`testing::RgbdContract`); empty when
  /// every rule holds.
  std::string breach;

  /// @return Whether the contract holds and no frame shows another's
  ///         picture.
  bool ok() const noexcept { return breach.empty() && wrong == 0; }
};

/// @brief Checks a session's frames one at a time (above).
class SessionCheck {
 public:
  /// @brief A check of the frames of a sensor that opened as @p info, read
  ///        on @p device, where the frames' pictures are.
  /// @return The check; `testing::PatternReader::create`'s errors. @p device
  ///         and @p allocator are borrowed: they must outlive the check.
  static vkc::Result<std::unique_ptr<SessionCheck>> create(
      const vkc::Device& device, vkc::Allocator& allocator,
      sensor::SensorInfo info) {
    VKC_ASSIGN(std::unique_ptr<sensor::testing::PatternReader> reader,
               sensor::testing::PatternReader::create(device, allocator));
    return std::unique_ptr<SessionCheck>(
        new SessionCheck(std::move(reader), std::move(info)));
  }

  SessionCheck(const SessionCheck&) = delete;
  SessionCheck& operator=(const SessionCheck&) = delete;
  SessionCheck(SessionCheck&&) = delete;
  SessionCheck& operator=(SessionCheck&&) = delete;

  /// @brief Check @p frame, the next one taken: its contract, its indices
  ///        and its times. Its picture's ownership is updated on the device,
  ///        as `testing::PatternReader::read` does.
  /// @return OK; `testing::PatternReader::read`'s errors.
  vkc::Status take(sensor::RgbdFrame& frame) {
    contract_.take(frame);
    VKC_ASSIGN(const sensor::testing::FrameIndex index, reader_->read(frame));
    ++frames_;
    if (!index.depth) ++no_depth_index_;
    if (info_.color) {
      if (!index.picture) {
        ++no_picture_index_;
      } else if (index.depth) {
        ++(*index.picture == *index.depth ? own_picture_ : wrong_);
      }
    }
    if (info_.clock == sensor::ClockDomain::Host && frame.arrival_ns != 0 &&
        frame.timestamp_ns != 0) {
      latency_ms_.push_back(static_cast<double>(static_cast<std::int64_t>(
                                frame.arrival_ns - frame.timestamp_ns)) /
                            1e6);
    }
    const auto now = std::chrono::steady_clock::now();
    if (frames_ == 1) {
      first_ns_ = frame.timestamp_ns;
      first_wall_ = now;
    }
    last_ns_ = frame.timestamp_ns;
    last_wall_ = now;
    return {};
  }

  /// @return What the frames taken and @p sensor's counters, spans and link
  ///         say now.
  SessionReport report(const RgbdStreamSensor& sensor) const {
    SessionReport r;
    r.info = info_;
    r.stats = sensor.stats();
    r.frames = frames_;
    r.own_picture = own_picture_;
    r.wrong = wrong_;
    r.no_picture_index = no_picture_index_;
    r.no_depth_index = no_depth_index_;
    for (const RgbdStreamSensor::Skipped& s : sensor.skipped()) {
      ++r.depth_spans;
      r.depth_lost += s.count;
      if (s.after_ns != 0 && s.before_ns != 0 &&
          s.before_ns - s.after_ns >= kLossWindowSplitNs) {
        ++r.window_splits;
      }
    }
    for (const RgbdStreamSensor::Skipped& s : sensor.skipped_color()) {
      ++r.color_spans;
      r.color_lost += s.count;
    }
    r.link = sensor.link_stats();
    r.first_ns = first_ns_;
    r.last_ns = last_ns_;
    r.seconds = std::chrono::duration<double>(last_wall_ - first_wall_).count();
    if (!latency_ms_.empty()) {
      std::vector<double> sorted = latency_ms_;
      std::sort(sorted.begin(), sorted.end());
      r.latency_ms = {vgrid_example::nearest_rank(sorted, 50),
                      vgrid_example::nearest_rank(sorted, 95), sorted.back()};
    }
    r.breach = contract_.breach(r.stats);
    return r;
  }

 private:
  SessionCheck(std::unique_ptr<sensor::testing::PatternReader> reader,
               sensor::SensorInfo info)
      : reader_(std::move(reader)), contract_(info), info_(std::move(info)) {}

  std::unique_ptr<sensor::testing::PatternReader> reader_;
  sensor::testing::RgbdContract contract_;
  sensor::SensorInfo info_;
  std::uint64_t frames_ = 0;
  std::uint64_t own_picture_ = 0;
  std::uint64_t wrong_ = 0;
  std::uint64_t no_picture_index_ = 0;
  std::uint64_t no_depth_index_ = 0;
  std::vector<double> latency_ms_;
  std::uint64_t first_ns_ = 0;
  std::uint64_t last_ns_ = 0;
  std::chrono::steady_clock::time_point first_wall_;
  std::chrono::steady_clock::time_point last_wall_;
};

/// @brief A host and port as typed for a check: `host:port`.
struct Address {
  std::string host;        ///< As typed; the listener or dial checks it.
  std::uint16_t port = 0;  ///< 1 to 65535.
};

/// @return @p text as a count: decimal digits alone, at most 9; empty
///         otherwise.
inline std::optional<long> count_of(const char* text) {
  const std::size_t n = std::strlen(text);
  if (n == 0 || n > 9) return std::nullopt;
  for (std::size_t i = 0; i < n; ++i) {
    if (text[i] < '0' || text[i] > '9') return std::nullopt;
  }
  return std::strtol(text, nullptr, 10);
}

/// @return @p text, `host:port` with a port of 1 to 65535 in decimal
///         digits, split at its last colon; empty otherwise. TODO: core
///         0.44.0's `net::parse_host_port`, at recon's next core re-pin.
inline std::optional<Address> parse_address(const std::string& text) {
  const std::size_t colon = text.rfind(':');
  if (colon == std::string::npos || colon == 0) return std::nullopt;
  const std::optional<long> port = count_of(text.c_str() + colon + 1);
  if (!port || *port < 1 || *port > 65535) return std::nullopt;
  return Address{text.substr(0, colon), static_cast<std::uint16_t>(*port)};
}

/// @return @p r as session_check prints it, a line a topic: the frames, the
///         pattern, the losses, the link (live), the times, and last
///         `"rgbd_breach: none"` or the breach.
inline std::string format(const SessionReport& r) {
  const auto u = [](std::uint64_t v) { return std::to_string(v); };
  const auto f = [](const char* form, double v) {
    char text[32];
    std::snprintf(text, sizeof text, form, v);
    return std::string(text);
  };
  const auto size = [&](const std::optional<sensor::camera::CameraModel>& m) {
    return m ? u(m->size.width) + "x" + u(m->size.height) : std::string("none");
  };
  std::string out = "sensor: " + r.info.id + " (" + r.info.model + "), depth " +
                    size(r.info.depth) + ", colour " + size(r.info.color) +
                    ", " + u(r.info.fps) + " fps, " + to_string(r.info.clock) +
                    " clock\n";
  out += "frames: " + u(r.frames) + " of " + u(r.stats.received) +
         " depth records (delivered " + u(r.stats.delivered) + ", failed " +
         u(r.stats.failed) + ", dropped " + u(r.stats.dropped) + ")\n";
  out += "pattern: ";
  if (r.info.color) {
    out += u(r.own_picture) + " own picture, " + u(r.wrong) + " wrong, " +
           u(r.no_picture_index) + " without a picture index, ";
  }
  out += u(r.no_depth_index) + " without a depth index\n";
  out += "lost: depth " + u(r.depth_lost) + ", colour " + u(r.color_lost) +
         "; spans: depth " + u(r.depth_spans) + ", colour " + u(r.color_spans) +
         ", of 100 ms or more " + u(r.window_splits) + "\n";
  if (r.link.connections != 0) {
    out += "link: connections " + u(r.link.connections) + ", refused " +
           u(r.link.refused) + ", failures " + u(r.link.failures) +
           ", repeats " + u(r.link.repeated) + ", recorded " +
           u(r.link.recorded) + "\n";
  }
  const double capture = r.last_ns > r.first_ns
                             ? static_cast<double>(r.last_ns - r.first_ns) / 1e9
                             : 0.0;
  out += "times: " + f("%.3f", capture) + " s of capture in " +
         f("%.3f", r.seconds) + " s";
  if (r.seconds > 0.0 && r.frames > 1) {
    out += ", " + f("%.2f", static_cast<double>(r.frames - 1) / r.seconds) +
           " frames/s";
  }
  if (r.latency_ms) {
    out += "; arrival - capture p50 " + f("%.2f", (*r.latency_ms)[0]) +
           " ms, p95 " + f("%.2f", (*r.latency_ms)[1]) + " ms, max " +
           f("%.2f", (*r.latency_ms)[2]) + " ms";
  }
  out +=
      "\nrgbd_breach: " + (r.breach.empty() ? std::string("none") : r.breach) +
      "\n";
  return out;
}

}  // namespace vr_example::link
