// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/set_log.hpp
/// @brief A sender's figures per set -- each set's stage rows, the device's
///        counters across it and its latency -- and the run's table of them,
///        which fuse_orbbec, codec_replica and rig_viewer print at the end.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "stage_table.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/codec/codec_params.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @return The system clock now, in nanoseconds since its epoch: the clock a
///         frame's arrival stamp (`sensor::RgbdFrame::arrival_ns`) and a
///         rig's capture time are on.
inline std::uint64_t system_now_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

/// @brief A sender's figures per set, and the run's table of them.
///
/// @code
/// vr_example::SetLog log([&device] { return device.counters(); });
/// for (;;) {
///   vr_example::SetLog::Set& s = log.begin();       // counters start here
///   VKC_ASSIGN(const auto set, sets.next(&s.rows));  // "wait: cameras"
///   s.arrived(set.arrival_ns, set.timestamp_ns);     // "set" starts here
///   // ... the set's stages, timed into s.rows ...
///   s.published();  // counters, latency and "set" end
/// }
/// std::fputs(log.table("per set").c_str(), stdout);
/// @endcode
class SetLog {
 public:
  /// Reads the device's counters (`Device::counters`).
  using Counters = std::function<vkc::DeviceCounters()>;

  /// @brief One set's figures, filled while it runs.
  class Set {
   public:
    /// @brief Start a set: its counters count from here.
    /// @param counters  As the log's.
    explicit Set(Counters counters)
        : counters_(std::move(counters)),
          begin_(counters_ ? counters_() : vkc::DeviceCounters{}) {}

    /// Its stages. `"set"` spans it from @ref arrived to @ref published, its
    /// wait the waits of the rows timed in that span.
    vkc::StageMetrics rows;
    /// Its coded frame's type, as the encoder finished it; none for a set
    /// not coded.
    std::optional<vr::codec::FrameType> type;
    /// For a set not coded, the name of the sets it is tabled with -- a
    /// viewer's sets that published a mesh apart from those that only
    /// fused -- with string-literal lifetime; null reads "uncoded".
    const char* kind = nullptr;
    /// Its fusion was done again on the host (`tsdf::FuseReport::redone`).
    bool redone = false;
    /// Its frame's payload outgrew what its batch copied and took a tail.
    bool tail = false;

    /// @brief The set is in hand: @ref rows' `"set"` starts now.
    /// @param arrival_ns  When its last frame reached the host, on the
    ///                    system clock; 0 when unknown.
    /// @param capture_ns  Its trigger, on the system clock; 0 when its
    ///                    sensors stamp on a clock of their own.
    void arrived(std::uint64_t arrival_ns, std::uint64_t capture_ns) {
      arrival_ns_ = arrival_ns;
      capture_ns_ = capture_ns;
      handed_ = Clock::now();
      waited_before_ms_ = rows.total_wait_ms();
    }

    /// @brief Its frame is out, written and published -- or, for a set not
    ///        coded, its last stage is done: its counters, its latency and
    ///        `"set"` end here, so a diagnostic decode after it counts in
    ///        none. Once only; a later call does nothing.
    void published() {
      if (published_) return;
      published_ = true;
      end_ = counters_ ? counters_() : vkc::DeviceCounters{};
      published_ns_ = system_now_ns();
      // Busy plus wait is its wall time: the waits are its rows' since
      // arrived, as a scope over the span would count them.
      const double wall =
          std::chrono::duration<double, std::milli>(Clock::now() - handed_)
              .count();
      rows.add_wait("set",
                    std::min(wall, rows.total_wait_ms() - waited_before_ms_));
      rows.add_cpu("set", wall);
    }

    /// @return Whether @ref published ran.
    bool is_published() const noexcept { return published_; }

    /// @return The device's counters across the set, @ref published's less
    ///         the begin's.
    vkc::DeviceCounters counted() const noexcept {
      vkc::DeviceCounters d;
      d.submits = end_.submits - begin_.submits;
      d.host_waits = end_.host_waits - begin_.host_waits;
      d.blocked_ns = end_.blocked_ns - begin_.blocked_ns;
      d.staging_buffers = end_.staging_buffers - begin_.staging_buffers;
      d.dispatches = end_.dispatches - begin_.dispatches;
      d.barriers = end_.barriers - begin_.barriers;
      return d;
    }

    /// @return Arrival to published, in milliseconds; none when unknown.
    std::optional<double> arrival_ms() const noexcept {
      return since(arrival_ns_);
    }
    /// @return Capture to published, in milliseconds; none when unknown.
    std::optional<double> capture_ms() const noexcept {
      return since(capture_ns_);
    }

   private:
    using Clock = std::chrono::steady_clock;

    std::optional<double> since(std::uint64_t ns) const noexcept {
      if (ns == 0 || !published_ || published_ns_ < ns) return std::nullopt;
      return static_cast<double>(published_ns_ - ns) * 1e-6;
    }

    Counters counters_;
    vkc::DeviceCounters begin_;
    vkc::DeviceCounters end_;
    Clock::time_point handed_ = Clock::now();
    std::uint64_t arrival_ns_ = 0;
    std::uint64_t capture_ns_ = 0;
    std::uint64_t published_ns_ = 0;
    double waited_before_ms_ = 0.0;  // the rows' waits before arrived
    bool published_ = false;
  };

  /// @param counters  Reads the device's counters at each set's begin and
  ///                  publish; empty counts nothing.
  /// @param keep      The sets kept, the newest: a run with no end of its own,
  ///                  a viewer's, is bounded by it. 0 keeps every set.
  explicit SetLog(Counters counters, std::size_t keep = 0)
      : counters_(std::move(counters)), keep_(keep) {}

  /// @brief Start a set: its counters count from here. The reference holds
  ///        for as long as the log does, or, with a `keep`, until the set is
  ///        let go, `keep` sets later.
  Set& begin() {
    if (keep_ != 0 && sets_.size() == keep_) {
      sets_.pop_front();  // a deque: the other sets' references hold
      ++dropped_;
    }
    sets_.emplace_back(counters_);
    return sets_.back();
  }

  /// @return The sets kept, a set never published among them.
  const std::deque<Set>& sets() const noexcept { return sets_; }

  /// @return The sets let go for the `keep` bound, the oldest.
  std::uint64_t dropped() const noexcept { return dropped_; }

  /// @return The table of the sets kept and published (see the file's
  ///         notes): P-frame and I-frame sets, then the sets not coded, by
  ///         @ref Set::kind in the order first seen, each group apart. For
  ///         each row, the sets that ran it and the mean, p50, p95 and max of
  ///         its host busy, wait and device milliseconds over them (n/t or
  ///         n/a for device time some set did not measure, a dash for a
  ///         host-only row); per set, the submits, host waits, blocked
  ///         milliseconds, staging buffers, dispatches and barriers; the
  ///         latencies arrival -> published and capture -> published (n/a
  ///         where unknown); and the sets fused again and the payload tails.
  ///         Empty when no set was published.
  std::string table(const std::string& title) const {
    std::vector<const char*> names = {"P-frame", "I-frame"};
    for (const Set& s : sets_) {
      const char* name = group_of(s);
      if (std::none_of(names.begin(), names.end(), [name](const char* n) {
            return std::strcmp(n, name) == 0;
          })) {
        names.push_back(name);
      }
    }
    std::string out;
    for (const char* name : names) {
      std::vector<const Set*> sets;
      for (const Set& s : sets_) {
        if (s.is_published() && std::strcmp(group_of(s), name) == 0) {
          sets.push_back(&s);
        }
      }
      if (!sets.empty()) out += group_table(title, name, sets);
    }
    return out;
  }

 private:
  // A set's group: its frame's type, or, not coded, its kind.
  static const char* group_of(const Set& s) {
    if (s.type) {
      return *s.type == vr::codec::FrameType::kInter ? "P-frame" : "I-frame";
    }
    return s.kind != nullptr ? s.kind : "uncoded";
  }

  // Every row any of `sets` ran, each set's in its order: a row first seen in
  // a later set goes after the row before it there, past that row's
  // breakdowns, so a breakdown stays inside its stage and a stage after the
  // breakdowns of the one before it.
  static std::vector<const char*> merged_rows(
      const std::vector<const Set*>& sets) {
    const auto breakdown = [](const char* name) {
      return std::strncmp(name, vkc::StageMetrics::kBreakdownPrefix,
                          std::strlen(vkc::StageMetrics::kBreakdownPrefix)) ==
             0;
    };
    std::vector<const char*> rows;
    const auto at = [&rows](const char* name) {
      return std::find_if(rows.begin(), rows.end(), [name](const char* row) {
        return std::strcmp(row, name) == 0;
      });
    };
    for (const Set* s : sets) {
      const char* before = nullptr;
      for (const vkc::StageRow& row : s->rows.rows()) {
        if (at(row.name) == rows.end()) {
          auto place = before != nullptr ? at(before) + 1 : rows.begin();
          while (before != nullptr && place != rows.end() &&
                 breakdown(*place)) {
            ++place;
          }
          rows.insert(place, row.name);
        }
        before = row.name;
      }
    }
    return rows;
  }

  // Four cells of `what`, as wide as format_stats()'s.
  static std::string cells(const char* what) {
    char text[64];
    std::snprintf(text, sizeof text, "%8s %8s %8s %8s", what, what, what, what);
    return text;
  }

  static std::string group_table(const std::string& title, const char* name,
                                 const std::vector<const Set*>& sets) {
    std::string out;
    char line[512];
    std::snprintf(line, sizeof line,
                  "%s: %zu %s sets (ms; mean / p50 / p95 / max over the n "
                  "sets that ran a row)\n",
                  title.c_str(), sets.size(), name);
    out += line;
    std::snprintf(line, sizeof line, "  %-20s %5s  %-35s  %-35s  %s\n", "row",
                  "n", "host busy", "wait", "device");
    out += line;
    const std::vector<const char*> rows = merged_rows(sets);
    for (const char* row_name : rows) {
      std::vector<double> busy, wait, device;
      vkc::DeviceTime worst = vkc::DeviceTime::None;
      for (const Set* s : sets) {
        const vkc::StageRow* row = find_row(s->rows, row_name);
        if (row == nullptr) continue;
        busy.push_back(row->cpu_ms - row->wait_ms);
        wait.push_back(row->wait_ms);
        worst = std::max(worst, row->device);
        if (row->device == vkc::DeviceTime::Measured) {
          device.push_back(row->gpu_ms);
        }
      }
      std::string device_cells = cells("-");
      if (worst == vkc::DeviceTime::Measured) {
        device_cells = format_stats(device);
      } else if (worst == vkc::DeviceTime::NotTimed) {
        device_cells = cells("n/t");
      } else if (worst == vkc::DeviceTime::Unavailable) {
        device_cells = cells("n/a");
      }
      std::snprintf(line, sizeof line, "  %-20s %5zu  %s  %s  %s\n", row_name,
                    busy.size(), format_stats(busy).c_str(),
                    format_stats(wait).c_str(), device_cells.c_str());
      out += line;
    }
    // The device's counters per set.
    const struct {
      const char* name;
      double (*of)(const vkc::DeviceCounters&);
    } counts[] = {
        {"submits",
         [](const vkc::DeviceCounters& c) { return double(c.submits); }},
        {"host waits",
         [](const vkc::DeviceCounters& c) { return double(c.host_waits); }},
        {"blocked ms",
         [](const vkc::DeviceCounters& c) {
           return double(c.blocked_ns) * 1e-6;
         }},
        {"staging buffers",
         [](const vkc::DeviceCounters& c) {
           return double(c.staging_buffers);
         }},
        {"dispatches",
         [](const vkc::DeviceCounters& c) { return double(c.dispatches); }},
        {"barriers",
         [](const vkc::DeviceCounters& c) { return double(c.barriers); }},
    };
    out += "  per set (mean / p50 / p95 / max):\n";
    for (const auto& count : counts) {
      std::vector<double> values;
      for (const Set* s : sets) values.push_back(count.of(s->counted()));
      std::snprintf(line, sizeof line, "    %-18s %s\n", count.name,
                    format_stats(values).c_str());
      out += line;
    }
    // The latencies, where the stamps are known.
    const struct {
      const char* name;
      std::optional<double> (Set::*of)() const noexcept;
    } latencies[] = {{"arrival->published", &Set::arrival_ms},
                     {"capture->published", &Set::capture_ms}};
    for (const auto& latency : latencies) {
      std::vector<double> values;
      for (const Set* s : sets) {
        if (const std::optional<double> ms = (s->*latency.of)()) {
          values.push_back(*ms);
        }
      }
      std::snprintf(
          line, sizeof line, "    %-18s %s\n", latency.name,
          values.empty() ? cells("n/a").c_str() : format_stats(values).c_str());
      out += line;
    }
    std::size_t redone = 0;
    std::size_t tails = 0;
    for (const Set* s : sets) {
      redone += s->redone ? 1 : 0;
      tails += s->tail ? 1 : 0;
    }
    std::snprintf(line, sizeof line, "    fused again %zu, payload tails %zu\n",
                  redone, tails);
    out += line;
    return out;
  }

  Counters counters_;
  std::size_t keep_ = 0;  // 0: every set
  std::uint64_t dropped_ = 0;
  std::deque<Set> sets_;  // a deque: begin() keeps earlier references
};

}  // namespace vr_example
