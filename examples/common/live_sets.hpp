// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/live_sets.hpp
/// @brief The live examples' sensor array, whichever cameras it holds: a
///        camera model's part of a camera's line, the report of the array's
///        counters and a rig's skew over its sets (`sensor::RigSets`), its
///        clock and its first set's age. Header-only, on the array alone.

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <string>

#include "volumetric_kit/sensor/array/rig_sets.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace sensor = volumetric_kit::sensor;

/// @brief A camera's counters at the end of a run, the part of its line after
///        its id: @ref sensor_counters, or a driver's own.
using CameraCounters = std::string (*)(const sensor::IRgbdSensor&);

/// @return A camera model's part of a camera's line: " WxH <lens> @ fx=..
///         fy=.. cx=.. cy=..", @p lens naming its lens model; empty, none.
inline std::string camera_summary(const sensor::camera::CameraModel& model,
                                  const std::string& lens) {
  char text[160];
  std::snprintf(text, sizeof text,
                " %ux%u %s%s@ fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                model.size.width, model.size.height, lens.c_str(),
                lens.empty() ? "" : " ", model.intrinsics.fx,
                model.intrinsics.fy, model.intrinsics.cx, model.intrinsics.cy);
  return text;
}

/// @return Counters as a camera's line shows them: "<received> received,
///         <delivered> delivered, <dropped> dropped, <failed> failed".
inline std::string counters_text(const sensor::SensorStats& s) {
  char text[160];
  std::snprintf(text, sizeof text,
                "%llu received, %llu delivered, %llu dropped, %llu failed",
                static_cast<unsigned long long>(s.received),
                static_cast<unsigned long long>(s.delivered),
                static_cast<unsigned long long>(s.dropped),
                static_cast<unsigned long long>(s.failed));
  return text;
}

/// @return Any camera's counters (`IRgbdSensor::stats`), as
///         @ref counters_text.
inline std::string sensor_counters(const sensor::IRgbdSensor& camera) {
  return counters_text(camera.stats());
}

/// @return @p array's sets, then a line per camera, "  <id>: " and its
///         @p counters, and for a rig each secondary's mean and worst skew to
///         the primary over @p sets; each line ends in a newline.
/// @param sets  The run's sets; null, for a run that polls the array itself
///              (rig_viewer's), reports no skew.
inline std::string array_report(sensor::SensorArray& array,
                                const sensor::RigSets* sets,
                                CameraCounters counters) {
  const sensor::SensorArrayStats st = array.stats();
  char line[256];
  std::snprintf(line, sizeof line,
                "%llu sets, %llu missing a camera, %llu frames unmatched\n",
                static_cast<unsigned long long>(st.sets),
                static_cast<unsigned long long>(st.incomplete),
                static_cast<unsigned long long>(st.unmatched));
  std::string report = line;
  for (std::size_t i = 0; i < array.size(); ++i) {
    const sensor::IRgbdSensor& camera = array.sensor(i);
    report += "  " + camera.info().id + ": " + counters(camera);
    const sensor::RigSets::Skew lag =
        sets != nullptr ? sets->skew(i) : sensor::RigSets::Skew{};
    if (lag.frames > 0) {
      std::snprintf(line, sizeof line,
                    "; skew to the primary %+.2f ms mean, %.2f ms worst",
                    lag.sum_ms / static_cast<double>(lag.frames), lag.worst_ms);
      report += line;
    }
    report += "\n";
  }
  return report;
}

/// @return Whether every sensor of @p array stamps its frames on the host's
///         clock, so a set's time is on the system clock: a rig's always
///         are, a single Orbbec camera's only with `source.host_clock`, a
///         RealSense camera's always.
inline bool on_host_clock(sensor::SensorArray& array) {
  for (std::size_t i = 0; i < array.size(); ++i) {
    if (array.sensor(i).info().clock != sensor::ClockDomain::Host) {
      return false;
    }
  }
  return true;
}

/// @brief Print a single camera's first set's age against the host's clock,
///        once @p sets has handed out @p set, its first; nothing for a rig or
///        a later set. On the host's clock the frame is milliseconds old; a
///        camera's own clock is where it was last set.
inline void print_first_frame_age(sensor::SensorArray& array,
                                  const sensor::RigSets& sets,
                                  const sensor::FrameSet& set) {
  if (sets.sets() != 1 || array.size() != 1) return;
  const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  std::printf("  first frame %.1f ms before the host's clock\n",
              (static_cast<double>(host.count()) -
               static_cast<double>(set.timestamp_ns)) /
                  1e6);
}

}  // namespace vr_example
