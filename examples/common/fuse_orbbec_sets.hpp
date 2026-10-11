// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_orbbec_sets.hpp
/// @brief fuse_orbbec's cameras: opened and read as a sensor array, each set
///        waited for and cut to the run's length, and the report of the
///        array's counters and a rig's skew. Header-only, compiled only into
///        the examples built with the Orbbec driver.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "orbbec_config.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sensor.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @brief Print an opened camera: what it reports, its cameras' factory
///        models at the opened modes, its clock, and where @p calibration
///        puts it.
inline void print_orbbec_camera(
    const vr::sensor::OrbbecSensor& opened,
    const vr::camera::ArrayCalibration& calibration) {
  const vr::sensor::OrbbecDeviceInfo& info = opened.device_info();
  const vr::camera::CameraModel& color = *opened.info().color;
  const vr::camera::CameraModel& depth = *opened.info().depth;
  const vr::camera::SensorCalibration* posed =
      vr::camera::find_sensor(calibration, info.serial);
  const vr::camera::Mat4d pose =
      posed != nullptr ? posed->color_to_world : vr::camera::Mat4d(1.0);
  std::printf(
      "camera: %s %s, firmware %s, %s %s, sync %s, %s clock\n"
      "  colour %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f, at (%.3f, %.3f, "
      "%.3f) m\n"
      "  depth %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n",
      info.name.c_str(), info.serial.c_str(), info.firmware_version.c_str(),
      info.connection_type.c_str(), info.ip_address.c_str(),
      vr::sensor::to_string(info.sync_mode),
      vr::sensor::to_string(opened.info().clock), color.size.width,
      color.size.height, color.intrinsics.fx, color.intrinsics.fy,
      color.intrinsics.cx, color.intrinsics.cy, pose[3].x, pose[3].y, pose[3].z,
      depth.size.width, depth.size.height, depth.intrinsics.fx,
      depth.intrinsics.fy, depth.intrinsics.cx, depth.intrinsics.cy);
}

/// @brief Open fuse_orbbec's cameras, each printed: one, or every camera of
///        the `source.sync_config` file (`open_orbbec_sensors`), the colour
///        decoded onto
///        @p device. A single camera posed by @p calibration is the one it
///        names, or its only one. The depth gate is validated by the driver,
///        which names both values when it refuses one.
/// @return The cameras; `Status::Code::NotFound` when @p calibration poses
///         several and `source.serial` names none, or not the named one; or the
///         driver's error.
inline vkc::Result<std::vector<std::unique_ptr<vr::sensor::IRgbdSensor>>>
open_orbbec_cameras(const config::Config& opt,
                    const vr::camera::ArrayCalibration& calibration,
                    const vkc::Device& device, vkc::Allocator& allocator) {
  vr::sensor::OrbbecSensor::Options o;
  apply_orbbec_config(opt, device, allocator, o);
  std::vector<std::unique_ptr<vr::sensor::IRgbdSensor>> sensors;
  const auto t_open = std::chrono::steady_clock::now();
  if (!opt.source.sync_config.empty()) {
    o.apply_sync = opt.source.apply_sync;
    VKC_ASSIGN(sensors,
               vr::sensor::open_orbbec_sensors(opt.source.sync_config, o));
  } else {
    o.serial = opt.source.serial;
    const std::vector<vr::camera::SensorCalibration>& posed =
        calibration.sensors;
    if (!posed.empty()) {
      const vr::camera::SensorCalibration* sensor =
          !opt.source.serial.empty()
              ? vr::camera::find_sensor(calibration, opt.source.serial)
          : posed.size() == 1 ? &posed[0]
                              : nullptr;
      if (sensor == nullptr) {
        return vkc::Status::not_found(
            opt.source.calibration +
            (opt.source.serial.empty()
                 ? std::string(
                       " poses several cameras; name one with source.serial")
                 : " does not pose camera " + opt.source.serial));
      }
      o.serial = sensor->id;
    }
    o.sync_clock_to_host = opt.source.host_clock;
    VKC_ASSIGN(vr::sensor::OrbbecSensor opened,
               vr::sensor::OrbbecSensor::open(o));
    sensors.push_back(
        std::make_unique<vr::sensor::OrbbecSensor>(std::move(opened)));
  }
  for (const std::unique_ptr<vr::sensor::IRgbdSensor>& opened : sensors) {
    // Every camera here is an OrbbecSensor.
    print_orbbec_camera(static_cast<const vr::sensor::OrbbecSensor&>(*opened),
                        calibration);
  }
  if (opt.source.sync_config.empty() &&
      sensors[0]->info().role == vr::sensor::SyncRole::Secondary) {
    std::printf(
        "  note: a sync secondary delivers frames only on another camera's "
        "signal\n");
  }
  std::printf(
      "opened %zu camera(s) in %.1f s\n", sensors.size(),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_open)
          .count());
  return sensors;
}

/// @brief fuse_orbbec's cameras (@ref open_orbbec_cameras) read as an array
///        posed by `source.calibration`, preparing each set on @p device --
///        unless the stream operation, whose array builds no GPU pass.
/// @return The array; the calibration file's error, the cameras', or
///         `SensorArray::open`'s.
inline vkc::Result<vr::sensor::SensorArray> open_orbbec_array(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator) {
  vr::sensor::SensorArray::Options options;
  if (!opt.source.calibration.empty()) {
    VKC_ASSIGN(options.calibration,
               vr::camera::read_array_calibration(opt.source.calibration));
  }
  if (opt.operation != config::Operation::Stream) {
    options.device = &device;
    options.allocator = &allocator;
  }
  VKC_ASSIGN(auto sensors,
             open_orbbec_cameras(opt, options.calibration, device, allocator));
  return vr::sensor::SensorArray::open(std::move(sensors), options);
}

/// @brief Keep @p set's first @p frames frames, in the array's order, and
///        drop the rest: a run of N frames stops mid-set.
/// @return The frames kept.
inline std::size_t cut_set(vr::sensor::FrameSet& set, std::size_t frames) {
  std::size_t kept = 0;
  for (std::optional<vr::sensor::RgbdFrame>& frame : set.frames) {
    if (!frame) continue;
    if (kept == frames) {
      frame.reset();
    } else {
      ++kept;
    }
  }
  return kept;
}

/// @brief A run's sets from a started array, each waited for and cut to the
///        run's length, and what its report prints.
///
/// The array is borrowed: it must outlive every @ref next and
/// @ref print_stats. Only @ref print_stats needs its sensors to be
/// `OrbbecSensor`s.
class RigSets {
 public:
  /// How long @ref next waits for a set before calling the cameras silent.
  /// A streaming camera delivers one every ~33 ms; a secondary with no
  /// primary, or a camera that dropped off the network, delivers none.
  static constexpr std::chrono::seconds kSilenceLimit{10};

  /// @brief A secondary's frame's time less its set's, the primary's, over
  ///        the sets @ref next handed out, before their cut.
  struct Skew {
    double sum_ms = 0.0;       ///< The sum, so the mean is over @ref frames.
    double worst_ms = 0.0;     ///< The largest magnitude.
    std::uint64_t frames = 0;  ///< The camera's frames among those sets.
  };

  /// @param array   Started.
  /// @param frames  The frames the run takes.
  RigSets(vr::sensor::SensorArray& array, std::size_t frames)
      : array_(array),
        limit_(frames),
        skews_(array.size()),
        last_set_(std::chrono::steady_clock::now()) {}

  /// @return Whether the run has taken its frames.
  bool done() const noexcept { return frames_ >= limit_; }
  /// @return The sets @ref next has handed out.
  std::size_t sets() const noexcept { return sets_; }
  /// @return The frames those sets held, as cut.
  std::size_t frames() const noexcept { return frames_; }
  /// @return Camera @p camera's skew to the primary; the primary's is empty.
  /// @pre @p camera is below the array's size.
  const Skew& skew(std::size_t camera) const { return skews_[camera]; }

  /// @brief Wait for the next set and cut it to the frames the run still
  ///        takes (@ref cut_set).
  ///
  /// The host time of the poll that hands the set out is @p stages' `"poll"`
  /// row; an empty poll is the loop waiting, not the driver working. Each
  /// secondary's skew to the primary is kept for @ref print_stats, and a
  /// single camera's first set prints its age against the host's clock.
  /// @param stages  Optional stage rows; null measures nothing.
  /// @return The set; `Status::Code::IoError` after @ref kSilenceLimit with
  ///         none, naming a sync secondary's cause; or the array's error.
  vkc::Result<vr::sensor::FrameSet> next(vkc::StageMetrics* stages) {
    for (;;) {
      const auto t_poll = std::chrono::steady_clock::now();
      VKC_ASSIGN(std::optional<vr::sensor::FrameSet> set, array_.poll_set());
      const auto now = std::chrono::steady_clock::now();
      if (set) {
        if (stages != nullptr) {
          stages->add_cpu(
              "poll",
              std::chrono::duration<double, std::milli>(now - t_poll).count());
        }
        last_set_ = now;
        keep(*set);
        return std::move(*set);
      }
      if (now - last_set_ > kSilenceLimit) return silence();
      // Polled faster than the cameras run.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  /// @brief Print the array's sets, each camera's counters, `lost`
  ///        included, and for a rig each secondary's mean and worst skew to
  ///        the primary.
  /// @pre The array's sensors are all `OrbbecSensor`s.
  void print_stats() const {
    const vr::sensor::SensorArrayStats st = array_.stats();
    std::printf("%llu sets, %llu missing a camera, %llu frames unmatched\n",
                static_cast<unsigned long long>(st.sets),
                static_cast<unsigned long long>(st.incomplete),
                static_cast<unsigned long long>(st.unmatched));
    for (std::size_t i = 0; i < array_.size(); ++i) {
      const auto& camera =
          static_cast<const vr::sensor::OrbbecSensor&>(array_.sensor(i));
      const vr::sensor::OrbbecStreamStats s = camera.orbbec_stats();
      std::printf(
          "  %s: %llu received, %llu delivered, %llu dropped, %llu failed, "
          "%llu lost",
          camera.info().id.c_str(), static_cast<unsigned long long>(s.received),
          static_cast<unsigned long long>(s.delivered),
          static_cast<unsigned long long>(s.dropped),
          static_cast<unsigned long long>(s.failed),
          static_cast<unsigned long long>(s.lost));
      const Skew& lag = skews_[i];
      if (lag.frames > 0) {
        std::printf("; skew to the primary %+.2f ms mean, %.2f ms worst",
                    lag.sum_ms / static_cast<double>(lag.frames), lag.worst_ms);
      }
      std::printf("\n");
    }
  }

 private:
  // Count `set`, keep its skews, and cut it to the run's remaining frames.
  void keep(vr::sensor::FrameSet& set) {
    if (sets_ == 0 && array_.size() == 1) {
      // The SDK's host clock is the system clock: on it (source.host_clock),
      // the frame is milliseconds old; the camera's own is where it was last
      // set.
      const auto host = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch());
      std::printf("  first frame %.1f ms before the host's clock\n",
                  (static_cast<double>(host.count()) -
                   static_cast<double>(set.timestamp_ns)) /
                      1e6);
    }
    for (std::size_t c = 0; c < set.frames.size(); ++c) {
      if (c == array_.primary() || !set.frames[c]) continue;
      const double ms = static_cast<double>(static_cast<std::int64_t>(
                            set.frames[c]->timestamp_ns - set.timestamp_ns)) /
                        1e6;
      skews_[c].sum_ms += ms;
      skews_[c].worst_ms = std::max(skews_[c].worst_ms, std::fabs(ms));
      ++skews_[c].frames;
    }
    frames_ += cut_set(set, limit_ - frames_);
    ++sets_;
  }

  vkc::Status silence() {
    const bool one = array_.size() == 1;
    const bool secondary =
        one && array_.sensor(0).info().role == vr::sensor::SyncRole::Secondary;
    return vkc::Status::io_error(
        "no frame from " +
        (one ? "camera " + array_.sensor(0).info().id
             : std::string("the rig")) +
        " in " + std::to_string(kSilenceLimit.count()) + " s" +
        (secondary ? std::string(" -- it is a sync secondary, which captures "
                                 "only on another camera's signal; start the "
                                 "camera that drives it")
                   : std::string()));
  }

  vr::sensor::SensorArray& array_;
  std::size_t limit_;
  std::size_t sets_ = 0;
  std::size_t frames_ = 0;
  std::vector<Skew> skews_;  // per camera; the primary's stays empty
  std::chrono::steady_clock::time_point last_set_;
};

}  // namespace vr_example
