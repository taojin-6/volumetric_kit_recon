// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Internal (not installed), SDK-free: how OrbbecRig turns per-camera frames
// into one set per sync trigger. Pure bookkeeping over timestamps and the
// caller's opaque ids, so host tests drive it with synthetic streams.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"

namespace volumetric_kit::recon::sensor::orbbec {

/// The order a rig's cameras start in -- every secondary before the primary,
/// since the primary's first trigger is what the secondaries wait for -- or
/// why the rig cannot run: no primary, more than one, or a camera that would
/// stream on its own clock rather than on the primary's trigger.
Result<std::vector<std::size_t>> rig_start_order(
    const std::vector<OrbbecSyncMode>& modes,
    const std::vector<std::string>& serials);

class TriggerGrouper {
 public:
  struct Config {
    std::size_t cameras = 0;
    /// The primary. Its timestamp names a trigger when its frame is present.
    std::size_t anchor = 0;
    /// Frames within this of each other (rig clock) belong to one trigger.
    std::uint64_t tolerance_us = 5000;
    /// How long (host clock) a trigger waits for a camera that has sent
    /// nothing newer before it is handed out without that camera.
    std::uint64_t max_wait_us = 50000;
    /// Frames held per camera; a fuller queue lets its oldest go.
    std::size_t queue_depth = 4;
  };

  /// One trigger: an id per camera, empty where that camera's frame for it
  /// never arrived.
  struct Group {
    std::uint64_t timestamp_us = 0;  ///< The anchor's, else the earliest.
    std::vector<std::optional<std::uint64_t>> ids;
  };

  explicit TriggerGrouper(const Config& config);

  /// A frame from @p camera stamped @p ts_us on the rig clock, arrived at host
  /// time @p now_us. Ids a full queue let go are appended to @p released.
  void add(std::size_t camera, std::uint64_t ts_us, std::uint64_t id,
           std::uint64_t now_us, std::vector<std::uint64_t>* released);

  /// The newest trigger ready to hand out: every camera present, or every
  /// missing camera past it (it sent a later frame) or silent for
  /// `max_wait_us`. Among ready triggers the newest one holding the anchor is
  /// preferred, so a camera whose clock is off by more than the tolerance
  /// costs its own frames rather than the rig's sets. Every other ready
  /// trigger's ids are appended to @p released.
  std::optional<Group> take(std::uint64_t now_us,
                            std::vector<std::uint64_t>* released);

  /// Forget everything held; the ids are appended to @p released.
  void clear(std::vector<std::uint64_t>* released);

 private:
  struct Entry {
    std::uint64_t ts_us;
    std::uint64_t id;
    std::uint64_t arrived_us;
  };
  Config config_;
  std::vector<std::deque<Entry>> queues_;
};

}  // namespace volumetric_kit::recon::sensor::orbbec
