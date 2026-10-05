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

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"

namespace volumetric_kit::recon::sensor::orbbec {

/// The order a rig's cameras start in -- every secondary before the primary,
/// since the primary's first trigger is what the secondaries wait for -- or
/// why the rig cannot run: no primary, more than one, or a camera that would
/// stream on its own clock rather than on the primary's trigger.
core::Result<std::vector<std::size_t>> rig_start_order(
    const std::vector<OrbbecSyncMode>& modes,
    const std::vector<std::string>& serials);

class TriggerGrouper {
 public:
  struct Config {
    std::size_t cameras = 0;
    /// The primary. Its frames name the triggers: a trigger without one is
    /// never handed out.
    std::size_t anchor = 0;
    /// A camera's frame within this of an anchor frame (rig clock) belongs to
    /// its trigger.
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
    std::uint64_t timestamp_us = 0;  ///< The anchor's frame's.
    std::vector<std::optional<std::uint64_t>> ids;
  };

  explicit TriggerGrouper(const Config& config);

  /// A frame from @p camera stamped @p ts_us on the rig clock, arrived at host
  /// time @p now_us. Ids a full queue let go are appended to @p released.
  void add(std::size_t camera, std::uint64_t ts_us, std::uint64_t id,
           std::uint64_t now_us, std::vector<std::uint64_t>* released);

  /// The newest trigger ready to hand out: an anchor frame, and each other
  /// camera's frame within the tolerance of it, once every missing camera is
  /// past it (it sent a later frame) or silent for `max_wait_us`. A frame
  /// earlier than the tolerance allows of the earliest anchor frame held is
  /// near no trigger still to come -- its set already went out, the anchor
  /// missed it, or its camera's clock is off -- so it is let go, and such a
  /// camera costs its own frames, not the rig's sets. Those ids, and every
  /// older ready trigger's, are appended to @p released.
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
