// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/live_mobile.hpp
/// @brief The phone lane of a live run (`source.mobile`): the phone's frames,
///        tracked against the rig's model and fused as fill where the rig's
///        cameras do not see.
///
/// After each set's send, in the gap before the next set, the lane takes the
/// phone record due by the predicted next set's time, prepares it and runs
/// every fit due at it (`track::Tracker` on @ref lane_tracking) in one batch,
/// waited on there; then it hands the next send up to @ref kDrainCap of the
/// frames the tracker has placed, as fillers (`sender::Filler`,
/// @ref to_filler). The tracker holds a frame on the device until a fit has
/// placed it, Anchored or between two Anchored fits within its hold (5 s
/// either side), and drops it otherwise, so fill lands 2 to 7 s after
/// capture. What the lane does each step goes into its log (`track_log.hpp`).
///
/// **Pairing.** The phone starts with the first gap. Its frames are taken in
/// order (`IRgbdSensor::drain`, a queue of 64) whenever none taken waits.
/// The first take pins the phone's clock to the rig's at its newest record:
/// that record's time pairs with the predicted time of the set it is taken
/// for, so what a live link held while the cameras opened is passed by, and
/// every later record is due at the set whose predicted time, on that offset,
/// is at or past its own. A live link's records (`RgbdFrame::arrival_ns`
/// set) are due once they have come: a take whose newest is fresher than the
/// offset says moves the offset on, never back, so a backlog that comes over
/// several gaps never leaves the lane behind. A gap takes the newest record
/// due and passes the older ones by. A file's records come as fast as they
/// are read and carry no arrival, so only its first take pins and a replay
/// pairs exactly; a live link's are read on a thread of their own
/// (@ref polled_in_background), so a set never waits for the phone. An error
/// the phone returns is counted and the lane goes on.
///
/// **Splits.** Where records were lost over @ref kSplitGapNs or more (a gap in
/// their sequence), the lane flushes the tracker before the first frame after
/// the loss (`Tracker::flush`): a fit never holds frames on both sides of a
/// loss, which may hide a map step the device's own step removal did not
/// see.
///
/// **Registration.** By `source.mobile_registration` (the phone opened with
/// it, depth alone), or without one by the origin tag (live_tag.hpp): the
/// phone opened with its pictures, every paired frame shown to the watch and
/// none tracked until it registers the session (counted `unregistered`);
/// from then on each frame is placed by that registration
/// (`tracking_to_world * color_to_world`, the product `RgbdStreamSensor`
/// applies to one it is given). The first Anchored fit of the rig chain
/// prints how far it moved the tag's registration; a later hold that passes
/// is printed with its gap from the rig chain and logged, never applied. Both
/// gaps are @ref pose_gap at the newest paired frame's camera.
///
/// **The log** has no `arkit_raw` row live: the wire carries no log of the
/// device's removed map steps. A replay that has one gives it to its log
/// (`RunSource::device_steps`).
///
/// **Fill's colour** (pending owner decision): until the phone's picture is
/// fused, fill is drawn in a fixed tint of its tier (@ref kAnchoredFillTint,
/// @ref kUnanchoredFillTint), the colour every filler fuses with; the
/// phone's own picture is never read.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "config.hpp"
#include "live_tag.hpp"
#include "track_log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/sender/fuse_inputs.hpp"
#include "volumetric_kit/recon/track/tracker.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace config = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
namespace vkc = volumetric_kit::core;

/// Fillers fused a set, at most: the frames a gap placed beyond it wait for
/// the sets after.
inline constexpr std::size_t kDrainCap = 3;

/// The phone's queue (`IRgbdSensor::set_queue_depth`): 6 s of a 10 Hz
/// phone, its oldest let go past it.
inline constexpr std::size_t kPhoneQueue = 64;

/// A loss this long or longer between two records starts the tracker's
/// windows again (a split).
inline constexpr std::uint64_t kSplitGapNs = 100'000'000;

/// The colour fill of each tier fuses with, as a prepared frame's colour word
/// (R, G, B in the low bytes, sRGB-encoded; coverage 0xFF): anchored fill
/// teal, unanchored fill orange.
inline constexpr std::uint32_t kAnchoredFillTint = 0xFFC8B428u;
inline constexpr std::uint32_t kUnanchoredFillTint = 0xFF288CF0u;

/// @return What the lane tracks with -- code, not configuration, since the
///         driver decides what fuses: 2 s windows every 0.5 s
///         (`track::window_method`) on two chains, `rig` against the rig
///         alone (`tsdf::rig_cast`) and `track` against the rig and anchored
///         fill (`tsdf::tracking_cast`), and the device's own poses beside
///         them (`arkit`, no fit). Shadows only log.
track::TrackerConfig lane_tracking();

/// @return @p released as a filler: its frame moved by its correction, both
///         cameras (`track::corrected`); Anchored fill only when the rig-only
///         chain (driver 0) anchored it, else Unanchored, so fill placed with
///         fill's help is never tracked against; mobile sensor 0.
volumetric_kit::recon::sender::Filler to_filler(track::Released&& released);

/// @brief What a scenario with `source.mobile` needs that the run cannot do,
///        checked before any device opens.
/// @return OK; `Status::Code::Unsupported` beside FFS depth
///         (`source.depth` ffs), which stage 1 does not run the lane beside;
///         `Status::Code::InvalidArgument` under Classic fusion
///         (`fusion.dynamic` false), since fill needs Dynamic, or with
///         `operation: "stream"`, which fuses nothing; without
///         `source.mobile_registration`, `source.calibration`'s read error
///         or @ref TagWatch::check's refusal of its world
///         (`Status::Code::Unsupported` in a build without
///         `VR_WITH_CALIB`).
vkc::Status check_mobile(const config::Config& opt);

/// @return @p sensor polled on a thread of its own from its start, so a
///         @ref sensor::IRgbdSensor::poll or `drain` never waits on it: what
///         a live link needs, whose own poll waits for records. It keeps the
///         interface's queue: `set_queue_depth` frames (1 by default), the
///         oldest let go (counted dropped) when a newer one arrives on a
///         full one; `poll` takes the newest, `drain` every frame, oldest
///         first. Its other counters and `exhausted` are the inner sensor's
///         as of its last poll; an error the inner poll returns comes out of
///         the next poll or drain once, and the polling goes on.
std::unique_ptr<sensor::IRgbdSensor> polled_in_background(
    std::unique_ptr<sensor::IRgbdSensor> sensor);

/// @brief The phone lane (above). An aggregate: neither copied nor moved.
class MobileLane {
 public:
  /// @brief What the lane did.
  struct Counts {
    std::uint64_t polled = 0;  ///< Records taken from the phone.
    std::uint64_t paired = 0;  ///< Frames prepared and tracked.
    /// Records passed by: an older one due in the same gap as a newer one.
    std::uint64_t skipped = 0;
    /// Frames the prep or the tracker refused: counted, and the lane goes on.
    std::uint64_t passed_over = 0;
    std::uint64_t splits = 0;      ///< Flushes at a loss.
    std::uint64_t errors = 0;      ///< Errors the phone returned.
    std::uint64_t anchored = 0;    ///< Placed as Anchored fill.
    std::uint64_t unanchored = 0;  ///< Placed as Unanchored fill.
    std::uint64_t dropped = 0;     ///< Never placed.
    std::uint64_t fused = 0;       ///< Handed to a send.
    /// Paired before the tag registered the session: not tracked.
    std::uint64_t unregistered = 0;
  };

  /// @param device     The run's device; it and @p allocator outlive the lane.
  /// @param phone      The phone, not started: its queue is set and it is
  ///                   started at the first @ref next. Null makes the empty
  ///                   lane, whose every call does nothing.
  /// @param tracker    Its tracker; required with a phone.
  /// @param log        Its log; required with a phone (an empty directory
  ///                   writes no files).
  /// @param tags       The origin tag's watch, which registers the phone,
  ///                   its frames in the phone's own world; with a log
  ///                   directory, the registration is written there
  ///                   (`registration.json`, never over one). Null: placed
  ///                   as the phone was opened. Destroyed before the phone
  ///                   stops (its thread may hold a picture of the phone's
  ///                   decoder).
  /// @return The lane; `Status::Code::InvalidArgument` for a phone without a
  ///         tracker or a log; or the prep's error.
  static vkc::Result<std::unique_ptr<MobileLane>> create(
      vkc::Device& device, vkc::Allocator& allocator,
      std::unique_ptr<sensor::IRgbdSensor> phone,
      std::unique_ptr<track::Tracker> tracker, std::unique_ptr<TrackLog> log,
      std::unique_ptr<TagWatch> tags = {});
  ~MobileLane();
  MobileLane(const MobileLane&) = delete;
  MobileLane& operator=(const MobileLane&) = delete;
  MobileLane(MobileLane&&) = delete;
  MobileLane& operator=(MobileLane&&) = delete;

  /// @brief After a set's send: take the record due by @p next_set_ns,
  ///        prepare it and record its due fits against @p grid as the set
  ///        left it, in one batch, waited on here; then judge, place and log.
  ///        Once the phone is exhausted, the tracker is flushed and what it
  ///        placed still drains.
  /// @param grid         The run's model, not written until this returns.
  /// @param next_set_ns  The next set's predicted time, on the rig's clock.
  /// @return The next send's fillers, at most @ref kDrainCap, oldest first;
  ///         a frame the prep or the tracker refuses is passed over and
  ///         counted. Only the device's, a poll's, the log's, the tag
  ///         watch's device or copy errors and the registration file's write
  ///         error return.
  vkc::Result<std::vector<volumetric_kit::recon::sender::Filler>> next(
      const volumetric_kit::grid::volume::VoxelBlockGrid& grid,
      std::uint64_t next_set_ns);

  /// @brief The run's end: the tag watch's last news taken and the watch
  ///        let go (a hold its thread is solving then, one search of 10 to
  ///        55 ms, is not logged), the tracker flushed (held frames decided,
  ///        placed or dropped, and logged, not fused), the log closed and its
  ///        summary printed. Idempotent.
  /// @return OK; the tracker's or the log's error; or the watch's device or
  ///         copy error, once the rest of the close has run.
  vkc::Status close();

  /// @return What the lane did so far.
  const Counts& counts() const noexcept;

  /// @return Its line for the run's report: the counts and the phone's
  ///         counters; empty for the empty lane.
  std::string report() const;

  /// @return The lane's log; null for the empty lane.
  const TrackLog* log() const noexcept;

 private:
  struct State;
  explicit MobileLane(std::unique_ptr<State> state) noexcept;
  std::unique_ptr<State> state_;
};

/// @brief The run's lane: with `source.mobile` its session -- a file, or the
///        phone dialling `tcp://ADDRESS:PORT`, waited for here up to 5
///        minutes (a Ctrl+C ends the wait with the empty lane) and polled in
///        the background -- read at confidence 2 or more
///        (`link::RgbdStreamSensor`), its tracker on @ref lane_tracking and
///        its log in `diagnostics.track_log`; without it, the empty lane.
///        Placed by `source.mobile_registration`, read depth alone; or
///        without one by the origin tag, read with its pictures: a
///        @ref TagWatch in `source.calibration`'s world, on its own thread
///        (every lane runs beside a live rig). A file read so decodes its
///        pictures in @ref MobileLane::next, on the set loop (about 1 ms at
///        p95 a gap, measured on an RTX 5090): a replay's cost, not a live
///        run's, whose phone decodes on its poll thread. With
///        `diagnostics.track_log`, a live phone is recorded there
///        (`phone.vrst`), and the tag's registration is written beside it
///        (`registration.json`), so the run replays.
/// @return The lane; @ref check_mobile's refusal, before any wait; with the
///         tag, `Status::Code::InvalidArgument` before any wait for a
///         `diagnostics.track_log` that holds a `registration.json` already
///         (an earlier run's), and once the session has come for one with no
///         pictures; or the opener's, the registration's, the watch's or the
///         tracker's error.
vkc::Result<std::unique_ptr<MobileLane>> open_mobile_lane(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator);

}  // namespace vr_example
