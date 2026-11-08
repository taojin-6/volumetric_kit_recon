// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/live_mobile.hpp
/// @brief The phone lane of a live run (`source.mobile`): phones' frames,
///        tracked against the rig's model and fused as fill where the rig's
///        cameras do not see.
///
/// After each set's send, in the gap before the next set, each of the lane's
/// sessions takes its phone's record due by the predicted next set's time,
/// prepares it and runs every fit due at it (`track::Tracker` on
/// @ref lane_tracking) in one batch, waited on there; then the lane hands the
/// next send up to @ref kDrainCap of the frames the trackers have placed, as
/// fillers (`sender::Filler`, @ref to_filler). A tracker holds a frame on the
/// device until a fit has placed it, Anchored or between two Anchored fits
/// within its hold (5 s either side), and drops it otherwise, so fill lands 2
/// to 7 s after capture. What a session does each step goes into its log
/// (`track_log.hpp`).
///
/// **Sessions.** A session is one phone's: its sensor, its tracker, its log
/// and, placed by the tag, its watch (@ref MobileLane::SessionParts). One
/// added (@ref MobileLane::add) joins at the next gap, after those joined; a
/// run's lane holds one (@ref open_mobile_lane). At its join it takes its
/// mobile index, (its join's ordinal - 1) mod 63 past any a session still
/// holds, so each phone fills under its own (`tsdf::mobile_source`); it holds
/// it until it is done: its phone exhausted, its tracker flushed and what it
/// placed handed over. Done, it lets its frame prep and timer go, and keeps
/// its counts, log and phone for the close. The drain takes one placed frame
/// a session in turn, in join order from the session after the last send's
/// last, up to @ref kDrainCap in all, each session's oldest first: a fixed
/// order, since fused averages depend on it. A session keeps at most
/// @ref kPlacedCap placed frames waiting, its oldest let go past it. A filler
/// the send would refuse (@ref check_filler), which would refuse the whole set,
/// is let go. The rules below are each session's own.
///
/// **Pairing.** A session's phone starts at its join. Its frames are taken in
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
/// **What the phone is told.** A live phone searching by the tag is told how
/// the search goes, in one line of words for a person to read (@ref lane_note),
/// up the socket it dialled (`link::RgbdStreamSensor::say`), on a change and at
/// least every @ref kNoteAtLeastNs, so it can tell a stalled run from a steady
/// one. A session placed by a registration file, or reading a file, says
/// nothing.
///
/// **The log** has no `arkit_raw` row live: the wire carries no log of the
/// device's removed map steps. A replay that has one gives it to its log
/// (`RunSource::device_steps`).
///
/// **Fill's colour** (pending owner decision): until the phone's picture is
/// fused, fill is drawn in a fixed tint of its tier (@ref kAnchoredFillTint,
/// @ref kUnanchoredFillTint), the colour every filler fuses with, made once
/// a depth size for the whole lane; the phone's own picture is never read.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config.hpp"
#include "live_tag.hpp"
#include "track_log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/sender/fuse_inputs.hpp"
#include "volumetric_kit/recon/track/tracker.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace config = volumetric_kit::core::config;
namespace gnet = volumetric_kit::grid::networking;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
namespace vkc = volumetric_kit::core;

/// Fillers fused a set, at most, from every session together: the frames a
/// gap placed beyond it wait for the sets after.
inline constexpr std::size_t kDrainCap = 3;

/// A session's placed frames waiting for a send, at most: past it, its oldest
/// are let go (@ref MobileLane::Counts::shed).
inline constexpr std::size_t kPlacedCap = 40;

/// At least one note a second, in set time, changed or not.
inline constexpr std::uint64_t kNoteAtLeastNs = 1'000'000'000;

/// How long a refused hold's words show, in set time.
inline constexpr std::uint64_t kRefusedShownNs = 3'000'000'000;

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
///         fill's help is never tracked against; mobile sensor @p index (its
///         session's, @ref MobileLane::SessionView::index).
volumetric_kit::recon::sender::Filler to_filler(track::Released&& released,
                                                std::uint32_t index);

/// @brief What the send would refuse of @p filler, and the whole set with it
///        (`tsdf::TsdfIntegrator::integrate`): checked before the lane hands
///        a filler over.
/// @return OK; `Status::Code::InvalidArgument` for an index of
///         `tsdf::kMaxSourcesPerClass` (63) or more, or a depth camera, or a
///         colour camera with a colour, with a non-finite intrinsic, depth
///         bound or pose entry.
vkc::Status check_filler(const volumetric_kit::recon::sender::Filler& filler);

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
  /// @brief What a session did, or every session together.
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
    /// Placed, then let go past the session's @ref kPlacedCap, oldest first.
    std::uint64_t shed = 0;
    /// Placed, then let go as the send would refuse it (@ref check_filler).
    std::uint64_t unfit = 0;
  };

  /// @brief Where the tag search stands, for a person (@ref lane_note).
  struct Status {
    bool registered = false;     ///< By the tag's first hold that passed.
    std::uint32_t gathered = 0;  ///< @ref TagWatch::Counts::gathered.
    /// The set time of the newest refused hold; 0 for none.
    std::uint64_t refused_ns = 0;
  };

  /// @brief How a session's frames are placed in the rig's world.
  enum class Placement {
    /// As its phone was opened: by `source.mobile_registration`, or a world
    /// it shares already.
    Registration,
    /// By the origin tag (@ref SessionParts::tags): nothing is tracked until
    /// the watch registers the session.
    Tag,
  };

  /// @brief One phone's session, as @ref add takes it.
  struct SessionParts {
    /// The phone, not started: its queue is set and it is started at the
    /// session's join. Required.
    std::unique_ptr<sensor::IRgbdSensor> phone;
    /// Its tracker. Required.
    std::unique_ptr<track::Tracker> tracker;
    /// Its log. Required (an empty directory writes no files).
    std::unique_ptr<TrackLog> log;
    /// The origin tag's watch, which registers the session, its frames in
    /// the phone's own world; with a log directory, the registration is
    /// written there (`registration.json`, never over one). Required by
    /// @ref Placement::Tag, refused otherwise. Destroyed before the phone
    /// stops (its thread may hold a picture of the phone's decoder).
    std::unique_ptr<TagWatch> tags;
    /// Whether the phone is connected now, for the session's leave, which
    /// does not read it yet; called on the lane's thread, it must not wait.
    /// Empty: always.
    std::function<bool()> presence;
    /// With @ref tags, told @ref lane_note after a gap when @ref note_due
    /// says so, on the lane's thread; it must not wait. Empty, or without
    /// them: told nothing.
    std::function<void(gnet::ViewerNote)> say;
    /// What the lane's report calls the session (a sensor's id).
    std::string name;
    /// How its frames are placed.
    Placement placement = Placement::Registration;
  };

  /// @brief A session as the lane holds it (@ref sessions).
  struct SessionView {
    std::string name;  ///< @ref SessionParts::name.
    /// @ref SessionParts::placement.
    Placement placement = Placement::Registration;
    /// Its place in the join order, from 1; 0 until it joins.
    std::uint32_t ordinal = 0;
    /// Its mobile index (@ref to_filler), taken at its join; 0 before.
    std::uint32_t index = 0;
    /// Whether it holds @ref index: joined, and not done (its phone
    /// exhausted, its tracker flushed and what it placed handed over).
    bool holding = false;
    /// Its placed frames not yet handed to a send.
    std::size_t waiting = 0;
    Counts counts;                  ///< What it did.
    const TrackLog* log = nullptr;  ///< Its log.
  };

  /// @brief The lane, with one session (@ref add) named by @p phone's sensor
  ///        id, placed by @p tags or as @p phone was opened, joined at the
  ///        first @ref next.
  /// @param device     The run's device; it and @p allocator outlive the lane.
  /// @param phone      As @ref SessionParts::phone. Null makes the empty
  ///                   lane, whose every call does nothing until a session
  ///                   is added.
  /// @param tracker    Its tracker; required with a phone.
  /// @param log        Its log; required with a phone (an empty directory
  ///                   writes no files).
  /// @param tags       As @ref SessionParts::tags. Null: placed as the phone
  ///                   was opened.
  /// @param say        As @ref SessionParts::say.
  /// @return The lane; `Status::Code::InvalidArgument` for a phone without a
  ///         tracker or a log; or the prep's error.
  static vkc::Result<std::unique_ptr<MobileLane>> create(
      vkc::Device& device, vkc::Allocator& allocator,
      std::unique_ptr<sensor::IRgbdSensor> phone,
      std::unique_ptr<track::Tracker> tracker, std::unique_ptr<TrackLog> log,
      std::unique_ptr<TagWatch> tags = {},
      std::function<void(gnet::ViewerNote)> say = {});
  ~MobileLane();
  MobileLane(const MobileLane&) = delete;
  MobileLane& operator=(const MobileLane&) = delete;
  MobileLane(MobileLane&&) = delete;
  MobileLane& operator=(MobileLane&&) = delete;

  /// @brief A session to join at the next @ref next, after those joined, or
  ///        at the first after it with an index free while a session holds
  ///        each of the 63. Its frame prep and timer are made here.
  /// @param parts  The session.
  /// @return OK; `Status::Code::InvalidArgument` for a missing phone,
  ///         tracker or log, a watch its placement does not match, or a
  ///         closed lane; or the prep's error.
  vkc::Status add(SessionParts parts);

  /// @brief After a set's send: the sessions added since the last gap join
  ///        (@ref add); then each joined session, in join order, takes its
  ///        record due by @p next_set_ns, prepares it and records its due
  ///        fits against @p grid as the set left it, in one batch of its own,
  ///        waited on here, then judges, places and logs. Once a session's
  ///        phone is exhausted, its tracker is flushed and what it placed
  ///        still drains.
  /// @param grid         The run's model, not written until this returns.
  /// @param next_set_ns  The next set's predicted time, on the rig's clock.
  /// @return The next send's fillers, at most @ref kDrainCap, one a session
  ///         in turn (above), each session's oldest first; a frame the prep
  ///         or the tracker refuses is passed over and counted, as is a
  ///         filler @ref check_filler refuses. Only the device's, a poll's,
  ///         a log's, a tag watch's device or copy errors, a phone's start
  ///         error and a registration file's write error return.
  vkc::Result<std::vector<volumetric_kit::recon::sender::Filler>> next(
      const volumetric_kit::grid::volume::VoxelBlockGrid& grid,
      std::uint64_t next_set_ns);

  /// @brief The run's end, for each session in join order: its tag watch's
  ///        last news taken and the watch let go (a hold its thread is
  ///        solving then, one search of 10 to 55 ms, is not logged), its
  ///        tracker flushed (held frames decided, placed or dropped, and
  ///        logged, not fused), its log closed and its summary printed.
  ///        Idempotent.
  /// @return OK; or the first session's error to return one: its tracker's
  ///         or its log's, or its watch's device or copy error once the rest
  ///         of its close has run. Every session's close runs.
  vkc::Status close();

  /// @return What the lane's sessions did so far, together.
  Counts counts() const noexcept;

  /// @return Each session, in join order, those added since the last gap
  ///         last.
  std::vector<SessionView> sessions() const;

  /// @return Its lines for the run's report: each session's counts and its
  ///         phone's counters, in join order, each named by its ordinal and
  ///         name when there are more than one; empty for the empty lane.
  std::string report() const;

  /// @return The first session's log; null for the empty lane.
  const TrackLog* log() const noexcept;

 private:
  struct State;
  explicit MobileLane(std::unique_ptr<State> state) noexcept;
  std::unique_ptr<State> state_;
};

/// @brief The one line a person reads of the tag search, every word of it, by
///        the first that holds: registered, a hold refused within
///        @ref kRefusedShownNs, stills gathered, still looking.
/// @param status  Where the search stands.
/// @param set_ns  The set it is said at, on the rig's clock.
/// @return The line and its tone.
gnet::ViewerNote lane_note(const MobileLane::Status& status,
                           std::uint64_t set_ns);

/// @return Whether @p now goes out, @p since_ns of set time after @p last did:
///         with no last, on a change, or at @ref kNoteAtLeastNs.
bool note_due(const gnet::ViewerNote& now,
              const std::optional<gnet::ViewerNote>& last,
              std::uint64_t since_ns);

/// @brief The run's lane: with `source.mobile` its session -- a file, or the
///        phone dialling `tcp://ADDRESS:PORT`, waited for here up to 5
///        minutes (a Ctrl+C ends the wait with the empty lane) and polled in
///        the background -- read at confidence 2 or more
///        (`link::RgbdStreamSensor`), its tracker on @ref lane_tracking and
///        its log in `diagnostics.track_log`; without it, the empty lane.
///        Placed by `source.mobile_registration`, read depth alone; or
///        without one by the origin tag, read with its pictures: a
///        @ref TagWatch in `source.calibration`'s world, on its own thread
///        (every lane runs beside a live rig), a live phone told how the
///        search goes. A file read so decodes its pictures in
///        @ref MobileLane::next, on the set loop (about 1 ms at p95 a gap,
///        measured on an RTX 5090): a replay's cost, not a live run's, whose
///        phone decodes on its poll thread. With `diagnostics.track_log`, a
///        live phone is recorded there (`phone.vrst`), and the tag's
///        registration is written beside it (`registration.json`), so the run
///        replays.
/// @return The lane; @ref check_mobile's refusal, before any wait; with the
///         tag, `Status::Code::InvalidArgument` before any wait for a
///         `diagnostics.track_log` that holds a `registration.json` already
///         (an earlier run's), and once the session has come for one with no
///         pictures; or the opener's, the registration's, the watch's or the
///         tracker's error.
vkc::Result<std::unique_ptr<MobileLane>> open_mobile_lane(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator);

}  // namespace vr_example
