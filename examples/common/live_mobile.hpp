// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/live_mobile.hpp
/// @brief The phone lane of a live run (`source.mobile`): phones' frames,
///        tracked against the rig's model and fused as fill where the rig's
///        cameras do not see.
///
/// After each set's send, in the gap before the next set, each of the lane's
/// sessions takes its phone's record due by the predicted next set's time;
/// the gap's scheduler (below) has it prepare that record and run every fit
/// due at it (`track::Tracker` on @ref lane_tracking) in one batch of its
/// own, or puts the record back for a later gap; then the lane hands the
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
/// file's lane holds one, a live lane those its phones bring
/// (@ref open_mobile_lane). At its join it takes its mobile index, (its
/// join's ordinal - 1) mod 63 past any a session still holds, so each phone
/// fills under its own (`tsdf::mobile_source`); it holds it until it is
/// done: its phone exhausted or the session retired, its tracker flushed and
/// what it placed handed over. Done, it lets its frame prep and timer go; a
/// file's keeps its counts, log and phone for the close, and a live phone's
/// leaves (below). The drain takes one placed frame a session in turn, in
/// join order from the session after the last send's last, up to
/// @ref kDrainCap in all, each session's oldest first: a fixed order, since
/// fused averages depend on it. A session keeps at most @ref kPlacedCap placed
/// frames waiting, its oldest let go past it. A filler the send would refuse
/// (@ref check_filler), which would refuse the whole set, is let go. The rules
/// below are each session's own; the gap's scheduler is the lane's.
///
/// **The gap's scheduler.** A gap runs in this order: a session joins (one
/// the source made, a gap at most, and those added since the last gap); each
/// session does its host work (its presence, its watch's news, its record
/// due, as paired below); then each record is classed, before anything of it
/// is shown to a watch or tracked: by the tag, not yet registered (shown to
/// its watch alone), a prep (no fit due) or a hop (placed, and a fit due at
/// it, `Tracker::fit_due`). One hop runs a gap, of all the sessions': the one
/// whose hop has waited the most gaps, ties taken in turn: the turn moves to
/// the session after the winner only in a gap two or more hops contest, so
/// phones whose hops keep falling in the same gaps take the first place in
/// turn, period by period. A record not run goes back to the front of its
/// session's records, its split and its watch's sight of it untaken, and
/// counts a deferral (@ref MobileLane::Counts::deferred); a newer record due
/// before it runs takes its place, as any older one due is passed by, and a
/// hop stays due at it. The records taken are each prepared and recorded into
/// their session's own batch, submitted in join order and then waited in
/// join order, so the sessions' preps overlap one another and the hop. A live
/// lane reads its gap's @ref GapBudget: with an end, the hop runs only if now
/// plus @ref HopEstimate's p90 is by it, and once past it the preps go back
/// too (@ref MobileLane::Counts::over_budget): the rig's sets come before a
/// phone's fill. So that the estimate, fed only by hops that run, is never
/// stuck above every gap's room, a budget that has held hops back for
/// @ref kHopProbeNs of set time with none run has the next gap not yet past
/// its end run its hop whatever the estimate says: a probe, one a second at
/// most, which may overrun its gap; one that ends by the gap's end replaces
/// every sample longer than that gap's room (@ref HopEstimate::probed). A
/// file's lane reads no budget, whatever it is given, and `run_live_fusion`
/// gives one to a live lane alone (@ref MobileLane::live); with none (a
/// file's run, the tests' `GapBudget{}`) nothing but a second hop in one gap
/// goes back, so a replay repeats, and one phone defers nothing. Over each
/// @ref kDeferralWindowNs of set time in which more than half the records
/// classed a prep or a hop went back, the lane prints a warning with the
/// estimate and the last room.
///
/// **Coming and going** (a session with a presence,
/// @ref MobileLane::SessionParts::presence: a live phone's). Its stages are
/// @ref MobileLane::Stage's. A gap that finds it not connected makes it Away
/// from that set's time, printed; one not yet seen connected since its join
/// only from 1 s after it (@ref kFirstSeenNs), Away from the join, as its
/// phone's thread takes its connection after the start. Connected again
/// within the grace (@ref LaneOptions::grace_ns, 30 s of set time), it is
/// Live again, the same session, its registration kept, a loss in its
/// records a split as any.
/// Away for @ref LaneOptions::settle_ns (7 s: the tracker's 5 s hold and its
/// 2 s window, so no frame then shares a fit or an interpolation with a
/// later one), its tracker is flushed once, which places nothing differently
/// and lets what it holds go sooner. It retires at the grace, when its phone
/// is exhausted (the link's failure cap or its decoder), at an error of its
/// own, or at the run's end: its watch's last news taken, its tracker
/// flushed; from then it pairs and says nothing, its phone's queue is taken
/// and let go each gap, and what it placed still drains. Once that is
/// handed over, it leaves: the lane's reaper thread lets its watch go, then
/// stops and destroys its phone (the link's slot frees with it), then its
/// tracker, prep and timer; then closes its log, prints its summary and
/// writes its row of `sessions.csv`. Its log goes too, but one in the run at
/// the close stays for the caller. A session that comes back after it left
/// is a new one: a new ordinal, index, directory and watch.
///
/// **Errors.** Of a session that comes and goes, only the device's loss
/// (`VK_ERROR_DEVICE_LOST`) comes out of @ref MobileLane::next and
/// @ref MobileLane::close, the phone's and the opener's too. Anything else it
/// meets retires that session alone, printed once a session id, and told
/// (@ref MobileLane::SessionParts::on_error): its prep's or its tracker's
/// error other than `Status::Code::InvalidArgument` (a frame they refuse is
/// passed over, as before), its phone's start, a watch that fails before it
/// registers. A live lane then keeps that phone's id from its door for
/// @ref LaneOptions::error_back_off, doubled with each repeat up to 32 times,
/// so a phone whose error recurs is not made a session over and over; the
/// device out of memory also pauses its admissions for a minute. A file's
/// session, always there, returns its error as before: a replay asked for
/// that file. A log's, a registration file's or a recording's write error is
/// printed once and the session goes on: the log never changes what fuses. A
/// phone a live lane turns away is never an error.
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
/// the phone returns is counted and the lane goes on, but the device's loss,
/// which returns.
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
/// up the socket it dialled, through its session's handle
/// (`link::SessionHandle::say`, never the sensor), on a change and at least
/// every @ref kNoteAtLeastNs, so it can tell a stalled run from a steady one.
/// A session placed by a registration file, reading a file, retired or turned
/// away, says nothing.
///
/// **The log** has no `arkit_raw` row live: the wire carries no log of the
/// device's removed map steps. A replay that has one gives it to its log
/// (`RunSource::device_steps`).
///
/// **Fill's colour** (pending owner decision): until the phone's picture is
/// fused, fill is drawn in a fixed tint of its tier (@ref kAnchoredFillTint,
/// @ref kUnanchoredFillTint), the colour every filler fuses with, made once
/// a depth size for the whole lane; the phone's own picture is never read.
///
/// **A live run** (`source.mobile` `tcp://ADDRESS:PORT`) never waits for a
/// phone, and never depends on one: @ref open_mobile_lane returns at once and
/// the cameras open. A door (`link::RgbdStreamSensor::Door`) listens on the
/// address for the whole run, at most @ref kMaxPhones sessions at once, a
/// new one past them refused right after its session's chunk, which a writer
/// that redials reads as a dial that failed (it shows Retrying, and joins
/// once a slot frees); a held session's reconnect always gets through. An
/// address it cannot bind at the start (not this host's yet, in use, its
/// interface down) is warned of and bound again every 2 s, off the set loop,
/// until it takes. The opener's thread admits each new session (its decoder
/// made there), makes its watch, frame prep, timer, fitters, tracker and
/// log, and hands it to the lane, which joins one a gap. A session is placed
/// by `source.mobile_registration` if that names its id (depth alone), else
/// by the origin tag where this build and the calibration's world allow it,
/// else turned away; a registration that cannot be read is warned of and
/// names none. A setting under which no phone can be taken (Classic fusion,
/// FFS depth, `operation: "stream"`, no registration where the tag cannot
/// place one, a device that cannot run the fitters) turns every phone away,
/// the start line says why, and the rig runs. A phone turned away is refused
/// at the door as one past the cap is, logged once a spell, and counted in
/// `door.csv` and the report.
///
/// **A live run's log** (`diagnostics.track_log` D): `D/sessions.csv`, a row
/// for each join and each leave (`event`, `set_ns`, `ordinal`, `session_id`,
/// `sensor`, `peer`, `mobile_index`, `placement`, `directory`, `polled`,
/// `paired`, `fused` and `reason`, the rest of the line; the counts and the
/// reason a leave's), written by the reaper alone; `D/door.csv`, the door's
/// (`link/rgbd_stream_sensor.hpp`); each made at its first row, so a run no
/// phone dialled leaves D as it was; and a directory a session,
/// `D/NN-<sensor>-<hex8>/` (NN its join's ordinal, two digits or more; its
/// sensor id, any byte but a letter, a digit, `.`, `_` or `-` made `_`; its
/// session id's first 8 hex digits), holding its log (`track_log.hpp`), its
/// recording (`phone.vrst`) and its tag's registration
/// (`registration.json`), so each replays alone as a file. A D an earlier
/// run logged into (its `sessions.csv` or `door.csv`, a session's
/// directory, a flat `registration.json` or `phone.vrst`), or one that cannot
/// be made, is warned of at the start and named in the report, and this run
/// writes nothing there: its phones join, unlogged and unrecorded. A file's
/// run keeps one flat log in D. An empty D writes nothing: no directory,
/// recording or row, and every summary still prints.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config.hpp"
#include "live_tag.hpp"
#include "polled_sensor.hpp"  // polled_in_background, declared here before
#include "track_log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/sender/fuse_inputs.hpp"
#include "volumetric_kit/recon/track/tracker.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"
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

/// A live run's phones at once, at most: each session from its arrival at
/// the door until its sensor is destroyed (`link::DoorOptions::max_sessions`).
/// Code, not configuration, as @ref lane_tracking is: past it a new session
/// is refused silently and joins once a slot frees. Four, as the gap's
/// scheduler runs one hop a gap among them.
/// TODO: the lab run at one, two and four phones under load (PERF.md, PH1)
/// confirms four or lowers it.
inline constexpr std::size_t kMaxPhones = 4;

/// The hops a live gap's guard estimates the next by (@ref HopEstimate).
inline constexpr std::size_t kHopSamples = 32;

/// What each of them is until a hop takes its place: the guard's seed
/// (@ref LaneOptions::hop_seed's default).
inline constexpr std::chrono::nanoseconds kHopSeed{20'000'000};

/// How long, in set time, a live lane's budget may hold its hops back with
/// none run before a gap not yet past its end runs one whatever the estimate
/// says (a probe, @ref HopEstimate::probed), so the estimate learns again.
inline constexpr std::uint64_t kHopProbeNs = 1'000'000'000;

/// The set time over which the share of records put back is warned of, past
/// half of those classed a prep or a hop.
inline constexpr std::uint64_t kDeferralWindowNs = 10'000'000'000;

/// How long a live session may go without a connection, in set time, before
/// it retires: @ref LaneOptions::grace_ns's default.
inline constexpr std::uint64_t kGraceNs = 30'000'000'000;

/// How long Away before its tracker is flushed, in set time: the tracker's
/// 5 s hold and its 2 s window; @ref LaneOptions::settle_ns's default.
inline constexpr std::uint64_t kAwaySettleNs = 7'000'000'000;

/// How long after its join, in set time, a session not yet seen connected
/// stays Live: its phone's polling thread takes its connection after the
/// start, so the join's own gap may read it not connected.
inline constexpr std::uint64_t kFirstSeenNs = 1'000'000'000;

/// @brief A lane's lifecycle settings, and where a live lane writes
///        `sessions.csv`.
struct LaneOptions {
  /// Away this long, in set time, a session retires; positive.
  std::uint64_t grace_ns = kGraceNs;
  /// Away this long, its tracker is flushed once; positive, below the grace.
  std::uint64_t settle_ns = kAwaySettleNs;
  /// How long, in steady time, a live lane's door refuses a phone's id after
  /// a session of it retired for an error of its own or was not opened:
  /// doubled with each repeat of that id, 32 times this at most; positive.
  std::chrono::milliseconds error_back_off{10'000};
  /// What the gap's guard estimates a hop at until hops take its place
  /// (@ref HopEstimate): each of its samples at the start; positive. A test
  /// sets it to reach the guard's branches with no hop that slow.
  std::chrono::nanoseconds hop_seed = kHopSeed;
  /// Where `sessions.csv` goes (D, above), made with its directory at its
  /// first row, never over one already there: one that cannot be made is
  /// printed, and none written. Empty writes none. @ref open_mobile_lane
  /// gives `diagnostics.track_log`, or nothing where that holds an earlier
  /// run's logs or cannot be made.
  std::string log_directory;
};

/// @brief What a gap may spend.
struct GapBudget {
  /// The steady time the gap ends by: when the set loop's set came, plus the
  /// rig's period, less 3 ms. A live lane reads it (the gap's scheduler,
  /// above): its hop runs only if now plus @ref HopEstimate's p90 is by it,
  /// or as a probe (@ref kHopProbeNs), and once past it the preps wait too.
  /// Empty, no limit. `run_live_fusion` gives it to a live lane alone
  /// (@ref MobileLane::live), and a file's lane (@ref MobileLane::create's
  /// first form) reads none, whatever it is given, so a replay repeats.
  std::optional<std::chrono::steady_clock::time_point> gap_end;
};

/// @brief What a hop costs a live gap, by which its guard decides
///        (@ref GapBudget): the 90th percentile, by nearest rank (the 29th
///        of 32), of the last @ref kHopSamples hops' wall times, each of the
///        32 the seed until a hop takes its place, oldest first. A hop's wall
///        time runs from the gap's choice to its last batch waited. A value.
class HopEstimate {
 public:
  /// @brief Every sample @p seed.
  /// @param seed  What a hop is taken to cost before one ran; positive.
  explicit HopEstimate(std::chrono::nanoseconds seed = kHopSeed) noexcept;
  /// @brief A hop's wall time, in place of the oldest sample.
  void add(std::chrono::nanoseconds wall) noexcept;
  /// @brief A probe's wall time (a hop run whatever the estimate said), in
  ///        place of the oldest sample. One that ended within @p room, the
  ///        time its gap had left at the choice, shows the estimate stale:
  ///        every sample longer than @p room becomes @p wall first, so the
  ///        next gap with that room runs its hop. One that did not is a
  ///        sample as @ref add takes it.
  /// @param wall  The probe's wall time.
  /// @param room  Its gap's end less its choice.
  void probed(std::chrono::nanoseconds wall,
              std::chrono::nanoseconds room) noexcept;
  /// @return The estimate.
  std::chrono::nanoseconds p90() const noexcept;

 private:
  std::array<std::chrono::nanoseconds, kHopSamples> samples_{};
  std::size_t next_ = 0;  // the oldest
};

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
///        checked before any device opens. A live `source.mobile` is never
///        refused: what this refuses of a file turns its phones away
///        (@ref open_mobile_lane), and the rig runs.
/// @return OK, always for a live one; for a file,
///         `Status::Code::Unsupported` beside FFS depth
///         (`source.depth` ffs), which stage 1 does not run the lane beside;
///         `Status::Code::InvalidArgument` under Classic fusion
///         (`fusion.dynamic` false), since fill needs Dynamic, or with
///         `operation: "stream"`, which fuses nothing; without
///         `source.mobile_registration`, `source.calibration`'s read error
///         or @ref TagWatch::check's refusal of its world
///         (`Status::Code::Unsupported` in a build without
///         `VR_WITH_CALIB`).
vkc::Status check_mobile(const config::Config& opt);

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
    /// Records the gap's scheduler put back for a later gap, each time one
    /// went back (one put back twice counts twice): a hop past the gap's
    /// one, or a hop or a prep a live gap's budget held back.
    std::uint64_t deferred = 0;
    /// Of @ref deferred, those the budget held back (@ref GapBudget): a hop
    /// its estimate did not fit, or any past the gap's end.
    std::uint64_t over_budget = 0;
  };

  /// @brief Where the tag search stands, for a person (@ref lane_note).
  struct Status {
    bool registered = false;     ///< By the tag's first hold that passed.
    std::uint32_t gathered = 0;  ///< @ref TagWatch::Counts::gathered.
    /// The set time of the newest refused hold; 0 for none.
    std::uint64_t refused_ns = 0;
  };

  /// @brief Where a session is in its life (above).
  enum class Stage : std::uint8_t {
    Added,     ///< Added, waiting for its join.
    Live,      ///< Joined, its phone connected (or a file's).
    Away,      ///< Joined, its phone not connected since a gap.
    Retiring,  ///< Retired: what it placed drains, nothing more paired.
    Reaping,   ///< Handed to the reaper, which lets it go.
    Left,      ///< Gone: what is kept of it is its counts and lines.
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
    /// Whether the phone is connected now, read each gap on the lane's
    /// thread; it must not wait. Given, the session comes and goes (above);
    /// empty, it is always there (a file's), and leaves at the close.
    std::function<bool()> presence;
    /// With @ref tags, told @ref lane_note after a gap when @ref note_due
    /// says so, on the lane's thread; it must not wait. Empty, or without
    /// them: told nothing.
    std::function<void(gnet::ViewerNote)> say;
    /// Told the error the session retires for, once, on the lane's thread,
    /// if it comes and goes (@ref presence) and the error is its own (not
    /// the device's loss); it must not wait. Let go as the session leaves.
    std::function<void(const vkc::Status&)> on_error;
    /// What the lane's report calls the session (a sensor's id).
    std::string name;
    /// How its frames are placed.
    Placement placement = Placement::Registration;
    /// Its frame prep and timer, made off the set loop (a live lane's
    /// opener); empty, @ref add makes them.
    std::optional<sensor::GpuFramePrep> prep;
    std::optional<vkc::GpuTimer> timer;  ///< As @ref prep.
  };

  /// @brief Where a live lane's sessions come from (@ref open_mobile_lane's
  ///        door and opener): each one made off the set loop, its phone not
  ///        started, taken by the lane at a gap.
  class SessionSource {
   public:
    virtual ~SessionSource() = default;
    /// @return The next session ready to join; none yet; or, once, the
    ///         device's loss the source met making one. Never waits.
    virtual vkc::Result<std::optional<SessionParts>> take() = 0;
    /// @brief The device ran out of memory in a session: no session is
    ///        admitted for a while.
    virtual void out_of_memory() noexcept = 0;
    /// @brief No session more: its threads stopped and joined, and any
    ///        session made and not taken let go. Idempotent.
    virtual void close() noexcept = 0;
    /// @return Its lines of the run's report.
    virtual std::string report() const = 0;
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
    /// exhausted or it retired, its tracker flushed and what it placed
    /// handed over).
    bool holding = false;
    Stage stage = Stage::Added;  ///< Where it is in its life.
    /// Its placed frames not yet handed to a send.
    std::size_t waiting = 0;
    Counts counts;  ///< What it did.
    /// Its log; null while the reaper has it, and once it left before the
    /// close.
    const TrackLog* log = nullptr;
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

  /// @brief A live lane: sessions from @p source, one joined a gap, and any
  ///        added (@ref add); its reaper's thread started, and
  ///        `sessions.csv` made in @ref LaneOptions::log_directory.
  /// @param device     The run's device; it and @p allocator outlive the lane.
  /// @param source     Where its sessions come from; null: those added alone.
  /// @param options    The grace, the settle, the back-off and the log's
  ///                   directory.
  /// @return The lane, @ref enabled; `Status::Code::InvalidArgument` for a
  ///         settle of 0 or not below the grace, or a back-off or a hop's
  ///         seed not positive; `Status::Code::IoError` for a reaper's thread
  ///         that cannot start.
  static vkc::Result<std::unique_ptr<MobileLane>> create(
      vkc::Device& device, vkc::Allocator& allocator,
      std::unique_ptr<SessionSource> source, LaneOptions options);
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

  /// @brief After a set's send: a session the source has ready, if one is,
  ///        and those added since the last gap join (@ref add); each joined
  ///        session's presence is read and its record due by
  ///        @p next_set_ns taken (above); then the gap's scheduler (above)
  ///        classes the records, keeps one hop of them all and, within a
  ///        live lane's @p budget, the preps, and puts the rest back; each
  ///        record kept
  ///        is prepared and its due fits recorded against @p grid as the set
  ///        left it, in one batch of its session's own, the batches
  ///        submitted in join order and then waited here in join order, each
  ///        session then judging, placing and logging. Once a session's
  ///        phone is exhausted, its tracker is flushed and what it placed
  ///        still drains; a session that retired then leaves.
  /// @param grid         The run's model, not written until this returns.
  /// @param next_set_ns  The next set's predicted time, on the rig's clock.
  /// @param budget       What the gap may spend (@ref GapBudget); a file's
  ///                     lane reads none.
  /// @return The next send's fillers, at most @ref kDrainCap, one a session
  ///         in turn (above), each session's oldest first; a frame the prep
  ///         or the tracker refuses is passed over and counted, as is a
  ///         filler @ref check_filler refuses. Only the device's loss
  ///         returns, or a file's session's own error (above).
  vkc::Result<std::vector<volumetric_kit::recon::sender::Filler>> next(
      const volumetric_kit::grid::volume::VoxelBlockGrid& grid,
      std::uint64_t next_set_ns, GapBudget budget = {});

  /// @brief The run's end: a live lane's source closed (its door and opener
  ///        gone); then each session still in the run, in join order,
  ///        retires: its tag watch's last news taken (a hold its thread is
  ///        solving then, one search of 10 to 55 ms, is not logged), its
  ///        tracker flushed (held frames decided, placed or dropped, and
  ///        logged, not fused); then it leaves through the reaper, as above,
  ///        its log kept, and this waits for every leave. Idempotent.
  /// @return OK; or the device's loss.
  vkc::Status close();

  /// @return Whether the lane runs a gap at all: a live lane, phones or
  ///         none, or one with a session; not the empty lane.
  bool enabled() const noexcept;

  /// @return Whether the lane is live (@ref create's second form): the one
  ///         whose gaps read a @ref GapBudget, which `run_live_fusion` gives
  ///         it alone; a file's lane, or the empty lane, reads none.
  bool live() const noexcept;

  /// @return What the lane's sessions did so far, together.
  Counts counts() const noexcept;

  /// @return Each session, in join order, those added since the last gap
  ///         last.
  std::vector<SessionView> sessions() const;

  /// @return Its lines for the run's report: each session's counts and its
  ///         phone's counters, in join order, those that left included, each
  ///         named by its ordinal and name when there are more than one or
  ///         the lane is live, a session that came and went with its join
  ///         and its leave in seconds of set time from the lane's first
  ///         gap, and its deferrals (those of the budget among them); a
  ///         live lane's totals, its gap's deferrals, probes, hop estimate
  ///         (@ref HopEstimate) and last budgeted gap's room once a session
  ///         joined, and its source's lines after; empty for the empty lane.
  std::string report() const;

  /// @return The first session's log, as @ref SessionView::log; null for the
  ///         empty lane.
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

/// @brief The run's lane: with `source.mobile` a file, its one session; or
///        `tcp://ADDRESS:PORT`, a live lane whose phones join and leave
///        (above), returned at once, the door bound or binding; without it,
///        the empty lane. Each session is read at confidence 2 or more
///        (`link::RgbdStreamSensor`), its tracker on @ref lane_tracking and
///        its log in `diagnostics.track_log` (a live session's in a
///        directory of its own). Placed by `source.mobile_registration`,
///        read depth alone; or by the origin tag, read with its pictures: a
///        @ref TagWatch in `source.calibration`'s world, on its own thread
///        (every lane runs beside a live rig), a live phone told how the
///        search goes. A file read so decodes its pictures in
///        @ref MobileLane::next, on the set loop (about 1 ms at p95 a gap,
///        measured on an RTX 5090): a replay's cost, not a live run's, whose
///        phones decode on their poll threads. With `diagnostics.track_log`,
///        a live phone is recorded (`phone.vrst`) and the tag's
///        registration written (`registration.json`), so each session
///        replays.
/// @param options  A live lane's grace, settle and back-off; its log
///                 directory is `diagnostics.track_log`'s (above).
/// @return The lane. A file: @ref check_mobile's refusal; with the tag,
///         `Status::Code::InvalidArgument` for a `diagnostics.track_log`
///         that holds a `registration.json` already (an earlier run's), or
///         for a session with no pictures; or the reader's, the
///         registration's, the watch's or the tracker's error. Live, what
///         would refuse a file turns its phones away instead (above), and a
///         `diagnostics.track_log` that holds an earlier run's logs or
///         cannot be made is printed and written to nowhere; only the
///         device's loss returns, `Status::Code::InvalidArgument` for
///         @p options a live lane refuses, or `Status::Code::IoError` for a
///         thread that cannot start.
vkc::Result<std::unique_ptr<MobileLane>> open_mobile_lane(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator,
    LaneOptions options = {});

}  // namespace vr_example
