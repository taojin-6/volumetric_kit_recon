// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/live_mobile_internal.hpp
/// @brief The phone lane's own types, shared by its sources alone: not
///        installed and not an API (the lane's is live_mobile.hpp). A
///        session (`detail::PhoneSession`), the reaper that lets one go,
///        what the gap's scheduler classes a record as, and the errors
///        the lane tells apart. live_mobile.cpp holds the lane and the
///        gap's scheduler, live_mobile_session.cpp a session's work and
///        its leave, live_mobile_open.cpp the open path (`check_mobile`,
///        `open_mobile_lane`, a live run's door, opener and back-off).

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "live_mobile.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"

namespace vr_example {

namespace sender = volumetric_kit::recon::sender;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace vol = volumetric_kit::grid::volume;
using volumetric_kit::recon::Mat4f;

namespace detail {

using Stage = MobileLane::Stage;

// What the gap's scheduler classes a session's record as, before anything of
// it is shown or tracked.
enum class Kind : std::uint8_t {
  None,          // nothing due, or the session is not to pair this gap
  Unregistered,  // by the tag, not yet registered: shown to its watch alone
  Prep,          // placed, no fit due
  Hop,           // placed, a fit due
};

// Under FillColour::Picture, a frame's picture prepared in a batch of its
// own, submitted and not yet waited: the whole frame, which the batch reads
// (the phone's decoded picture), the batch, the picture as the prep hands it
// out (its depth let go) and its pending submit.
struct PictureInFlight {
  PictureInFlight(vkc::Device& device, vkc::Allocator& allocator)
      : batch(device, allocator) {}
  sensor::RgbdFrame frame;
  vkc::CommandBatch batch;
  sensor::DeviceFrame picture;
  vkc::PendingBatch pending;  // last: it goes first, waiting for its work
};

// A session's record prepared and its fits recorded into a batch of its own,
// submitted and not yet waited: the batch, its pending submit and the step;
// and, when its picture is kept, the picture's own.
struct InFlight {
  InFlight(vkc::Device& device, vkc::Allocator& allocator)
      : batch(device, allocator) {}
  vkc::CommandBatch batch;
  track::PendingStep step;
  vkc::PendingBatch pending;  // it goes before the batch, waiting
  std::unique_ptr<PictureInFlight> picture;  // goes first, waiting for its own
};

// Whether `s` is the device's loss, the one error a lane returns.
bool device_lost(const vkc::Status& s);

// Whether `s` is memory running out, the device's or the host's.
bool ran_out_of_memory(const vkc::Status& s);

// The counts' line of a report, beginning with `label`.
std::string counts_line(const std::string& label, const MobileLane::Counts& c,
                        const sensor::SensorStats& st, std::size_t waiting);

// One phone's session: what the lane was before it held sessions, but the
// tints, which the lane keeps for them all. The lane's thread alone uses it
// until it is handed to the reaper (Stage::Reaping), whose it is then, but
// for what the lane reads of a session gone: its stage, counts and lines,
// which the reaper never writes.
struct PhoneSession {
  vkc::Device* device = nullptr;
  vkc::Allocator* allocator = nullptr;
  std::unique_ptr<sensor::IRgbdSensor> phone;
  std::optional<sensor::GpuFramePrep> prep;
  std::optional<vkc::GpuTimer> timer;
  // The tracker goes after the log, which its rows are named by.
  std::unique_ptr<track::Tracker> tracker;
  std::unique_ptr<TrackLog> log;
  // The origin tag's watch, shown frames until the session is placed: let go
  // before the phone stops.
  std::unique_ptr<TagWatch> tags;
  // Whether the phone is connected: a live phone's, which comes and goes;
  // let go, with the say and the error's hearer, as the session goes to the
  // reaper.
  std::function<bool()> presence;
  std::function<void(const vkc::Status&)> on_error;
  bool lives = false;  // given a presence: it comes and goes
  bool seen = false;   // its presence read connected since its join
  std::string name;
  MobileLane::Placement placement = MobileLane::Placement::Registration;
  std::atomic<Stage> stage{Stage::Added};
  std::uint32_t ordinal = 0;  // its join's place, from 1; 0 until it joins
  std::uint32_t index = 0;    // its mobile index, from its join
  bool by_tag = false;
  bool placed = true;       // false until the tag registers the phone
  bool log_failed = false;  // a write of its log failed, printed once
  sensor::camera::Mat4d tracking_to_world{1.0};  // the tag's registration
  // The first paired frame's time, and the newest's colour camera in the
  // phone's own world, where the first rig fit's gap is measured.
  std::optional<std::uint64_t> first_paired;
  sensor::camera::Mat4d newest_tracking{1.0};
  bool rig_anchored = false;  // the rig chain's first Anchored fit printed
  std::optional<std::uint64_t> registered_after;  // the first frame to it
  TagWatch::Counts tag_counts;                    // as of the last take
  std::string first_tag_failure;
  std::uint64_t set_ns = 0;      // the set the newest gap came before
  std::uint64_t refused_ns = 0;  // the set time of the newest refused hold
  // What the phone is told, with the tag: the last note, and its set time.
  std::function<void(gnet::ViewerNote)> say;
  std::optional<gnet::ViewerNote> said;
  std::uint64_t said_ns = 0;
  // The frames placed, not yet handed to a send: kPlacedCap at most.
  std::deque<sender::Filler> queue;
  // The records taken from the phone, oldest first, not yet due, or put back
  // by the gap's scheduler.
  std::deque<sensor::RgbdFrame> held;
  // The record due this gap, taken by its host work and not yet paired.
  std::optional<sensor::RgbdFrame> due;
  // The gaps the hop due now has waited for the gap's one hop.
  std::uint32_t waited = 0;
  // The phone's time less the rig's: pinned by the first take's newest
  // record, moved on by a live take's (take_records).
  std::optional<std::int64_t> offset;
  // The last record taken, for the loss test.
  std::optional<std::uint64_t> last_seq;
  std::uint64_t last_pts = 0;
  // The losses, oldest first: the tracker is flushed before the first frame
  // at or after each.
  std::deque<std::uint64_t> splits;
  bool started = false;  // the phone, at its join
  bool ended = false;    // the phone is done or it retired; the tracker flushed
  // The device's loss the phone returned, for the gap to return.
  vkc::Status lost;
  // Set by the reaper once its phone is gone, and with it the door's slot.
  std::atomic<bool> released{false};
  // Coming and going, in set time: Away since; flushed once in this spell.
  std::uint64_t away_since = 0;
  bool settled = false;
  std::uint64_t joined_ns = 0;  // the set its join came before
  std::uint64_t left_ns = 0;    // the set it was handed to the reaper at
  std::string leave_reason;     // why it retired
  // The phone's counters as it was handed to the reaper, which lets it go.
  sensor::SensorStats last_stats{};
  MobileLane::Counts counts;
  std::string first_refusal;  // why the first frame passed over was
  std::string first_error;    // the first error the phone returned
  std::string first_unfit;    // why the first filler let go was
  // What its fillers carry (the lane's). Under FillColour::Picture: its
  // picture prep; the frames its tracker took, and the count of them from
  // which the next picture is due (passed on to the next frame while due);
  // the pictures prepared for frames the tracker holds, by the frame's time,
  // each attached to its filler as the frame is placed and let go as it is
  // dropped; and the pictures the prep refused.
  FillOptions fill;
  std::optional<sensor::GpuFramePrep> picture_prep;
  std::uint64_t tracked = 0;
  std::uint64_t next_picture = 0;
  std::map<std::uint64_t, sensor::DeviceFrame> pictures;
  std::uint64_t pictures_refused = 0;
  std::string first_picture_refusal;

  PhoneSession() = default;
  ~PhoneSession() {
    tags.reset();  // its thread may hold a picture of the phone's
    if (phone) phone->stop();
  }
  PhoneSession(const PhoneSession&) = delete;
  PhoneSession& operator=(const PhoneSession&) = delete;

  // Whether it comes and goes: a live phone's, given a presence.
  bool comes_and_goes() const { return lives; }

  // In the run and pairing: joined, not retired.
  bool active() const {
    const Stage s = stage.load(std::memory_order_acquire);
    return s == Stage::Live || s == Stage::Away;
  }

  // Done: its phone done or the session retired, its tracker flushed and
  // what it placed handed over, for good.
  bool done() const { return ended && queue.empty(); }

  // Whether it holds its index: joined, and not done.
  bool holding() const { return started && !done(); }

  // Its name in the run's lines: its ordinal, its session's first 8 hex
  // digits where known, and its sensor.
  std::string label() const;

  // At its join: its place and index, then the phone started.
  vkc::Status join(std::uint32_t at, std::uint32_t mobile_index,
                   std::uint64_t now_ns);

  // A write of its log or its registration: one that fails is printed once,
  // and the session goes on, since the log never changes what fuses.
  void log_written(const vkc::Status& written);

  void passed_over(const vkc::Status& why) {
    if (counts.passed_over++ == 0) first_refusal = why.message();
  }

  void unfit(const vkc::Status& why) {
    if (counts.unfit++ == 0) first_unfit = why.message();
  }

  // The newest paired frame's camera as the tag registered it.
  Mat4f registered_camera() const {
    return Mat4f(tracking_to_world * newest_tracking);
  }

  // The rig chain's fits; on the tag, its first Anchored one prints how far
  // it moved the registration.
  void watch_rig(const track::TrackStep& step);

  // The session registered by the tag's first hold that passed: frames are
  // placed from now on, and the run's log and registration file say so.
  void register_by(const sensor::camera::TrackedRegistration& reg,
                   const TagHold& hold);

  // What the watch found: each hold logged and printed, the one that passed
  // registering the session; the first failure printed, every one counted.
  void apply(TagNews news);

  // The watch's news since the last gap; a watch that fails is the session's
  // error. It fails only before the registration, since it searches nothing
  // after it.
  vkc::Status take_news();

  // Where the tag search stands: the one source of the words.
  MobileLane::Status status() const {
    return {by_tag && placed, tag_counts.gathered, refused_ns};
  }

  // The words said when they are due.
  void tell();

  // A step's placements into the queue, each with its picture if one was
  // kept, its oldest let go past the cap; a dropped frame's picture let go;
  // its counts, and the log.
  void take(const track::TrackStep& step);

  // The tracker flushed (`take` of its flush): it holds nothing after, so
  // no picture is left waiting either.
  void flush();

  // Whether `frame`'s picture is kept for its fill: under
  // FillColour::Picture, the first frame the tracker takes and every
  // picture_every-th after the last kept; a due frame with no picture passes
  // its turn to the next. No frame tracked was shown to the tag's watch.
  bool picture_due(const sensor::RgbdFrame& frame) const;

  // `frame`'s picture prepared into a batch of its own, submitted into
  // `f.picture`; a refusal or failure returned for picture_refused, nothing
  // left in flight.
  vkc::Status begin_picture(InFlight& f, sensor::RgbdFrame frame);

  // `f`'s picture batch, if any, waited and its picture kept by its time
  // (counted); one that fails is let go (picture_refused). Only the device's
  // loss returns.
  vkc::Status finish_picture(InFlight& f);

  // A picture let go for `why`: counted, the first printed in the report.
  void picture_refused(const vkc::Status& why) {
    if (pictures_refused++ == 0) first_picture_refusal = why.message();
  }

  // The records the phone holds, taken in order for the set at `next_set`; a
  // loss is noted for its split. An error the phone returns is counted, and
  // the lane goes on. The first take pins the offset at its newest record,
  // so what waited in a live link while the cameras opened is passed by. A
  // live record (one with an arrival time) is due once it has come, so a
  // take whose newest is fresher than the offset says moves it on, never
  // back: a backlog that comes over several gaps does not leave the lane
  // behind. A recording's records are there whenever read, so only its
  // first take pins, and a replay pairs exactly.
  bool take_records(std::uint64_t next_set);

  // The newest record due by the set at `next_set_ns`; older ones due are
  // passed by. Reads on to the first record not yet due, which waits.
  std::optional<sensor::RgbdFrame> pair(std::uint64_t next_set);

  // What the record due is to the gap's scheduler: shown to the watch alone
  // before the tag registers the session; else a hop where the tracker would
  // fit at it (a split's flush first leaves its hops as they are), a prep
  // where not.
  Kind classify() const;

  // The record due put back, first, for a later gap: nothing of it shown or
  // tracked, so its split and its watch's sight of it wait with it; by the
  // gap's budget, or by the one hop a gap.
  void defer(bool by_budget);

  // The record due paired: shown to the tag's watch, then, once placed,
  // prepared and its due fits recorded in one batch of its own, submitted
  // and not waited (finish waits); after a loss, the tracker flushed first.
  // Every loss before it is one split (no frame between them was paired).
  // Nothing in flight: one shown to the watch alone, or passed over.
  vkc::Result<std::unique_ptr<InFlight>> begin(const vol::VoxelBlockGrid& grid);

  // `f`'s batch waited, then its step judged, placed and logged.
  vkc::Status finish(InFlight& f);

  // What the tracker holds decided now: the phone is done, or the session
  // retired.
  void end();

  // Retired, for `why`: its watch's last news taken and its tracker
  // flushed; from now on it pairs and says nothing. A session that comes
  // and goes says so. Only the device's loss returns.
  vkc::Status retire(std::string why);

  // A gap's host work, joined and not retired: its presence (one that comes
  // and goes), the watch's news, then the record due by the set at
  // `next_set_ns` taken (`due`), for the gap's scheduler. One that comes and
  // goes may retire here (the grace), with nothing due. An error returned is
  // the session's own, for the lane to retire it by, or the device's loss.
  vkc::Status host(std::uint64_t next_set_ns, const LaneOptions& options);

  // After the gap's batches: the phone's end, nothing held or put back,
  // retires the session, as the grace does; a file's places what it holds,
  // which still drains, and leaves at the close. Only the device's loss
  // returns.
  vkc::Status at_end();

  // Retired: the phone's queue taken and let go, its decoded pictures with
  // it, and nothing paired. Only the device's loss returns.
  vkc::Status discard();

  // Its lines of the run's report, the first beginning with `label`; its
  // join and leave in seconds from `origin_ns`, the lane's first gap.
  std::string report(const std::string& label, std::uint64_t origin_ns) const;

  // Its row of sessions.csv: a join's, or a leave's with its counts and
  // reason.
  std::string row(bool join) const;
};

// What lets a session go: release, then its log closed and its summary
// printed, either's error printed and passed by. Its log goes too unless
// `keep_log`. The reaper's thread runs it, or the lane's at a file's close.
void let_go(PhoneSession& s, bool keep_log);

// The lane's reaper: a thread that lets the sessions handed to it go
// (let_go), in turn, and the one writer of sessions.csv, whose join rows
// the lane's thread queues to it.
class Reaper {
 public:
  // `csv_path`: sessions.csv, made at its first row; empty, none. Its thread
  // started here: std::system_error if it cannot be.
  explicit Reaper(std::string csv_path) : csv_path_(std::move(csv_path)) {
    thread_ = std::thread([this] { run(); });
  }
  ~Reaper() { finish(); }
  Reaper(const Reaper&) = delete;
  Reaper& operator=(const Reaper&) = delete;

  // A join's row, written in turn.
  void join_row(std::string row) { push({nullptr, false, std::move(row)}); }

  // `s` let go, then its leave's row written and its stage Left. The lane
  // no longer touches what the reaper does.
  void reap(PhoneSession* s, bool keep_log) { push({s, keep_log, {}}); }

  // Every job done, the thread joined. Idempotent.
  void finish();

 private:
  struct Job {
    PhoneSession* session = nullptr;  // null: a join's row alone
    bool keep_log = false;
    std::string row;
  };

  void push(Job job);

  void run();

  // `row` written, sessions.csv made with its header (and its directory) at
  // the first, never over anything already there; a file that cannot be
  // made or written is printed, and written no more.
  void write(const std::string& row);

  std::ofstream csv_;        // the thread's
  bool csv_failed_ = false;  // the thread's
  std::string csv_path_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Job> jobs_;   // under mutex_
  bool stopping_ = false;  // under mutex_
  std::thread thread_;     // last: started once the rest is
};

}  // namespace detail
}  // namespace vr_example
