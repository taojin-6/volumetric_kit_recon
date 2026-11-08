// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "live_mobile.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "interrupt.hpp"
#include "link/rgbd_stream_sensor.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"

namespace vr_example {

namespace sender = volumetric_kit::recon::sender;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace vol = volumetric_kit::grid::volume;
using volumetric_kit::recon::Mat4f;

// How long a run waits for its live phone to dial.
constexpr std::chrono::minutes kPhoneWait{5};

track::TrackerConfig lane_tracking() {
  track::TrackerConfig c;
  c.driver = track::window_method();
  c.drivers = {{"rig", tsdf::rig_cast()}, {"track", tsdf::tracking_cast()}};
  c.shadows = {track::device_method()};
  return c;
}

sender::Filler to_filler(track::Released&& released, std::uint32_t index) {
  const bool rig_anchored =
      released.placement == track::Placement::Anchored && released.driver == 0;
  sender::Filler f;
  f.frame = track::corrected(std::move(released.frame), released.correction);
  f.tier = rig_anchored ? tsdf::FillTier::Anchored : tsdf::FillTier::Unanchored;
  f.index = index;
  return f;
}

gnet::ViewerNote lane_note(const MobileLane::Status& status,
                           std::uint64_t set_ns) {
  using Tone = gnet::ViewerNote::Tone;
  gnet::ViewerNote note;
  if (status.registered) {
    note.tone = Tone::Good;
    note.line = "Tag found, registered";
  } else if (status.refused_ns != 0 &&
             set_ns < status.refused_ns + kRefusedShownNs) {
    note.tone = Tone::Caution;
    note.line = "Tag refused, trying again";
  } else if (status.gathered != 0) {
    note.tone = Tone::Good;
    note.line = "Tag seen: " + std::to_string(status.gathered) + " of " +
                std::to_string(kHoldStills);
  } else {
    note.tone = Tone::Neutral;
    note.line = "Looking for the tag";
  }
  return note;
}

bool note_due(const gnet::ViewerNote& now,
              const std::optional<gnet::ViewerNote>& last,
              std::uint64_t since_ns) {
  return !last || now.tone != last->tone || now.line != last->line ||
         since_ns >= kNoteAtLeastNs;
}

vkc::Status check_mobile(const config::Config& opt) {
  if (!opt.source.mobile) return {};
  if (opt.source.depth == config::DepthSource::Ffs) {
    return vkc::Status::unsupported(
        "source.mobile: the phone's lane does not run beside FFS depth "
        "(source.depth ffs) yet");
  }
  if (!opt.fusion.dynamic) {
    return vkc::Status::invalid_argument(
        "source.mobile needs fusion.dynamic: the phone's frames fill only "
        "under Dynamic fusion");
  }
  if (opt.operation == config::Operation::Stream) {
    return vkc::Status::invalid_argument(
        "source.mobile is not run by operation stream, which fuses nothing");
  }
  if (opt.source.mobile->registration.empty()) {
    // Registered by the origin tag: this build must find it, and the
    // calibration's world must be a tag.
    VKC_ASSIGN(const sensor::camera::ArrayCalibration rig,
               sensor::camera::read_array_calibration(opt.source.calibration));
    VKC_TRY(TagWatch::check(rig.world).with_context(
        "source.mobile without source.mobile_registration"));
  }
  return {};
}

// --- The background poll ----------------------------------------------------

namespace {

class PolledInBackground final : public sensor::IRgbdSensor {
 public:
  explicit PolledInBackground(std::unique_ptr<sensor::IRgbdSensor> inner)
      : inner_(std::move(inner)) {}
  ~PolledInBackground() override { stop(); }
  PolledInBackground(const PolledInBackground&) = delete;
  PolledInBackground& operator=(const PolledInBackground&) = delete;

  const sensor::SensorInfo& info() const noexcept override {
    return inner_->info();
  }
  vkc::Status set_queue_depth(std::size_t frames) override {
    if (frames == 0 || thread_.joinable()) {
      return vkc::Status::invalid_argument(
          "polled_in_background: set_queue_depth takes 1 or more, before the "
          "start");
    }
    depth_ = frames;
    return {};
  }
  // The inner sensor is started here and then used by the polling thread
  // alone until stop joins it: one thread at a time, as it requires.
  vkc::Status start() override {
    if (thread_.joinable()) return {};
    VKC_TRY(inner_->start());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = false;
      ended_ = false;
      error_ = {};
      queue_.clear();
      dropped_ = 0;
      stats_ = inner_->stats();
    }
    thread_ = std::thread([this] { run(); });
    return {};
  }
  void stop() noexcept override {
    if (!thread_.joinable()) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    idle_.notify_all();
    thread_.join();
    inner_->stop();
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
  }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    std::lock_guard<std::mutex> lock(mutex_);
    VKC_TRY(take_error());
    if (queue_.empty()) return std::optional<sensor::RgbdFrame>{};
    dropped_ += queue_.size() - 1;  // the newest only
    sensor::RgbdFrame f = std::move(queue_.back());
    queue_.clear();
    ++delivered_;
    return std::optional<sensor::RgbdFrame>(std::move(f));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    VKC_CHECK(out != nullptr, "polled_in_background: drain into nothing");
    std::lock_guard<std::mutex> lock(mutex_);
    VKC_TRY(take_error());
    for (sensor::RgbdFrame& f : queue_) out->push_back(std::move(f));
    delivered_ += queue_.size();
    queue_.clear();
    return {};
  }
  bool exhausted() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    return ended_ && queue_.empty() && error_.ok();
  }
  sensor::SensorStats stats() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::SensorStats s = stats_;
    s.delivered = delivered_;
    s.dropped += dropped_;
    return s;
  }

 private:
  // An error the inner poll returned, once; then OK.
  vkc::Status take_error() {
    vkc::Status e = std::move(error_);
    error_ = {};
    return e;
  }

  void run() {
    for (;;) {
      vkc::Result<std::optional<sensor::RgbdFrame>> polled = inner_->poll();
      const bool ended = inner_->exhausted();
      std::unique_lock<std::mutex> lock(mutex_);
      stats_ = inner_->stats();
      if (!polled.ok()) {
        if (error_.ok()) error_ = polled.status();
      } else if (*polled) {
        // A full queue lets its oldest go, so the inner link is always read.
        while (queue_.size() >= depth_) {
          queue_.pop_front();
          ++dropped_;
        }
        queue_.push_back(std::move(**polled));
      } else if (ended) {
        ended_ = true;
        return;
      } else {
        // A poll that does not wait of its own: wait a little before the next.
        idle_.wait_for(lock, std::chrono::milliseconds(1),
                       [this] { return stopping_; });
      }
      if (stopping_) return;
    }
  }

  std::unique_ptr<sensor::IRgbdSensor> inner_;
  std::size_t depth_ = 1;
  mutable std::mutex mutex_;
  std::condition_variable idle_;
  std::deque<sensor::RgbdFrame> queue_;
  vkc::Status error_;
  sensor::SensorStats stats_{};
  std::uint64_t delivered_ = 0;
  std::uint64_t dropped_ = 0;
  bool stopping_ = false;
  bool ended_ = false;
  std::thread thread_;
};

}  // namespace

std::unique_ptr<sensor::IRgbdSensor> polled_in_background(
    std::unique_ptr<sensor::IRgbdSensor> sensor) {
  return std::make_unique<PolledInBackground>(std::move(sensor));
}

// --- The lane ---------------------------------------------------------------

namespace {

// `c` counted into `to`, field by field: a field added to Counts is added
// here too, or this fails.
static_assert(sizeof(MobileLane::Counts) == 13 * sizeof(std::uint64_t),
              "add_to sums every field of MobileLane::Counts");
void add_to(MobileLane::Counts& to, const MobileLane::Counts& c) {
  to.polled += c.polled;
  to.paired += c.paired;
  to.skipped += c.skipped;
  to.passed_over += c.passed_over;
  to.splits += c.splits;
  to.errors += c.errors;
  to.anchored += c.anchored;
  to.unanchored += c.unanchored;
  to.dropped += c.dropped;
  to.fused += c.fused;
  to.unregistered += c.unregistered;
  to.shed += c.shed;
  to.unfit += c.unfit;
}

// One phone's session: what the lane was before it held sessions, but the
// tints, which the lane keeps for them all.
struct PhoneSession {
  vkc::Device* device = nullptr;
  vkc::Allocator* allocator = nullptr;
  std::unique_ptr<sensor::IRgbdSensor> phone;
  std::optional<sensor::GpuFramePrep> prep;
  std::optional<vkc::GpuTimer> timer;
  // The tracker goes after the log, which its rows are named by.
  std::unique_ptr<track::Tracker> tracker;
  std::unique_ptr<TrackLog> log;
  // The origin tag's watch: let go before the phone stops.
  std::unique_ptr<TagWatch> tags;
  // TODO: read at each gap once a session leaves (away while it reads
  // false, retired after a grace), which phones that come and go need.
  std::function<bool()> presence;
  std::string name;
  MobileLane::Placement placement = MobileLane::Placement::Registration;
  std::uint32_t ordinal = 0;  // its join's place, from 1; 0 until it joins
  std::uint32_t index = 0;    // its mobile index, from its join
  bool by_tag = false;
  bool placed = true;  // false until the tag registers the phone
  sensor::camera::Mat4d tracking_to_world{1.0};  // the tag's registration
  // The newest paired frame: its time, and its colour camera in the phone's
  // own world, where the gaps are measured.
  std::optional<std::uint64_t> first_paired;
  sensor::camera::Mat4d newest_tracking{1.0};
  std::optional<Mat4f> rig_correction;  // the rig chain's, after its last fit
  bool rig_anchored = false;            // its first Anchored fit printed
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
  // The records taken from the phone, oldest first, not yet due.
  std::deque<sensor::RgbdFrame> held;
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
  bool ended = false;    // the phone is exhausted and the tracker flushed
  bool closed = false;
  MobileLane::Counts counts;
  std::string first_refusal;  // why the first frame passed over was
  std::string first_error;    // the first error the phone returned
  std::string first_unfit;    // why the first filler let go was

  PhoneSession() = default;
  ~PhoneSession() {
    tags.reset();  // its thread may hold a picture of the phone's
    if (phone) phone->stop();
  }
  PhoneSession(const PhoneSession&) = delete;
  PhoneSession& operator=(const PhoneSession&) = delete;

  // Done: its phone exhausted, its tracker flushed and what it placed handed
  // over, for good.
  bool done() const { return ended && queue.empty(); }

  // Whether it holds its index: joined, and not done.
  bool holding() const { return started && !done(); }

  // At its join: its place and index, and the phone started.
  vkc::Status join(std::uint32_t at, std::uint32_t mobile_index) {
    // What the link held while the cameras opened comes in the first takes,
    // passed by but for each take's newest.
    VKC_TRY(phone->set_queue_depth(kPhoneQueue));
    VKC_TRY(phone->start());
    started = true;
    ordinal = at;
    index = mobile_index;
    return {};
  }

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
  void watch_rig(const track::TrackStep& step) {
    for (const track::ChainFit& f : step.fits) {
      if (f.chain != 0 || !f.refused.empty()) continue;  // the rig chain's
      rig_correction = f.judged.correction;
      if (!by_tag || rig_anchored ||
          f.judged.state != track::TrackState::Anchored) {
        continue;
      }
      rig_anchored = true;
      const Mat4f at = registered_camera();
      const auto [mm, deg] = pose_gap(f.judged.correction * at, at);
      std::printf(
          "phone: first rig fit: %.1f cm / %.1f deg from the tag's "
          "registration\n",
          mm / 10.0, deg);
    }
  }

  // The session registered by the tag's first hold that passed: frames are
  // placed from now on, and the run's log and registration file say so.
  vkc::Status register_by(const sensor::camera::TrackedRegistration& reg,
                          const TagHold& hold) {
    placed = true;
    tracking_to_world = reg.tracking_to_world;
    registered_after = hold.last_pts_ns - first_paired.value_or(0);
    std::string where = "tag";
    if (!log->directory().empty()) {
      where = (std::filesystem::path(log->directory()) / "registration.json")
                  .string();
      VKC_TRY(sensor::camera::write_tracked_registration(
          where, reg, vkc::WriteMode::CreateNew));
    }
    return log->registered(std::move(where), Mat4f(reg.tracking_to_world));
  }

  // What the watch found: each hold logged and printed, the first that
  // passed registering the session, a passed one after it measured against
  // the rig chain; the first failure printed, every one counted.
  vkc::Status apply(TagNews news) {
    for (const std::string& f : news.failures) {
      if (!first_tag_failure.empty()) break;
      first_tag_failure = f;
      std::printf("phone: tag search failed (counted, the run goes on): %s\n",
                  f.c_str());
    }
    for (const TagHold& h : news.holds) {
      VKC_TRY(log->write(h));
      if (h.use == TagHold::Use::Refused) {
        refused_ns = set_ns;
        std::printf("phone: tag hold refused: %s: %s\n", h.miss.c_str(),
                    h.reason.c_str());
        continue;
      }
      std::string outcome = "registered";
      if (h.use == TagHold::Use::Registered) {
        VKC_CHECK(news.registered.has_value(),
                  "MobileLane: a registering hold without its registration");
        VKC_TRY(register_by(*news.registered, h));
      } else if (!rig_correction) {
        outcome = "passed, no rig fit yet (logged)";
      } else {
        const auto [mm, deg] =
            pose_gap(Mat4f(h.tracking_to_world * newest_tracking),
                     *rig_correction * registered_camera());
        char buf[96];
        std::snprintf(buf, sizeof(buf),
                      "passed, %.1f cm / %.1f deg from the rig chain (logged)",
                      mm / 10.0, deg);
        outcome = buf;
      }
      std::printf(
          "phone: tag hold: %u stills over %.1f s, RMS %.1f px, worst %.1f "
          "px, %.1f m, %.0f deg off normal -> %s\n",
          h.stills, double(h.last_pts_ns - h.first_pts_ns) * 1e-9, h.rms_px,
          h.worst_px, h.range_m, h.off_normal_deg, outcome.c_str());
    }
    tag_counts = tags->counts();
    return {};
  }

  // Where the tag search stands: the one source of the words.
  MobileLane::Status status() const {
    return {by_tag && placed, tag_counts.gathered, refused_ns};
  }

  // The words said when they are due.
  void tell() {
    if (!say) return;
    gnet::ViewerNote now = lane_note(status(), set_ns);
    if (!note_due(now, said, set_ns - said_ns)) return;
    say(now);
    said = std::move(now);
    said_ns = set_ns;
  }

  // A step's placements into the queue, its oldest let go past the cap; its
  // counts, and the log.
  vkc::Status take(const track::TrackStep& step) {
    watch_rig(step);
    VKC_TRY(log->write(step));
    for (const track::Released& r : step.released) {
      track::Released copy = r;
      sender::Filler f = to_filler(std::move(copy), index);
      ++(f.tier == tsdf::FillTier::Anchored ? counts.anchored
                                            : counts.unanchored);
      queue.push_back(std::move(f));
      if (queue.size() > kPlacedCap) {
        queue.pop_front();
        ++counts.shed;
      }
    }
    counts.dropped += step.dropped_ns.size();
    return {};
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
  bool take_records(std::uint64_t next_set) {
    std::vector<sensor::RgbdFrame> got;
    const vkc::Status st = phone->drain(&got);
    if (!st.ok() && counts.errors++ == 0) first_error = st.message();
    if (!got.empty()) {
      const sensor::RgbdFrame& newest = got.back();
      const std::int64_t pin =
          std::int64_t(newest.timestamp_ns) - std::int64_t(next_set);
      if (!offset || (newest.arrival_ns != 0 && pin > *offset)) offset = pin;
    }
    for (sensor::RgbdFrame& f : got) {
      ++counts.polled;
      if (last_seq && f.sequence > *last_seq + 1 &&
          f.timestamp_ns >= last_pts + kSplitGapNs) {
        splits.push_back(f.timestamp_ns);
      }
      last_seq = f.sequence;
      last_pts = f.timestamp_ns;
      held.push_back(std::move(f));
    }
    return !got.empty();
  }

  // The newest record due by the set at `next_set_ns`; older ones due are
  // passed by. Reads on to the first record not yet due, which waits.
  std::optional<sensor::RgbdFrame> pair(std::uint64_t next_set) {
    std::optional<sensor::RgbdFrame> due;
    for (;;) {
      if (held.empty() && !take_records(next_set)) break;
      if (std::int64_t(held.front().timestamp_ns) >
          std::int64_t(next_set) + *offset) {
        break;
      }
      if (due) ++counts.skipped;
      due = std::move(held.front());
      held.pop_front();
    }
    return due;
  }

  // A frame paired: shown to the tag's watch, then tracked once placed. Every
  // loss before it is one split (no frame between them was paired).
  vkc::Status paired(const vol::VoxelBlockGrid& grid, sensor::RgbdFrame frame) {
    bool split = false;
    while (!splits.empty() && splits.front() <= frame.timestamp_ns) {
      splits.pop_front();
      split = true;
    }
    if (!first_paired) first_paired = frame.timestamp_ns;
    newest_tracking = frame.color_to_world;
    if (tags) {
      if (split) tags->split();
      VKC_TRY(tags->see(frame));  // before track_one lets its picture go
    }
    if (!placed) {
      ++counts.unregistered;  // nothing tracked before the tag
      return {};
    }
    if (tags) frame.color_to_world = tracking_to_world * frame.color_to_world;
    return track_one(grid, std::move(frame), split);
  }

  // One phone frame: prepared and its due fits recorded in one batch,
  // waited on here, then judged, placed and logged; after a loss, the
  // tracker flushed first.
  vkc::Status track_one(const vol::VoxelBlockGrid& grid,
                        sensor::RgbdFrame frame, bool split) {
    if (split) {
      ++counts.splits;
      VKC_TRY(take(tracker->flush()));
    }
    // The phone's picture is never fused: fill fuses its tier's tint.
    frame.color = {};
    frame.color_packed = nullptr;
    frame.color_bitstream = {};
    vkc::CommandBatch batch(*device, *allocator);
    batch.time(*timer);
    vkc::StageMetrics& rows = log->rows();
    vkc::Result<sensor::DeviceFrame> prepared =
        prep->record(batch, frame, &rows);
    if (!prepared.ok()) {
      if (prepared.status().domain() != vkc::Status::Code::InvalidArgument) {
        return prepared.status();
      }
      passed_over(prepared.status());
      return {};
    }
    vkc::Result<track::PendingStep> step =
        tracker->record(batch, grid, std::move(*prepared), &rows);
    if (!step.ok()) {
      if (step.status().domain() != vkc::Status::Code::InvalidArgument) {
        return step.status();
      }
      passed_over(step.status());
      return {};  // the batch goes unsubmitted, the prep's slot with it
    }
    vkc::Status ran;
    {
      vkc::Result<vkc::PendingBatch> submitted = batch.submit_async({}, {});
      if (submitted.ok()) {
        {
          vkc::WaitScope wait(&rows, "wait: mobile");
          ran = submitted->wait();
        }
        if (ran.ok()) rows.merge_device(submitted->device_rows());
      } else {
        ran = submitted.status();
      }
    }  // a failed batch is out of flight once its pending batch is gone
    if (!ran.ok()) {
      tracker->abandon(*step);
      return ran;
    }
    VKC_ASSIGN(const track::TrackStep done, tracker->finish(*step));
    ++counts.paired;
    return take(done);
  }

  // The phone is done: what the tracker holds is decided now.
  vkc::Status end() {
    if (ended) return {};
    ended = true;
    return take(tracker->flush());
  }

  // A gap, once joined: the watch's news, then the record due by the set at
  // `next_set_ns` paired; once the phone is exhausted, the tracker flushed.
  vkc::Status gap(const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns) {
    set_ns = next_set_ns;
    if (tags) {
      VKC_ASSIGN(TagNews news, tags->take());
      VKC_TRY(apply(std::move(news)));
    }
    if (!ended) {
      std::optional<sensor::RgbdFrame> due = pair(next_set_ns);
      if (due) VKC_TRY(paired(grid, std::move(*due)));
      if (held.empty() && phone->exhausted()) VKC_TRY(end());
    }
    return {};
  }

  // The run's end: the watch's last news, then the watch let go before the
  // phone stops; its error comes back after the rest of the close, which
  // still runs.
  vkc::Status close() {
    if (closed) return {};
    closed = true;
    vkc::Status tagged;
    if (tags) {
      vkc::Result<TagNews> news = tags->take();
      tagged = news.ok() ? apply(std::move(news).value()) : news.status();
      tags.reset();
    }
    VKC_TRY(end());  // placed, not fused: the run is over
    phone->stop();
    VKC_TRY(log->close());
    VKC_ASSIGN(const std::string table,
               summarize(log->run(), log->directory()));
    std::fputs(table.c_str(), stdout);
    return tagged;
  }

  // Its lines of the run's report, the first beginning with `label`.
  std::string report(const std::string& label) const {
    const MobileLane::Counts& c = counts;
    const sensor::SensorStats st = phone->stats();
    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        "%s: %llu records (%llu received, %llu failed), %llu tracked, %llu "
        "skipped, %llu passed over, %llu splits; fill %llu anchored, %llu "
        "unanchored, %llu dropped, %llu fused, %zu waiting\n",
        label.c_str(), static_cast<unsigned long long>(c.polled),
        static_cast<unsigned long long>(st.received),
        static_cast<unsigned long long>(st.failed),
        static_cast<unsigned long long>(c.paired),
        static_cast<unsigned long long>(c.skipped),
        static_cast<unsigned long long>(c.passed_over),
        static_cast<unsigned long long>(c.splits),
        static_cast<unsigned long long>(c.anchored),
        static_cast<unsigned long long>(c.unanchored),
        static_cast<unsigned long long>(c.dropped),
        static_cast<unsigned long long>(c.fused), queue.size());
    std::string out = buf;
    if (by_tag) {
      const TagWatch::Counts& t = tag_counts;
      char when[96];
      if (registered_after) {
        std::snprintf(when, sizeof(when),
                      "registered at %.1f s, %llu frames before it",
                      double(*registered_after) * 1e-9,
                      static_cast<unsigned long long>(c.unregistered));
      } else {
        std::snprintf(when, sizeof(when), "not registered, %llu frames",
                      static_cast<unsigned long long>(c.unregistered));
      }
      std::snprintf(buf, sizeof(buf),
                    "  tag: %llu searched, %llu stills, %llu holds (%llu "
                    "refused), %llu replaced, %llu failed; %s\n",
                    static_cast<unsigned long long>(t.candidates),
                    static_cast<unsigned long long>(t.stills),
                    static_cast<unsigned long long>(t.holds),
                    static_cast<unsigned long long>(t.refused),
                    static_cast<unsigned long long>(t.replaced),
                    static_cast<unsigned long long>(t.failed), when);
      out += buf;
    }
    if (!first_refusal.empty()) {
      out += "  first passed over: " + first_refusal + "\n";
    }
    if (c.errors != 0) {
      out += "  " + std::to_string(c.errors) +
             " phone error(s), the first: " + first_error + "\n";
    }
    if (c.shed != 0) {
      out += "  " + std::to_string(c.shed) + " placed let go past " +
             std::to_string(kPlacedCap) + " waiting\n";
    }
    if (c.unfit != 0) {
      out += "  " + std::to_string(c.unfit) +
             " placed the send would refuse, the first: " + first_unfit + "\n";
    }
    return out;
  }
};

bool finite(const Mat4f& m) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(m[c][r])) return false;
    }
  }
  return true;
}

}  // namespace

vkc::Status check_filler(const sender::Filler& filler) {
  if (filler.index >= tsdf::kMaxSourcesPerClass) {
    return vkc::Status::invalid_argument(
        "filler: mobile index " + std::to_string(filler.index) +
        " is past the source byte's " +
        std::to_string(tsdf::kMaxSourcesPerClass) + " sensors a class");
  }
  const sensor::DepthCameraParams& d = filler.frame.depth_camera;
  if (!std::isfinite(d.fx) || !std::isfinite(d.fy) || !std::isfinite(d.cx) ||
      !std::isfinite(d.cy) || !std::isfinite(d.min_depth) ||
      !std::isfinite(d.max_depth) || !finite(d.cam_to_world)) {
    return vkc::Status::invalid_argument(
        "filler: its depth camera has a non-finite intrinsic, depth bound or "
        "pose entry");
  }
  const sensor::ColorCameraParams& c = filler.frame.color_camera;
  if (filler.frame.has_color() &&
      (!std::isfinite(c.fx) || !std::isfinite(c.fy) || !std::isfinite(c.cx) ||
       !std::isfinite(c.cy) || !finite(c.cam_to_world))) {
    return vkc::Status::invalid_argument(
        "filler: its colour camera has a non-finite intrinsic or pose entry");
  }
  return {};
}

struct MobileLane::State {
  vkc::Device* device = nullptr;
  vkc::Allocator* allocator = nullptr;
  // The sessions in join order, those added since the last gap last.
  std::vector<std::unique_ptr<PhoneSession>> sessions;
  std::uint32_t joins = 0;     // sessions joined so far
  std::size_t drain_next = 0;  // the session the next turn of the drain is
  // The tint pictures, by tier, at each depth size a filler has had.
  std::map<std::pair<std::uint32_t, std::uint32_t>,
           std::array<std::shared_ptr<const vkc::Buffer>, 2>>
      tints;
  bool closed = false;

  // The index of the session joining as `ordinal`: (ordinal - 1) mod 63,
  // past any a session holds; none while all are held.
  std::optional<std::uint32_t> free_index(std::uint32_t ordinal) const {
    std::array<bool, tsdf::kMaxSourcesPerClass> held{};
    for (const std::unique_ptr<PhoneSession>& s : sessions) {
      if (s->holding()) held[s->index] = true;
    }
    for (std::uint32_t k = 0; k < tsdf::kMaxSourcesPerClass; ++k) {
      const std::uint32_t i = static_cast<std::uint32_t>(
          (std::uint64_t{ordinal} - 1 + k) % tsdf::kMaxSourcesPerClass);
      if (!held[i]) return i;
    }
    return std::nullopt;
  }

  // The sessions added since the last gap, in turn, each with an index free.
  vkc::Status adopt() {
    for (const std::unique_ptr<PhoneSession>& s : sessions) {
      if (s->started) continue;
      const std::optional<std::uint32_t> index = free_index(joins + 1);
      if (!index) break;  // waits for a session to be done
      VKC_TRY(s->join(joins + 1, *index));
      ++joins;
    }
    return {};
  }

  // A filler coloured by its tier's tint, through a colour camera of its
  // depth camera's geometry; the tints at its depth size made on the device
  // once.
  vkc::Status tint(sender::Filler& f) {
    const sensor::DepthCameraParams& d = f.frame.depth_camera;
    std::array<std::shared_ptr<const vkc::Buffer>, 2>& made =
        tints[{d.width, d.height}];
    if (!made[0]) {
      const VkDeviceSize bytes = VkDeviceSize(d.width) * d.height * 4;
      vkc::CommandBatch batch(*device, *allocator);
      const std::uint32_t words[2] = {kAnchoredFillTint, kUnanchoredFillTint};
      std::array<std::shared_ptr<const vkc::Buffer>, 2> fresh;
      for (int t = 0; t < 2; ++t) {
        VKC_ASSIGN(vkc::Buffer b,
                   vkc::device_storage_buffer(*allocator, bytes));
        VKC_TRY(batch.fill(b, 0, bytes, words[t]));
        fresh[t] = std::make_shared<const vkc::Buffer>(std::move(b));
      }
      VKC_TRY(batch.submit());
      made = std::move(fresh);
    }
    f.frame.color = made[f.tier == tsdf::FillTier::Anchored ? 0 : 1];
    f.frame.color_camera = {d.fx,    d.fy,     d.cx,          d.cy,
                            d.width, d.height, d.cam_to_world};
    f.frame.color_encoding = {};
    return {};
  }

  // The next send's fillers: one a session in turn, in join order from
  // where the last send's turns stopped, kDrainCap in all; a session with
  // none waiting gives its turn away, and a filler the send would refuse is
  // let go.
  vkc::Status drain(std::vector<sender::Filler>* out) {
    const std::size_t n = sessions.size();
    for (std::size_t idle = 0; out->size() < kDrainCap && idle < n;) {
      PhoneSession& s = *sessions[drain_next];
      drain_next = (drain_next + 1) % n;
      if (s.queue.empty()) {
        ++idle;
        continue;
      }
      idle = 0;
      sender::Filler f = std::move(s.queue.front());
      s.queue.pop_front();
      const vkc::Status fit = check_filler(f);
      if (!fit.ok()) {
        s.unfit(fit);
        continue;
      }
      VKC_TRY(tint(f));
      ++s.counts.fused;
      out->push_back(std::move(f));
    }
    return {};
  }
};

MobileLane::MobileLane(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}
MobileLane::~MobileLane() = default;

vkc::Result<std::unique_ptr<MobileLane>> MobileLane::create(
    vkc::Device& device, vkc::Allocator& allocator,
    std::unique_ptr<sensor::IRgbdSensor> phone,
    std::unique_ptr<track::Tracker> tracker, std::unique_ptr<TrackLog> log,
    std::unique_ptr<TagWatch> tags, std::function<void(gnet::ViewerNote)> say) {
  auto s = std::make_unique<State>();
  s->device = &device;
  s->allocator = &allocator;
  std::unique_ptr<MobileLane> lane(new MobileLane(std::move(s)));
  if (phone) {
    if (!tracker || !log) {
      return vkc::Status::invalid_argument(
          "MobileLane::create: a phone needs its tracker and its log");
    }
    SessionParts parts;
    parts.name = phone->info().id;
    parts.placement = tags ? Placement::Tag : Placement::Registration;
    parts.phone = std::move(phone);
    parts.tracker = std::move(tracker);
    parts.log = std::move(log);
    parts.tags = std::move(tags);
    parts.say = std::move(say);
    VKC_TRY(lane->add(std::move(parts)));
  }
  return lane;
}

vkc::Status MobileLane::add(SessionParts parts) {
  State& st = *state_;
  if (st.closed) {
    return vkc::Status::invalid_argument("MobileLane::add: the lane is closed");
  }
  if (!parts.phone || !parts.tracker || !parts.log) {
    return vkc::Status::invalid_argument(
        "MobileLane::add: a session needs its phone, its tracker and its log");
  }
  const bool by_tag = parts.placement == Placement::Tag;
  if (by_tag != (parts.tags != nullptr)) {
    return vkc::Status::invalid_argument(
        "MobileLane::add: a session placed by the tag needs its watch, and "
        "only one placed so takes one");
  }
  auto s = std::make_unique<PhoneSession>();
  s->device = st.device;
  s->allocator = st.allocator;
  VKC_ASSIGN(sensor::GpuFramePrep prep,
             sensor::GpuFramePrep::create(*st.device, *st.allocator));
  s->prep.emplace(std::move(prep));
  VKC_ASSIGN(vkc::GpuTimer timer, vkc::GpuTimer::create(*st.device));
  s->timer.emplace(std::move(timer));
  s->phone = std::move(parts.phone);
  s->tracker = std::move(parts.tracker);
  s->log = std::move(parts.log);
  s->by_tag = by_tag;
  s->placed = !by_tag;
  s->tags = std::move(parts.tags);
  if (by_tag) s->say = std::move(parts.say);
  s->presence = std::move(parts.presence);
  s->name = std::move(parts.name);
  s->placement = parts.placement;
  st.sessions.push_back(std::move(s));
  return {};
}

vkc::Result<std::vector<sender::Filler>> MobileLane::next(
    const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns) {
  State& st = *state_;
  std::vector<sender::Filler> out;
  if (st.closed) return out;
  VKC_TRY(st.adopt());
  for (const std::unique_ptr<PhoneSession>& s : st.sessions) {
    if (s->started) VKC_TRY(s->gap(grid, next_set_ns));
  }
  VKC_TRY(st.drain(&out));
  for (const std::unique_ptr<PhoneSession>& s : st.sessions) {
    if (!s->started) continue;
    s->tell();
    // Done, it prepares nothing more: its prep and timer go (a frame handed
    // out holds its own buffers), and their query pools with them, before
    // its watch and phone, which stay until the close: what lets those go
    // finds the prep and timer gone.
    if (s->done()) {
      s->prep.reset();
      s->timer.reset();
    }
  }
  return out;
}

vkc::Status MobileLane::close() {
  State& st = *state_;
  if (st.closed) return {};
  st.closed = true;
  vkc::Status first;
  for (const std::unique_ptr<PhoneSession>& s : st.sessions) {
    const vkc::Status closed = s->close();
    if (first.ok()) first = closed;
  }
  return first;
}

MobileLane::Counts MobileLane::counts() const noexcept {
  Counts total;
  for (const std::unique_ptr<PhoneSession>& s : state_->sessions) {
    add_to(total, s->counts);
  }
  return total;
}

std::vector<MobileLane::SessionView> MobileLane::sessions() const {
  std::vector<SessionView> out;
  for (const std::unique_ptr<PhoneSession>& s : state_->sessions) {
    SessionView v;
    v.name = s->name;
    v.placement = s->placement;
    v.ordinal = s->ordinal;
    v.index = s->index;
    v.holding = s->holding();
    v.waiting = s->queue.size();
    v.counts = s->counts;
    v.log = s->log.get();
    out.push_back(std::move(v));
  }
  return out;
}

const TrackLog* MobileLane::log() const noexcept {
  return state_->sessions.empty() ? nullptr
                                  : state_->sessions.front()->log.get();
}

std::string MobileLane::report() const {
  const std::vector<std::unique_ptr<PhoneSession>>& sessions = state_->sessions;
  if (sessions.size() == 1) return sessions.front()->report("mobile");
  std::string out;
  for (const std::unique_ptr<PhoneSession>& s : sessions) {
    char label[64];
    if (s->ordinal != 0) {
      std::snprintf(label, sizeof(label), "mobile %02u",
                    static_cast<unsigned>(s->ordinal));
    } else {
      std::snprintf(label, sizeof(label), "mobile --");
    }
    out += s->report(std::string(label) + " (" + s->name + ")");
  }
  return out;
}

vkc::Result<std::unique_ptr<MobileLane>> open_mobile_lane(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator) {
  if (!opt.source.mobile)
    return MobileLane::create(device, allocator, {}, {}, {});
  VKC_TRY(check_mobile(opt));  // every refusal before any wait
  const config::MobileSession& m = *opt.source.mobile;
  const std::filesystem::path dir = opt.diagnostics.track_log;
  const bool by_tag = m.registration.empty();
  link::RgbdStreamSensor::Options options;
  options.min_confidence = 2;  // the phone's high, whatever it fused at
  RunSource source;
  source.device = device.caps().properties().deviceName;
  sensor::camera::WorldFrame world;
  if (by_tag) {
    VKC_ASSIGN(const sensor::camera::ArrayCalibration rig,
               sensor::camera::read_array_calibration(opt.source.calibration));
    world = rig.world;
    options.device = &device;  // the pictures, for the tag
    options.allocator = &allocator;
    source.registration = "tag";  // until the watch registers it
    std::error_code ignored;
    if (!dir.empty() &&
        std::filesystem::exists(dir / "registration.json", ignored)) {
      return vkc::Status::invalid_argument(
          "diagnostics.track_log: " + dir.string() +
          " holds an earlier run's registration.json: give each run its own "
          "directory");
    }
  } else {
    VKC_ASSIGN(options.registration,
               sensor::camera::read_tracked_registration(m.registration));
    source.registration = m.registration;
    source.tracking_to_world = Mat4f(options.registration->tracking_to_world);
  }
  std::unique_ptr<link::RgbdStreamSensor> stream;
  if (m.host.empty()) {
    source.session = m.path;
    VKC_ASSIGN(stream, link::RgbdStreamSensor::open_file(m.path, options));
  } else {
    source.session = "tcp://" + m.host + ":" + std::to_string(m.port);
    // With a log, the phone is recorded beside it from its first session,
    // so the run replays; listen refuses a recording already there.
    std::string record;
    if (!dir.empty()) {
      std::error_code made;
      std::filesystem::create_directories(dir, made);
      if (made) {
        return vkc::Status::io_error("diagnostics.track_log: cannot make " +
                                     dir.string() + ": " + made.message());
      }
      record = (dir / "phone.vrst").string();
    }
    std::printf("waiting for the phone to dial %s (Ctrl+C ends the wait)\n",
                source.session.c_str());
    // A second at a time, so a Ctrl+C ends the wait; the empty lane then
    // lets the run return before its cameras open.
    const auto until = std::chrono::steady_clock::now() + kPhoneWait;
    for (;;) {
      if (vgrid_example::interrupted()) {
        return MobileLane::create(device, allocator, {}, {}, {});
      }
      vkc::Result<std::unique_ptr<link::RgbdStreamSensor>> heard =
          link::RgbdStreamSensor::listen(m.host, m.port, options, record,
                                         std::chrono::seconds(1));
      if (heard.ok()) {
        stream = std::move(*heard);
        break;
      }
      // Only the wait running out is waited through again.
      const bool waited =
          heard.status().domain() == vkc::Status::Code::IoError &&
          heard.status().message().find("no session within the wait") !=
              std::string::npos;
      if (!waited || std::chrono::steady_clock::now() >= until) {
        return heard.status();
      }
    }
  }
  // The tag's watch, for the session that came: it needs its pictures.
  std::unique_ptr<TagWatch> tags;
  if (by_tag) {
    const std::string id = stream->info().id;
    if (!stream->info().color) {
      return vkc::Status::invalid_argument(
          "source.mobile: " + source.session + ": sensor " + id +
          " sends no pictures, which the origin tag is found in: give "
          "source.mobile_registration");
    }
    const std::array<std::uint8_t, 16>& session = stream->session_id();
    char hex[9];
    std::snprintf(hex, sizeof(hex), "%02x%02x%02x%02x", session[0], session[1],
                  session[2], session[3]);
    std::printf(
        "phone: session %s… (%s): registering by the origin tag, %s id "
        "%d, at its printed edge %.4g mm\n",
        hex, id.c_str(), world.apriltag->family.c_str(), world.apriltag->id,
        world.apriltag->size_m * 1e3);
    VKC_ASSIGN(tags,
               TagWatch::create(device, allocator,
                                {id, session, world, /*background=*/true}));
  }
  // A live phone on the tag is told how the search goes. The sensor stays where
  // it is inside the background poll, and the lane says nothing after its
  // close, before the phone stops.
  std::function<void(gnet::ViewerNote)> say;
  if (by_tag && !m.host.empty()) {
    say = [reader = stream.get()](gnet::ViewerNote note) {
      reader->say(std::move(note));
    };
  }
  std::unique_ptr<sensor::IRgbdSensor> phone;
  if (m.host.empty()) {
    phone = std::move(stream);
  } else {
    phone = polled_in_background(std::move(stream));
  }
  const track::TrackerConfig tracking = lane_tracking();
  VKC_ASSIGN(std::vector<std::unique_ptr<track::IFitter>> fitters,
             track::create_fitters(device, allocator, tracking));
  VKC_ASSIGN(std::unique_ptr<track::Tracker> tracker,
             track::Tracker::create(tracking, std::move(fitters)));
  VKC_ASSIGN(
      std::unique_ptr<TrackLog> log,
      TrackLog::open(opt.diagnostics.track_log, *tracker, std::move(source)));
  return MobileLane::create(device, allocator, std::move(phone),
                            std::move(tracker), std::move(log), std::move(tags),
                            std::move(say));
}

}  // namespace vr_example
