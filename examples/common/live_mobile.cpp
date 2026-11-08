// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "live_mobile.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"

namespace vr_example {

namespace sender = volumetric_kit::recon::sender;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace vol = volumetric_kit::grid::volume;
using volumetric_kit::recon::Mat4f;

namespace stream = volumetric_kit::grid::stream;

// How often a live lane tries its address again while it cannot bind it,
// and how often it says again that it is not listening.
constexpr std::chrono::seconds kBindRetry{2};
constexpr std::chrono::seconds kBindReminder{60};
// How long a live lane admits no session after the device ran out of memory
// in one.
constexpr std::chrono::seconds kOutOfMemoryPause{60};
// The longest the opener waits for an arrival a pass, so a close is seen.
constexpr std::chrono::milliseconds kOpenerWait{100};
// A phone's back-off grows to this many times LaneOptions::error_back_off.
constexpr int kBackOffMaxDoublings = 5;

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

HopEstimate::HopEstimate(std::chrono::nanoseconds seed) noexcept {
  samples_.fill(seed);
}

void HopEstimate::add(std::chrono::nanoseconds wall) noexcept {
  samples_[next_] = wall;
  next_ = (next_ + 1) % kHopSamples;
}

void HopEstimate::probed(std::chrono::nanoseconds wall,
                         std::chrono::nanoseconds room) noexcept {
  if (wall <= room) {
    for (std::chrono::nanoseconds& s : samples_) {
      if (s > room) s = wall;
    }
  }
  add(wall);
}

std::chrono::nanoseconds HopEstimate::p90() const noexcept {
  // Nearest rank: the ceil(0.9 n)-th smallest.
  constexpr std::size_t kRank = (kHopSamples * 9 + 9) / 10 - 1;
  std::array<std::chrono::nanoseconds, kHopSamples> sorted = samples_;
  std::nth_element(sorted.begin(), sorted.begin() + kRank, sorted.end());
  return sorted[kRank];
}

namespace {

// What the run's settings refuse of any phone, a file's or a live one's.
vkc::Status settings_refusal(const config::Config& opt) {
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
  return {};
}

// Whether the origin tag can place a phone: this build finds it, and the
// calibration's world is it; then that world.
vkc::Result<sensor::camera::WorldFrame> tag_world(const config::Config& opt) {
  VKC_ASSIGN(const sensor::camera::ArrayCalibration rig,
             sensor::camera::read_array_calibration(opt.source.calibration));
  VKC_TRY(TagWatch::check(rig.world));
  return rig.world;
}

}  // namespace

vkc::Status check_mobile(const config::Config& opt) {
  // A live one's refusals turn its phones away (open_mobile_lane).
  if (!opt.source.mobile || !opt.source.mobile->host.empty()) return {};
  VKC_TRY(settings_refusal(opt));
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

using Stage = MobileLane::Stage;

// `c` counted into `to`, field by field: a field added to Counts is added
// here too, or this fails.
static_assert(sizeof(MobileLane::Counts) == 15 * sizeof(std::uint64_t),
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
  to.deferred += c.deferred;
  to.over_budget += c.over_budget;
}

// What the gap's scheduler classes a session's record as, before anything of
// it is shown or tracked.
enum class Kind : std::uint8_t {
  None,          // nothing due, or the session is not to pair this gap
  Unregistered,  // by the tag, not yet registered: shown to its watch alone
  Prep,          // placed, no fit due
  Hop,           // placed, a fit due
};

// A session's record prepared and its fits recorded into a batch of its own,
// submitted and not yet waited: the batch, its pending submit and the step.
struct InFlight {
  InFlight(vkc::Device& device, vkc::Allocator& allocator)
      : batch(device, allocator) {}
  vkc::CommandBatch batch;
  track::PendingStep step;
  vkc::PendingBatch pending;  // last: it goes first, waiting for its work
};

// Whether `s` is the device's loss, the one error a lane returns.
bool device_lost(const vkc::Status& s) {
  const std::optional<VkResult> r = vkc::vk_result(s);
  return r && *r == VK_ERROR_DEVICE_LOST;
}

// Whether `s` is memory running out, the device's or the host's.
bool ran_out_of_memory(const vkc::Status& s) {
  if (s.domain() == vkc::Status::Code::OutOfMemory) return true;
  const std::optional<VkResult> r = vkc::vk_result(s);
  return r && (*r == VK_ERROR_OUT_OF_DEVICE_MEMORY ||
               *r == VK_ERROR_OUT_OF_HOST_MEMORY);
}

// A session id's 32 hex digits.
std::string hex32(const std::array<std::uint8_t, 16>& id) {
  constexpr char kDigits[] = "0123456789abcdef";
  std::string out;
  for (const std::uint8_t byte : id) {
    out += kDigits[byte >> 4];
    out += kDigits[byte & 0xf];
  }
  return out;
}

// A CSV field: a comma or a line break made harmless.
std::string field(std::string s) {
  std::replace(s.begin(), s.end(), ',', '_');
  std::replace(s.begin(), s.end(), '\n', ' ');
  std::replace(s.begin(), s.end(), '\r', ' ');
  return s;
}

// The counts' line of a report, beginning with `label`.
std::string counts_line(const std::string& label, const MobileLane::Counts& c,
                        const sensor::SensorStats& st, std::size_t waiting) {
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
      static_cast<unsigned long long>(c.fused), waiting);
  return buf;
}

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
  // The origin tag's watch: let go before the phone stops.
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
  bool placed = true;         // false until the tag registers the phone
  bool watch_failed = false;  // its watch died after the registration
  bool log_failed = false;    // a write of its log failed, printed once
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
  std::string label() const {
    char nn[16];
    std::snprintf(nn, sizeof(nn), "%02u", static_cast<unsigned>(ordinal));
    const std::string& id = log->run().source.session_id;
    return std::string(nn) + " " + (id.empty() ? name : id.substr(0, 8)) +
           " (" + name + ")";
  }

  // At its join: its place and index, then the phone started.
  vkc::Status join(std::uint32_t at, std::uint32_t mobile_index,
                   std::uint64_t now_ns) {
    ordinal = at;
    index = mobile_index;
    joined_ns = now_ns;
    set_ns = now_ns;
    stage.store(Stage::Live, std::memory_order_release);
    log_written(log->joined(at, mobile_index, now_ns));
    // What the link held while the cameras opened, or before a late join,
    // comes in the first takes, passed by but for each take's newest.
    VKC_TRY(phone->set_queue_depth(kPhoneQueue));
    VKC_TRY(phone->start());
    started = true;
    return {};
  }

  // A write of its log or its registration: one that fails is printed once,
  // and the session goes on, since the log never changes what fuses.
  void log_written(const vkc::Status& written) {
    if (written.ok() || log_failed) return;
    log_failed = true;
    std::fprintf(stderr,
                 "warning: phone %s: a write of its log failed (%s); the "
                 "session goes on\n",
                 name.c_str(), written.message().c_str());
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
  void register_by(const sensor::camera::TrackedRegistration& reg,
                   const TagHold& hold) {
    placed = true;
    tracking_to_world = reg.tracking_to_world;
    registered_after = hold.last_pts_ns - first_paired.value_or(0);
    std::string where = "tag";
    if (!log->directory().empty()) {
      where = (std::filesystem::path(log->directory()) / "registration.json")
                  .string();
      const vkc::Status written = sensor::camera::write_tracked_registration(
          where, reg, vkc::WriteMode::CreateNew);
      log_written(written);
      if (!written.ok()) where = "tag";
    }
    log_written(
        log->registered(std::move(where), Mat4f(reg.tracking_to_world)));
  }

  // What the watch found: each hold logged and printed, the first that
  // passed registering the session, a passed one after it measured against
  // the rig chain; the first failure printed, every one counted.
  void apply(TagNews news) {
    for (const std::string& f : news.failures) {
      if (!first_tag_failure.empty()) break;
      first_tag_failure = f;
      std::printf("phone: tag search failed (counted, the run goes on): %s\n",
                  f.c_str());
    }
    for (const TagHold& h : news.holds) {
      log_written(log->write(h));
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
        register_by(*news.registered, h);
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
  }

  // The watch's news since the last gap. A watch that fails before the
  // session registers is the session's error; after it, it is let be, and
  // the session goes on placed. The device's loss returns either way.
  vkc::Status take_news() {
    if (!tags || watch_failed) return {};
    vkc::Result<TagNews> news = tags->take();
    if (news.ok()) {
      apply(std::move(news).value());
      return {};
    }
    if (device_lost(news.status()) || !placed) return news.status();
    watch_failed = true;
    std::fprintf(stderr,
                 "warning: phone %s: its tag watch failed after the "
                 "registration (%s); the session goes on placed\n",
                 name.c_str(), news.status().message().c_str());
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
  void take(const track::TrackStep& step) {
    watch_rig(step);
    log_written(log->write(step));
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
    if (device_lost(st) && lost.ok()) lost = st;
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

  // What the record due is to the gap's scheduler: shown to the watch alone
  // before the tag registers the session; else a hop where the tracker would
  // fit at it (a split's flush first leaves its hops as they are), a prep
  // where not.
  Kind classify() const {
    if (!due) return Kind::None;
    if (!placed) return Kind::Unregistered;
    return tracker->fit_due(due->timestamp_ns) ? Kind::Hop : Kind::Prep;
  }

  // The record due put back, first, for a later gap: nothing of it shown or
  // tracked, so its split and its watch's sight of it wait with it; by the
  // gap's budget, or by the one hop a gap.
  void defer(bool by_budget) {
    held.push_front(std::move(*due));
    due.reset();
    ++counts.deferred;
    if (by_budget) ++counts.over_budget;
  }

  // The record due paired: shown to the tag's watch, then, once placed,
  // prepared and its due fits recorded in one batch of its own, submitted
  // and not waited (finish waits); after a loss, the tracker flushed first.
  // Every loss before it is one split (no frame between them was paired).
  // Nothing in flight: one shown to the watch alone, or passed over.
  vkc::Result<std::unique_ptr<InFlight>> begin(
      const vol::VoxelBlockGrid& grid) {
    VKC_CHECK(due.has_value(), "MobileLane: a session begun with nothing due");
    sensor::RgbdFrame frame = std::move(*due);
    due.reset();
    bool split = false;
    while (!splits.empty() && splits.front() <= frame.timestamp_ns) {
      splits.pop_front();
      split = true;
    }
    if (!first_paired) first_paired = frame.timestamp_ns;
    newest_tracking = frame.color_to_world;
    if (tags && !watch_failed) {
      if (split) tags->split();
      VKC_TRY(tags->see(frame));  // before its picture is let go below
    }
    if (!placed) {
      ++counts.unregistered;  // nothing tracked before the tag
      return std::unique_ptr<InFlight>();
    }
    if (tags) frame.color_to_world = tracking_to_world * frame.color_to_world;
    if (split) {
      ++counts.splits;
      take(tracker->flush());
    }
    // The phone's picture is never fused: fill fuses its tier's tint.
    frame.color = {};
    frame.color_packed = nullptr;
    frame.color_bitstream = {};
    auto f = std::make_unique<InFlight>(*device, *allocator);
    f->batch.time(*timer);
    vkc::StageMetrics& rows = log->rows();
    vkc::Result<sensor::DeviceFrame> prepared =
        prep->record(f->batch, frame, &rows);
    if (!prepared.ok()) {
      if (prepared.status().domain() != vkc::Status::Code::InvalidArgument) {
        return prepared.status();
      }
      passed_over(prepared.status());
      return std::unique_ptr<InFlight>();
    }
    vkc::Result<track::PendingStep> step =
        tracker->record(f->batch, grid, std::move(*prepared), &rows);
    if (!step.ok()) {
      if (step.status().domain() != vkc::Status::Code::InvalidArgument) {
        return step.status();
      }
      passed_over(step.status());
      // The batch goes unsubmitted, the prep's slot with it.
      return std::unique_ptr<InFlight>();
    }
    f->step = *step;
    vkc::Result<vkc::PendingBatch> submitted = f->batch.submit_async({}, {});
    if (!submitted.ok()) {
      tracker->abandon(f->step);  // none of it runs
      return submitted.status();
    }
    f->pending = std::move(submitted).value();
    return f;
  }

  // `f`'s batch waited, then its step judged, placed and logged.
  vkc::Status finish(InFlight& f) {
    vkc::StageMetrics& rows = log->rows();
    vkc::Status ran;
    {
      vkc::WaitScope wait(&rows, "wait: mobile");
      ran = f.pending.wait();
    }
    if (!ran.ok()) {
      f.pending = vkc::PendingBatch();  // out of flight once it is gone
      tracker->abandon(f.step);
      return ran;
    }
    rows.merge_device(f.pending.device_rows());
    VKC_ASSIGN(const track::TrackStep done, tracker->finish(f.step));
    ++counts.paired;
    take(done);
    return {};
  }

  // What the tracker holds decided now: the phone is done, or the session
  // retired.
  void end() {
    if (ended) return;
    ended = true;
    take(tracker->flush());
  }

  // Retired, for `why`: its watch's last news taken and its tracker
  // flushed; from now on it pairs and says nothing. A session that comes
  // and goes says so. Only the device's loss returns.
  vkc::Status retire(std::string why) {
    const Stage now = stage.load(std::memory_order_acquire);
    if (now == Stage::Retiring || now == Stage::Reaping || now == Stage::Left) {
      return {};
    }
    leave_reason = std::move(why);
    stage.store(Stage::Retiring, std::memory_order_release);
    due.reset();
    if (comes_and_goes()) {
      std::printf("phones: %s retires: %s\n", label().c_str(),
                  leave_reason.c_str());
    }
    const vkc::Status news = take_news();
    end();
    return device_lost(news) ? news : vkc::Status{};
  }

  // A gap's host work, joined and not retired: its presence (one that comes
  // and goes), the watch's news, then the record due by the set at
  // `next_set_ns` taken (`due`), for the gap's scheduler. One that comes and
  // goes may retire here (the grace), with nothing due. An error returned is
  // the session's own, for the lane to retire it by, or the device's loss.
  vkc::Status host(std::uint64_t next_set_ns, const LaneOptions& options) {
    set_ns = next_set_ns;
    if (comes_and_goes()) {
      const bool here = presence();
      const Stage now = stage.load(std::memory_order_acquire);
      if (here && now == Stage::Away) {
        stage.store(Stage::Live, std::memory_order_release);
        std::printf("phones: %s is back after %.1f s away\n", label().c_str(),
                    double(set_ns - away_since) * 1e-9);
      } else if (!here && now == Stage::Live &&
                 (seen || set_ns >= joined_ns + kFirstSeenNs)) {
        // One never seen connected is away from its join.
        stage.store(Stage::Away, std::memory_order_release);
        away_since = seen ? set_ns : joined_ns;
        settled = false;
        std::printf(
            "phones: %s away; it retires at %.0f s away unless it comes "
            "back\n",
            label().c_str(), double(options.grace_ns) * 1e-9);
      }
      seen = seen || here;
      if (stage.load(std::memory_order_acquire) == Stage::Away) {
        const std::uint64_t away =
            set_ns > away_since ? set_ns - away_since : 0;
        if (away >= options.grace_ns) {
          char why[64];
          std::snprintf(why, sizeof(why), "away %.0f s", double(away) * 1e-9);
          return retire(why);
        }
        if (away >= options.settle_ns && !settled) {
          settled = true;  // what it holds decided, as no later frame shares
          take(tracker->flush());
        }
      }
    }
    VKC_TRY(take_news());
    due = pair(next_set_ns);
    if (!lost.ok()) return lost;
    return {};
  }

  // After the gap's batches: the phone's end, nothing held or put back,
  // retires the session, as the grace does; a file's places what it holds,
  // which still drains, and leaves at the close. Only the device's loss
  // returns.
  vkc::Status at_end() {
    if (!held.empty() || !phone->exhausted()) return {};
    return retire(comes_and_goes() ? "its stream ended" : "its file ended");
  }

  // Retired: the phone's queue taken and let go, its decoded pictures with
  // it, and nothing paired. Only the device's loss returns.
  vkc::Status discard() {
    held.clear();
    due.reset();
    std::vector<sensor::RgbdFrame> got;
    if (!phone || !started) return {};
    const vkc::Status st = phone->drain(&got);
    return device_lost(st) ? st : vkc::Status{};
  }

  // Its lines of the run's report, the first beginning with `label`; its
  // join and leave in seconds from `origin_ns`, the lane's first gap.
  std::string report(const std::string& label, std::uint64_t origin_ns) const {
    const MobileLane::Counts& c = counts;
    const Stage now = stage.load(std::memory_order_acquire);
    const bool gone = now == Stage::Reaping || now == Stage::Left;
    const sensor::SensorStats st = gone ? last_stats : phone->stats();
    std::string out = counts_line(label, c, st, queue.size());
    char buf[512];
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
    if (c.deferred != 0) {
      out += "  " + std::to_string(c.deferred) +
             " deferred to a later gap (one hop a gap; " +
             std::to_string(c.over_budget) + " by the gap's budget)\n";
    }
    if (comes_and_goes() && !leave_reason.empty()) {
      const auto in = [origin_ns](std::uint64_t ns) {
        return double(ns > origin_ns ? ns - origin_ns : 0) * 1e-9;
      };
      std::snprintf(buf, sizeof(buf),
                    "  joined %.1f s in, %s %.1f s in: ", in(joined_ns),
                    now == Stage::Left ? "left" : "retired",
                    in(now == Stage::Left ? left_ns : set_ns));
      out += buf + leave_reason + "\n";
    }
    return out;
  }

  // Its row of sessions.csv: a join's, or a leave's with its counts and
  // reason.
  std::string row(bool join) const {
    const RunSource& src = log->run().source;
    const std::string directory =
        log->directory().empty()
            ? std::string()
            : std::filesystem::path(log->directory()).filename().string();
    std::string s = join ? "join," : "leave,";
    s += std::to_string(join ? joined_ns : left_ns) + "," +
         std::to_string(ordinal) + "," + field(src.session_id) + "," +
         field(name) + "," + field(src.peer) + "," + std::to_string(index) +
         "," + (by_tag ? "tag" : "registration") + "," + field(directory) + ",";
    if (join) return s + "0,0,0,\n";
    std::string why = leave_reason;
    std::replace(why.begin(), why.end(), '\n', ' ');
    std::replace(why.begin(), why.end(), '\r', ' ');
    return s + std::to_string(counts.polled) + "," +
           std::to_string(counts.paired) + "," + std::to_string(counts.fused) +
           "," + why + "\n";
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

// What a session holds let go, in the plan's order: its watch, then its
// phone (the link's slot frees as the sensor goes), then its tracker, prep
// and timer.
void release(PhoneSession& s) noexcept {
  s.tags.reset();  // its thread may hold a picture of the phone's
  if (s.phone) {
    s.phone->stop();
    s.phone.reset();
  }
  s.released.store(true, std::memory_order_release);
  s.tracker.reset();
  s.prep.reset();
  s.timer.reset();
}

// What lets a session go: release, then its log closed and its summary
// printed, either's error printed and passed by. Its log goes too unless
// `keep_log`. The reaper's thread runs it, or the lane's at a file's close.
void let_go(PhoneSession& s, bool keep_log) {
  release(s);
  if (!s.log) return;
  const vkc::Status closed = s.log->close();
  s.log_written(closed);
  vkc::Result<std::string> table = summarize(s.log->run(), s.log->directory());
  if (table.ok()) {
    std::fputs(table->c_str(), stdout);
  } else {
    s.log_written(table.status());
  }
  if (!keep_log) s.log.reset();
}

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
  void finish() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
  }

 private:
  struct Job {
    PhoneSession* session = nullptr;  // null: a join's row alone
    bool keep_log = false;
    std::string row;
  };

  void push(Job job) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      jobs_.push_back(std::move(job));
    }
    wake_.notify_one();
  }

  void run() {
    for (;;) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
        if (jobs_.empty()) return;  // stopping, every job done
        job = std::move(jobs_.front());
        jobs_.pop_front();
      }
      // Host memory running out on the way is printed, and the next job
      // goes on: no phone's trouble ends the run.
      if (job.session != nullptr) {
        PhoneSession& s = *job.session;
        try {
          // Its row before let_go may let its log go: the row reads it.
          job.row = s.row(/*join=*/false);
          let_go(s, job.keep_log);
        } catch (const std::exception& e) {
          release(s);
          std::fprintf(stderr, "warning: phones: letting %s go: %s\n",
                       s.name.c_str(), e.what());
        }
        s.stage.store(Stage::Left, std::memory_order_release);
      }
      try {
        write(job.row);
      } catch (const std::exception& e) {
        std::fprintf(stderr, "warning: phones: %s: %s\n", csv_path_.c_str(),
                     e.what());
      }
    }
  }

  // `row` written, sessions.csv made with its header (and its directory) at
  // the first, never over anything already there; a file that cannot be
  // made or written is printed, and written no more.
  void write(const std::string& row) {
    if (csv_path_.empty() || csv_failed_ || row.empty()) return;
    if (!csv_.is_open()) {
      const std::filesystem::path path(csv_path_);
      std::error_code ec;
      if (std::filesystem::exists(std::filesystem::symlink_status(path, ec))) {
        std::fprintf(stderr,
                     "warning: phones: %s is there already; this run writes "
                     "none\n",
                     csv_path_.c_str());
        csv_failed_ = true;
        return;
      }
      std::filesystem::create_directories(path.parent_path(), ec);
      csv_.open(path, std::ios::binary | std::ios::trunc);
      csv_ << "event,set_ns,ordinal,session_id,sensor,peer,mobile_index,"
              "placement,directory,polled,paired,fused,reason\n";
    }
    csv_ << row;
    csv_.flush();
    if (!csv_) {
      std::fprintf(stderr,
                   "warning: phones: a write of %s failed; it is written no "
                   "more\n",
                   csv_path_.c_str());
      csv_.close();
      csv_failed_ = true;
    }
  }

  std::ofstream csv_;        // the thread's
  bool csv_failed_ = false;  // the thread's
  std::string csv_path_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Job> jobs_;   // under mutex_
  bool stopping_ = false;  // under mutex_
  std::thread thread_;     // last: started once the rest is
};

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
  // The sessions in join order, those added since the last gap last; a
  // session stays here once it left, for its counts and lines.
  std::vector<std::unique_ptr<PhoneSession>> sessions;
  std::uint32_t joins = 0;     // sessions joined so far
  std::size_t drain_next = 0;  // the session the next turn of the drain is
  // The tint pictures, by tier, at each depth size a filler has had.
  std::map<std::pair<std::uint32_t, std::uint32_t>,
           std::array<std::shared_ptr<const vkc::Buffer>, 2>>
      tints;
  bool closed = false;
  // A live lane's: its options, its sessions' source and its reaper, which
  // goes before the sessions it reaps.
  bool live = false;
  LaneOptions options;
  std::unique_ptr<SessionSource> source;
  std::unique_ptr<Reaper> reaper;
  // The first gap's set, which a report's times count from.
  std::optional<std::uint64_t> origin_ns;
  // The sessions whose own error was printed, by id (or name), the oldest
  // forgotten past 64.
  std::deque<std::string> warned;
  // The gap's scheduler: the session the next contested tie for the hop
  // starts at; what a live gap's hop costs; the set of the first gap whose
  // budget held a hop back since a hop last ran, the probes run and the
  // newest budgeted gap's room; and, over the deferral window from its set,
  // the records classed a prep or a hop and those of them put back.
  std::size_t hop_next = 0;
  HopEstimate hop_cost;
  std::optional<std::uint64_t> held_since;
  std::uint64_t probes = 0;
  std::optional<std::chrono::nanoseconds> last_room;
  std::optional<std::uint64_t> window_ns;
  std::uint64_t window_classed = 0;
  std::uint64_t window_deferred = 0;
  std::uint64_t window_over_budget = 0;

  State() = default;
  // The source closed and the reaper's jobs done before the sessions go.
  ~State() {
    if (source) source->close();
    reaper.reset();
  }
  State(const State&) = delete;
  State& operator=(const State&) = delete;

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

  // The sessions in the run now, from their join until the reaper let their
  // phone go, which frees the door's slot: what a live join's line counts
  // against kMaxPhones.
  std::size_t in_run() const {
    std::size_t n = 0;
    for (const std::unique_ptr<PhoneSession>& s : sessions) {
      const Stage st = s->stage.load(std::memory_order_acquire);
      if (st == Stage::Live || st == Stage::Away || st == Stage::Retiring ||
          (st == Stage::Reaping &&
           !s->released.load(std::memory_order_acquire))) {
        ++n;
      }
    }
    return n;
  }

  // The sessions added since the last gap, in turn, each with an index free,
  // at the set `set_ns`: its phone started, a live lane's line printed and
  // its join's row queued. A phone that does not start retires its session.
  vkc::Status adopt(std::uint64_t set_ns) {
    for (const std::unique_ptr<PhoneSession>& s : sessions) {
      if (s->stage.load(std::memory_order_acquire) != Stage::Added) continue;
      const std::optional<std::uint32_t> index = free_index(joins + 1);
      if (!index) break;  // waits for a session to be done
      const vkc::Status started = s->join(joins + 1, *index, set_ns);
      ++joins;
      if (live) {
        const RunSource& src = s->log->run().source;
        const std::string by = s->by_tag                  ? "the tag"
                               : src.registration.empty() ? "its registration"
                                                          : src.registration;
        std::printf("phones: %s%s%s joined by %s; %zu of %zu\n",
                    s->label().c_str(), src.peer.empty() ? "" : " from ",
                    src.peer.c_str(), by.c_str(), in_run(), kMaxPhones);
        if (reaper) reaper->join_row(s->row(/*join=*/true));
      }
      if (!started.ok()) {
        VKC_TRY(contain(*s, started.with_context("its phone's start")));
      }
    }
    return {};
  }

  // A session's own error retires one that comes and goes alone, printed
  // once its id, and its hearer told; out of memory, the source admits
  // nothing for a while. The device's loss returns, as does a file's
  // session's error: a replay asked for that file.
  vkc::Status contain(PhoneSession& s, const vkc::Status& error) {
    if (error.ok()) return {};
    if (device_lost(error) || !s.comes_and_goes()) return error;
    if (ran_out_of_memory(error) && source) source->out_of_memory();
    const std::string& id = s.log->run().source.session_id;
    const std::string key = id.empty() ? s.name : id;
    if (std::find(warned.begin(), warned.end(), key) == warned.end()) {
      if (warned.size() >= 64) warned.pop_front();
      warned.push_back(key);
      std::fprintf(stderr, "warning: phone %s: %s; the session retires\n",
                   s.name.c_str(), error.message().c_str());
    }
    if (std::function<void(const vkc::Status&)> told =
            std::exchange(s.on_error, {})) {
      told(error);
    }
    return s.retire("its error: " + error.message());
  }

  // The gap's work (live_mobile.hpp, the gap's scheduler): each session in
  // the run does its host work, a retiring one's phone queue let go; each
  // record due is classed; one hop of them all is kept and, within a live
  // lane's `given` budget, every prep, the rest put back; the records kept
  // are begun in join order and finished in join order; each session's own
  // error is contained once every batch is waited; then each phone's end.
  // Only the device's loss returns, or a file's session's own error.
  vkc::Status gap(const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns,
                  const GapBudget& given) {
    // A file's lane reads no budget, so a replay repeats.
    const GapBudget budget = live ? given : GapBudget{};
    const std::size_t n = sessions.size();
    std::vector<bool> paired(n, false);  // its host work done, still in it
    for (std::size_t i = 0; i < n; ++i) {
      PhoneSession& s = *sessions[i];
      if (s.active()) {
        const vkc::Status hosted = s.host(next_set_ns, options);
        VKC_TRY(contain(s, hosted));
        paired[i] = hosted.ok() && s.active();  // not retired at its grace
      } else if (s.stage.load(std::memory_order_acquire) == Stage::Retiring) {
        s.set_ns = next_set_ns;
        if (s.comes_and_goes()) VKC_TRY(s.discard());
      }
    }
    // Classed before anything of a record is shown or tracked, so one put
    // back keeps its split and its watch's sight of it.
    std::vector<Kind> kind(n, Kind::None);
    std::size_t hops = 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (paired[i]) kind[i] = sessions[i]->classify();
      if (kind[i] == Kind::Hop) ++hops;
    }
    // One hop: the longest waited, ties in turn from hop_next; with a
    // budget's end, only if the estimate ends it by then (or as a probe, the
    // budget having held hops back kHopProbeNs with none run), and the preps
    // only before that end.
    const std::chrono::steady_clock::time_point chose =
        std::chrono::steady_clock::now();
    bool late = false;
    bool hop_fits = true;
    bool probe = false;
    if (budget.gap_end) {
      last_room = std::chrono::duration_cast<std::chrono::nanoseconds>(
          *budget.gap_end - chose);
      late = chose > *budget.gap_end;
      hop_fits = chose + hop_cost.p90() <= *budget.gap_end;
      if (!hop_fits && !late && hops != 0 && held_since &&
          next_set_ns >= *held_since + kHopProbeNs) {
        hop_fits = probe = true;
      }
    }
    std::size_t hop = n;  // none
    for (std::size_t k = 0; hop_fits && k < n; ++k) {
      const std::size_t i = (hop_next + k) % n;
      if (kind[i] == Kind::Hop &&
          (hop == n || sessions[i]->waited > sessions[hop]->waited)) {
        hop = i;
      }
    }
    if (hop == n && hops != 0 && !held_since) held_since = next_set_ns;
    for (std::size_t i = 0; i < n; ++i) {
      const bool hop_back = kind[i] == Kind::Hop && i != hop;
      const bool prep_back = kind[i] == Kind::Prep && late;
      if (kind[i] == Kind::Hop || kind[i] == Kind::Prep) ++window_classed;
      if (!hop_back && !prep_back) continue;
      const bool by_budget = prep_back || !hop_fits;
      if (hop_back) ++sessions[i]->waited;
      sessions[i]->defer(by_budget);
      kind[i] = Kind::None;
      ++window_deferred;
      if (by_budget) ++window_over_budget;
    }
    // Each kept record's batch submitted in join order, then each waited in
    // join order; an error is contained once none is in flight.
    struct Run {
      PhoneSession* session = nullptr;
      std::unique_ptr<InFlight> flight;
      vkc::Status error;
      bool hop = false;
    };
    std::vector<Run> runs;
    for (std::size_t i = 0; i < n; ++i) {
      if (kind[i] == Kind::None) continue;
      Run r;
      r.session = sessions[i].get();
      r.hop = i == hop;
      vkc::Result<std::unique_ptr<InFlight>> begun = r.session->begin(grid);
      if (begun.ok()) {
        r.flight = std::move(begun).value();
      } else {
        r.error = begun.status();
      }
      runs.push_back(std::move(r));
    }
    bool hop_ran = false;
    for (Run& r : runs) {
      if (!r.flight) continue;
      r.error = r.session->finish(*r.flight);
      r.flight.reset();
      hop_ran = hop_ran || (r.hop && r.error.ok());
    }
    // A hop that ran: its wall time a sample, its session's wait over, and,
    // where two or more contested the gap, the turn moved on past it. One
    // chosen and not run (its frame passed over, or an error) keeps both.
    if (hop_ran) {
      const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - chose);
      if (probe) {
        ++probes;
        hop_cost.probed(wall, *last_room);
      } else {
        hop_cost.add(wall);
      }
      held_since.reset();
      sessions[hop]->waited = 0;
      if (hops > 1) hop_next = hop + 1;
    }
    for (Run& r : runs) VKC_TRY(contain(*r.session, r.error));
    for (std::size_t i = 0; i < n; ++i) {
      if (paired[i] && sessions[i]->active()) {
        VKC_TRY(contain(*sessions[i], sessions[i]->at_end()));
      }
    }
    note_deferrals(next_set_ns);
    return {};
  }

  // A warning for each deferral window, from the first gap's set, in which
  // more than half the records classed a prep or a hop were put back: how
  // many by the budget, the estimate and the newest budgeted gap's room, so
  // too many phones reads apart from a rig too slow for their hops.
  void note_deferrals(std::uint64_t set_ns) {
    if (!window_ns) window_ns = set_ns;
    if (set_ns < *window_ns + kDeferralWindowNs) return;
    if (2 * window_deferred > window_classed) {
      std::fprintf(stderr,
                   "warning: phones: %llu of %llu records deferred over %.0f s "
                   "of sets (%llu by the gap's budget, the rest one hop a "
                   "gap; a hop estimated at %.1f ms, the last gap's room "
                   "%s); their fill comes late or not at all\n",
                   static_cast<unsigned long long>(window_deferred),
                   static_cast<unsigned long long>(window_classed),
                   double(set_ns - *window_ns) * 1e-9,
                   static_cast<unsigned long long>(window_over_budget),
                   double(hop_cost.p90().count()) * 1e-6, room_text().c_str());
    }
    window_ns = set_ns;
    window_classed = 0;
    window_deferred = 0;
    window_over_budget = 0;
  }

  // The newest budgeted gap's room, for a person: "none" without a budget.
  std::string room_text() const {
    if (!last_room) return "none";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f ms",
                  double(last_room->count()) * 1e-6);
    return buf;
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
  // none waiting, or gone to the reaper, gives its turn away, and a filler
  // the send would refuse is let go. A tint that cannot be made is the
  // device's: out of memory, the filler is let go; its loss returns.
  vkc::Status drain(std::vector<sender::Filler>* out) {
    const std::size_t n = sessions.size();
    if (n == 0) return {};
    drain_next %= n;
    for (std::size_t idle = 0; out->size() < kDrainCap && idle < n;) {
      PhoneSession& s = *sessions[drain_next];
      drain_next = (drain_next + 1) % n;
      const Stage st = s.stage.load(std::memory_order_acquire);
      if (st == Stage::Reaping || st == Stage::Left || s.queue.empty()) {
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
      const vkc::Status tinted = tint(f);
      if (!tinted.ok()) {
        if (device_lost(tinted)) return tinted;
        s.unfit(tinted);
        continue;
      }
      ++s.counts.fused;
      out->push_back(std::move(f));
    }
    return {};
  }

  // A session done, handed to the reaper: its counters kept, its presence,
  // say and error's hearer let go here; from now on the reaper's.
  void hand_over(PhoneSession& s, bool keep_log) {
    s.last_stats = s.phone ? s.phone->stats() : sensor::SensorStats{};
    s.left_ns = s.set_ns;
    s.presence = {};
    s.say = {};
    s.on_error = {};
    s.stage.store(Stage::Reaping, std::memory_order_release);
    reaper->reap(&s, keep_log);
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

vkc::Result<std::unique_ptr<MobileLane>> MobileLane::create(
    vkc::Device& device, vkc::Allocator& allocator,
    std::unique_ptr<SessionSource> source, LaneOptions options) {
  if (options.settle_ns == 0 || options.settle_ns >= options.grace_ns) {
    return vkc::Status::invalid_argument("MobileLane::create: a settle of " +
                                         std::to_string(options.settle_ns) +
                                         " ns, which must be positive and "
                                         "below the grace's " +
                                         std::to_string(options.grace_ns));
  }
  if (options.error_back_off.count() <= 0) {
    return vkc::Status::invalid_argument(
        "MobileLane::create: a back-off of " +
        std::to_string(options.error_back_off.count()) +
        " ms, which must be positive");
  }
  if (options.hop_seed.count() <= 0) {
    return vkc::Status::invalid_argument(
        "MobileLane::create: a hop's seed of " +
        std::to_string(options.hop_seed.count()) +
        " ns, which must be positive");
  }
  auto s = std::make_unique<State>();
  s->device = &device;
  s->allocator = &allocator;
  s->live = true;
  s->hop_cost = HopEstimate(options.hop_seed);
  s->source = std::move(source);
  // sessions.csv, made at its first row, never over one there.
  std::string csv_path;
  if (!options.log_directory.empty()) {
    csv_path = (std::filesystem::path(options.log_directory) / "sessions.csv")
                   .string();
  }
  s->options = std::move(options);
  try {
    s->reaper = std::make_unique<Reaper>(std::move(csv_path));
  } catch (const std::system_error& e) {
    return vkc::Status::io_error(
        std::string("MobileLane::create: cannot start the reaper's thread: ") +
        e.what());
  }
  return std::unique_ptr<MobileLane>(new MobileLane(std::move(s)));
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
  if (parts.prep) {
    s->prep = std::move(parts.prep);
  } else {
    VKC_ASSIGN(sensor::GpuFramePrep prep,
               sensor::GpuFramePrep::create(*st.device, *st.allocator));
    s->prep.emplace(std::move(prep));
  }
  if (parts.timer) {
    s->timer = std::move(parts.timer);
  } else {
    VKC_ASSIGN(vkc::GpuTimer timer, vkc::GpuTimer::create(*st.device));
    s->timer.emplace(std::move(timer));
  }
  s->phone = std::move(parts.phone);
  s->tracker = std::move(parts.tracker);
  s->log = std::move(parts.log);
  s->by_tag = by_tag;
  s->placed = !by_tag;
  s->tags = std::move(parts.tags);
  if (by_tag) s->say = std::move(parts.say);
  s->lives = static_cast<bool>(parts.presence);
  s->presence = std::move(parts.presence);
  if (s->lives) s->on_error = std::move(parts.on_error);
  s->name = std::move(parts.name);
  s->placement = parts.placement;
  st.sessions.push_back(std::move(s));
  return {};
}

vkc::Result<std::vector<sender::Filler>> MobileLane::next(
    const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns,
    GapBudget budget) {
  State& st = *state_;
  std::vector<sender::Filler> out;
  if (st.closed) return out;
  if (!st.origin_ns) st.origin_ns = next_set_ns;
  // One session the source made, a gap at most.
  if (st.source) {
    VKC_ASSIGN(std::optional<SessionParts> ready, st.source->take());
    if (ready) {
      const vkc::Status added = add(std::move(*ready));
      if (!added.ok()) {
        if (device_lost(added)) return added;
        std::fprintf(stderr, "warning: phones: a session not joined: %s\n",
                     added.message().c_str());
      }
    }
  }
  VKC_TRY(st.adopt(next_set_ns));
  VKC_TRY(st.gap(grid, next_set_ns, budget));
  VKC_TRY(st.drain(&out));
  for (const std::unique_ptr<PhoneSession>& s : st.sessions) {
    if (s->active()) s->tell();
    if (s->stage.load(std::memory_order_acquire) != Stage::Retiring ||
        !s->done()) {
      continue;
    }
    if (s->comes_and_goes() && st.reaper) {
      st.hand_over(*s, /*keep_log=*/false);  // it leaves
    } else {
      // A file's, done, prepares nothing more: its prep and timer go (a
      // frame handed out holds its own buffers), and their query pools with
      // them, before its watch and phone, which stay until the close.
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
  if (st.source) st.source->close();
  vkc::Status lost;
  for (const std::unique_ptr<PhoneSession>& s : st.sessions) {
    const Stage now = s->stage.load(std::memory_order_acquire);
    if (now == Stage::Added || now == Stage::Reaping || now == Stage::Left) {
      continue;
    }
    // Placed, not fused: the run is over. One retired before takes its
    // watch's news once more, as a search may have ended since.
    const vkc::Status retired =
        now == Stage::Retiring ? s->take_news() : s->retire("the run's end");
    if (lost.ok() && device_lost(retired)) lost = retired;
    if (st.reaper) {
      st.hand_over(*s, /*keep_log=*/true);
    } else {
      s->last_stats = s->phone->stats();
      s->left_ns = s->set_ns;
      let_go(*s, /*keep_log=*/true);
      s->stage.store(Stage::Left, std::memory_order_release);
    }
  }
  if (st.reaper) st.reaper->finish();
  return lost;
}

bool MobileLane::enabled() const noexcept {
  return state_->live || !state_->sessions.empty();
}

bool MobileLane::live() const noexcept { return state_->live; }

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
    v.stage = s->stage.load(std::memory_order_acquire);
    v.waiting = s->queue.size();
    v.counts = s->counts;
    // The reaper's while it lets the session go.
    v.log = v.stage == Stage::Reaping ? nullptr : s->log.get();
    out.push_back(std::move(v));
  }
  return out;
}

const TrackLog* MobileLane::log() const noexcept {
  if (state_->sessions.empty()) return nullptr;
  const PhoneSession& s = *state_->sessions.front();
  if (s.stage.load(std::memory_order_acquire) == Stage::Reaping) return nullptr;
  return s.log.get();
}

std::string MobileLane::report() const {
  const State& st = *state_;
  const std::vector<std::unique_ptr<PhoneSession>>& sessions = st.sessions;
  const std::uint64_t origin = st.origin_ns.value_or(0);
  if (!st.live && sessions.size() == 1) {
    return sessions.front()->report("mobile", origin);
  }
  std::string out;
  for (const std::unique_ptr<PhoneSession>& s : sessions) {
    char label[64];
    if (s->ordinal != 0) {
      std::snprintf(label, sizeof(label), "mobile %02u",
                    static_cast<unsigned>(s->ordinal));
    } else {
      std::snprintf(label, sizeof(label), "mobile --");
    }
    out += s->report(std::string(label) + " (" + s->name + ")", origin);
  }
  if (!st.live) return out;
  // The lane's totals, then its source's lines.
  sensor::SensorStats phones{};
  std::size_t waiting = 0;
  std::size_t left = 0;
  for (const std::unique_ptr<PhoneSession>& s : sessions) {
    const Stage now = s->stage.load(std::memory_order_acquire);
    const bool gone = now == Stage::Reaping || now == Stage::Left;
    if (now == Stage::Added) continue;
    const sensor::SensorStats ps = gone ? s->last_stats : s->phone->stats();
    phones.received += ps.received;
    phones.failed += ps.failed;
    waiting += s->queue.size();
    if (now == Stage::Left) ++left;
  }
  char label[96];
  std::snprintf(label, sizeof(label), "mobile, %u session(s), %zu left",
                static_cast<unsigned>(st.joins), left);
  const Counts total = counts();
  out += counts_line(label, total, phones, waiting);
  if (st.joins != 0) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "  the gap: %llu deferred (%llu by its budget), %llu "
                  "probe(s); a hop estimated at %.1f ms (p90 of the last "
                  "%zu), the last gap's room %s\n",
                  static_cast<unsigned long long>(total.deferred),
                  static_cast<unsigned long long>(total.over_budget),
                  static_cast<unsigned long long>(st.probes),
                  double(st.hop_cost.p90().count()) * 1e-6, kHopSamples,
                  st.room_text().c_str());
    out += buf;
  }
  if (st.source) out += st.source->report();
  return out;
}

// --- A live run's sessions -------------------------------------------------

namespace {

using Door = link::RgbdStreamSensor::Door;

// A sensor id as a directory's name takes it: any byte but a letter, a
// digit, '.', '_' or '-' made '_'; "phone" for none.
std::string sanitized(const std::string& id) {
  std::string out = id.empty() ? std::string("phone") : id;
  for (char& c : out) {
    const bool kept = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                      (c >= '0' && c <= '9') || c == '.' || c == '_' ||
                      c == '-';
    if (!kept) c = '_';
  }
  return out;
}

// The sensor id a session's depth track names; empty for none.
std::string depth_sensor(const stream::Session& session) {
  for (const stream::Track& t : session.tracks) {
    if (t.kind == stream::TrackKind::RgbdDepth) return t.sensor;
  }
  return {};
}

// Whether a session carries pictures: a colour track, which the tag is found
// in.
bool has_pictures(const stream::Session& session) {
  return std::any_of(session.tracks.begin(), session.tracks.end(),
                     [](const stream::Track& t) {
                       return t.kind == stream::TrackKind::RgbdColor;
                     });
}

// What a live run takes of a phone, settled as it opens.
struct LivePlan {
  std::string host;
  std::uint16_t port = 0;
  std::string address;        // "tcp://host:port"
  std::filesystem::path dir;  // D; empty for none
  std::string device_name;    // the GPU, for each log
  // Why every phone is turned away; empty: none is.
  std::string turned_away;
  // Why nothing is written in D (an earlier run's logs, or none can be made);
  // empty: D is written, or none was given.
  std::string unlogged;
  // The registration file and the one session it places, depth alone; or
  // why it cannot be read, which places none.
  std::string registration_path;
  std::optional<sensor::camera::TrackedRegistration> registration;
  std::string unread;
  // The origin tag's world, when the tag can place a session; else why not.
  std::optional<sensor::camera::WorldFrame> tag;
  std::string no_tag;
};

// Why a session is refused at the door, if it is: the plan's turning away,
// or one no registration names that the tag cannot place.
std::string screen_of(const LivePlan& plan, const stream::Session& session) {
  if (!plan.turned_away.empty()) return plan.turned_away;
  if (plan.registration && session.id == plan.registration->session) return {};
  if (!plan.tag) {
    return "no registration names it and the origin tag places none: " +
           plan.no_tag;
  }
  if (!has_pictures(session)) {
    return "it sends no pictures, which the origin tag is found in: give "
           "source.mobile_registration";
  }
  return {};
}

// The phones' ids a live lane keeps from its door for a while: each whose
// session retired for an error of its own or was not opened, for the
// back-off, doubled with each repeat of that id up to 32 times. Its own
// lock, never held while calling the door, which asks it under its own (the
// screen): every call from any thread.
class BackOff {
 public:
  using Id = std::array<std::uint8_t, 16>;
  using Clock = std::chrono::steady_clock;

  explicit BackOff(std::chrono::milliseconds base) : base_(base) {}

  // `id`'s session failed for `why`: it is kept away for the next back-off.
  void failed(const Id& id, const std::string& why) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find_if(ids_.begin(), ids_.end(),
                           [&](const Kept& k) { return k.id == id; });
    if (it == ids_.end()) {
      if (ids_.size() >= kMaxIds) ids_.pop_front();
      ids_.push_back({id, 0, {}, {}});
      it = ids_.end() - 1;
    }
    const std::chrono::milliseconds kept =
        base_ * (1 << std::min(it->failures, kBackOffMaxDoublings));
    it->until = Clock::now() + kept;
    ++it->failures;
    // The door logs a refusal once a spell, while its reason stays the same:
    // the reason names this back-off, never what is left of it.
    char buf[96];
    std::snprintf(buf, sizeof(buf),
                  "kept away %.1f s after %d failure(s) of its session, the "
                  "last: ",
                  double(kept.count()) * 1e-3, it->failures);
    it->why = buf + why.substr(0, 160);
  }

  // Why `id` is kept away now; empty: it is not.
  std::string refusal(const Id& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const Clock::time_point now = Clock::now();
    for (const Kept& k : ids_) {
      if (k.id == id && now < k.until) return k.why;
    }
    return {};
  }

 private:
  static constexpr std::size_t kMaxIds = 256;
  struct Kept {
    Id id{};
    int failures = 0;  // its sessions' so far
    Clock::time_point until;
    std::string why;  // the refusal while kept away
  };
  const std::chrono::milliseconds base_;
  mutable std::mutex mutex_;
  std::deque<Kept> ids_;  // under mutex_
};

// A live run's sessions: the door, bound at once or every 2 s until it is,
// and the opener's thread, which admits each arrival and makes its session
// whole (its watch, prep, timer, fitters, tracker and log) off the set loop.
class DoorSource final : public MobileLane::SessionSource {
 public:
  DoorSource(vkc::Device& device, vkc::Allocator& allocator, LivePlan plan,
             std::chrono::milliseconds back_off)
      : device_(device),
        allocator_(allocator),
        plan_(std::move(plan)),
        back_off_(std::make_shared<BackOff>(back_off)) {}
  ~DoorSource() override { close(); }
  DoorSource(const DoorSource&) = delete;
  DoorSource& operator=(const DoorSource&) = delete;

  // Bound now if the address takes it, the start line printed either way;
  // then the opener's thread: std::system_error if it cannot start.
  void start() {
    bind(/*first=*/true);
    opener_ = std::thread([this] { run(); });
  }

  vkc::Result<std::optional<MobileLane::SessionParts>> take() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!lost_.ok()) return std::exchange(lost_, {});
    if (ready_.empty()) return std::optional<MobileLane::SessionParts>{};
    std::optional<MobileLane::SessionParts> out(std::move(ready_.front()));
    ready_.pop_front();
    return out;
  }

  void out_of_memory() noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    paused_until_ = std::chrono::steady_clock::now() + kOutOfMemoryPause;
  }

  void close() noexcept override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return;
      closed_ = true;
    }
    wake_.notify_all();
    if (opener_.joinable()) opener_.join();
    std::unique_ptr<Door> door;
    std::deque<MobileLane::SessionParts> ready;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (door_) {
        last_stats_ = door_->stats();  // for the report, the door gone
        closed_door_ = true;
      }
      door = std::move(door_);
      ready.swap(ready_);
    }
    // A session made and not taken goes, its watch before its phone (the
    // parts' members go in that order), then the door, which ends an arrival
    // not admitted.
    ready.clear();
    door.reset();
  }

  std::string report() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string out;
    // What turned every phone away, or kept them unlogged, after the door's
    // line, or after its not listening.
    std::string after;
    if (!plan_.turned_away.empty()) {
      after += "  every phone turned away: " + plan_.turned_away + "\n";
    }
    if (!plan_.unlogged.empty()) {
      after += "  phones not logged: " + plan_.unlogged + "\n";
    }
    if (!door_ && !closed_door_) {
      return "phones: not listening: " + bind_error_ + "\n" + after;
    }
    const Door::Stats d = door_ ? door_->stats() : last_stats_;
    char buf[320];
    std::snprintf(
        buf, sizeof(buf),
        "phones: door on %s: %llu accepted, %llu arrived, %llu admitted, "
        "%llu refused (%llu turned away), %llu reconnects, %llu strays, "
        "%llu crowded\n",
        plan_.address.c_str(), static_cast<unsigned long long>(d.accepted),
        static_cast<unsigned long long>(d.arrivals),
        static_cast<unsigned long long>(d.admitted),
        static_cast<unsigned long long>(d.refused),
        static_cast<unsigned long long>(d.screened),
        static_cast<unsigned long long>(d.reconnects),
        static_cast<unsigned long long>(d.strays),
        static_cast<unsigned long long>(d.crowded));
    out = buf + after;
    if (failed_ != 0) {
      out += "  " + std::to_string(failed_) +
             " session(s) not opened, the first: " + first_failure_ + "\n";
    }
    return out;
  }

 private:
  // The door, bound if the address takes it: the start line, or a warning
  // the first time and once a reason changes, and a reminder each minute;
  // the line again once bound after.
  void bind(bool first) {
    link::DoorOptions options;
    options.max_sessions = kMaxPhones;
    // The plan is the source's for its life, which the door's ends inside;
    // the back-off is shared.
    options.screen = [plan = &plan_,
                      back_off = back_off_](const stream::Session& session) {
      std::string why = screen_of(*plan, session);
      if (why.empty()) why = back_off->refusal(session.id);
      return why;
    };
    if (!plan_.dir.empty()) options.csv = (plan_.dir / "door.csv").string();
    vkc::Result<std::unique_ptr<Door>> opened =
        Door::open(plan_.host, plan_.port, options);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!opened.ok()) {
      const std::string why = opened.status().message();
      const auto now = std::chrono::steady_clock::now();
      if (first || why != bind_error_) {
        std::fprintf(stderr,
                     "warning: phones: cannot listen on %s (%s); the rig runs "
                     "without phones, trying again every %lld s\n",
                     plan_.address.c_str(), why.c_str(),
                     static_cast<long long>(kBindRetry.count()));
        warned_at_ = now;
      } else if (now - warned_at_ >= kBindReminder) {
        std::fprintf(stderr,
                     "warning: phones: still not listening on %s (%s)\n",
                     plan_.address.c_str(), why.c_str());
        warned_at_ = now;
      }
      bind_error_ = why;
      return;
    }
    door_ = std::move(opened).value();
    std::printf("phones: listening on %s%s\n", plan_.address.c_str(),
                welcome().c_str());
  }

  // The start line's rest: who is taken, and how.
  std::string welcome() const {
    if (!plan_.turned_away.empty()) {
      return ", every phone turned away: " + plan_.turned_away;
    }
    std::string out =
        kMaxPhones == 1
            ? std::string(" for one phone at a time")
            : " for up to " + std::to_string(kMaxPhones) + " phones at once";
    if (plan_.registration) {
      char hex[9];
      const std::array<std::uint8_t, 16>& id = plan_.registration->session;
      std::snprintf(hex, sizeof(hex), "%02x%02x%02x%02x", id[0], id[1], id[2],
                    id[3]);
      out += std::string(", session ") + hex + " placed by " +
             plan_.registration_path;
      out += plan_.tag ? std::string(", any other by the tag")
                       : ", any other turned away: " + plan_.no_tag;
    } else {
      out += ", placed by the tag";
      if (!plan_.unread.empty()) {
        out += " (" + plan_.unread + ", which places none)";
      }
    }
    return out + "; the rig fuses alone until one joins";
  }

  bool stopping() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  // The opener's thread: the door bound again every 2 s until it is; then
  // each arrival admitted and made whole, in turn, or let go (its slot
  // free), and none for a while after the device ran out of memory. Host
  // memory running out in a pass is printed, the arrival let go, and the
  // next pass goes on: no phone's trouble ends the run.
  void run() noexcept {
    auto next_bind = std::chrono::steady_clock::now() + kBindRetry;
    while (!stopping()) {
      try {
        pass(next_bind);
      } catch (const std::exception& e) {
        try {
          std::fprintf(stderr, "warning: phones: the opener: %s\n", e.what());
        } catch (...) {
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  // One pass of the opener's thread: the door bound, or one arrival taken.
  void pass(std::chrono::steady_clock::time_point& next_bind) {
    Door* door = nullptr;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      door = door_.get();
      if (door == nullptr) {
        wake_.wait_until(lock, next_bind, [this] { return closed_; });
        if (closed_) return;
      }
    }
    if (door == nullptr) {
      if (std::chrono::steady_clock::now() >= next_bind) {
        bind(/*first=*/false);
        next_bind = std::chrono::steady_clock::now() + kBindRetry;
      }
      return;
    }
    std::optional<Door::Arrival> arrival = door->take_arrival(kOpenerWait);
    if (!arrival) return;
    const std::array<std::uint8_t, 16> id = arrival->session.id;
    const std::string peer = arrival->peer;  // admit consumes the arrival
    bool paused = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      paused = std::chrono::steady_clock::now() < paused_until_;
    }
    if (paused) {
      // The arrival goes, its slot with it; the pause keeps the rest away.
      fail(id, peer, "the device ran out of memory within the last minute",
           /*back_off=*/false);
      return;
    }
    vkc::Result<MobileLane::SessionParts> made = open(*door, *arrival);
    if (!made.ok()) {
      if (device_lost(made.status())) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!lost_seen_) lost_ = made.status();  // for the lane, once
        lost_seen_ = true;
      }
      if (ran_out_of_memory(made.status())) out_of_memory();
      fail(id, peer, made.status().message(), /*back_off=*/true);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ready_.push_back(std::move(made).value());
  }

  // A session not opened: counted, printed once an id, and its id kept away
  // from the door for a while (`back_off`).
  void fail(const std::array<std::uint8_t, 16>& id, const std::string& peer,
            const std::string& why, bool back_off) {
    if (back_off) back_off_->failed(id, why);
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_++ == 0) first_failure_ = why;
    if (std::find(failed_ids_.begin(), failed_ids_.end(), id) !=
        failed_ids_.end()) {
      return;
    }
    if (failed_ids_.size() >= 64) failed_ids_.pop_front();
    failed_ids_.push_back(id);
    std::fprintf(stderr,
                 "warning: phones: session %s from %s not opened (%s); the run "
                 "goes on\n",
                 hex32(id).substr(0, 8).c_str(), peer.c_str(), why.c_str());
  }

  // `arrival`'s session made whole: its directory, admitted (its decoder
  // made), its watch, its phone polled in the background, its prep, timer,
  // fitters, tracker and log. A failure lets it go, its directory with it.
  vkc::Result<MobileLane::SessionParts> open(Door& door,
                                             Door::Arrival& arrival) {
    const std::string id = hex32(arrival.session.id);
    const std::string sensor_id = depth_sensor(arrival.session);
    const bool named =
        plan_.registration && arrival.session.id == plan_.registration->session;
    // Its directory, NN its join's: the lane joins the sessions made in the
    // order they are made, and a failure gives its number back.
    std::filesystem::path dir;
    if (!plan_.dir.empty()) {
      char name[64];
      std::snprintf(name, sizeof(name), "%02u-",
                    static_cast<unsigned>(opened_ + 1));
      dir = plan_.dir / (name + sanitized(sensor_id) + "-" + id.substr(0, 8));
      std::error_code made;
      std::filesystem::create_directories(dir, made);
      if (made) {
        std::fprintf(stderr,
                     "warning: phones: cannot make %s (%s); session %s goes "
                     "on unlogged and unrecorded\n",
                     dir.string().c_str(), made.message().c_str(),
                     id.substr(0, 8).c_str());
        dir.clear();
      }
    }
    const auto unmade = [&](vkc::Status why) {
      if (!dir.empty()) {
        std::error_code ignored;
        std::filesystem::remove_all(dir, ignored);
      }
      return why;
    };
    link::RgbdStreamSensor::Options options;
    options.min_confidence = 2;  // the phone's high, whatever it fused at
    RunSource source;
    source.session = plan_.address;
    source.device = plan_.device_name;
    source.session_id = id;
    source.peer = arrival.peer;
    if (named) {
      options.registration = plan_.registration;
      source.registration = plan_.registration_path;
      source.tracking_to_world = Mat4f(plan_.registration->tracking_to_world);
    } else {
      options.device = &device_;  // the pictures, for the tag
      options.allocator = &allocator_;
      source.registration = "tag";  // until the watch registers it
    }
    const std::array<std::uint8_t, 16> session_id = arrival.session.id;
    vkc::Result<std::unique_ptr<link::RgbdStreamSensor>> admitted =
        door.admit(std::move(arrival), std::move(options),
                   dir.empty() ? std::string() : (dir / "phone.vrst").string());
    if (!admitted.ok()) return unmade(admitted.status());
    std::unique_ptr<link::RgbdStreamSensor> stream =
        std::move(admitted).value();
    source.sensor = stream->info().id;
    MobileLane::SessionParts parts;
    parts.name = stream->info().id;
    if (!named) {
      if (!plan_.tag) {  // the screen refuses these; never placed blind
        return unmade(vkc::Status::invalid_argument(
            "no registration names it and the origin tag places none: " +
            plan_.no_tag));
      }
      if (!stream->info().color) {
        return unmade(vkc::Status::invalid_argument(
            "sensor " + parts.name +
            " sends no pictures, which the origin tag is found in: give "
            "source.mobile_registration"));
      }
      const sensor::camera::WorldFrame& world = *plan_.tag;
      std::printf(
          "phones: session %s (%s): registering by the origin tag, %s id "
          "%d, at its printed edge %.4g mm\n",
          id.substr(0, 8).c_str(), parts.name.c_str(),
          world.apriltag->family.c_str(), world.apriltag->id,
          world.apriltag->size_m * 1e3);
      vkc::Result<std::unique_ptr<TagWatch>> tags = TagWatch::create(
          device_, allocator_, {parts.name, session_id, world, true});
      if (!tags.ok()) return unmade(tags.status());
      parts.tags = std::move(tags).value();
      parts.placement = MobileLane::Placement::Tag;
    }
    // Its presence and its notes through its handle, never the sensor, which
    // the reaper lets go on a thread of its own.
    const std::shared_ptr<link::SessionHandle> handle = stream->handle();
    parts.presence = [handle] { return handle->connected(); };
    parts.on_error = [back_off = back_off_,
                      session_id](const vkc::Status& error) {
      back_off->failed(session_id, error.message());
    };
    if (!named) {
      parts.say = [handle](gnet::ViewerNote note) {
        handle->say(std::move(note));
      };
    }
    parts.phone = polled_in_background(std::move(stream));
    vkc::Result<sensor::GpuFramePrep> prep =
        sensor::GpuFramePrep::create(device_, allocator_);
    if (!prep.ok()) return unmade(prep.status());
    parts.prep.emplace(std::move(prep).value());
    vkc::Result<vkc::GpuTimer> timer = vkc::GpuTimer::create(device_);
    if (!timer.ok()) return unmade(timer.status());
    parts.timer.emplace(std::move(timer).value());
    const track::TrackerConfig tracking = lane_tracking();
    vkc::Result<std::vector<std::unique_ptr<track::IFitter>>> fitters =
        track::create_fitters(device_, allocator_, tracking);
    if (!fitters.ok()) return unmade(fitters.status());
    vkc::Result<std::unique_ptr<track::Tracker>> tracker =
        track::Tracker::create(tracking, std::move(fitters).value());
    if (!tracker.ok()) return unmade(tracker.status());
    parts.tracker = std::move(tracker).value();
    // A log that cannot be made is printed, and the session is logged in
    // memory alone.
    vkc::Result<std::unique_ptr<TrackLog>> log =
        TrackLog::open(dir.string(), *parts.tracker, source);
    if (!log.ok()) {
      std::fprintf(stderr,
                   "warning: phones: session %s's log (%s); it is kept in "
                   "memory alone\n",
                   id.substr(0, 8).c_str(), log.status().message().c_str());
      log = TrackLog::open({}, *parts.tracker, std::move(source));
      if (!log.ok()) return unmade(log.status());
    }
    parts.log = std::move(log).value();
    ++opened_;
    return parts;
  }

  vkc::Device& device_;
  vkc::Allocator& allocator_;
  const LivePlan plan_;
  const std::shared_ptr<BackOff> back_off_;  // the door's screen's too
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::unique_ptr<Door> door_;  // set by start or the opener; under mutex_
  std::deque<MobileLane::SessionParts> ready_;         // under mutex_
  bool closed_ = false;                                // under mutex_
  bool closed_door_ = false;                           // under mutex_
  Door::Stats last_stats_{};                           // under mutex_
  std::string bind_error_;                             // under mutex_
  std::chrono::steady_clock::time_point warned_at_{};  // under mutex_
  vkc::Status lost_;        // the device's loss, for take; under mutex_
  bool lost_seen_ = false;  // under mutex_
  std::chrono::steady_clock::time_point paused_until_{};  // under mutex_
  std::uint64_t failed_ = 0;                              // under mutex_
  std::string first_failure_;                             // under mutex_
  std::deque<std::array<std::uint8_t, 16>> failed_ids_;   // under mutex_
  std::uint32_t opened_ = 0;  // the sessions made; the opener's
  std::thread opener_;        // last: started once the rest is
};

// Whether `name` is a session's directory's, as the opener names one:
// NN-<sensor>-<hex8>, NN two digits or more, the sensor not empty.
bool session_directory_name(const std::string& name) {
  std::size_t digits = 0;
  while (digits < name.size() && name[digits] >= '0' && name[digits] <= '9') {
    ++digits;
  }
  // The digits, '-', a sensor of one byte or more, '-' and 8 hex digits.
  if (digits < 2 || name.size() < digits + 1 + 1 + 1 + 8) return false;
  if (name[digits] != '-' || name[name.size() - 9] != '-') return false;
  return std::all_of(name.end() - 8, name.end(), [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}

// What an earlier run's logs left in D, which this one would write beside:
// its sessions.csv or door.csv (each made only with a row), a session's
// directory with its log, recording or registration in it, or a file run's
// flat registration.json or a recording; its name, or empty.
std::string earlier_run(const std::filesystem::path& dir) {
  std::error_code ec;
  const auto there = [&](const std::filesystem::path& p) {
    return std::filesystem::exists(std::filesystem::symlink_status(p, ec));
  };
  for (const char* file :
       {"sessions.csv", "door.csv", "registration.json", "phone.vrst"}) {
    if (there(dir / file)) return file;
  }
  for (std::filesystem::directory_iterator it(dir, ec), end; !ec && it != end;
       it.increment(ec)) {
    const std::string name = it->path().filename().string();
    std::error_code is_dir;
    if (!session_directory_name(name) || !it->is_directory(is_dir)) continue;
    for (const char* file : {"run.csv", "phone.vrst", "registration.json"}) {
      if (there(it->path() / file)) return name;
    }
  }
  return {};
}

// A live source.mobile's lane (above): what it takes of a phone settled
// here, never refusing the run; the door bound or binding; the lane.
vkc::Result<std::unique_ptr<MobileLane>> open_live_lane(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator,
    LaneOptions options) {
  const config::MobileSession& m = *opt.source.mobile;
  LivePlan plan;
  plan.host = m.host;
  plan.port = m.port;
  plan.address = "tcp://" + m.host + ":" + std::to_string(m.port);
  plan.device_name = device.caps().properties().deviceName;
  plan.dir = opt.diagnostics.track_log;
  const vkc::Status settings = settings_refusal(opt);
  if (!settings.ok()) plan.turned_away = settings.message();
  if (plan.turned_away.empty() && !m.registration.empty()) {
    vkc::Result<sensor::camera::TrackedRegistration> reg =
        sensor::camera::read_tracked_registration(m.registration);
    if (reg.ok()) {
      plan.registration = std::move(reg).value();
      plan.registration_path = m.registration;
    } else {
      // It names no session, so each is placed as one it does not name.
      plan.unread = "source.mobile_registration: " + reg.status().message();
      std::fprintf(stderr,
                   "warning: phones: %s; it places no session, and the origin "
                   "tag any it can\n",
                   plan.unread.c_str());
    }
  }
  vkc::Result<sensor::camera::WorldFrame> world = tag_world(opt);
  if (world.ok()) {
    plan.tag = std::move(world).value();
  } else {
    plan.no_tag = world.status().message();
  }
  if (plan.turned_away.empty() && !plan.registration && !plan.tag) {
    // Nothing places any phone: the registration that would have, or why
    // the tag cannot.
    plan.turned_away = plan.unread.empty() ? plan.no_tag : plan.unread;
  }
  if (!plan.dir.empty()) {
    // The log never changes what is taken: a D this run cannot write takes
    // the phones all the same, unlogged.
    const std::string earlier = earlier_run(plan.dir);
    std::error_code made;
    if (earlier.empty()) std::filesystem::create_directories(plan.dir, made);
    if (!earlier.empty()) {
      plan.unlogged = "diagnostics.track_log: " + plan.dir.string() +
                      " holds an earlier run's " + earlier +
                      ": give each run its own directory";
    } else if (made) {
      plan.unlogged = "diagnostics.track_log: cannot make " +
                      plan.dir.string() + " (" + made.message() + ")";
    }
    if (!plan.unlogged.empty()) {
      std::fprintf(stderr,
                   "warning: phones: %s; no phone is logged or recorded\n",
                   plan.unlogged.c_str());
      plan.dir.clear();
    }
  }
  if (plan.turned_away.empty()) {
    // A device that cannot track takes no phone: what each session makes,
    // made once here, which also fills the pipeline cache for them.
    vkc::Status can = [&]() -> vkc::Status {
      VKC_ASSIGN(const sensor::GpuFramePrep prep,
                 sensor::GpuFramePrep::create(device, allocator));
      (void)prep;
      VKC_ASSIGN(const std::vector<std::unique_ptr<track::IFitter>> fitters,
                 track::create_fitters(device, allocator, lane_tracking()));
      (void)fitters;
      return {};
    }();
    if (device_lost(can)) return can;
    if (!can.ok())
      plan.turned_away = "this device cannot track: " + can.message();
  }
  options.log_directory = plan.dir.string();
  if (options.error_back_off.count() <= 0) {
    return vkc::Status::invalid_argument(
        "open_mobile_lane: a back-off of " +
        std::to_string(options.error_back_off.count()) +
        " ms, which must be positive");
  }
  auto source = std::make_unique<DoorSource>(device, allocator, std::move(plan),
                                             options.error_back_off);
  try {
    source->start();
  } catch (const std::system_error& e) {
    return vkc::Status::io_error(
        std::string("open_mobile_lane: cannot start the opener's thread: ") +
        e.what());
  }
  return MobileLane::create(device, allocator, std::move(source),
                            std::move(options));
}

}  // namespace

vkc::Result<std::unique_ptr<MobileLane>> open_mobile_lane(
    const config::Config& opt, vkc::Device& device, vkc::Allocator& allocator,
    LaneOptions options) {
  if (!opt.source.mobile)
    return MobileLane::create(device, allocator, {}, {}, {});
  const config::MobileSession& m = *opt.source.mobile;
  if (!m.host.empty()) {
    return open_live_lane(opt, device, allocator, std::move(options));
  }
  // A file: its one session, as before.
  VKC_TRY(check_mobile(opt));
  const std::filesystem::path dir = opt.diagnostics.track_log;
  const bool by_tag = m.registration.empty();
  link::RgbdStreamSensor::Options reading;
  reading.min_confidence = 2;  // the phone's high, whatever it fused at
  RunSource source;
  source.device = device.caps().properties().deviceName;
  source.session = m.path;
  sensor::camera::WorldFrame world;
  if (by_tag) {
    VKC_ASSIGN(world, tag_world(opt));
    reading.device = &device;  // the pictures, for the tag
    reading.allocator = &allocator;
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
    VKC_ASSIGN(reading.registration,
               sensor::camera::read_tracked_registration(m.registration));
    source.registration = m.registration;
    source.tracking_to_world = Mat4f(reading.registration->tracking_to_world);
  }
  VKC_ASSIGN(std::unique_ptr<link::RgbdStreamSensor> stream,
             link::RgbdStreamSensor::open_file(m.path, reading));
  source.session_id = hex32(stream->session_id());
  source.sensor = stream->info().id;
  // The tag's watch, for the session: it needs its pictures.
  std::unique_ptr<TagWatch> tags;
  if (by_tag) {
    const std::string id = stream->info().id;
    if (!stream->info().color) {
      return vkc::Status::invalid_argument(
          "source.mobile: " + source.session + ": sensor " + id +
          " sends no pictures, which the origin tag is found in: give "
          "source.mobile_registration");
    }
    std::printf(
        "phone: session %s… (%s): registering by the origin tag, %s id "
        "%d, at its printed edge %.4g mm\n",
        source.session_id.substr(0, 8).c_str(), id.c_str(),
        world.apriltag->family.c_str(), world.apriltag->id,
        world.apriltag->size_m * 1e3);
    VKC_ASSIGN(tags, TagWatch::create(device, allocator,
                                      {id, stream->session_id(), world,
                                       /*background=*/true}));
  }
  const track::TrackerConfig tracking = lane_tracking();
  VKC_ASSIGN(std::vector<std::unique_ptr<track::IFitter>> fitters,
             track::create_fitters(device, allocator, tracking));
  VKC_ASSIGN(std::unique_ptr<track::Tracker> tracker,
             track::Tracker::create(tracking, std::move(fitters)));
  VKC_ASSIGN(
      std::unique_ptr<TrackLog> log,
      TrackLog::open(opt.diagnostics.track_log, *tracker, std::move(source)));
  return MobileLane::create(device, allocator, std::move(stream),
                            std::move(tracker), std::move(log),
                            std::move(tags));
}

}  // namespace vr_example
