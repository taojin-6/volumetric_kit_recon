// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "live_mobile_internal.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "link/rgbd_stream_sensor.hpp"
#include "live_mobile.hpp"
#include "polled_sensor.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"

namespace vr_example {

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

// --- A live run's sessions -------------------------------------------------

namespace {

using Door = link::RgbdStreamSensor::Door;
using detail::device_lost;
using detail::ran_out_of_memory;

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
  // Whether fill carries the phones' pictures (FillColour::Picture): each
  // session is read with its pictures, and given its picture prep.
  bool pictures = false;
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
      out += ", session " + hex32(plan_.registration->session).substr(0, 8) +
             " placed by " + plan_.registration_path;
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
      if (plan_.pictures) {  // for its fill
        options.device = &device_;
        options.allocator = &allocator_;
      }
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
    if (plan_.pictures) {
      vkc::Result<sensor::GpuFramePrep> picture_prep =
          sensor::GpuFramePrep::create(device_, allocator_,
                                       picture_prep_config());
      if (!picture_prep.ok()) return unmade(picture_prep.status());
      parts.picture_prep.emplace(std::move(picture_prep).value());
    }
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
  plan.pictures = options.fill.colour == FillColour::Picture;
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
  if (options.fill.picture_every == 0) {
    return vkc::Status::invalid_argument(
        "open_mobile_lane: a picture_every of 0, which must be positive");
  }
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
    if (options.fill.colour == FillColour::Picture) {  // for its fill
      reading.device = &device;
      reading.allocator = &allocator;
    }
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
                            std::move(tracker), std::move(log), std::move(tags),
                            {}, options.fill);
}

}  // namespace vr_example
