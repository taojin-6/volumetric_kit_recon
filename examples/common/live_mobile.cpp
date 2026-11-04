// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "live_mobile.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

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

sender::Filler to_filler(track::Released&& released) {
  const bool rig_anchored =
      released.placement == track::Placement::Anchored && released.driver == 0;
  sender::Filler f;
  f.frame = track::corrected(std::move(released.frame), released.correction);
  f.tier = rig_anchored ? tsdf::FillTier::Anchored : tsdf::FillTier::Unanchored;
  f.index = 0;
  return f;
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

struct MobileLane::State {
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
  // The frames placed, not yet handed to a send.
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
  bool started = false;  // the phone, at the first gap
  // The tint pictures, by tier, at the phone's depth size.
  std::shared_ptr<const vkc::Buffer> tint[2];
  std::uint32_t tint_width = 0;
  std::uint32_t tint_height = 0;
  bool ended = false;  // the phone is exhausted and the tracker flushed
  bool closed = false;
  Counts counts;
  std::string first_refusal;  // why the first frame passed over was
  std::string first_error;    // the first error the phone returned

  void passed_over(const vkc::Status& why) {
    if (counts.passed_over++ == 0) first_refusal = why.message();
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

  // A step's placements into the queue, its counts, and the log.
  vkc::Status take(const track::TrackStep& step) {
    watch_rig(step);
    VKC_TRY(log->write(step));
    for (const track::Released& r : step.released) {
      track::Released copy = r;
      sender::Filler f = to_filler(std::move(copy));
      ++(f.tier == tsdf::FillTier::Anchored ? counts.anchored
                                            : counts.unanchored);
      queue.push_back(std::move(f));
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

  // The tint pictures at the frame's depth size, made on the device once.
  vkc::Status ensure_tints(std::uint32_t width, std::uint32_t height) {
    if (tint[0] && tint_width == width && tint_height == height) return {};
    const VkDeviceSize bytes = VkDeviceSize(width) * height * 4;
    vkc::CommandBatch batch(*device, *allocator);
    const std::uint32_t words[2] = {kAnchoredFillTint, kUnanchoredFillTint};
    std::shared_ptr<const vkc::Buffer> made[2];
    for (int t = 0; t < 2; ++t) {
      VKC_ASSIGN(vkc::Buffer b, vkc::device_storage_buffer(*allocator, bytes));
      VKC_TRY(batch.fill(b, 0, bytes, words[t]));
      made[t] = std::make_shared<const vkc::Buffer>(std::move(b));
    }
    VKC_TRY(batch.submit());
    tint[0] = std::move(made[0]);
    tint[1] = std::move(made[1]);
    tint_width = width;
    tint_height = height;
    return {};
  }

  // A filler coloured by its tier's tint, through a colour camera of its
  // depth camera's geometry.
  void tinted(sender::Filler& f) const {
    const sensor::DepthCameraParams& d = f.frame.depth_camera;
    f.frame.color = tint[f.tier == tsdf::FillTier::Anchored ? 0 : 1];
    f.frame.color_camera = {d.fx,    d.fy,     d.cx,          d.cy,
                            d.width, d.height, d.cam_to_world};
    f.frame.color_encoding = {};
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
};

MobileLane::MobileLane(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}
MobileLane::~MobileLane() {
  if (!state_) return;
  state_->tags.reset();  // its thread may hold a picture of the phone's
  if (state_->phone) state_->phone->stop();
}

vkc::Result<std::unique_ptr<MobileLane>> MobileLane::create(
    vkc::Device& device, vkc::Allocator& allocator,
    std::unique_ptr<sensor::IRgbdSensor> phone,
    std::unique_ptr<track::Tracker> tracker, std::unique_ptr<TrackLog> log,
    std::unique_ptr<TagWatch> tags) {
  auto s = std::make_unique<State>();
  s->device = &device;
  s->allocator = &allocator;
  if (phone) {
    if (!tracker || !log) {
      return vkc::Status::invalid_argument(
          "MobileLane::create: a phone needs its tracker and its log");
    }
    VKC_ASSIGN(sensor::GpuFramePrep prep,
               sensor::GpuFramePrep::create(device, allocator));
    s->prep.emplace(std::move(prep));
    VKC_ASSIGN(vkc::GpuTimer timer, vkc::GpuTimer::create(device));
    s->timer.emplace(std::move(timer));
    s->phone = std::move(phone);
    s->tracker = std::move(tracker);
    s->log = std::move(log);
    s->by_tag = tags != nullptr;
    s->placed = !s->by_tag;
    s->tags = std::move(tags);
  }
  return std::unique_ptr<MobileLane>(new MobileLane(std::move(s)));
}

vkc::Result<std::vector<sender::Filler>> MobileLane::next(
    const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns) {
  State& s = *state_;
  std::vector<sender::Filler> out;
  if (!s.phone || s.closed) return out;
  if (!s.started) {
    // At the first gap: what the link held while the cameras opened comes
    // in the first takes, passed by but for each take's newest.
    VKC_TRY(s.phone->set_queue_depth(kPhoneQueue));
    VKC_TRY(s.phone->start());
    s.started = true;
  }
  if (s.tags) {
    VKC_ASSIGN(TagNews news, s.tags->take());
    VKC_TRY(s.apply(std::move(news)));
  }
  if (!s.ended) {
    std::optional<sensor::RgbdFrame> due = s.pair(next_set_ns);
    if (due) VKC_TRY(s.paired(grid, std::move(*due)));
    if (s.held.empty() && s.phone->exhausted()) VKC_TRY(s.end());
  }
  while (!s.queue.empty() && out.size() < kDrainCap) {
    sender::Filler f = std::move(s.queue.front());
    s.queue.pop_front();
    VKC_TRY(s.ensure_tints(f.frame.depth_camera.width,
                           f.frame.depth_camera.height));
    s.tinted(f);
    out.push_back(std::move(f));
  }
  s.counts.fused += out.size();
  return out;
}

vkc::Status MobileLane::close() {
  State& s = *state_;
  if (!s.phone || s.closed) return {};
  s.closed = true;
  vkc::Status tagged;
  if (s.tags) {  // its last news, then let go before the phone stops
    vkc::Result<TagNews> news = s.tags->take();
    tagged = news.ok() ? s.apply(std::move(news).value()) : news.status();
    s.tags.reset();
  }
  VKC_TRY(tagged);
  VKC_TRY(s.end());  // placed, not fused: the run is over
  s.phone->stop();
  VKC_TRY(s.log->close());
  VKC_ASSIGN(const std::string table,
             summarize(s.log->run(), s.log->directory()));
  std::fputs(table.c_str(), stdout);
  return {};
}

const MobileLane::Counts& MobileLane::counts() const noexcept {
  return state_->counts;
}

const TrackLog* MobileLane::log() const noexcept { return state_->log.get(); }

std::string MobileLane::report() const {
  const State& s = *state_;
  if (!s.phone) return {};
  const Counts& c = s.counts;
  const sensor::SensorStats st = s.phone->stats();
  char buf[512];
  std::snprintf(
      buf, sizeof(buf),
      "mobile: %llu records (%llu received, %llu failed), %llu tracked, %llu "
      "skipped, %llu passed over, %llu splits; fill %llu anchored, %llu "
      "unanchored, %llu dropped, %llu fused, %zu waiting\n",
      static_cast<unsigned long long>(c.polled),
      static_cast<unsigned long long>(st.received),
      static_cast<unsigned long long>(st.failed),
      static_cast<unsigned long long>(c.paired),
      static_cast<unsigned long long>(c.skipped),
      static_cast<unsigned long long>(c.passed_over),
      static_cast<unsigned long long>(c.splits),
      static_cast<unsigned long long>(c.anchored),
      static_cast<unsigned long long>(c.unanchored),
      static_cast<unsigned long long>(c.dropped),
      static_cast<unsigned long long>(c.fused), s.queue.size());
  std::string out = buf;
  if (s.by_tag) {
    const TagWatch::Counts& t = s.tag_counts;
    char when[96];
    if (s.registered_after) {
      std::snprintf(when, sizeof(when),
                    "registered at %.1f s, %llu frames before it",
                    double(*s.registered_after) * 1e-9,
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
  if (!s.first_refusal.empty()) {
    out += "  first passed over: " + s.first_refusal + "\n";
  }
  if (c.errors != 0) {
    out += "  " + std::to_string(c.errors) +
           " phone error(s), the first: " + s.first_error + "\n";
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
                            std::move(tracker), std::move(log),
                            std::move(tags));
}

}  // namespace vr_example
