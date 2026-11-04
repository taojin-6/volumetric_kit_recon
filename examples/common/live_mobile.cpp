// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "live_mobile.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
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
  // The frames placed, not yet handed to a send.
  std::deque<sender::Filler> queue;
  // The records taken from the phone, oldest first, not yet due.
  std::deque<sensor::RgbdFrame> held;
  // The phone's time less the rig's, pinned by the first record taken.
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

  // A step's placements into the queue, its counts, and the log.
  vkc::Status take(const track::TrackStep& step) {
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

  // The records the phone holds, taken in order; a loss is noted for its
  // split. An error the phone returns is counted, and the lane goes on.
  bool take_records() {
    std::vector<sensor::RgbdFrame> got;
    const vkc::Status st = phone->drain(&got);
    if (!st.ok() && counts.errors++ == 0) first_error = st.message();
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
      if (held.empty() && !take_records()) break;
      if (!offset) {
        offset =
            std::int64_t(held.front().timestamp_ns) - std::int64_t(next_set);
      }
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

  // One phone frame: prepared and its due fits recorded in one batch,
  // waited on here, then judged, placed and logged.
  vkc::Status track_one(const vol::VoxelBlockGrid& grid,
                        sensor::RgbdFrame frame) {
    // Every loss before this frame: one flush (no frame between them was
    // recorded).
    bool split = false;
    while (!splits.empty() && splits.front() <= frame.timestamp_ns) {
      splits.pop_front();
      split = true;
    }
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
  if (state_ && state_->phone) state_->phone->stop();
}

vkc::Result<std::unique_ptr<MobileLane>> MobileLane::create(
    vkc::Device& device, vkc::Allocator& allocator,
    std::unique_ptr<sensor::IRgbdSensor> phone,
    std::unique_ptr<track::Tracker> tracker, std::unique_ptr<TrackLog> log) {
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
  }
  return std::unique_ptr<MobileLane>(new MobileLane(std::move(s)));
}

vkc::Result<std::vector<sender::Filler>> MobileLane::next(
    const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns) {
  State& s = *state_;
  std::vector<sender::Filler> out;
  if (!s.phone || s.closed) return out;
  if (!s.started) {
    // At the first gap, so nothing sent while the cameras opened waits.
    VKC_TRY(s.phone->set_queue_depth(kPhoneQueue));
    VKC_TRY(s.phone->start());
    s.started = true;
  }
  if (!s.ended) {
    std::optional<sensor::RgbdFrame> due = s.pair(next_set_ns);
    if (due) VKC_TRY(s.track_one(grid, std::move(*due)));
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
  const config::MobileSession& m = *opt.source.mobile;
  link::RgbdStreamSensor::Options options;
  VKC_ASSIGN(options.registration,
             sensor::camera::read_tracked_registration(m.registration));
  options.min_confidence = 2;  // the phone's high, whatever it fused at
  RunSource source;
  source.registration = m.registration;
  source.tracking_to_world = Mat4f(options.registration->tracking_to_world);
  source.device = device.caps().properties().deviceName;
  std::unique_ptr<sensor::IRgbdSensor> phone;
  if (m.host.empty()) {
    source.session = m.path;
    VKC_ASSIGN(phone,
               link::RgbdStreamSensor::open_file(m.path, std::move(options)));
  } else {
    source.session = "tcp://" + m.host + ":" + std::to_string(m.port);
    std::printf("waiting for the phone to dial %s (Ctrl+C ends the wait)\n",
                source.session.c_str());
    // A second at a time, so a Ctrl+C ends the wait; the empty lane then
    // lets the run return before its cameras open.
    std::unique_ptr<link::RgbdStreamSensor> live;
    const auto until = std::chrono::steady_clock::now() + kPhoneWait;
    for (;;) {
      if (vgrid_example::interrupted()) {
        return MobileLane::create(device, allocator, {}, {}, {});
      }
      vkc::Result<std::unique_ptr<link::RgbdStreamSensor>> heard =
          link::RgbdStreamSensor::listen(m.host, m.port, options, {},
                                         std::chrono::seconds(1));
      if (heard.ok()) {
        live = std::move(*heard);
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
    phone = polled_in_background(std::move(live));
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
                            std::move(tracker), std::move(log));
}

}  // namespace vr_example
