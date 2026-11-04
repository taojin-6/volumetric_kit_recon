// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The phone lane (examples/common/live_mobile.hpp) on a fake phone, its
// frames prepared on the device and tracked by scripted fitters
// (scripted_fitter.hpp) on lane_tracking()'s chains, sets every 40 ms:
// - Tiers and one generation: frames the rig chain anchored fuse as Anchored
//   fill at its correction, frames only the track chain placed as Unanchored
//   fill at its own, each in its tier's tint through its depth camera; the
//   rig chain's priors are the same whether the track chain anchors or not.
// - The 5 s wait and the drop: frames between two rig anchors 7.5 s apart
//   wait for the second and fuse interpolated; those more than 5 s from
//   either, and those after the last, are dropped.
// - Replays repeat: the same phone twice gives the same fillers, bit for bit.
// - The drain cap: no send gets more than 3 fillers; every placed frame is
//   handed over in the end.
// - A frame the tracker refuses is passed over and the lane goes on.
// - The log: files only with a directory; close flushes, every frame decided
//   by every chain; close is idempotent; the phone's end flushes the tracker
//   and what it placed still drains.
// - A loss of 100 ms or more splits the windows: no fit holds frames from
//   both sides.
// - Pairing: one record a set at 25 Hz sets, the older of two passed by at
//   5 Hz sets.
// - check_mobile: no phone, or a phone under Dynamic, runs; Classic and FFS
//   depth are refused. The empty lane does nothing.
// - polled_in_background: a poll never waits on the inner sensor's, frames
//   come through in order, the end and an inner error come through, and stop
//   joins while the inner poll waits.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

#include "live_mobile.hpp"
#include "scripted_fitter.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/buffer_readback.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"

namespace vkc = volumetric_kit::core;
namespace cfg = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace sender = volumetric_kit::recon::sender;
namespace vol = volumetric_kit::grid::volume;
using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec4f;
using vr_example::MobileLane;
using vr_test::ScriptedFitter;

namespace {

constexpr std::uint64_t kMs = 1'000'000;
constexpr std::uint64_t kS = 1'000'000'000;
constexpr std::uint32_t kW = 32;
constexpr std::uint32_t kH = 24;

Mat4f translate(float x, float y, float z) {
  Mat4f m(1.0f);
  m[3] = Vec4f(x, y, z, 1.0f);
  return m;
}

bool same(const Mat4f& a, const Mat4f& b) {
  return std::memcmp(&a, &b, sizeof(Mat4f)) == 0;
}

// --- The fake phone ---------------------------------------------------------

struct PhoneFrame {
  std::uint64_t pts = 0;
  std::uint64_t seq = 0;
  bool noise = true;  // false: states none, which the tracker refuses
};

// Frames at 10 Hz from 1 s: `count` of them, seq 0 up.
std::vector<PhoneFrame> ten_hz(std::size_t count) {
  std::vector<PhoneFrame> out;
  for (std::size_t i = 0; i < count; ++i)
    out.push_back({kS + i * 100 * kMs, i});
  return out;
}

// The device's pose of the frame at `pts`: 1 cm along x a frame.
Mat4f device_pose(std::uint64_t pts) {
  return translate(0.01f * float((pts - kS) / (100 * kMs)), 0.0f, 0.0f);
}

// A phone whose records the script lists, each a 32 x 24 depth at 1.5 m with
// a confidence plane, as a recorded session hands them out: at once, in
// order, exhausted after the last.
class FakePhone final : public sensor::IRgbdSensor {
 public:
  explicit FakePhone(std::vector<PhoneFrame> script)
      : script_(std::move(script)) {
    info_.id = "phone";
    info_.clock = sensor::ClockDomain::Host;
    info_.fps = 10;
  }
  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override {
    started_ = true;
    return {};
  }
  void stop() noexcept override { started_ = false; }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    if (!started_ || next_ >= script_.size()) {
      return std::optional<sensor::RgbdFrame>{};
    }
    const PhoneFrame& p = script_[next_++];
    struct Pixels {
      std::vector<std::uint16_t> depth;
      std::vector<std::uint8_t> confidence;
    };
    auto pixels = std::make_shared<Pixels>();
    pixels->depth.assign(std::size_t{kW} * kH, 1500);
    pixels->confidence.assign(std::size_t{kW} * kH, 2);
    sensor::RgbdFrame f;
    f.depth = pixels->depth.data();
    f.depth_confidence = pixels->confidence.data();
    f.min_confidence = 2;  // as the lane's reader raises it
    f.depth_camera = {{kW, kH}, {40.0, 40.0, 15.5, 11.5}, {}};
    f.min_depth = 0.1f;
    f.max_depth = 5.0f;
    if (p.noise) f.depth_noise = {0.002f, 0.0f, 0.002f};
    f.color_to_world = sensor::camera::Mat4d(device_pose(p.pts));
    f.timestamp_ns = p.pts;
    f.sequence = p.seq;
    f.pixels = std::move(pixels);
    return std::optional<sensor::RgbdFrame>(std::move(f));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> f, poll());
    if (f) out->push_back(std::move(*f));
    return {};
  }
  bool exhausted() const noexcept override { return next_ >= script_.size(); }
  sensor::SensorStats stats() const noexcept override {
    sensor::SensorStats s{};
    s.received = next_;
    s.delivered = next_;
    return s;
  }

 private:
  std::vector<PhoneFrame> script_;
  std::size_t next_ = 0;
  bool started_ = false;
  sensor::SensorInfo info_;
};

// --- Scripts ----------------------------------------------------------------

track::TrackFit anchored_fit(const Mat4f& correction) {
  track::TrackFit fit;
  fit.correction = correction;
  fit.lambda_max = 1.0f;
  fit.relative_eigenvalues = {1, 1, 1, 1, 1, 1};
  for (int i = 0; i < 6; ++i) fit.directions[std::size_t(7 * i)] = 1.0f;
  fit.held = 0;
  fit.frames = 1;
  fit.attempts = 20000;
  fit.hits = 18000;
  fit.inliers = 17000;
  return fit;
}

// Anchored at `at` when `when(the newest frame's pts)`, else nothing solved.
ScriptedFitter::Script anchors_when(std::function<bool(std::uint64_t)> when,
                                    Mat4f at) {
  return [when = std::move(when), at](
             const std::vector<sensor::DeviceFrame>& frames,
             const track::PosePrior& prior) -> vkc::Result<track::TrackFit> {
    if (when(frames.back().timestamp_ns)) return anchored_fit(at);
    track::TrackFit none;
    none.correction = prior.correction;
    return none;
  };
}

bool never(std::uint64_t) { return false; }

struct Env {
  vkc::test::GpuContext& gpu;
  vol::VoxelBlockGrid& grid;
};

struct Lane {
  std::unique_ptr<MobileLane> lane;
  std::shared_ptr<const std::vector<vr_test::ScriptedRecord>> rig, track;
};

vkc::Result<Lane> make_lane(const Env& env, std::vector<PhoneFrame> phone,
                            ScriptedFitter::Script rig,
                            ScriptedFitter::Script track_script,
                            const std::string& dir = {}) {
  Lane out;
  auto r = std::make_unique<ScriptedFitter>(std::move(rig));
  auto t = std::make_unique<ScriptedFitter>(std::move(track_script));
  out.rig = r->log();
  out.track = t->log();
  std::vector<std::unique_ptr<track::IFitter>> fitters;
  fitters.push_back(std::move(r));
  fitters.push_back(std::move(t));
  fitters.emplace_back();  // arkit
  VKC_ASSIGN(
      std::unique_ptr<track::Tracker> tracker,
      track::Tracker::create(vr_example::lane_tracking(), std::move(fitters)));
  VKC_ASSIGN(std::unique_ptr<vr_example::TrackLog> log,
             vr_example::TrackLog::open(dir, *tracker, {}));
  VKC_ASSIGN(out.lane,
             MobileLane::create(env.gpu.device, env.gpu.allocator,
                                std::make_unique<FakePhone>(std::move(phone)),
                                std::move(tracker), std::move(log)));
  return out;
}

// What a send got: the set's predicted time and its fillers.
struct Sent {
  std::uint64_t next_set_ns = 0;
  std::vector<sender::Filler> fillers;
};

// `sets` sets from 1 s, `period` apart, each followed by the lane's gap.
vkc::Result<std::vector<Sent>> run(const Env& env, MobileLane& lane,
                                   std::size_t sets,
                                   std::uint64_t period = 40 * kMs) {
  std::vector<Sent> out;
  for (std::size_t i = 0; i < sets; ++i) {
    Sent s;
    s.next_set_ns = kS + (i + 1) * period;
    VKC_ASSIGN(s.fillers, lane.next(env.grid, s.next_set_ns));
    out.push_back(std::move(s));
  }
  return out;
}

std::vector<const sender::Filler*> all_fillers(const std::vector<Sent>& sent) {
  std::vector<const sender::Filler*> out;
  for (const Sent& s : sent) {
    for (const sender::Filler& f : s.fillers) out.push_back(&f);
  }
  return out;
}

// --- The cases --------------------------------------------------------------

// Tiers, tints and one generation.
int test_tiers(const Env& env) {
  const Mat4f rig_at = translate(0.02f, 0.0f, 0.0f);
  const Mat4f track_at = translate(0.0f, 0.03f, 0.0f);
  const auto rig_window = [](std::uint64_t t) {
    return t >= 3 * kS && t <= 4 * kS;
  };
  auto made =
      make_lane(env, ten_hz(60), anchors_when(rig_window, rig_at),
                anchors_when([](std::uint64_t) { return true; }, track_at));
  CHECK(made.ok());
  auto sent = run(env, *made->lane, 260);
  CHECK(sent.ok());
  // The rig's Anchored fits are those at 3.0, 3.5 and 4.0 s: they hold the
  // frames of (1, 4] s.
  std::size_t anchored = 0;
  std::size_t unanchored = 0;
  std::vector<std::uint32_t> words;
  for (const sender::Filler* f : all_fillers(*sent)) {
    const std::uint64_t t = f->frame.timestamp_ns;
    const bool by_rig = t > kS && t <= 4 * kS;
    CHECK(f->tier ==
          (by_rig ? tsdf::FillTier::Anchored : tsdf::FillTier::Unanchored));
    CHECK(f->index == 0);
    const Mat4f at = by_rig ? rig_at : track_at;
    CHECK(same(f->frame.depth_camera.cam_to_world, at * device_pose(t)));
    // Its colour is its tier's tint, through its depth camera.
    CHECK(f->frame.has_color());
    CHECK(f->frame.color_camera.width == kW &&
          f->frame.color_camera.height == kH);
    CHECK(same(f->frame.color_camera.cam_to_world,
               f->frame.depth_camera.cam_to_world));
    auto word = vkc::test::read_back<std::uint32_t>(
        env.gpu.device, env.gpu.allocator, *f->frame.color, 1, kW * kH - 1);
    CHECK(word.ok());
    CHECK((*word)[0] == (by_rig ? vr_example::kAnchoredFillTint
                                : vr_example::kUnanchoredFillTint));
    ++(by_rig ? anchored : unanchored);
  }
  std::fputs(made->lane->report().c_str(), stdout);
  CHECK(anchored > 0 && unanchored > 0);
  const MobileLane::Counts& c = made->lane->counts();
  CHECK(c.anchored == anchored && c.unanchored == unanchored && c.paired == 60);
  CHECK(made->lane->close().ok());
  // One generation: the rig chain's priors do not see the track chain's fits.
  auto quiet = make_lane(env, ten_hz(60), anchors_when(rig_window, rig_at),
                         anchors_when(never, track_at));
  CHECK(quiet.ok());
  CHECK(run(env, *quiet->lane, 260).ok());
  CHECK(quiet->lane->close().ok());
  CHECK(made->rig->size() == quiet->rig->size());
  for (std::size_t i = 0; i < made->rig->size(); ++i) {
    const auto& a = (*made->rig)[i];
    const auto& b = (*quiet->rig)[i];
    CHECK(a.pts == b.pts && same(a.prior.correction, b.prior.correction) &&
          a.prior.sigma_m == b.prior.sigma_m &&
          a.prior.sigma_rad == b.prior.sigma_rad);
  }
  return 0;
}

// The 5 s wait and the drop.
int test_wait_and_drop(const Env& env) {
  const auto anchors = [](std::uint64_t t) {
    return t == 1500 * kMs || t == 9 * kS;
  };
  auto made = make_lane(env, ten_hz(91), anchors_when(anchors, Mat4f(1.0f)),
                        anchors_when(never, Mat4f(1.0f)));
  CHECK(made.ok());
  auto sent = run(env, *made->lane, 400);
  CHECK(sent.ok());
  std::map<std::uint64_t, std::uint64_t> fused_at;  // pts -> its send's time
  std::size_t most = 0;
  for (const Sent& s : *sent) {
    most = std::max(most, s.fillers.size());
    for (const sender::Filler& f : s.fillers) {
      fused_at[f.frame.timestamp_ns] = s.next_set_ns;
    }
  }
  CHECK(most <= vr_example::kDrainCap);
  // 1.0-1.5 s Anchored by the 1.5 s fit, 7.1-9.0 s by the 9 s one. A frame
  // between waits 5 s for an Anchored fit after it: those of 4.0-6.5 s see
  // the 9 s fit in time (within 5 s of the 1.5 s one too) and fuse in the set
  // after it; earlier ones run out first, later ones and those after 9 s
  // have nothing within 5 s on one side. Those are dropped.
  for (std::uint64_t i = 0; i <= 90; ++i) {
    const std::uint64_t t = kS + i * 100 * kMs;
    const bool held = t <= 1500 * kMs || (t >= 7100 * kMs && t <= 9 * kS);
    const bool between = t >= 4 * kS && t <= 6500 * kMs;
    CHECK(fused_at.count(t) == ((held || between) ? 1u : 0u));
    if (between) CHECK(fused_at[t] >= 9 * kS);
  }
  const MobileLane::Counts& c = made->lane->counts();
  CHECK(c.dropped == 91 - fused_at.size());
  CHECK(c.fused == fused_at.size());
  CHECK(c.anchored == 6 + 20 && c.unanchored == 26);
  CHECK(made->lane->close().ok());
  return 0;
}

// Replays repeat, bit for bit.
int test_replay(const Env& env) {
  std::vector<std::vector<std::pair<std::uint64_t, Mat4f>>> runs;
  for (int r = 0; r < 2; ++r) {
    auto made = make_lane(
        env, ten_hz(50),
        anchors_when([](std::uint64_t t) { return t % (1500 * kMs) == 0; },
                     translate(0.01f, 0.0f, 0.0f)),
        anchors_when([](std::uint64_t t) { return t % kS == 0; },
                     translate(0.0f, 0.01f, 0.0f)));
    CHECK(made.ok());
    auto sent = run(env, *made->lane, 220);
    CHECK(sent.ok());
    std::vector<std::pair<std::uint64_t, Mat4f>> got;
    for (const sender::Filler* f : all_fillers(*sent)) {
      got.emplace_back(f->frame.timestamp_ns * 2 + std::uint64_t(f->tier),
                       f->frame.depth_camera.cam_to_world);
    }
    CHECK(made->lane->close().ok());
    runs.push_back(std::move(got));
  }
  CHECK(!runs[0].empty() && runs[0].size() == runs[1].size());
  for (std::size_t i = 0; i < runs[0].size(); ++i) {
    CHECK(runs[0][i].first == runs[1][i].first &&
          same(runs[0][i].second, runs[1][i].second));
  }
  return 0;
}

// A refused frame is passed over; the log, close and the phone's end.
int test_log_and_close(const Env& env, const std::filesystem::path& dir) {
  std::vector<PhoneFrame> phone = ten_hz(30);
  phone[7].noise = false;  // the tracker's frame check refuses it
  auto made = make_lane(
      env, phone, anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
      anchors_when(never, Mat4f(1.0f)), dir.string());
  CHECK(made.ok());
  // Sets past the phone's end: it flushes, and what it placed still drains.
  auto sent = run(env, *made->lane, 120);
  CHECK(sent.ok());
  const MobileLane::Counts& c = made->lane->counts();
  CHECK(c.passed_over == 1 && c.paired == 29 && c.polled == 30);
  // The fits at each 0.5 s to 3.5 s place all but the frames after the last,
  // 3.6-3.9 s, which the phone's end leaves with no fit: dropped.
  CHECK(c.fused == 25 && c.dropped == 4);
  CHECK(made->lane->close().ok());
  CHECK(made->lane->close().ok());  // idempotent
  for (const char* file :
       {"run.csv", "chains.csv", "frames.csv", "fits.csv", "fill.csv",
        "rows.csv", "poses_rig.tum", "poses_track.tum", "poses_arkit.tum"}) {
    CHECK(std::filesystem::exists(dir / file));
  }
  // Every frame tracked is decided by every chain.
  const vr_example::TrackRun& logged = made->lane->log()->run();
  std::map<std::size_t, std::size_t> per_chain;
  for (const auto& f : logged.frames) ++per_chain[f.chain];
  CHECK(per_chain.size() == 3);
  for (const auto& [chain, n] : per_chain) CHECK(n == 29);
  CHECK(made->lane->report().find("29 tracked") != std::string::npos);
  // No directory: the run is kept and summarized all the same.
  auto quiet =
      make_lane(env, ten_hz(10),
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)));
  CHECK(quiet.ok() && run(env, *quiet->lane, 10).ok());
  CHECK(quiet->lane->close().ok());
  CHECK(!quiet->lane->log()->run().frames.empty());
  return 0;
}

// A loss of 100 ms or more: the windows start again after it.
int test_split(const Env& env) {
  std::vector<PhoneFrame> phone = ten_hz(20);
  for (std::uint64_t i = 0; i < 20; ++i) {
    phone.push_back({kS + (23 + i) * 100 * kMs, 23 + i});  // 20-22 lost
  }
  // One lost record (100 ms of loss, 200 ms apart) splits too.
  phone.push_back({kS + 44 * 100 * kMs, 44});
  auto made = make_lane(
      env, phone, anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
      anchors_when(never, Mat4f(1.0f)));
  CHECK(made.ok());
  CHECK(run(env, *made->lane, 140).ok());
  CHECK(made->lane->counts().splits == 2);
  for (const auto& rec : *made->rig) {
    const bool before = rec.pts.front() < kS + 23 * 100 * kMs;
    const bool after = rec.pts.back() >= kS + 23 * 100 * kMs;
    CHECK(!(before && after));
  }
  CHECK(made->lane->close().ok());
  return 0;
}

// Pairing: one record a 25 Hz set; at 5 Hz the older of two is passed by.
int test_pairing(const Env& env) {
  auto fast =
      make_lane(env, ten_hz(20),
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)));
  CHECK(fast.ok() && run(env, *fast->lane, 60).ok());
  CHECK(fast->lane->counts().paired == 20 && fast->lane->counts().skipped == 0);
  CHECK(fast->lane->close().ok());
  auto slow =
      make_lane(env, ten_hz(20),
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)));
  CHECK(slow.ok() && run(env, *slow->lane, 15, 200 * kMs).ok());
  // The first record pairs alone, then two are due a set (the older passed
  // by), the last alone.
  CHECK(slow->lane->counts().paired == 11 && slow->lane->counts().skipped == 9);
  CHECK(slow->lane->close().ok());
  return 0;
}

int test_check_and_empty(const Env& env) {
  const auto parse = [](const std::string& extra) {
    return cfg::parse(R"({"version": 1, "operation": "fuse",
        "source": {"kind": "live", "sync_config": "sync.json",
                   "calibration": "poses.json")" +
                          extra + "}",
                      cfg::Application::FuseOrbbec);
  };
  auto rig = parse("}");
  auto phone = parse(R"(, "mobile": "phone.vrst"})");
  auto live = parse(R"(, "mobile": "tcp://10.8.0.1:7101",
      "mobile_registration": "phone.registration.json"})");
  auto classic =
      parse(R"(, "mobile": "phone.vrst"}, "fusion": {"dynamic": false})");
  CHECK(rig.ok() && phone.ok() && live.ok() && classic.ok());
  CHECK(vr_example::check_mobile(*rig).ok());
  CHECK(vr_example::check_mobile(*phone).ok());
  CHECK(vr_example::check_mobile(*live).ok());
  CHECK(vr_example::check_mobile(*classic).domain() ==
        vkc::Status::Code::InvalidArgument);
  cfg::Config ffs = *phone;
  ffs.source.depth = cfg::DepthSource::Ffs;
  const vkc::Status refused = vr_example::check_mobile(ffs);
  CHECK(refused.domain() == vkc::Status::Code::Unsupported);
  CHECK(refused.message().find("FFS") != std::string::npos);
  // The empty lane: no phone, nothing done.
  auto empty =
      vr_example::open_mobile_lane(*rig, env.gpu.device, env.gpu.allocator);
  CHECK(empty.ok());
  auto none = (*empty)->next(env.grid, 2 * kS);
  CHECK(none.ok() && none->empty());
  CHECK((*empty)->report().empty() && (*empty)->log() == nullptr);
  CHECK((*empty)->close().ok());
  // A phone needs its tracker and log.
  auto half =
      MobileLane::create(env.gpu.device, env.gpu.allocator,
                         std::make_unique<FakePhone>(ten_hz(1)), {}, {});
  CHECK(half.status().domain() == vkc::Status::Code::InvalidArgument);
  return 0;
}

// A sensor whose poll waits, as a live link's does, up to 20 ms for the
// frames the test lets through; an error on cue.
class GatedSensor final : public sensor::IRgbdSensor {
 public:
  explicit GatedSensor(std::size_t count) : count_(count) {}
  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override { return {}; }
  void stop() noexcept override {}
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!gate_.wait_for(lock, std::chrono::milliseconds(20),
                        [this] { return let_ > given_ || fail_; })) {
      return std::optional<sensor::RgbdFrame>{};
    }
    if (fail_) {
      fail_ = false;
      return vkc::Status::io_error("gated: an error");
    }
    sensor::RgbdFrame f;
    f.timestamp_ns = ++given_;
    return std::optional<sensor::RgbdFrame>(std::move(f));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>*) override { return {}; }
  bool exhausted() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    return given_ == count_;
  }
  sensor::SensorStats stats() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::SensorStats s{};
    s.received = given_;
    return s;
  }
  void let_through(std::size_t n) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      let_ = std::min(count_, let_ + n);
    }
    gate_.notify_all();
  }
  void fail_once() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fail_ = true;
    }
    gate_.notify_all();
  }

 private:
  sensor::SensorInfo info_;
  mutable std::mutex mutex_;
  std::condition_variable gate_;
  std::size_t count_;
  std::size_t let_ = 0;
  std::size_t given_ = 0;
  bool fail_ = false;
};

int test_background() {
  auto inner = std::make_unique<GatedSensor>(5);
  GatedSensor* gated = inner.get();
  auto polled = vr_example::polled_in_background(std::move(inner));
  CHECK(polled->start().ok());
  // Nothing let through: a poll returns at once, many times over, while the
  // inner poll waits.
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 100; ++i) {
    auto f = polled->poll();
    CHECK(f.ok() && !f->has_value());
  }
  CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(20));
  CHECK(!polled->exhausted());
  // Frames come through in order, an error once between them, then the end.
  const auto take = [&polled](std::vector<std::uint64_t>& got,
                              std::size_t want) {
    const auto until =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (got.size() < want && std::chrono::steady_clock::now() < until) {
      auto f = polled->poll();
      if (f.ok() && *f) got.push_back((*f)->timestamp_ns);
      if (!f.ok() || !*f) std::this_thread::yield();
    }
  };
  std::vector<std::uint64_t> got;
  gated->let_through(2);
  take(got, 2);
  CHECK((got == std::vector<std::uint64_t>{1, 2}));
  gated->fail_once();
  bool errored = false;
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!errored && std::chrono::steady_clock::now() < until) {
    auto f = polled->poll();
    errored = !f.ok() && f.status().domain() == vkc::Status::Code::IoError;
    if (!errored) std::this_thread::yield();
  }
  CHECK(errored);
  gated->let_through(3);
  take(got, 5);
  CHECK((got == std::vector<std::uint64_t>{1, 2, 3, 4, 5}));
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!polled->exhausted() && std::chrono::steady_clock::now() < end) {
    std::this_thread::yield();
  }
  CHECK(polled->exhausted() && polled->stats().received == 5);
  polled->stop();
  // Stopped while the inner poll waits for frames that never come.
  auto idle =
      vr_example::polled_in_background(std::make_unique<GatedSensor>(3));
  CHECK(idle->start().ok());
  idle->stop();
  idle->stop();
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::test::Scratch scratch("recon_example_live_mobile_");
  auto grid =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, 0.05f, 0.15f, 64);
  CHECK(grid.ok());
  const Env env{gpu, *grid};
  CHECK(test_tiers(env) == 0);
  CHECK(test_wait_and_drop(env) == 0);
  CHECK(test_replay(env) == 0);
  CHECK(test_log_and_close(env, scratch.path / "lane") == 0);
  CHECK(test_split(env) == 0);
  CHECK(test_pairing(env) == 0);
  CHECK(test_check_and_empty(env) == 0);
  CHECK(test_background() == 0);
  std::puts("recon_example_live_mobile: OK");
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
