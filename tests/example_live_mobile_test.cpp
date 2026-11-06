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
//   both sides, a frame alone between two losses included. An error the
//   phone returns is counted and the lane goes on.
// - Pairing: one record a set at 25 Hz sets, the older of two passed by at
//   5 Hz sets. A live link's backlog from the cameras' setup, taken at once
//   or over several gaps, is passed by but for each take's newest, and the
//   records after it are tracked in the first gap after they came.
// - check_mobile: no phone, or a phone under Dynamic, runs; Classic and FFS
//   depth are refused. A phone with no registration needs the origin tag: a
//   build without VR_WITH_CALIB refuses it, naming the option, before the
//   wait too; one with it takes a tag world and refuses another. The empty
//   lane does nothing.
// - polled_in_background: a poll never waits on the inner sensor's; it keeps
//   the interface's queue (a poll the newest, a drain every frame oldest
//   first, a full queue letting its oldest go); an inner error and the end
//   come through; stop joins while the inner poll waits.
// - What the phone is told: lane_note's every line and tone, in order, its 3 s
//   to the nanosecond, and note_due's rate, with no device. A lane with no tag
//   tells nothing, whatever it is given.

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
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace vkc = volumetric_kit::core;
namespace cfg = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace sender = volumetric_kit::recon::sender;
namespace vol = volumetric_kit::grid::volume;
namespace gnet = volumetric_kit::grid::networking;
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
    if (fail_at_ && next_ == *fail_at_) {
      fail_at_.reset();
      return vkc::Status::io_error("fake phone: a damaged record");
    }
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> f, poll());
    if (f) out->push_back(std::move(*f));
    return {};
  }
  // The drain before record `n` returns an error, once.
  void fail_before(std::size_t n) { fail_at_ = n; }
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
  std::optional<std::size_t> fail_at_;
  bool started_ = false;
  sensor::SensorInfo info_;
};

// How long after its capture a live phone's record comes.
constexpr std::uint64_t kLatency = 50 * kMs;

// The script's records as a live link hands them out: each comes kLatency
// after its capture, and a drain takes those come by `*now`, oldest first,
// at most `per_drain` (the rest wait in the link), each stamped with its
// arrival.
class LivePhone final : public sensor::IRgbdSensor {
 public:
  LivePhone(const std::vector<PhoneFrame>& script, const std::uint64_t* now,
            std::size_t per_drain)
      : inner_(script), script_(script), now_(now), per_drain_(per_drain) {}
  const sensor::SensorInfo& info() const noexcept override {
    return inner_.info();
  }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override { return inner_.start(); }
  void stop() noexcept override { inner_.stop(); }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    return std::optional<sensor::RgbdFrame>{};
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    for (std::size_t n = 0; n < per_drain_ && next_ < script_.size() &&
                            script_[next_].pts + kLatency <= *now_;
         ++n, ++next_) {
      VKC_ASSIGN(std::optional<sensor::RgbdFrame> f, inner_.poll());
      f->arrival_ns = *now_;
      out->push_back(std::move(*f));
    }
    return {};
  }
  bool exhausted() const noexcept override { return inner_.exhausted(); }
  sensor::SensorStats stats() const noexcept override { return inner_.stats(); }

 private:
  FakePhone inner_;
  std::vector<PhoneFrame> script_;
  const std::uint64_t* now_;
  std::size_t per_drain_;
  std::size_t next_ = 0;
};

// --- Scripts ----------------------------------------------------------------

using vr_test::anchors_when;

bool never(std::uint64_t) { return false; }

struct Env {
  vkc::test::GpuContext& gpu;
  vol::VoxelBlockGrid& grid;
};

struct Lane {
  std::unique_ptr<MobileLane> lane;
  std::shared_ptr<const std::vector<vr_test::ScriptedRecord>> rig, track;
  FakePhone* phone = nullptr;  // the lane's
};

// The lane on `phone`, tracked by the two scripts.
vkc::Result<Lane> make_lane(const Env& env,
                            std::unique_ptr<sensor::IRgbdSensor> phone,
                            ScriptedFitter::Script rig,
                            ScriptedFitter::Script track_script,
                            const std::string& dir = {},
                            std::function<void(gnet::ViewerNote)> say = {}) {
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
  VKC_ASSIGN(out.lane, MobileLane::create(env.gpu.device, env.gpu.allocator,
                                          std::move(phone), std::move(tracker),
                                          std::move(log), {}, std::move(say)));
  return out;
}

// The lane on a fake phone of `script`.
vkc::Result<Lane> make_lane(const Env& env, std::vector<PhoneFrame> script,
                            ScriptedFitter::Script rig,
                            ScriptedFitter::Script track_script,
                            const std::string& dir = {}) {
  auto fake = std::make_unique<FakePhone>(std::move(script));
  FakePhone* phone = fake.get();
  VKC_ASSIGN(Lane out, make_lane(env, std::move(fake), std::move(rig),
                                 std::move(track_script), dir));
  out.phone = phone;
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
  // A frame alone between two losses: the lane takes the second loss's
  // record while the first's is due, and still splits at both.
  std::vector<PhoneFrame> lonely = ten_hz(10);  // 1.0-1.9 s
  lonely.push_back({kS + 12 * 100 * kMs, 12});  // 2.2 s, after 10-11 lost
  for (std::uint64_t i = 15; i < 26; ++i) {     // from 2.5 s, 13-14 lost
    lonely.push_back({kS + i * 100 * kMs, i});
  }
  auto twice =
      make_lane(env, lonely,
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)));
  CHECK(twice.ok() && run(env, *twice->lane, 100).ok());
  CHECK(twice->lane->counts().splits == 2);
  for (const auto& rec : *twice->rig) {
    for (const std::uint64_t loss :
         {kS + 12 * 100 * kMs, kS + 15 * 100 * kMs}) {
      CHECK(!(rec.pts.front() < loss && rec.pts.back() >= loss));
    }
  }
  CHECK(twice->lane->close().ok());
  // An error the phone returns is counted, and the lane goes on.
  auto failing =
      make_lane(env, ten_hz(20),
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)));
  CHECK(failing.ok());
  failing->phone->fail_before(5);
  CHECK(run(env, *failing->lane, 80).ok());
  CHECK(failing->lane->counts().errors == 1 &&
        failing->lane->counts().paired == 20);
  CHECK(failing->lane->report().find("a damaged record") != std::string::npos);
  CHECK(failing->lane->close().ok());
  return 0;
}

// Pairing: one record a 25 Hz set, at the phone's own pace (the first, at
// 1.0 s, pairs with the set at 1.04 s, so the last, at 2.9 s, with the one
// at 2.94 s); at 5 Hz the older of two is passed by.
int test_pairing(const Env& env) {
  auto fast =
      make_lane(env, ten_hz(20),
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)));
  CHECK(fast.ok());
  for (std::size_t i = 0; i < 60; ++i) {
    const std::uint64_t next_set = kS + (i + 1) * 40 * kMs;
    CHECK(fast->lane->next(env.grid, next_set).ok());
    // Record k is due by the set at 1.04 s + k * 0.1 s.
    CHECK(fast->lane->counts().paired ==
          std::min<std::size_t>(20, 2 * i / 5 + 1));
  }
  CHECK(fast->lane->counts().skipped == 0);
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

// A live link's backlog: records at 10 Hz from 1 s, each coming 50 ms after
// capture, the lane's first gap at 1 s (none) or at 4 s (3 s of setup), the
// backlog taken at once or 8 records a gap. Every record is tracked or
// passed by, none split; of the backlog only a take's newest is tracked; and
// every record captured from half a second after the first gap on is
// tracked in the first gap after it came.
int test_live_backlog(const Env& env) {
  struct Case {
    std::uint64_t first_gap;
    std::size_t per_drain;
  };
  for (const Case c : {Case{kS, 1000}, Case{4 * kS, 1000}, Case{4 * kS, 8}}) {
    std::uint64_t now = 0;
    const std::vector<PhoneFrame> script = ten_hz(60);  // the last at 6.9 s
    auto made =
        make_lane(env, std::make_unique<LivePhone>(script, &now, c.per_drain),
                  anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                  anchors_when(never, Mat4f(1.0f)));
    CHECK(made.ok());
    const auto& frames = made->lane->log()->run().frames;
    std::size_t seen = 0;
    std::size_t fresh = 0;
    std::size_t backlog = 0;
    for (now = c.first_gap; now <= 7 * kS; now += 40 * kMs) {
      CHECK(made->lane->next(env.grid, now + 40 * kMs).ok());
      for (; seen < frames.size(); ++seen) {
        if (frames[seen].chain != 2) continue;  // arkit: decided as recorded
        const std::uint64_t pts = frames[seen].pts_ns;
        if (pts < c.first_gap) ++backlog;
        if (pts < c.first_gap + 500 * kMs) continue;
        CHECK(now >= pts + kLatency && now < pts + kLatency + 40 * kMs);
        ++fresh;
      }
    }
    const MobileLane::Counts& n = made->lane->counts();
    CHECK(n.polled == 60 && n.paired + n.skipped == 60);
    CHECK(n.passed_over == 0 && n.splits == 0);
    CHECK(fresh == std::size_t(60 - (c.first_gap - kS) / (100 * kMs) - 5));
    const std::size_t takes = c.first_gap == kS ? 0 : 30 / c.per_drain + 1;
    CHECK(backlog <= takes);
    CHECK(made->lane->close().ok());
  }
  return 0;
}

int test_check_and_empty(const Env& env, const std::filesystem::path& dir) {
  // Two calibration files: the world a tag, and the world a sensor.
  namespace camera = sensor::camera;
  std::filesystem::create_directories(dir);
  camera::ArrayCalibration array;
  array.sensors.push_back({"rig-0", camera::Mat4d(1.0), {}, {}, {}, 0.0});
  array.world.apriltag = camera::AprilTagWorld{"tag36h11", 0, 0.15};
  const std::string tag_world = (dir / "tag_world.json").string();
  CHECK(
      camera::write_array_calibration(tag_world, array, vkc::WriteMode::Replace)
          .ok());
  array.world = {"rig-0", std::nullopt};
  const std::string sensor_world = (dir / "sensor_world.json").string();
  CHECK(camera::write_array_calibration(sensor_world, array,
                                        vkc::WriteMode::Replace)
            .ok());
  const auto parse = [](const std::string& extra,
                        const std::string& calibration = "poses.json") {
    return cfg::parse(R"({"version": 1, "operation": "fuse",
        "source": {"kind": "live", "sync_config": "sync.json",
                   "calibration": ")" +
                          calibration + "\"" + extra + "}",
                      cfg::Application::FuseOrbbec);
  };
  auto rig = parse("}");
  auto phone = parse(R"(, "mobile": "phone.vrst",
      "mobile_registration": "phone.registration.json"})");
  auto live = parse(R"(, "mobile": "tcp://10.8.0.1:7101",
      "mobile_registration": "phone.registration.json"})");
  auto classic = parse(R"(, "mobile": "phone.vrst",
      "mobile_registration": "phone.registration.json"},
      "fusion": {"dynamic": false})");
  CHECK(rig.ok() && phone.ok() && live.ok() && classic.ok());
  CHECK(vr_example::check_mobile(*rig).ok());
  CHECK(vr_example::check_mobile(*phone).ok());
  CHECK(vr_example::check_mobile(*live).ok());
  CHECK(vr_example::check_mobile(*classic).domain() ==
        vkc::Status::Code::InvalidArgument);
  // Without a registration (the core defaults none since 0.49.0) the origin
  // tag registers the phone, which the build and the calibration's world
  // must allow: checked before any device opens, and again before any wait.
  auto by_tag = parse(R"(, "mobile": "tcp://10.8.0.1:7101"})", tag_world);
  auto by_sensor = parse(R"(, "mobile": "phone.vrst"})", sensor_world);
  auto no_file = parse(R"(, "mobile": "phone.vrst"})");
  CHECK(by_tag.ok() && by_sensor.ok() && no_file.ok());
  CHECK(vr_example::check_mobile(*no_file).domain() ==
        vkc::Status::Code::NotFound);
  const vkc::Status tagged = vr_example::check_mobile(*by_tag);
  const vkc::Status sensored = vr_example::check_mobile(*by_sensor);
#ifdef VR_TEST_WITH_CALIB
  CHECK(tagged.ok());
  CHECK(sensored.domain() == vkc::Status::Code::InvalidArgument);
  CHECK(sensored.message().find("source.mobile_registration") !=
        std::string::npos);
#else
  // live_tag_off.cpp: refused, naming the option and the key.
  for (const vkc::Status& s : {tagged, sensored}) {
    CHECK(s.domain() == vkc::Status::Code::Unsupported);
    CHECK(s.message().find("-DVR_WITH_CALIB=ON") != std::string::npos);
    CHECK(s.message().find("source.mobile_registration") != std::string::npos);
  }
  CHECK(vr_example::open_mobile_lane(*by_tag, env.gpu.device, env.gpu.allocator)
            .status()
            .domain() == vkc::Status::Code::Unsupported);
  CHECK(vr_example::TagWatch::create(env.gpu.device, env.gpu.allocator, {})
            .status()
            .domain() == vkc::Status::Code::Unsupported);
#endif
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

bool reads(const gnet::ViewerNote& note, gnet::ViewerNote::Tone tone,
           const char* line) {
  if (note.tone == tone && note.line == line) return true;
  std::fprintf(stderr, "reads \"%s\" at tone %d, not \"%s\" at %d\n",
               note.line.c_str(), int(note.tone), line, int(tone));
  return false;
}

// Every line and tone, in order, by hand. No device.
int test_lane_note() {
  using Tone = gnet::ViewerNote::Tone;
  using Status = MobileLane::Status;
  using vr_example::lane_note;
  using vr_example::note_due;
  const std::uint64_t t = 100 * kS;
  CHECK(reads(lane_note(Status{}, t), Tone::Neutral, "Looking for the tag"));
  CHECK(
      reads(lane_note(Status{false, 1, 0}, t), Tone::Good, "Tag seen: 1 of 6"));
  CHECK(
      reads(lane_note(Status{false, 5, 0}, t), Tone::Good, "Tag seen: 5 of 6"));
  // A refusal reads for 3 s of set time, to the nanosecond, over the count,
  // which then reads, or nothing does; and for a set time before it.
  const Status refused{false, 2, t};
  const char* const kRefused = "Tag refused, trying again";
  CHECK(reads(lane_note(refused, t), Tone::Caution, kRefused));
  CHECK(reads(lane_note(refused, t + 3 * kS - 1), Tone::Caution, kRefused));
  CHECK(reads(lane_note(refused, t - kS), Tone::Caution, kRefused));
  CHECK(reads(lane_note(refused, t + 3 * kS), Tone::Good, "Tag seen: 2 of 6"));
  CHECK(reads(lane_note(Status{false, 0, t}, t + 3 * kS), Tone::Neutral,
              "Looking for the tag"));
  // Registered reads over all of it, and stays.
  const char* const kFound = "Tag found, registered";
  CHECK(reads(lane_note(Status{true, 0, 0}, t), Tone::Good, kFound));
  CHECK(reads(lane_note(Status{true, 4, t}, t), Tone::Good, kFound));
  // The first, a change of line or tone, and once a second, to the nanosecond.
  const gnet::ViewerNote now = lane_note(Status{}, t);
  CHECK(note_due(now, std::nullopt, 0));
  CHECK(!note_due(now, now, 0));
  CHECK(!note_due(now, now, kS - 1));
  CHECK(note_due(now, now, kS));
  gnet::ViewerNote other = now;
  other.line += ".";
  CHECK(note_due(other, now, 0));
  other = now;
  other.tone = Tone::Caution;
  CHECK(note_due(other, now, 0));
  other = now;
  other.received = std::chrono::steady_clock::now();  // not on the wire
  CHECK(!note_due(other, now, 0));
  return 0;
}

// A lane with no tag has no search to tell of.
int test_no_tag_tells_nothing(const Env& env) {
  std::size_t told = 0;
  auto made =
      make_lane(env, std::make_unique<FakePhone>(ten_hz(5)),
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)),
                {}, [&told](gnet::ViewerNote) { ++told; });
  CHECK(made.ok());
  CHECK(run(env, *made->lane, 15).ok());
  CHECK(made->lane->close().ok());
  CHECK(made->lane->counts().paired > 0 && told == 0);
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
  // Until the inner sensor has received n (bounded), yielding.
  const auto received = [](sensor::IRgbdSensor& s, std::uint64_t n) {
    const auto until =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (s.stats().received < n && std::chrono::steady_clock::now() < until) {
      std::this_thread::yield();
    }
    return s.stats().received >= n;
  };
  // The default queue of 1: a poll takes the newest, the older let go.
  gated->let_through(2);
  CHECK(received(*polled, 2));
  auto newest = polled->poll();
  CHECK(newest.ok() && newest->has_value() && (*newest)->timestamp_ns == 2);
  CHECK(polled->stats().dropped == 1 && polled->stats().delivered == 1);
  // An error once, then the frames again; the end.
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
  CHECK(received(*polled, 5));
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!polled->exhausted() && std::chrono::steady_clock::now() < end) {
    std::vector<sensor::RgbdFrame> rest;
    CHECK(polled->drain(&rest).ok());
    std::this_thread::yield();
  }
  CHECK(polled->exhausted() && polled->stats().received == 5);
  polled->stop();
  // A queue of 3: a drain takes every frame held, oldest first; a full
  // queue lets its oldest go.
  auto deep_inner = std::make_unique<GatedSensor>(5);
  GatedSensor* deep_gate = deep_inner.get();
  auto deep = vr_example::polled_in_background(std::move(deep_inner));
  CHECK(deep->set_queue_depth(3).ok());
  CHECK(deep->start().ok());
  CHECK(deep->set_queue_depth(4).domain() ==
        vkc::Status::Code::InvalidArgument);
  deep_gate->let_through(2);
  CHECK(received(*deep, 2));
  std::vector<sensor::RgbdFrame> two;
  CHECK(deep->drain(&two).ok() && two.size() == 2);
  CHECK(two[0].timestamp_ns == 1 && two[1].timestamp_ns == 2);
  deep_gate->let_through(3);
  CHECK(received(*deep, 5));
  std::vector<sensor::RgbdFrame> three;
  CHECK(deep->drain(&three).ok() && three.size() == 3);
  CHECK(three[0].timestamp_ns == 3 && three[2].timestamp_ns == 5);
  CHECK(deep->stats().dropped == 0 && deep->stats().delivered == 5);
  deep->stop();
  auto small_inner = std::make_unique<GatedSensor>(5);
  GatedSensor* small_gate = small_inner.get();
  auto small = vr_example::polled_in_background(std::move(small_inner));
  CHECK(small->set_queue_depth(2).ok() && small->start().ok());
  small_gate->let_through(5);
  CHECK(received(*small, 5));
  std::vector<sensor::RgbdFrame> kept;
  CHECK(small->drain(&kept).ok() && kept.size() == 2);
  CHECK(kept[0].timestamp_ns == 4 && kept[1].timestamp_ns == 5);
  CHECK(small->stats().dropped == 3);
  small->stop();
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
  CHECK(test_live_backlog(env) == 0);
  CHECK(test_check_and_empty(env, scratch.path / "check") == 0);
  CHECK(test_no_tag_tells_nothing(env) == 0);
  CHECK(test_background() == 0);
  std::puts("recon_example_live_mobile: OK");
  return 0;
}

}  // namespace

int main() {
  if (test_lane_note() != 0) return 1;
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
