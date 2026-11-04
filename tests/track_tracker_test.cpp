// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// track::Tracker on scripted fitters (scripted_fitter.hpp): no kernel runs,
// and the device only gives the calls their batch and model. The rules of
// tracker.hpp, each a case: arkit poses every frame at once; shadows change
// nothing the driver does; one generation (the rig chain's priors never see
// the fill chain's fits); windows and hops; the four decisions, Interpolated
// against a hand interpolation; every release branch with two driver chains,
// the tie and the drop, and one chain; the judge through ChainFit (gate order,
// the first lock, a repeated jump, a Partial fit's strong directions, sigma's
// growth and reset, the inlier ratio over hits, the see-through check, a fit
// that solved nothing); the rows per chain; the contract (refusals, errors
// that leave nothing held, abandon, flush, create's refusals); replays repeat;
// and corrected() moves both cameras.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "scripted_fitter.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/track/tracker.hpp"

namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec3f;
using volumetric_kit::recon::Vec4f;
using vr_test::ScriptedFitter;

// `lhs = *expr` for a test returning an exit code: on an error, print it and
// return 1.
#define TRACK_TEST_CAT2(a, b) a##b
#define TRACK_TEST_CAT(a, b) TRACK_TEST_CAT2(a, b)
#define ASSIGN_OR_FAIL(lhs, expr) \
  ASSIGN_OR_FAIL_IMPL(TRACK_TEST_CAT(assigned_, __LINE__), lhs, expr)
#define ASSIGN_OR_FAIL_IMPL(tmp, lhs, expr)                      \
  auto tmp = (expr);                                             \
  if (!tmp.ok()) {                                               \
    std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                 tmp.status().message().c_str());                \
    return 1;                                                    \
  }                                                              \
  lhs = std::move(*tmp)

namespace {

constexpr std::uint64_t kMs = 1'000'000;
constexpr std::uint64_t kS = 1'000'000'000;
constexpr double kPi = 3.14159265358979323846;

Mat4f translate(float x, float y, float z) {
  Mat4f m(1.0f);
  m[3] = Vec4f(x, y, z, 1.0f);
  return m;
}

Mat4f rotate_z(double deg) {
  const auto c = float(std::cos(deg * kPi / 180.0));
  const auto s = float(std::sin(deg * kPi / 180.0));
  Mat4f m(1.0f);
  m[0] = Vec4f(c, s, 0, 0);
  m[1] = Vec4f(-s, c, 0, 0);
  return m;
}

bool same(const Mat4f& a, const Mat4f& b) {
  return std::memcmp(&a, &b, sizeof(Mat4f)) == 0;
}

bool near(const Mat4f& a, const Mat4f& b, float tol = 1e-5f) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (std::fabs(a[c][r] - b[c][r]) > tol) return false;
    }
  }
  return true;
}

// A frame at `pts`, its buffers empty (a scripted fitter never reads them).
sensor::DeviceFrame frame_at(std::uint64_t pts, const Mat4f& pose = Mat4f(1.0f),
                             std::uint32_t width = 32) {
  sensor::DeviceFrame f;
  f.depth = std::make_shared<const vkc::Buffer>();
  f.depth_camera = {40.0f, 40.0f, 15.5f, 11.5f, 0.1f, 5.0f, width, 24, pose};
  f.color_camera = {40.0f, 40.0f, 15.5f, 11.5f, width, 24, pose};
  f.timestamp_ns = pts;
  f.depth_noise = {0.002f, 0.0f, 0.002f};
  return f;
}

// A fit every gate takes, at `correction`.
track::TrackFit anchored(const Mat4f& correction) {
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

// A fit that solved nothing.
track::TrackFit unsolved(const Mat4f& correction) {
  track::TrackFit fit;
  fit.correction = correction;
  for (int i = 0; i < 6; ++i) fit.directions[std::size_t(7 * i)] = 1.0f;
  return fit;
}

using Script = ScriptedFitter::Script;

// Anchored at the prior when `anchor(newest pts)`, else nothing solved.
Script anchors_when(std::function<bool(std::uint64_t)> anchor,
                    Mat4f at = Mat4f(1.0f)) {
  return [anchor = std::move(anchor), at](
             const std::vector<sensor::DeviceFrame>& frames,
             const track::PosePrior&) -> vkc::Result<track::TrackFit> {
    if (anchor(frames.back().timestamp_ns)) return anchored(at);
    return unsolved(at);
  };
}

Script refuses() {
  return [](const std::vector<sensor::DeviceFrame>&,
            const track::PosePrior&) -> vkc::Result<track::TrackFit> {
    return vkc::Status::invalid_argument("scripted refusal");
  };
}

// A tracker on scripted fitters, one script per chain with a fit kind.
struct Rig {
  std::unique_ptr<track::Tracker> tracker;
  std::vector<ScriptedFitter*> fitters;  // per chain; null without a fit kind
};

vkc::Result<Rig> make(const track::TrackerConfig& config,
                      const std::vector<Script>& scripts,
                      const std::vector<double>& costs = {}) {
  std::vector<std::unique_ptr<track::IFitter>> fitters;
  Rig rig;
  std::size_t chains = config.drivers.size() + config.shadows.size();
  for (std::size_t c = 0; c < chains; ++c) {
    const track::Method& m = c < config.drivers.size()
                                 ? config.driver
                                 : config.shadows[c - config.drivers.size()];
    if (!m.fit) {
      fitters.emplace_back();
      rig.fitters.push_back(nullptr);
      continue;
    }
    auto f = std::make_unique<ScriptedFitter>(
        scripts.at(c), c < costs.size() ? costs[c] : 0.0);
    rig.fitters.push_back(f.get());
    fitters.push_back(std::move(f));
  }
  VKC_ASSIGN(rig.tracker, track::Tracker::create(config, std::move(fitters)));
  return rig;
}

// A driver of single-frame fits at every frame, as the release cases want.
track::TrackerConfig every_frame(std::vector<std::string> drivers,
                                 std::uint64_t hold = kS) {
  track::TrackerConfig c;
  c.driver = track::frame_method();
  for (const std::string& name : drivers) c.drivers.push_back({name, {}});
  c.shadows = {};
  c.hold_ns = hold;
  return c;
}

struct Env {
  vkc::test::GpuContext& gpu;
  vol::VoxelBlockGrid& grid;
};

vkc::Result<track::TrackStep> step(const Env& env, track::Tracker& t,
                                   sensor::DeviceFrame f,
                                   vkc::StageMetrics* rows = nullptr) {
  vkc::CommandBatch batch(env.gpu.device, env.gpu.allocator);
  VKC_ASSIGN(const track::PendingStep p,
             t.record(batch, env.grid, std::move(f), rows));
  return t.finish(p);
}

// Every step of a run, then its flush.
vkc::Result<std::vector<track::TrackStep>> run(
    const Env& env, track::Tracker& t,
    const std::vector<sensor::DeviceFrame>& frames) {
  std::vector<track::TrackStep> out;
  for (const sensor::DeviceFrame& f : frames) {
    VKC_ASSIGN(track::TrackStep s, step(env, t, f));
    out.push_back(std::move(s));
  }
  out.push_back(t.flush());
  return out;
}

std::vector<sensor::DeviceFrame> frames_every(std::uint64_t first,
                                              std::uint64_t period,
                                              std::size_t n) {
  std::vector<sensor::DeviceFrame> out;
  for (std::size_t i = 0; i < n; ++i)
    out.push_back(frame_at(first + i * period));
  return out;
}

// A step's driver half, in words (the fit serials named by chain and time,
// as shadows renumber them), for bit-identity checks: hex floats.
std::string driver_half(const std::vector<track::TrackStep>& steps,
                        std::size_t drivers) {
  std::string s;
  char buf[96];
  const auto mat = [&](const Mat4f& m) {
    for (int c = 0; c < 4; ++c) {
      for (int r = 0; r < 4; ++r) {
        std::snprintf(buf, sizeof(buf), "%a,", double(m[c][r]));
        s += buf;
      }
    }
  };
  std::map<std::uint64_t, std::string> fit_name;
  for (std::size_t i = 0; i < steps.size(); ++i) {
    s += "step " + std::to_string(i) + "\n";
    for (const track::ChainFit& f : steps[i].fits) {
      fit_name[f.serial] =
          std::to_string(f.chain) + "@" + std::to_string(f.last_pts_ns);
      if (f.chain >= drivers) continue;
      s += "fit " + fit_name[f.serial] + " " + std::to_string(f.first_pts_ns) +
           " " + std::to_string(int(f.judged.state)) + " ";
      mat(f.judged.correction);
      std::snprintf(buf, sizeof(buf), "%a %a\n", double(f.prior.sigma_m),
                    double(f.prior.sigma_rad));
      s += buf;
    }
    for (const track::ChainPose& p : steps[i].poses) {
      if (p.chain >= drivers) continue;
      s += "pose " + std::to_string(p.chain) + " " +
           std::to_string(p.frame.timestamp_ns) + " " +
           std::to_string(int(p.pose.from)) + " " +
           (p.pose.fit != 0 ? fit_name[p.pose.fit] : std::string("-")) + " " +
           std::to_string(p.pose.decided_pts_ns) + " ";
      mat(p.pose.correction);
      s += "\n";
    }
    for (const track::Released& r : steps[i].released) {
      s += "released " + std::to_string(r.frame.timestamp_ns) + " " +
           std::to_string(int(r.placement)) + " " + std::to_string(r.driver) +
           " ";
      mat(r.correction);
      s += "\n";
    }
    for (const std::uint64_t d : steps[i].dropped_ns) {
      s += "dropped " + std::to_string(d) + "\n";
    }
  }
  return s;
}

// The pose a chain decided for the frame at `pts`, over a run.
std::optional<track::FramePose> pose_of(
    const std::vector<track::TrackStep>& steps, std::size_t chain,
    std::uint64_t pts) {
  for (const track::TrackStep& s : steps) {
    for (const track::ChainPose& p : s.poses) {
      if (p.chain == chain && p.frame.timestamp_ns == pts) return p.pose;
    }
  }
  return std::nullopt;
}

// The release of the frame at `pts`, and the step it came in.
struct ReleaseAt {
  track::Released released;
  std::size_t step = 0;
};
std::optional<ReleaseAt> release_of(const std::vector<track::TrackStep>& steps,
                                    std::uint64_t pts) {
  for (std::size_t i = 0; i < steps.size(); ++i) {
    for (const track::Released& r : steps[i].released) {
      if (r.frame.timestamp_ns == pts) return ReleaseAt{r, i};
    }
  }
  return std::nullopt;
}

bool dropped(const std::vector<track::TrackStep>& steps, std::uint64_t pts) {
  for (const track::TrackStep& s : steps) {
    for (const std::uint64_t d : s.dropped_ns) {
      if (d == pts) return true;
    }
  }
  return false;
}

// 1. arkit: the device's pose, decided on its own frame.
int test_device(const Env& env) {
  track::TrackerConfig c = every_frame({"rig"});
  c.shadows = {track::device_method()};
  ASSIGN_OR_FAIL(Rig rig,
                 make(c, {anchors_when([](auto) { return true; }), {}}));
  for (std::uint64_t i = 0; i < 5; ++i) {
    ASSIGN_OR_FAIL(const track::TrackStep s,
                   step(env, *rig.tracker, frame_at(i * 100 * kMs)));
    bool seen = false;
    for (const track::ChainPose& p : s.poses) {
      if (p.chain != 1) continue;
      CHECK(!seen);
      seen = true;
      CHECK(p.frame.timestamp_ns == i * 100 * kMs);
      CHECK(p.pose.from == track::PoseFrom::Device);
      CHECK(same(p.pose.correction, Mat4f(1.0f)));
      CHECK(p.pose.decided_pts_ns == i * 100 * kMs);
      CHECK(p.pose.fit == 0);
    }
    CHECK(seen);
  }
  CHECK(rig.tracker->chains() == (std::vector<std::string>{"rig", "arkit"}));
  return 0;
}

// The driver's script for the shadow and replay cases: anchors three hops in
// four, at a small offset that drifts with time.
Script drifting() {
  return [](const std::vector<sensor::DeviceFrame>& frames,
            const track::PosePrior&) -> vkc::Result<track::TrackFit> {
    const std::uint64_t t = frames.back().timestamp_ns;
    const auto x = float(double(t) * 1e-9 * 0.004);
    if ((t / (500 * kMs)) % 4 == 3) return unsolved(translate(x, 0, 0));
    track::TrackFit fit = anchored(translate(x, 0.001f, 0) * rotate_z(0.05));
    fit.frames = std::uint32_t(frames.size());
    return fit;
  };
}

// 2. Shadows change nothing the driver does, nor when its fits record.
int test_shadows_isolated(const Env& env) {
  track::TrackerConfig base;
  base.drivers = {{"rig", {}}, {"track", {}}};
  base.shadows = {};
  const auto frames = frames_every(30 * kMs, 100 * kMs, 80);
  ASSIGN_OR_FAIL(Rig a, make(base, {drifting(), drifting()}));
  ASSIGN_OR_FAIL(const auto steps_a, run(env, *a.tracker, frames));
  track::TrackerConfig shadowed = base;
  track::Method long_window = track::window_method();
  long_window.name = "window4";
  long_window.window_ns = 4 * kS;
  track::Method refuser = track::frame_method();
  refuser.name = "refuser";
  shadowed.shadows = {track::device_method(), track::frame_method(),
                      long_window, refuser};
  ASSIGN_OR_FAIL(
      Rig b,
      make(shadowed,
           {drifting(), drifting(), {}, drifting(), drifting(), refuses()}));
  ASSIGN_OR_FAIL(const auto steps_b, run(env, *b.tracker, frames));
  CHECK(driver_half(steps_a, 2) == driver_half(steps_b, 2));
  // The driver's records, frame for frame and prior for prior.
  for (std::size_t c = 0; c < 2; ++c) {
    const auto& la = *a.fitters[c]->log();
    const auto& lb = *b.fitters[c]->log();
    CHECK(la.size() == lb.size() && !la.empty());
    for (std::size_t i = 0; i < la.size(); ++i) {
      CHECK(la[i].pts == lb[i].pts);
      CHECK(same(la[i].prior.correction, lb[i].prior.correction));
    }
  }
  // The refusing shadow's fits read as refused, and the steps went on.
  std::size_t refused = 0;
  for (const auto& s : steps_b) {
    for (const auto& f : s.fits) {
      if (f.chain == 5) {
        CHECK(f.refused == "scripted refusal");
        CHECK(f.judged.state == track::TrackState::Coasting);
        ++refused;
      }
    }
  }
  CHECK(refused == frames.size());
  return 0;
}

// 3. One generation: the rig chain's priors are the same whether the fill
// chain anchors or not.
int test_one_generation(const Env& env) {
  track::TrackerConfig c;
  c.drivers = {{"rig", {}}, {"track", {}}};
  c.shadows = {};
  const auto frames = frames_every(0, 100 * kMs, 60);
  ASSIGN_OR_FAIL(Rig a,
                 make(c, {drifting(), anchors_when([](auto) { return true; },
                                                   translate(0.02f, 0, 0))}));
  ASSIGN_OR_FAIL(const auto steps_a, run(env, *a.tracker, frames));
  ASSIGN_OR_FAIL(
      Rig b, make(c, {drifting(), anchors_when([](auto) { return false; })}));
  ASSIGN_OR_FAIL(const auto steps_b, run(env, *b.tracker, frames));
  const auto& la = *a.fitters[0]->log();
  const auto& lb = *b.fitters[0]->log();
  CHECK(la.size() == lb.size() && la.size() > 3);
  for (std::size_t i = 0; i < la.size(); ++i) {
    CHECK(same(la[i].prior.correction, lb[i].prior.correction));
    CHECK(la[i].prior.sigma_m == lb[i].prior.sigma_m);
    CHECK(la[i].prior.sigma_rad == lb[i].prior.sigma_rad);
  }
  // And the fill chain's did differ: its anchors moved it.
  CHECK(!same(a.fitters[1]->log()->back().prior.correction,
              b.fitters[1]->log()->back().prior.correction));
  return 0;
}

// 4. Windows and hops.
int test_windows_and_hops(const Env& env) {
  track::TrackerConfig c;  // window_method: 2 s, hop 0.5 s
  c.drivers = {{"rig", {}}, {"track", {}}};
  c.shadows = {};
  // 30 Hz from 1.03 s: a 2 s window holds 60 frames, so 32 are kept.
  const std::uint64_t period = 33'333'333;
  const auto frames = frames_every(1030 * kMs, period, 120);
  ASSIGN_OR_FAIL(Rig rig, make(c, {anchors_when([](auto) { return true; }),
                                   anchors_when([](auto) { return true; })}));
  std::vector<std::uint64_t> hop_frames;
  std::uint64_t last_hop = UINT64_MAX;
  for (const sensor::DeviceFrame& f : frames) {
    const std::uint64_t hop = f.timestamp_ns / (500 * kMs);
    if (hop != last_hop) hop_frames.push_back(f.timestamp_ns);
    last_hop = hop;
    ASSIGN_OR_FAIL(const track::TrackStep s, step(env, *rig.tracker, f));
    std::size_t fits = 0;
    for (const auto& fit : s.fits) {
      CHECK(fit.last_pts_ns == f.timestamp_ns);
      ++fits;
    }
    CHECK(fits == (hop_frames.back() == f.timestamp_ns ? 2u : 0u));
  }
  const auto& rig_log = *rig.fitters[0]->log();
  const auto& track_log = *rig.fitters[1]->log();
  CHECK(rig_log.size() == hop_frames.size());
  CHECK(track_log.size() == hop_frames.size());
  for (std::size_t i = 0; i < rig_log.size(); ++i) {
    const std::vector<std::uint64_t>& pts = rig_log[i].pts;
    CHECK(pts == track_log[i].pts);  // the same frames, at the same step
    CHECK(pts.back() == hop_frames[i]);
    CHECK(pts.size() <= track::kMaxFitFrames);
    for (const std::uint64_t t : pts) {
      CHECK(t + 2 * kS > pts.back() && t <= pts.back());
    }
    // Every frame of the window is in, unless 32 newer ones are.
    const std::size_t in_window = std::size_t(std::count_if(
        frames.begin(), frames.end(), [&](const sensor::DeviceFrame& f) {
          return f.timestamp_ns + 2 * kS > pts.back() &&
                 f.timestamp_ns <= pts.back();
        }));
    CHECK(pts.size() == std::min<std::size_t>(in_window, track::kMaxFitFrames));
  }
  CHECK(rig_log.back().pts.size() == track::kMaxFitFrames);
  // An abandoned step leaves its hop due at the next frame.
  ASSIGN_OR_FAIL(Rig again, make(c, {anchors_when([](auto) { return true; }),
                                     anchors_when([](auto) { return true; })}));
  const auto ten_hz = frames_every(0, 100 * kMs, 12);
  for (const sensor::DeviceFrame& f : ten_hz) {
    vkc::CommandBatch batch(env.gpu.device, env.gpu.allocator);
    ASSIGN_OR_FAIL(const track::PendingStep p,
                   again.tracker->record(batch, env.grid, f));
    if (f.timestamp_ns == 500 * kMs) {
      again.tracker->abandon(p);
      CHECK(again.fitters[0]->log()->back().abandoned);
      continue;
    }
    ASSIGN_OR_FAIL(const track::TrackStep s, again.tracker->finish(p));
    if (f.timestamp_ns == 600 * kMs) CHECK(s.fits.size() == 2);
    if (f.timestamp_ns == 700 * kMs) CHECK(s.fits.empty());
  }
  return 0;
}

// 5. The decisions: Anchored, Interpolated against a hand interpolation,
// Coasting once the hold runs out, and Device.
int test_decisions(const Env& env) {
  track::TrackerConfig c;
  c.driver = track::frame_method();
  c.driver.hop_ns = kS;  // single-frame fits at 0, 1, 2, ... s
  c.drivers = {{"rig", {}}};
  c.shadows = {track::device_method()};
  c.hold_ns = 2 * kS;
  const Mat4f ca = translate(0.01f, 0, 0);
  const Mat4f cb = translate(0.03f, 0.01f, 0) * rotate_z(0.8);
  const Script script =
      [&](const std::vector<sensor::DeviceFrame>& frames,
          const track::PosePrior&) -> vkc::Result<track::TrackFit> {
    const std::uint64_t t = frames.back().timestamp_ns;
    if (t == 1 * kS) return anchored(ca);
    if (t == 2 * kS) return anchored(cb);
    return unsolved(Mat4f(1.0f));
  };
  ASSIGN_OR_FAIL(Rig rig, make(c, {script, {}}));
  const auto frames = frames_every(0, 100 * kMs, 61);
  ASSIGN_OR_FAIL(const auto steps, run(env, *rig.tracker, frames));
  // Anchored: the fit's own frame, at once (a window of 0).
  const auto at1 = pose_of(steps, 0, 1 * kS);
  CHECK(at1 && at1->from == track::PoseFrom::Anchored);
  CHECK(same(at1->correction, ca) && at1->decided_pts_ns == 1 * kS);
  // Interpolated: between the fits at 1 s and 2 s, decided at 2 s.
  for (int k = 1; k < 10; ++k) {
    const std::uint64_t t = 1 * kS + std::uint64_t(k) * 100 * kMs;
    const auto p = pose_of(steps, 0, t);
    CHECK(p && p->from == track::PoseFrom::Interpolated);
    CHECK(p->decided_pts_ns == 2 * kS);
    const double s = double(k) / 10.0;
    const Mat4f hand = translate(float(0.01 + 0.02 * s), float(0.01 * s), 0) *
                       rotate_z(0.8 * s);
    CHECK(near(p->correction, hand, 2e-6f));
  }
  // Before the first anchor: Coasting once 2 s have passed, at the chain's
  // correction then (the second anchor's, at 2 s).
  const auto early = pose_of(steps, 0, 300 * kMs);
  CHECK(early && early->from == track::PoseFrom::Coasting);
  CHECK(early->decided_pts_ns == 2300 * kMs);
  CHECK(same(early->correction, cb));
  // After the last anchor: Coasting when the hold runs out.
  const auto late = pose_of(steps, 0, 3500 * kMs);
  CHECK(late && late->from == track::PoseFrom::Coasting);
  CHECK(late->decided_pts_ns == 5500 * kMs);
  CHECK(same(late->correction, cb));
  // Device, and every frame decided once per chain.
  for (const sensor::DeviceFrame& f : frames) {
    const auto d = pose_of(steps, 1, f.timestamp_ns);
    CHECK(d && d->from == track::PoseFrom::Device);
    std::size_t n = 0;
    for (const auto& s : steps) {
      for (const auto& p : s.poses) n += p.frame.timestamp_ns == f.timestamp_ns;
    }
    CHECK(n == 2);
  }
  return 0;
}

// 6. Release with {rig, track}: the rig's anchor first, the tie to the rig,
// the fill chain's anchor, the first interpolation, the drop; and one chain.
int test_release(const Env& env) {
  // Per frame (100 ms apart, hold 1 s): which chain anchors.
  //   0.0 both  0.1 track  0.2 none  0.3 rig  0.4 none  0.5 track  0.6.. none
  const auto anchors = [](std::uint64_t t, bool rig) {
    const std::uint64_t i = t / (100 * kMs);
    if (i == 0) return true;
    if (i == 1 || i == 5) return !rig;
    if (i == 3) return rig;
    return false;
  };
  const track::TrackerConfig c = every_frame({"rig", "track"});
  ASSIGN_OR_FAIL(
      Rig rig,
      make(c, {anchors_when([&](auto t) { return anchors(t, true); }),
               anchors_when([&](auto t) { return anchors(t, false); })}));
  const auto frames = frames_every(0, 100 * kMs, 20);
  ASSIGN_OR_FAIL(const auto steps, run(env, *rig.tracker, frames));
  const auto r0 = release_of(steps, 0);
  CHECK(r0 && r0->released.placement == track::Placement::Anchored &&
        r0->released.driver == 0 && r0->step == 0);
  const auto r1 = release_of(steps, 100 * kMs);
  CHECK(r1 && r1->released.placement == track::Placement::Anchored &&
        r1->released.driver == 1 && r1->step == 1);
  const auto r2 = release_of(steps, 200 * kMs);  // the rig interpolates at 0.3
  CHECK(r2 && r2->released.placement == track::Placement::Interpolated &&
        r2->released.driver == 0 && r2->step == 3);
  const auto r3 = release_of(steps, 300 * kMs);
  CHECK(r3 && r3->released.placement == track::Placement::Anchored &&
        r3->released.driver == 0);
  const auto r4 = release_of(steps, 400 * kMs);  // the fill chain first, at 0.5
  CHECK(r4 && r4->released.placement == track::Placement::Interpolated &&
        r4->released.driver == 1 && r4->step == 5);
  CHECK(!release_of(steps, 600 * kMs) && dropped(steps, 600 * kMs));
  // Each frame placed exactly once.
  for (const sensor::DeviceFrame& f : frames) {
    CHECK(release_of(steps, f.timestamp_ns).has_value() !=
          dropped(steps, f.timestamp_ns));
  }
  // The drop comes when the hold runs out: at 1.6 s, not before.
  for (std::size_t i = 0; i < steps.size(); ++i) {
    for (const std::uint64_t d : steps[i].dropped_ns) {
      if (d == 600 * kMs) CHECK(i == 16);
    }
  }
  // One driver chain (a dataset's).
  const track::TrackerConfig one = every_frame({"rig"});
  ASSIGN_OR_FAIL(
      Rig solo,
      make(one, {anchors_when([&](auto t) { return anchors(t, true); })}));
  ASSIGN_OR_FAIL(const auto solo_steps, run(env, *solo.tracker, frames));
  const auto s1 = release_of(solo_steps, 100 * kMs);
  CHECK(s1 && s1->released.placement == track::Placement::Interpolated &&
        s1->step == 3);
  CHECK(dropped(solo_steps, 400 * kMs));
  return 0;
}

// 7. The judge, through ChainFit::judged and the next prior.
int test_judge(const Env& env) {
  std::vector<track::TrackFit> queue;
  std::size_t next = 0;
  const Script script =
      [&](const std::vector<sensor::DeviceFrame>&,
          const track::PosePrior&) -> vkc::Result<track::TrackFit> {
    return queue.at(next++);
  };
  track::TrackerConfig c = every_frame({"rig"}, 30 * kS);
  c.driver = track::window_method();  // 0b's gates
  c.driver.window_ns = 0;
  c.driver.hop_ns = 0;
  ASSIGN_OR_FAIL(Rig rig, make(c, {script}));
  std::uint64_t t = 0;
  std::vector<track::ChainFit> fits;
  const auto feed = [&](const track::TrackFit& fit) -> int {
    queue.push_back(fit);
    ASSIGN_OR_FAIL(track::TrackStep s, step(env, *rig.tracker, frame_at(t)));
    CHECK(s.fits.size() == 1);
    fits.push_back(s.fits.front());
    t += 500 * kMs;
    return 0;
  };
  const Mat4f far = translate(1.0f, 0, 0);
  // Gate order: coverage first refuses, though the jump would be free.
  track::TrackFit low = anchored(far);
  low.hits = 2000;  // coverage 0.1 < 0.15
  low.inliers = 1900;
  CHECK(feed(low) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Rejected);
  CHECK(same(fits.back().judged.correction, Mat4f(1.0f)));
  CHECK(fits.back().prior.sigma_m == 0.0f);  // no prior before the first lock
  // The first lock takes any jump.
  CHECK(feed(anchored(far)) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Anchored);
  CHECK(same(fits.back().judged.correction, far));
  // After it, a 10 cm jump is refused once, and taken the second time.
  const Mat4f jump = translate(1.1f, 0, 0);
  CHECK(feed(anchored(jump)) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Rejected);
  CHECK(std::fabs(fits.back().prior.sigma_m - (0.005f + 0.008f * 0.5f)) <
        1e-6f);
  CHECK(feed(anchored(jump)) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Anchored);
  CHECK(same(fits.back().judged.correction, jump));
  // Sigma grows since the last anchor and resets on one.
  CHECK(std::fabs(fits.back().prior.sigma_m - (0.005f + 0.008f * 1.0f)) <
        1e-6f);
  const double rad = (0.3 + 0.3 * 1.0) * kPi / 180.0;
  CHECK(std::fabs(double(fits.back().prior.sigma_rad) - rad) < 1e-7);
  // A fit that solved nothing: Coasting, nothing moves.
  CHECK(feed(unsolved(far)) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Coasting);
  CHECK(same(fits.back().judged.correction, jump));
  // Its prior grew from the anchor at 1.5 s, not the first at 0.5 s.
  CHECK(std::fabs(fits.back().prior.sigma_m - (0.005f + 0.008f * 0.5f)) <
        1e-6f);
  // The inlier ratio is over hits: 9000 of 10000 hits, though of 40000
  // attempts.
  track::TrackFit sparse = anchored(jump);
  sparse.attempts = 40000;
  sparse.hits = 10000;
  sparse.inliers = 9000;
  CHECK(feed(sparse) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Anchored);
  track::TrackFit few = anchored(jump);
  few.inliers = 4999;
  CHECK(feed(few) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Rejected);
  // Partial: direction 0 (v_x) is weak, so only y moves.
  track::TrackFit part = anchored(jump * translate(0.02f, 0.01f, 0));
  part.relative_eigenvalues = {1e-4f, 1, 1, 1, 1, 1};
  part.held = 1;
  CHECK(feed(part) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Partial);
  CHECK(near(fits.back().judged.correction, jump * translate(0, 0.01f, 0)));
  // A Partial fit does not anchor: sigma keeps growing.
  CHECK(feed(unsolved(jump)) == 0);
  CHECK(fits.back().prior.sigma_m > 0.005f + 0.008f * 1.4f);
  // An ill-conditioned fit is never Anchored, whatever its gates say.
  track::TrackFit weak = anchored(jump);
  weak.relative_eigenvalues = {1e-5f, 1e-5f, 1, 1, 1, 1};
  weak.held = 3;
  CHECK(feed(weak) == 0);
  CHECK(fits.back().judged.state == track::TrackState::Partial);
  return 0;
}

// The see-through check: hits far off the surface leave the ratio.
int test_see_through(const Env& env) {
  for (const float distance : {0.0f, 0.1f}) {
    track::TrackerConfig c = every_frame({"rig"});
    c.driver = track::window_method();
    c.driver.window_ns = 0;
    c.driver.hop_ns = 0;
    c.driver.judge.see_through_m = distance;
    const Script script =
        [](const std::vector<sensor::DeviceFrame>&,
           const track::PosePrior&) -> vkc::Result<track::TrackFit> {
      track::TrackFit fit = anchored(Mat4f(1.0f));
      fit.hits = 18000;
      fit.inliers = 11000;  // 0.61 of the hits
      fit.behind = 2000;    // a door opened: the view sees past the model
      fit.in_front = 1000;  // a person in view
      return fit;
    };
    ASSIGN_OR_FAIL(Rig rig, make(c, {script}));
    ASSIGN_OR_FAIL(const track::TrackStep s,
                   step(env, *rig.tracker, frame_at(0)));
    CHECK(s.fits.size() == 1);
    CHECK(s.fits[0].judged.state == (distance > 0.0f
                                         ? track::TrackState::Anchored
                                         : track::TrackState::Rejected));
  }
  return 0;
}

// 8. Rows: each chain's under its own names.
int test_rows(const Env& env) {
  const track::TrackerConfig c = every_frame({"rig", "track"});
  ASSIGN_OR_FAIL(Rig rig, make(c,
                               {anchors_when([](auto) { return true; }),
                                anchors_when([](auto) { return true; })},
                               {1.0, 2.0}));
  vkc::StageMetrics rows;
  for (std::uint64_t i = 0; i < 3; ++i) {
    ASSIGN_OR_FAIL(const track::TrackStep s,
                   step(env, *rig.tracker, frame_at(i * 100 * kMs), &rows));
    CHECK(s.fits.size() == 2);
  }
  CHECK(rig.fitters[0]->log()->front().total_row == "track: rig");
  CHECK(rig.fitters[1]->log()->front().icp_row == "  ..track icp");
  double rig_ms = -1.0;
  double track_ms = -1.0;
  for (const vkc::StageRow& r : rows.rows()) {
    if (std::string(r.name) == "track: rig") rig_ms = r.cpu_ms;
    if (std::string(r.name) == "track: track") track_ms = r.cpu_ms;
  }
  CHECK(rig_ms == 3.0 && track_ms == 6.0);
  return 0;
}

// 9. The contract.
int test_contract(const Env& env) {
  const track::TrackerConfig c = every_frame({"rig", "track"});
  bool refuse_rig = false;
  const Script rig_script =
      [&](const std::vector<sensor::DeviceFrame>&,
          const track::PosePrior&) -> vkc::Result<track::TrackFit> {
    if (refuse_rig) return vkc::Status::invalid_argument("rig refuses");
    return anchored(Mat4f(1.0f));
  };
  ASSIGN_OR_FAIL(
      Rig rig, make(c, {rig_script, anchors_when([](auto) { return true; })}));
  track::Tracker& t = *rig.tracker;
  {
    vkc::CommandBatch batch(env.gpu.device, env.gpu.allocator);
    ASSIGN_OR_FAIL(const track::PendingStep p,
                   t.record(batch, env.grid, frame_at(0)));
    // A second record while one is outstanding.
    CHECK(!t.record(batch, env.grid, frame_at(100 * kMs)).ok());
    CHECK(t.flush().poses.empty());  // flush waits for the step
    CHECK(!t.finish(track::PendingStep{p.serial + 1}).ok());
    ASSIGN_OR_FAIL(const track::TrackStep s, t.finish(p));
    CHECK(s.released.size() == 1);
  }
  // Frames the tracker refuses, each leaving nothing held.
  sensor::DeviceFrame no_depth = frame_at(200 * kMs);
  no_depth.depth.reset();
  sensor::DeviceFrame no_noise = frame_at(200 * kMs);
  no_noise.depth_noise = {};
  sensor::DeviceFrame skewed = frame_at(200 * kMs);
  skewed.depth_camera.cam_to_world[0][0] = 2.0f;
  const sensor::DeviceFrame wider = frame_at(200 * kMs, Mat4f(1.0f), 64);
  const sensor::DeviceFrame stale = frame_at(0);
  for (const sensor::DeviceFrame& bad :
       {no_depth, no_noise, skewed, wider, stale}) {
    vkc::CommandBatch batch(env.gpu.device, env.gpu.allocator);
    CHECK(!t.record(batch, env.grid, bad).ok());
  }
  // A driver chain's refusal: nothing held, no fitter outstanding.
  refuse_rig = true;
  {
    vkc::CommandBatch batch(env.gpu.device, env.gpu.allocator);
    CHECK(!t.record(batch, env.grid, frame_at(200 * kMs)).ok());
  }
  refuse_rig = false;
  CHECK(!rig.fitters[0]->outstanding() && !rig.fitters[1]->outstanding());
  // The refused frame was never held: the same time records again, and only
  // one decision per chain ever names it.
  ASSIGN_OR_FAIL(const track::TrackStep s2, step(env, t, frame_at(200 * kMs)));
  CHECK(s2.released.size() == 1 &&
        s2.released[0].frame.timestamp_ns == 200 * kMs);
  // Flush decides everything held.
  const track::TrackStep last = t.flush();
  CHECK(last.fits.empty());
  // create's refusals.
  const auto refused = [](track::TrackerConfig config,
                          std::vector<std::unique_ptr<track::IFitter>> f) {
    return !track::Tracker::create(std::move(config), std::move(f)).ok();
  };
  const auto two = [] {
    std::vector<std::unique_ptr<track::IFitter>> f;
    f.push_back(std::make_unique<ScriptedFitter>(refuses()));
    f.push_back(std::make_unique<ScriptedFitter>(refuses()));
    return f;
  };
  track::TrackerConfig no_fit = c;
  no_fit.driver = track::device_method();
  CHECK(refused(no_fit, {}));
  track::TrackerConfig no_driver = c;
  no_driver.drivers.clear();
  CHECK(refused(no_driver, {}));
  track::TrackerConfig dup = c;
  dup.drivers[1].name = "rig";
  CHECK(refused(dup, two()));
  track::TrackerConfig bad_name = c;
  bad_name.drivers[1].name = "Track";
  CHECK(refused(bad_name, two()));
  track::TrackerConfig no_hold = c;
  no_hold.hold_ns = 0;
  CHECK(refused(no_hold, two()));
  track::TrackerConfig bad_judge = c;
  bad_judge.driver.judge.min_coverage = 1.5f;
  CHECK(refused(bad_judge, two()));
  track::TrackerConfig bent = c;
  bent.driver.initial_correction[0][0] = 2.0f;
  CHECK(refused(bent, two()));
  CHECK(refused(c, {}));  // too few fitters
  std::vector<std::unique_ptr<track::IFitter>> one_null = two();
  one_null[1].reset();
  CHECK(refused(c, std::move(one_null)));  // null where a fit kind is
  track::TrackerConfig with_arkit = c;
  with_arkit.shadows = {track::device_method()};
  std::vector<std::unique_ptr<track::IFitter>> three = two();
  three.push_back(std::make_unique<ScriptedFitter>(refuses()));
  CHECK(refused(with_arkit, std::move(three)));  // a fitter where none is
  return 0;
}

// Abandon leaves the judges and the ring as they were: a run with an
// abandoned step equals one that never saw its frame.
int test_abandon(const Env& env) {
  track::TrackerConfig c;
  c.drivers = {{"rig", {}}, {"track", {}}};
  c.shadows = {track::device_method()};
  const auto frames = frames_every(0, 100 * kMs, 50);
  ASSIGN_OR_FAIL(Rig a, make(c, {drifting(), drifting(), {}}));
  std::vector<track::TrackStep> steps_a;
  for (const sensor::DeviceFrame& f : frames) {
    vkc::CommandBatch batch(env.gpu.device, env.gpu.allocator);
    ASSIGN_OR_FAIL(const track::PendingStep p,
                   a.tracker->record(batch, env.grid, f));
    if (f.timestamp_ns == 1500 * kMs) {
      a.tracker->abandon(p);
      continue;
    }
    ASSIGN_OR_FAIL(track::TrackStep s, a.tracker->finish(p));
    steps_a.push_back(std::move(s));
  }
  steps_a.push_back(a.tracker->flush());
  std::vector<sensor::DeviceFrame> without;
  for (const sensor::DeviceFrame& f : frames) {
    if (f.timestamp_ns != 1500 * kMs) without.push_back(f);
  }
  ASSIGN_OR_FAIL(Rig b, make(c, {drifting(), drifting(), {}}));
  ASSIGN_OR_FAIL(const auto steps_b, run(env, *b.tracker, without));
  CHECK(driver_half(steps_a, 3) == driver_half(steps_b, 3));
  return 0;
}

// 10. Replays repeat; and flush places every frame.
int test_replay(const Env& env) {
  track::TrackerConfig c;
  c.drivers = {{"rig", {}}, {"track", {}}};
  c.shadows = {track::device_method(), track::frame_method()};
  const auto frames = frames_every(10 * kMs, 100 * kMs, 70);
  std::string first;
  for (int i = 0; i < 2; ++i) {
    ASSIGN_OR_FAIL(Rig r, make(c, {drifting(), drifting(), {}, drifting()}));
    ASSIGN_OR_FAIL(const auto steps, run(env, *r.tracker, frames));
    const std::string all = driver_half(steps, 4);
    if (i == 0) first = all;
    CHECK(all == first);
    for (const sensor::DeviceFrame& f : frames) {
      CHECK(release_of(steps, f.timestamp_ns).has_value() !=
            dropped(steps, f.timestamp_ns));
      for (std::size_t ch = 0; ch < 4; ++ch) {
        CHECK(pose_of(steps, ch, f.timestamp_ns).has_value());
      }
    }
  }
  return 0;
}

int test_corrected() {
  const Mat4f pose = translate(1, 2, 3) * rotate_z(30);
  sensor::DeviceFrame f = frame_at(5, pose);
  f.color_camera.cam_to_world = pose * translate(0.03f, 0, 0);
  const Mat4f c = translate(0.1f, 0, 0) * rotate_z(2);
  const sensor::DeviceFrame g = track::corrected(f, c);
  CHECK(same(g.depth_camera.cam_to_world, c * pose));
  CHECK(same(g.color_camera.cam_to_world, c * f.color_camera.cam_to_world));
  CHECK(g.timestamp_ns == 5 && g.depth == f.depth);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vol::VoxelGridParams p{};
  p.voxel_size = 0.05f;
  p.block_size = 8;
  p.voxels_per_block = 512;
  p.trunc_dist = 0.15f;
  p.bucket_size = 8;
  p.num_buckets = 8;
  p.num_blocks = 64;
  p.max_chain = 8;
  const vol::AttributeSpec attrs[] = {{vol::kTsdfAttribute, 4},
                                      {vol::kWeightAttribute, 4}};
  vkc::Result<vol::VoxelBlockGrid> grid =
      vol::VoxelBlockGrid::create(gpu.device, gpu.allocator, p, attrs, 2);
  CHECK(grid.ok());
  const Env env{gpu, *grid};
  CHECK(test_device(env) == 0);
  CHECK(test_shadows_isolated(env) == 0);
  CHECK(test_one_generation(env) == 0);
  CHECK(test_windows_and_hops(env) == 0);
  CHECK(test_decisions(env) == 0);
  CHECK(test_release(env) == 0);
  CHECK(test_judge(env) == 0);
  CHECK(test_see_through(env) == 0);
  CHECK(test_rows(env) == 0);
  CHECK(test_contract(env) == 0);
  CHECK(test_abandon(env) == 0);
  CHECK(test_replay(env) == 0);
  CHECK(test_corrected() == 0);
  std::printf("track tracker: all cases passed\n");
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
