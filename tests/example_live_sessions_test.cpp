// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The phone lane's sessions (examples/common/live_mobile.hpp), each a fake
// phone (fake_phone.hpp) added through MobileLane::add, its frames prepared on
// the device and tracked by scripted fitters (scripted_fitter.hpp):
// - Two sessions added get ordinals 1 and 2 and indices 0 and 1, each filler
//   under its session's index at its session's correction; a send takes at
//   most 3 in all, one a session in turn, so two phones placing alike (their
//   clocks a record apart, so their hops fall in different gaps) alternate
//   filler by filler across the sends; every placed frame is
//   handed over in the end, and the lane's counts are the sessions' summed.
//   The same two phones twice give the same fillers, bit for bit.
// - A session with nothing placed gives its turns away: beside it (its clock
//   a record after, so their hops fall in different gaps), a phone's sends
//   are those it gets alone.
// - A session added at gap 50 pins its own clock at its first take: it pairs
//   one record every 2.5 sets from its join, and the first session pairs as
//   if it were alone.
// - Five phones placing more than the sends take, their hops in different
//   gaps: a session never keeps more than 40 waiting, its oldest let go;
//   what it hands over keeps its order, its newest placed among it, and
//   every placed frame is fused, let go or dropped.
// - Indices wrap: 71 sessions joined over 70 gaps, one holding its index
//   throughout: each takes (ordinal - 1) mod 63 past those held, the 64th
//   index 1, and no two sessions hold one at once.
// - add refuses a session without its phone, its tracker or its log, each
//   missing alone, one placed by the tag without its watch, one placed by a
//   registration with one (where the build makes a watch), and any after the
//   close; check_filler refuses an index of 63 and a camera not finite.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "fake_phone.hpp"
#include "live_mobile.hpp"
#include "live_tag.hpp"
#include "scripted_fitter.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace sender = volumetric_kit::recon::sender;
namespace vol = volumetric_kit::grid::volume;
using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec4f;
using vr_example::MobileLane;
using vr_test::anchors_when;
using vr_test::device_pose;
using vr_test::FakePhone;
using vr_test::ScriptedFitter;
using vr_test::ten_hz;

namespace {

constexpr std::uint64_t kMs = 1'000'000;
constexpr std::uint64_t kS = 1'000'000'000;

Mat4f translate(float x, float y, float z) {
  Mat4f m(1.0f);
  m[3] = Vec4f(x, y, z, 1.0f);
  return m;
}

bool same(const Mat4f& a, const Mat4f& b) {
  return std::memcmp(&a, &b, sizeof(Mat4f)) == 0;
}

// Whether `part` is `whole` with some taken out, the rest in its order.
bool subsequence(const std::vector<std::uint64_t>& part,
                 const std::vector<std::uint64_t>& whole) {
  std::size_t k = 0;
  for (std::size_t i = 0; i < whole.size() && k < part.size(); ++i) {
    if (whole[i] == part[k]) ++k;
  }
  return k == part.size();
}

bool always(std::uint64_t) { return true; }
bool never(std::uint64_t) { return false; }

struct Env {
  vkc::test::GpuContext& gpu;
  vol::VoxelBlockGrid& grid;
};

// The empty lane, which sessions are added to.
vkc::Result<std::unique_ptr<MobileLane>> empty_lane(const Env& env) {
  return MobileLane::create(env.gpu.device, env.gpu.allocator, {}, {}, {});
}

// A session of a fake phone of `script`, tracked by the two scripts, logged
// in memory.
vkc::Result<MobileLane::SessionParts> session(
    std::vector<vr_test::PhoneFrame> script, ScriptedFitter::Script rig,
    ScriptedFitter::Script track_script, std::string name) {
  std::vector<std::unique_ptr<track::IFitter>> fitters;
  fitters.push_back(std::make_unique<ScriptedFitter>(std::move(rig)));
  fitters.push_back(std::make_unique<ScriptedFitter>(std::move(track_script)));
  fitters.emplace_back();  // arkit
  MobileLane::SessionParts parts;
  VKC_ASSIGN(parts.tracker, track::Tracker::create(vr_example::lane_tracking(),
                                                   std::move(fitters)));
  VKC_ASSIGN(parts.log, vr_example::TrackLog::open({}, *parts.tracker, {}));
  parts.phone = std::make_unique<FakePhone>(std::move(script));
  parts.name = std::move(name);
  return parts;
}

// One filler as a send got it, to compare runs by.
struct Got {
  std::size_t send = 0;  // which send
  std::uint64_t pts = 0;
  tsdf::FillTier tier = tsdf::FillTier::Anchored;
  std::uint32_t index = 0;
  Mat4f pose{1.0f};

  bool operator==(const Got& o) const {
    return send == o.send && pts == o.pts && tier == o.tier &&
           index == o.index && same(pose, o.pose);
  }
};

// `sets` gaps, `period` apart, the first before the set at 1 s + period; the
// fillers each send got, and the most one got.
vkc::Result<std::vector<Got>> run(const Env& env, MobileLane& lane,
                                  std::size_t sets, std::uint64_t period,
                                  std::size_t* most = nullptr) {
  std::vector<Got> out;
  for (std::size_t i = 0; i < sets; ++i) {
    VKC_ASSIGN(const std::vector<sender::Filler> sent,
               lane.next(env.grid, kS + (i + 1) * period));
    if (most != nullptr) *most = std::max(*most, sent.size());
    for (const sender::Filler& f : sent) {
      out.push_back({i, f.frame.timestamp_ns, f.tier, f.index,
                     f.frame.depth_camera.cam_to_world});
    }
  }
  return out;
}

// --- The cases --------------------------------------------------------------

// Two phones placing alike, each at its own correction, the second's clock
// a record after the first's: past their first fits (the second's a gap
// later) their hops fall in different gaps, so one hop a gap never holds
// either back and each pairs each record at its gap.
vkc::Result<std::vector<Got>> two_alike(const Env& env, const Mat4f (&at)[2],
                                        std::string* report) {
  VKC_ASSIGN(std::unique_ptr<MobileLane> lane, empty_lane(env));
  for (int k = 0; k < 2; ++k) {
    std::vector<vr_test::PhoneFrame> script = ten_hz(60);
    for (vr_test::PhoneFrame& p : script) p.pts += k * 100 * kMs;
    VKC_ASSIGN(MobileLane::SessionParts parts,
               session(std::move(script), anchors_when(always, at[k]),
                       anchors_when(never, Mat4f(1.0f)), k == 0 ? "a" : "b"));
    VKC_TRY(lane->add(std::move(parts)));
  }
  std::size_t most = 0;
  VKC_ASSIGN(std::vector<Got> got, run(env, *lane, 260, 40 * kMs, &most));
  if (most > vr_example::kDrainCap) {
    return vkc::Status::invalid_argument("a send got more than 3");
  }
  const std::vector<MobileLane::SessionView> views = lane->sessions();
  if (views.size() != 2 || views[0].ordinal != 1 || views[0].index != 0 ||
      views[1].ordinal != 2 || views[1].index != 1) {
    return vkc::Status::invalid_argument("not ordinals 1, 2 at indices 0, 1");
  }
  // Each handed everything over; the lane's counts are theirs summed.
  const MobileLane::Counts total = lane->counts();
  std::uint64_t fused = 0;
  std::uint64_t paired = 0;
  for (const MobileLane::SessionView& v : views) {
    const MobileLane::Counts& c = v.counts;
    if (c.paired != 60 || c.fused != c.anchored + c.unanchored ||
        c.fused == 0 || v.waiting != 0 || c.shed != 0 || c.unfit != 0) {
      return vkc::Status::invalid_argument("a session kept or lost fill");
    }
    fused += c.fused;
    paired += c.paired;
  }
  if (total.fused != fused || total.paired != paired || fused != got.size()) {
    return vkc::Status::invalid_argument("the totals are not the sum");
  }
  *report = lane->report();
  VKC_TRY(lane->close());
  return got;
}

int test_two_sessions(const Env& env) {
  const Mat4f at[2] = {translate(0.02f, 0.0f, 0.0f),
                       translate(0.0f, 0.03f, 0.0f)};
  std::string report;
  auto got = two_alike(env, at, &report);
  if (!got.ok()) std::fprintf(stderr, "%s\n", got.status().message().c_str());
  CHECK(got.ok() && !got->empty());
  std::fputs(report.c_str(), stdout);
  CHECK(report.find("mobile 01 (a): 60 records") != std::string::npos);
  CHECK(report.find("mobile 02 (b): 60 records") != std::string::npos);
  // One a session in turn, across the sends: 0, 1, 0, 1, ... while both have
  // frames to hand, each at its session's correction, anchored by the rig
  // chain. The second's last frames close the run: its last record is a hop,
  // whose fit places them, while the first's come after its last fit and are
  // dropped.
  std::size_t firsts = 0;
  for (const Got& g : *got) firsts += g.index == 0 ? 1 : 0;
  CHECK(firsts > 0 && 2 * firsts < got->size());
  for (std::size_t k = 0; k < got->size(); ++k) {
    const Got& g = (*got)[k];
    CHECK(g.index == (k < 2 * firsts ? k % 2 : 1));
    CHECK(g.tier == tsdf::FillTier::Anchored);
    CHECK(same(g.pose, at[g.index] * device_pose(g.pts)));
  }
  // Again, bit for bit.
  std::string again_report;
  auto again = two_alike(env, at, &again_report);
  CHECK(again.ok() && *again == *got);
  return 0;
}

// A session with nothing placed gives its turns away. The idle phone's clock
// starts a record after the other's: past their first fits (the idle one's a
// gap later) their hops fall in different gaps, so one hop a gap never holds
// the other back.
int test_turn_given_away(const Env& env) {
  const auto anchors = [](std::uint64_t t) { return t % (1500 * kMs) == 0; };
  std::vector<Got> runs[2];
  for (int beside = 0; beside < 2; ++beside) {
    auto lane = empty_lane(env);
    CHECK(lane.ok());
    auto a = session(ten_hz(60), anchors_when(anchors, Mat4f(1.0f)),
                     anchors_when(always, translate(0.0f, 0.01f, 0.0f)), "a");
    CHECK(a.ok() && (*lane)->add(std::move(*a)).ok());
    if (beside == 1) {
      std::vector<vr_test::PhoneFrame> script = ten_hz(60);
      for (vr_test::PhoneFrame& p : script) p.pts += 100 * kMs;
      auto idle = session(std::move(script), anchors_when(never, Mat4f(1.0f)),
                          anchors_when(never, Mat4f(1.0f)), "idle");
      CHECK(idle.ok() && (*lane)->add(std::move(*idle)).ok());
    }
    auto got = run(env, **lane, 260, 40 * kMs);
    CHECK(got.ok() && !got->empty());
    runs[beside] = std::move(*got);
    if (beside == 1) {
      const auto views = (*lane)->sessions();
      CHECK(views[1].counts.paired == 60 && views[1].counts.fused == 0);
    }
    CHECK((*lane)->close().ok());
  }
  CHECK(runs[0] == runs[1]);
  return 0;
}

// A session added at gap 50 pins its own clock.
int test_late_join(const Env& env) {
  auto lane = empty_lane(env);
  CHECK(lane.ok());
  auto first = session(ten_hz(20), anchors_when(always, Mat4f(1.0f)),
                       anchors_when(never, Mat4f(1.0f)), "first");
  CHECK(first.ok() && (*lane)->add(std::move(*first)).ok());
  for (std::size_t i = 0; i < 120; ++i) {
    if (i == 50) {
      auto late = session(ten_hz(20), anchors_when(always, Mat4f(1.0f)),
                          anchors_when(never, Mat4f(1.0f)), "late");
      CHECK(late.ok() && (*lane)->add(std::move(*late)).ok());
      CHECK((*lane)->sessions()[1].ordinal == 0);  // until the gap
    }
    CHECK((*lane)->next(env.grid, kS + (i + 1) * 40 * kMs).ok());
    const auto views = (*lane)->sessions();
    // Record k of each is due by the set its first take paired with, plus
    // k * 0.1 s: the first's at 1.04 s, the late one's at 3.04 s.
    CHECK(views[0].counts.paired == std::min<std::size_t>(20, 2 * i / 5 + 1));
    if (i >= 50) {
      CHECK(views[1].ordinal == 2 && views[1].index == 1);
      CHECK(views[1].counts.paired ==
            std::min<std::size_t>(20, 2 * (i - 50) / 5 + 1));
    }
  }
  const auto views = (*lane)->sessions();
  CHECK(views[0].counts.skipped == 0 && views[1].counts.skipped == 0);
  CHECK((*lane)->close().ok());
  return 0;
}

// Five phones at 12.5 Hz sets, each placing 0.8 frames a set: more than the
// sends' 3 take. One hop a gap leaves 6.25 gaps a 0.5 s hop for their five,
// and each phone's clock starts a record after the one before, so once their
// first fits are done their hops fall in different gaps and each pairs
// every record.
int test_queue_cap(const Env& env) {
  auto lane = empty_lane(env);
  CHECK(lane.ok());
  constexpr std::size_t kPhones = 5;
  for (std::size_t k = 0; k < kPhones; ++k) {
    std::vector<vr_test::PhoneFrame> script = ten_hz(150);
    for (vr_test::PhoneFrame& p : script) p.pts += k * 100 * kMs;
    auto parts =
        session(std::move(script), anchors_when(always, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)), "p" + std::to_string(k + 1));
    CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
  }
  std::vector<std::vector<std::uint64_t>> fused(kPhones);
  std::size_t most = 0;
  std::size_t longest = 0;
  for (std::size_t i = 0; i < 260; ++i) {
    auto sent = (*lane)->next(env.grid, kS + (i + 1) * 80 * kMs);
    CHECK(sent.ok());
    most = std::max(most, sent->size());
    for (const sender::Filler& f : *sent) {
      CHECK(f.index < kPhones);
      fused[f.index].push_back(f.frame.timestamp_ns);
    }
    for (const MobileLane::SessionView& v : (*lane)->sessions()) {
      CHECK(v.waiting <= vr_example::kPlacedCap);
      longest = std::max(longest, v.waiting);
    }
  }
  CHECK(most == vr_example::kDrainCap && longest == vr_example::kPlacedCap);
  const auto views = (*lane)->sessions();
  std::uint64_t shed = 0;
  for (std::size_t k = 0; k < kPhones; ++k) {
    const MobileLane::SessionView& v = views[k];
    CHECK(v.index == k && v.waiting == 0 && !v.holding);
    // What it placed, in the order it placed it.
    std::vector<std::uint64_t> placed;
    for (const vr_example::PlacementRecord& p : v.log->run().placements) {
      if (!p.dropped) placed.push_back(p.pts_ns);
    }
    const MobileLane::Counts& c = v.counts;
    CHECK(placed.size() == c.anchored + c.unanchored);
    CHECK(c.fused == fused[k].size() && c.unfit == 0);
    CHECK(placed.size() == c.fused + c.shed);
    // Handed over in order, the newest placed among it: the oldest went.
    CHECK(subsequence(fused[k], placed));
    CHECK(!fused[k].empty() && fused[k].back() == placed.back());
    shed += c.shed;
  }
  CHECK(shed > 0 && (*lane)->counts().shed == shed);
  const std::string report = (*lane)->report();
  CHECK(report.find("placed let go past 40 waiting") != std::string::npos);
  CHECK((*lane)->close().ok());
  return 0;
}

// Indices wrap and never collide: a session holding index 0 throughout, and
// one more joining each gap that holds its own for four.
int test_index_wrap(const Env& env) {
  auto lane = empty_lane(env);
  CHECK(lane.ok());
  auto held_long = session(ten_hz(100), anchors_when(never, Mat4f(1.0f)),
                           anchors_when(never, Mat4f(1.0f)), "long");
  CHECK(held_long.ok() && (*lane)->add(std::move(*held_long)).ok());
  std::size_t past_held = 0;  // joins whose (ordinal - 1) mod 63 was held
  for (std::size_t g = 0; g < 70; ++g) {
    std::set<std::uint32_t> held;
    for (const MobileLane::SessionView& v : (*lane)->sessions()) {
      if (v.holding) held.insert(v.index);
    }
    auto brief =
        session(ten_hz(2), anchors_when(never, Mat4f(1.0f)),
                anchors_when(never, Mat4f(1.0f)), "s" + std::to_string(g));
    CHECK(brief.ok() && (*lane)->add(std::move(*brief)).ok());
    CHECK((*lane)->next(env.grid, kS + (g + 1) * 40 * kMs).ok());
    const auto views = (*lane)->sessions();
    CHECK(views.size() == g + 2);
    // The newcomer: its ordinal, and its index past those held at its join.
    const MobileLane::SessionView& newest = views.back();
    CHECK(newest.ordinal == g + 2);
    std::uint32_t want = (newest.ordinal - 1) % tsdf::kMaxSourcesPerClass;
    if (held.count(want) != 0) ++past_held;
    while (held.count(want) != 0) want = (want + 1) % tsdf::kMaxSourcesPerClass;
    CHECK(newest.index == want);
    // No two hold one index; the first holds 0 throughout.
    std::set<std::uint32_t> holding;
    for (const MobileLane::SessionView& v : views) {
      if (v.holding) CHECK(holding.insert(v.index).second);
    }
    CHECK(views[0].holding && views[0].index == 0);
  }
  const auto views = (*lane)->sessions();
  CHECK(views[63].ordinal == 64 && views[63].index == 1);
  CHECK(past_held >= 8);  // ordinals 64-71
  // Not closed: 71 summaries tell nothing here; the lane's destructor stops
  // the phones.
  return 0;
}

int test_refusals(const Env& env) {
  auto lane = empty_lane(env);
  CHECK(lane.ok());
  CHECK((*lane)->report().empty() && (*lane)->log() == nullptr);
  CHECK((*lane)->add({}).domain() == vkc::Status::Code::InvalidArgument);
  // Its phone, its tracker or its log missing alone.
  for (int missing = 0; missing < 3; ++missing) {
    auto parts = session(ten_hz(1), anchors_when(never, Mat4f(1.0f)),
                         anchors_when(never, Mat4f(1.0f)), "missing");
    CHECK(parts.ok());
    if (missing == 0) parts->phone.reset();
    if (missing == 1) parts->tracker.reset();
    if (missing == 2) parts->log.reset();
    CHECK((*lane)->add(std::move(*parts)).domain() ==
          vkc::Status::Code::InvalidArgument);
  }
  auto no_watch = session(ten_hz(1), anchors_when(never, Mat4f(1.0f)),
                          anchors_when(never, Mat4f(1.0f)), "no watch");
  CHECK(no_watch.ok());
  no_watch->placement = MobileLane::Placement::Tag;
  CHECK((*lane)->add(std::move(*no_watch)).domain() ==
        vkc::Status::Code::InvalidArgument);
  // A watch beside a registration's placement, where this build makes one
  // (VR_WITH_CALIB); without, the watch is refused.
  sensor::camera::WorldFrame tag;
  tag.apriltag = sensor::camera::AprilTagWorld{"tag36h11", 0, 0.15};
  std::array<std::uint8_t, 16> id{};
  id[0] = 1;
  auto watch = vr_example::TagWatch::create(env.gpu.device, env.gpu.allocator,
                                            {"phone", id, tag, false});
  if (watch.ok()) {
    auto watched = session(ten_hz(1), anchors_when(never, Mat4f(1.0f)),
                           anchors_when(never, Mat4f(1.0f)), "watched");
    CHECK(watched.ok());
    watched->tags = std::move(*watch);
    CHECK((*lane)->add(std::move(*watched)).domain() ==
          vkc::Status::Code::InvalidArgument);
  } else {
    CHECK(watch.status().domain() == vkc::Status::Code::Unsupported);
  }
  CHECK((*lane)->sessions().empty());
  CHECK((*lane)->close().ok());
  auto late = session(ten_hz(1), anchors_when(never, Mat4f(1.0f)),
                      anchors_when(never, Mat4f(1.0f)), "late");
  CHECK(late.ok());
  CHECK((*lane)->add(std::move(*late)).domain() ==
        vkc::Status::Code::InvalidArgument);
  // What the send would refuse.
  sender::Filler f;
  f.frame.depth_camera = {40.0f, 40.0f, 15.5f, 11.5f,      0.1f,
                          5.0f,  32,    24,    Mat4f(1.0f)};
  CHECK(vr_example::check_filler(f).ok());
  f.index = tsdf::kMaxSourcesPerClass - 1;
  CHECK(vr_example::check_filler(f).ok());
  f.index = tsdf::kMaxSourcesPerClass;
  CHECK(vr_example::check_filler(f).domain() ==
        vkc::Status::Code::InvalidArgument);
  f.index = 0;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  sender::Filler bad = f;
  bad.frame.depth_camera.cam_to_world[3][1] = nan;
  CHECK(vr_example::check_filler(bad).domain() ==
        vkc::Status::Code::InvalidArgument);
  bad = f;
  bad.frame.depth_camera.max_depth = std::numeric_limits<float>::infinity();
  CHECK(vr_example::check_filler(bad).domain() ==
        vkc::Status::Code::InvalidArgument);
  // A colour camera counts only with a colour.
  bad = f;
  bad.frame.color_camera.fx = nan;
  CHECK(vr_example::check_filler(bad).ok());
  auto word = vkc::device_storage_buffer(env.gpu.allocator, 4);
  CHECK(word.ok());
  bad.frame.color = std::make_shared<const vkc::Buffer>(std::move(*word));
  CHECK(vr_example::check_filler(bad).domain() ==
        vkc::Status::Code::InvalidArgument);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  auto grid =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, 0.05f, 0.15f, 64);
  CHECK(grid.ok());
  const Env env{gpu, *grid};
  CHECK(test_two_sessions(env) == 0);
  CHECK(test_turn_given_away(env) == 0);
  CHECK(test_late_join(env) == 0);
  CHECK(test_queue_cap(env) == 0);
  CHECK(test_index_wrap(env) == 0);
  CHECK(test_refusals(env) == 0);
  std::puts("recon_example_live_sessions: OK");
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
