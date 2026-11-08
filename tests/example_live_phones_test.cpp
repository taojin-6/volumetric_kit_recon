// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Phones that come and go in a live run (examples/common/live_mobile.hpp),
// with no origin tag: sessions added to a live lane through MobileLane::add,
// each a fake phone (fake_phone.hpp) on scripted fitters
// (scripted_fitter.hpp) and a presence the test sets; and open_mobile_lane
// on 127.0.0.1, phones that dial it as an iPad's writer does
// (link_dialler.hpp), depth alone, placed by a registration file:
// - Leave through the presence: Away from the first gap that reads it gone;
//   the tracker flushed once at 7 s of set time Away, not a gap before; the
//   session retired at 30 s, then let go by the reaper: its log closed (its
//   summary.csv), its join's and its leave's rows in sessions.csv, its log
//   let go and its lines kept.
// - Fill still drains once retired: of five phones placing more than the
//   sends take (12.5 Hz sets, their clocks a record apart, so their hops
//   fall in different gaps), one that goes retires with frames waiting,
//   hands them over and only then leaves, its index free again.
// - A return within the grace is the same session: Live again, its ordinal
//   and index kept, the loss in its records one split.
// - Containment: a fitter's error retires its session alone while next()
//   stays OK, its error's hearer told once; the device's loss comes out of
//   next(), and so does a file's session's own error, as before.
// - The gap's scheduler: the guard's estimate is 20 ms until hops replace
//   its seeds, then the 29th of the last 32 hops' wall times; a probe that
//   fits its room brings it back down, one that overran is a sample as any.
//   Two phones on one clock, their hops due in the same gaps, get one hop a
//   gap, one put back to the next, the first place by turns, and their fits
//   stay within one of each other over 10 s of sets, every record paired. A
//   live gap whose end has passed runs no hop and puts its prep back, and
//   once gaps have room the loss in between is still one split; a file's
//   lane given that end pairs as with none. With the estimate above every
//   gap's room, the hops go back and the preps pair, every deferral the
//   budget's, until the probe a second of sets later, whose fit brings the
//   estimate down; an end an hour off defers nothing, and the hops that run
//   feed the estimate. Three phones on one clock replay the same fillers in
//   the same sends and the same counts, each put back alike.
// - open_mobile_lane returns within a second with no phone (the pipeline
//   cache warmed first, as a run's earlier lanes leave it); a run no phone
//   dials writes nothing in D. A phone that dials while the sets run joins
//   (D/01-ipad-<hex8>, its recording, its run.csv's session keys), beside a
//   dated folder that is no session's, and at the close leaves;
//   sessions.csv holds its join and leave, and no door.csv is made, as
//   nothing was refused. A second run into that directory is neither
//   refused nor its phones turned away: it says it logs nothing, naming
//   sessions.csv, and its phone joins, unlogged.
// - An address held by another socket: open and next() go on, the report
//   says it is not listening, and once the address frees a phone joins.
// - A setting that rules phones out (Classic fusion; a calibration whose
//   world is no tag, with no registration) turns every phone away: open
//   succeeds, a phone that dials is refused and counted, in door.csv too,
//   and nothing returns an error. With a registration and no tag, the session
//   it names joins and another is turned away.
// - A phone that leaves and dials again after it retired is a new session: a
//   new ordinal and directory.
// - An empty diagnostics.track_log writes nothing, the summaries still kept.
// - A session that fails of its own (a registration naming its session but
//   another sensor, which admit refuses and the door does not remember)
//   keeps its id away from the door for the back-off: a redial at once is
//   turned away, one after it is opened again and fails, and the back-off
//   doubles; door.csv holds each refusal.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_phone.hpp"
#include "link_dialler.hpp"
#include "live_mobile.hpp"
#include "scripted_fitter.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/net_loopback.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"

namespace vkc = volumetric_kit::core;
namespace cfg = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace camera = volumetric_kit::sensor::camera;
namespace track = volumetric_kit::recon::track;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace sender = volumetric_kit::recon::sender;
namespace vol = volumetric_kit::grid::volume;
namespace stream = volumetric_kit::grid::stream;
using volumetric_kit::recon::Mat4f;
using vr_example::LaneOptions;
using vr_example::MobileLane;
using vr_test::anchors_when;
using vr_test::FakePhone;
using vr_test::PhoneFrame;
using vr_test::ScriptedFitter;
using vr_test::ten_hz;
using Stage = MobileLane::Stage;

namespace {

constexpr std::uint64_t kMs = 1'000'000;
constexpr std::uint64_t kS = 1'000'000'000;

bool always(std::uint64_t) { return true; }
bool never(std::uint64_t) { return false; }

struct Env {
  vkc::test::GpuContext& gpu;
  vol::VoxelBlockGrid& grid;
};

std::vector<std::string> lines_of(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::vector<std::string> out;
  for (std::string line; std::getline(in, line);) out.push_back(line);
  return out;
}

bool has(const std::string& text, const std::string& part) {
  if (text.find(part) != std::string::npos) return true;
  std::fprintf(stderr, "no \"%s\" in:\n%s\n", part.c_str(), text.c_str());
  return false;
}

// A live phone's records, as far as the test lets them come (`upto`), each
// once: a phone that stops sending when it goes. A live stream never ends
// alone.
class GatedPhone final : public sensor::IRgbdSensor {
 public:
  GatedPhone(std::vector<PhoneFrame> script, std::shared_ptr<std::size_t> upto)
      : inner_(std::move(script)), upto_(std::move(upto)) {}
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
    if (given_ >= *upto_) return {};
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> f, inner_.poll());
    if (f) {
      out->push_back(std::move(*f));
      ++given_;
    }
    return {};
  }
  bool exhausted() const noexcept override { return false; }
  sensor::SensorStats stats() const noexcept override { return inner_.stats(); }

 private:
  FakePhone inner_;
  std::shared_ptr<std::size_t> upto_;
  std::size_t given_ = 0;
};

// A session of `phone` on the two scripts, logged in `dir` (empty: in
// memory), its presence `here` (null: none, a file's).
vkc::Result<MobileLane::SessionParts> session(
    std::unique_ptr<sensor::IRgbdSensor> phone, ScriptedFitter::Script rig,
    ScriptedFitter::Script track_script, std::string name,
    std::shared_ptr<bool> here, const std::string& dir = {}) {
  std::vector<std::unique_ptr<track::IFitter>> fitters;
  fitters.push_back(std::make_unique<ScriptedFitter>(std::move(rig)));
  fitters.push_back(std::make_unique<ScriptedFitter>(std::move(track_script)));
  fitters.emplace_back();  // arkit
  MobileLane::SessionParts parts;
  VKC_ASSIGN(parts.tracker, track::Tracker::create(vr_example::lane_tracking(),
                                                   std::move(fitters)));
  vr_example::RunSource source;
  source.session_id = "0123456789abcdef0123456789abcdef";
  VKC_ASSIGN(parts.log, vr_example::TrackLog::open(dir, *parts.tracker,
                                                   std::move(source)));
  parts.phone = std::move(phone);
  parts.name = std::move(name);
  if (here) parts.presence = [here] { return *here; };
  return parts;
}

// A live lane with no source: its sessions added.
vkc::Result<std::unique_ptr<MobileLane>> live_lane(const Env& env,
                                                   LaneOptions options) {
  return MobileLane::create(env.gpu.device, env.gpu.allocator, nullptr,
                            std::move(options));
}

// Waits up to 5 s, the reaper's thread working, for session `k` to leave.
bool left(const MobileLane& lane, std::size_t k) {
  return vr_test::wait_for(
      [&] { return lane.sessions().at(k).stage == Stage::Left; });
}

// --- Through add ------------------------------------------------------------

// Away from 5 s; flushed once at 7 s of it, retired at 30 s, then let go.
int test_leave(const Env& env, const std::filesystem::path& dir) {
  LaneOptions options;  // the defaults: settle 7 s, grace 30 s
  options.log_directory = dir.string();
  auto lane = live_lane(env, options);
  CHECK(lane.ok() && (*lane)->enabled());
  auto here = std::make_shared<bool>(true);
  auto upto = std::make_shared<std::size_t>(40);
  // Every fit anchors: the frames after the last (4.6-4.9 s) wait for one.
  auto parts = session(std::make_unique<GatedPhone>(ten_hz(40), upto),
                       anchors_when(always, Mat4f(1.0f)),
                       anchors_when(never, Mat4f(1.0f)), "phone", here,
                       (dir / "01-phone").string());
  CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
  std::optional<std::uint64_t> away;
  bool settled = false;
  bool retired = false;
  for (std::uint64_t set = kS + 40 * kMs; !retired; set += 40 * kMs) {
    if (set >= 5 * kS) *here = false;
    CHECK((*lane)->next(env.grid, set).ok());
    const MobileLane::SessionView v = (*lane)->sessions().at(0);
    if (set < 5 * kS) {
      CHECK(v.stage == Stage::Live && v.ordinal == 1);
      continue;
    }
    if (!away) away = set;  // the first gap that reads it gone
    const std::uint64_t gone = set - *away;
    if (gone < 7 * kS) {
      CHECK(v.stage == Stage::Away && v.counts.dropped == 0);
    } else if (gone < 30 * kS) {
      // Flushed at the first gap 7 s in: the four after the last fit are
      // dropped then, and never again.
      CHECK(v.stage == Stage::Away && v.counts.dropped == 4);
      settled = true;
    } else {
      CHECK(v.stage == Stage::Reaping || v.stage == Stage::Left);
      retired = true;
    }
  }
  CHECK(settled && left(**lane, 0));
  const MobileLane::SessionView v = (*lane)->sessions().at(0);
  CHECK(v.log == nullptr && !v.holding && v.counts.paired == 40 &&
        v.counts.fused == 36);
  CHECK(has((*lane)->report(), "joined 0.0 s in, left 34.0 s in: away 30 s"));
  CHECK(std::filesystem::exists(dir / "01-phone" / "summary.csv"));
  const std::vector<std::string> rows = lines_of(dir / "sessions.csv");
  CHECK(rows.size() == 3);
  CHECK(rows[0] ==
        "event,set_ns,ordinal,session_id,sensor,peer,mobile_index,placement,"
        "directory,polled,paired,fused,reason");
  CHECK(rows[1] ==
        "join,1040000000,1,0123456789abcdef0123456789abcdef,phone,,0,"
        "registration,01-phone,0,0,0,");
  CHECK(rows[2] ==
        "leave,35000000000,1,0123456789abcdef0123456789abcdef,phone,,0,"
        "registration,01-phone,40,40,36,away 30 s");
  CHECK((*lane)->close().ok());
  CHECK(lines_of(dir / "sessions.csv").size() == 3);  // nothing more
  return 0;
}

// Five phones placing more than the sends take; one goes, and retires with
// frames waiting, which drain before it leaves. At 12.5 Hz sets, each phone's
// clock a record after the one before, as the sessions test's queue cap case
// runs them: their hops fall in different gaps and each pairs every record.
int test_drains_when_retired(const Env& env) {
  LaneOptions options;
  options.settle_ns = kS;
  options.grace_ns = 2 * kS;
  auto lane = live_lane(env, options);
  CHECK(lane.ok());
  auto here = std::make_shared<bool>(true);
  auto upto = std::make_shared<std::size_t>(150);
  for (std::size_t k = 0; k < 5; ++k) {
    std::unique_ptr<sensor::IRgbdSensor> phone;
    std::vector<PhoneFrame> script = ten_hz(150);
    for (PhoneFrame& p : script) p.pts += k * 100 * kMs;
    if (k == 0) {
      phone = std::make_unique<GatedPhone>(std::move(script), upto);
    } else {
      phone = std::make_unique<FakePhone>(std::move(script));
    }
    auto parts = session(std::move(phone), anchors_when(always, Mat4f(1.0f)),
                         anchors_when(never, Mat4f(1.0f)),
                         "p" + std::to_string(k + 1), k == 0 ? here : nullptr);
    CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
  }
  bool drained_retired = false;  // fused while Retiring
  std::uint64_t fused_at_retire = 0;
  std::size_t waiting_at_retire = 0;
  for (std::size_t i = 0; i < 250; ++i) {
    const std::uint64_t set = kS + (i + 1) * 80 * kMs;
    if (set >= 11 * kS && *here) {
      *here = false;
      *upto = 0;  // it sends no more
    }
    CHECK((*lane)->next(env.grid, set).ok());
    const MobileLane::SessionView v = (*lane)->sessions().at(0);
    if (v.stage == Stage::Retiring) {
      if (waiting_at_retire == 0) {
        waiting_at_retire = v.waiting;
        fused_at_retire = v.counts.fused;
      }
      if (v.counts.fused > fused_at_retire) drained_retired = true;
    }
  }
  CHECK(waiting_at_retire > 3 && drained_retired);
  CHECK(left(**lane, 0));
  const std::vector<MobileLane::SessionView> views = (*lane)->sessions();
  CHECK(views[0].waiting == 0 && !views[0].holding);
  CHECK(views[0].counts.fused + views[0].counts.shed ==
        views[0].counts.anchored + views[0].counts.unanchored);
  // The others go on.
  for (std::size_t k = 1; k < 5; ++k) CHECK(views[k].stage != Stage::Left);
  CHECK((*lane)->close().ok());
  return 0;
}

// Away for a second, back within the grace: the same session, a split.
int test_return(const Env& env) {
  auto lane = live_lane(env, {});
  CHECK(lane.ok());
  auto here = std::make_shared<bool>(true);
  auto upto = std::make_shared<std::size_t>(20);
  std::vector<PhoneFrame> script = ten_hz(20);  // 1.0-2.9 s
  for (std::uint64_t i = 30; i < 50; ++i) {
    script.push_back({kS + i * 100 * kMs, i});  // 4.0-5.9 s, 20-29 lost
  }
  auto parts = session(std::make_unique<GatedPhone>(script, upto),
                       anchors_when(always, Mat4f(1.0f)),
                       anchors_when(never, Mat4f(1.0f)), "phone", here);
  CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
  bool was_away = false;
  for (std::size_t i = 0; i < 200; ++i) {
    const std::uint64_t set = kS + (i + 1) * 40 * kMs;
    *here = set < 3 * kS || set >= 4 * kS;
    if (set >= 4 * kS) *upto = 40;
    CHECK((*lane)->next(env.grid, set).ok());
    const MobileLane::SessionView v = (*lane)->sessions().at(0);
    if (v.stage == Stage::Away) was_away = true;
    CHECK(v.stage == (*here ? Stage::Live : Stage::Away));
  }
  const MobileLane::SessionView v = (*lane)->sessions().at(0);
  CHECK(was_away && v.ordinal == 1 && v.index == 0 && v.holding);
  CHECK(v.counts.paired == 40 && v.counts.splits == 1);
  CHECK((*lane)->sessions().size() == 1);
  CHECK((*lane)->close().ok());
  return 0;
}

// A fitter's error retires its session alone; the device's loss returns.
int test_containment(const Env& env) {
  auto lane = live_lane(env, {});
  CHECK(lane.ok());
  auto here = std::make_shared<bool>(true);
  const ScriptedFitter::Script failing =
      [](const std::vector<sensor::DeviceFrame>& frames,
         const track::PosePrior& prior) -> vkc::Result<track::TrackFit> {
    if (frames.back().timestamp_ns == 2 * kS) {
      return vkc::Status::io_error("a scripted fitter's error");
    }
    return anchors_when(always, Mat4f(1.0f))(frames, prior);
  };
  auto all = std::make_shared<std::size_t>(30);
  auto good = session(std::make_unique<GatedPhone>(ten_hz(30), all),
                      anchors_when(always, Mat4f(1.0f)),
                      anchors_when(never, Mat4f(1.0f)), "good", here);
  auto bad = session(std::make_unique<GatedPhone>(ten_hz(30), all), failing,
                     anchors_when(never, Mat4f(1.0f)), "bad", here);
  CHECK(good.ok() && bad.ok());
  auto heard = std::make_shared<std::vector<std::string>>();
  bad->on_error = [heard](const vkc::Status& e) {
    heard->push_back(e.message());
  };
  CHECK((*lane)->add(std::move(*good)).ok() &&
        (*lane)->add(std::move(*bad)).ok());
  for (std::size_t i = 0; i < 120; ++i) {
    CHECK((*lane)->next(env.grid, kS + (i + 1) * 40 * kMs).ok());
  }
  CHECK(left(**lane, 1));
  const std::vector<MobileLane::SessionView> views = (*lane)->sessions();
  CHECK(views[0].stage == Stage::Live && views[0].counts.paired == 30);
  CHECK(views[1].counts.paired == 10);  // 1.0-1.9 s, then its error
  CHECK(has((*lane)->report(), "its error: a scripted fitter's error"));
  CHECK(heard->size() == 1 && heard->front() == "a scripted fitter's error");
  CHECK((*lane)->close().ok());

  // A file's session (no presence): its own error comes out of next().
  auto file_lane = live_lane(env, {});
  CHECK(file_lane.ok());
  auto filed = session(std::make_unique<FakePhone>(ten_hz(30)), failing,
                       anchors_when(never, Mat4f(1.0f)), "file", nullptr);
  CHECK(filed.ok() && (*file_lane)->add(std::move(*filed)).ok());
  vkc::Status failed;
  for (std::size_t i = 0; i < 120 && failed.ok(); ++i) {
    failed = (*file_lane)->next(env.grid, kS + (i + 1) * 40 * kMs).status();
  }
  CHECK(failed.domain() == vkc::Status::Code::IoError &&
        failed.message() == "a scripted fitter's error");
  CHECK((*file_lane)->close().ok());

  auto lost_lane = live_lane(env, {});
  CHECK(lost_lane.ok());
  const ScriptedFitter::Script losing =
      [](const std::vector<sensor::DeviceFrame>& frames,
         const track::PosePrior& prior) -> vkc::Result<track::TrackFit> {
    if (frames.back().timestamp_ns == 2 * kS) {
      return vkc::vk_error(VK_ERROR_DEVICE_LOST, "a scripted fitter");
    }
    return anchors_when(always, Mat4f(1.0f))(frames, prior);
  };
  auto doomed = session(std::make_unique<GatedPhone>(ten_hz(30), all), losing,
                        anchors_when(never, Mat4f(1.0f)), "doomed", here);
  CHECK(doomed.ok() && (*lost_lane)->add(std::move(*doomed)).ok());
  vkc::Status lost;
  for (std::size_t i = 0; i < 120 && lost.ok(); ++i) {
    lost = (*lost_lane)->next(env.grid, kS + (i + 1) * 40 * kMs).status();
  }
  const std::optional<VkResult> result = vkc::vk_result(lost);
  CHECK(result && *result == VK_ERROR_DEVICE_LOST);
  CHECK((*lost_lane)->close().ok());
  return 0;
}

// --- The gap's scheduler ----------------------------------------------------

// The guard's estimate: 20 ms until hops take the seeds' places, then the
// 29th smallest of the last 32 hops' wall times.
int test_hop_estimate() {
  using std::chrono::milliseconds;
  vr_example::HopEstimate e;
  CHECK(e.p90() == milliseconds(20));
  // 28 quick hops leave four seeds above them; a 29th, three.
  for (int i = 0; i < 28; ++i) e.add(milliseconds(5));
  CHECK(e.p90() == milliseconds(20));
  e.add(milliseconds(5));
  CHECK(e.p90() == milliseconds(5));
  // Three slow hops in the last seeds' places are above the rank; a fourth,
  // in the oldest quick one's, is at it.
  for (int i = 0; i < 3; ++i) e.add(milliseconds(100));
  CHECK(e.p90() == milliseconds(5));
  e.add(milliseconds(100));
  CHECK(e.p90() == milliseconds(100));
  // 32 more push every older one out.
  for (int i = 0; i < 32; ++i) e.add(milliseconds(8));
  CHECK(e.p90() == milliseconds(8));
  // Ranked, not averaged, whatever the order: 32 ms down to 1 ms is 29 ms.
  vr_example::HopEstimate ranked;
  for (int i = 32; i >= 1; --i) ranked.add(milliseconds(i));
  CHECK(ranked.p90() == milliseconds(29));
  // Four slow hops hold it above any gap's 30 ms of room, and no hop the
  // guard holds back can bring it down. A probe that overran its room is a
  // sample as any; one that ended within it replaces every sample longer
  // than that room, and the estimate is the seed's again.
  vr_example::HopEstimate slow;
  for (int i = 0; i < 4; ++i) slow.add(milliseconds(100));
  CHECK(slow.p90() == milliseconds(100));
  slow.probed(milliseconds(40), milliseconds(30));
  CHECK(slow.p90() == milliseconds(100));
  slow.probed(milliseconds(15), milliseconds(30));
  CHECK(slow.p90() == milliseconds(20));
  // A seed of its own: above every room until a probe that fits.
  vr_example::HopEstimate seeded(std::chrono::hours(1));
  CHECK(seeded.p90() == std::chrono::hours(1));
  seeded.probed(milliseconds(3), std::chrono::seconds(10));
  CHECK(seeded.p90() == milliseconds(3));
  return 0;
}

// A session of a fake phone of `script` whose rig chain anchors every fit
// and logs it into `*rig_fits`; logged in memory, a file's (no presence).
vkc::Result<MobileLane::SessionParts> logged_session(
    std::vector<PhoneFrame> script, std::string name,
    std::shared_ptr<const std::vector<vr_test::ScriptedRecord>>* rig_fits) {
  auto rig =
      std::make_unique<ScriptedFitter>(anchors_when(always, Mat4f(1.0f)));
  *rig_fits = rig->log();
  std::vector<std::unique_ptr<track::IFitter>> fitters;
  fitters.push_back(std::move(rig));
  fitters.push_back(
      std::make_unique<ScriptedFitter>(anchors_when(never, Mat4f(1.0f))));
  fitters.emplace_back();  // arkit
  MobileLane::SessionParts parts;
  VKC_ASSIGN(parts.tracker, track::Tracker::create(vr_example::lane_tracking(),
                                                   std::move(fitters)));
  VKC_ASSIGN(parts.log, vr_example::TrackLog::open({}, *parts.tracker, {}));
  parts.phone = std::make_unique<FakePhone>(std::move(script));
  parts.name = std::move(name);
  return parts;
}

// The lane with no session, which sessions are added to: a file's, which
// reads no budget.
vkc::Result<std::unique_ptr<MobileLane>> empty_lane(const Env& env) {
  return MobileLane::create(env.gpu.device, env.gpu.allocator, {}, {}, {});
}

bool same_counts(const MobileLane::Counts& a, const MobileLane::Counts& b) {
  return std::memcmp(&a, &b, sizeof a) == 0;
}

// The gaps' sets at 25 Hz: gap i before the set at 1 s + (i + 1) * 40 ms.
std::uint64_t gap_set(std::size_t i) { return kS + (i + 1) * 40 * kMs; }

// A budget whose end is `ahead` from now.
vr_example::GapBudget ending_in(std::chrono::steady_clock::duration ahead) {
  vr_example::GapBudget b;
  b.gap_end = std::chrono::steady_clock::now() + ahead;
  return b;
}

// Two phones on one clock, their hops due in the same gaps: one hop a gap of
// the two, one phone's at the gap and the other's at the next, the first
// place taking turns period by period (the turn moves on only in a contested
// gap), and over 10 s of sets their fits within one of each other, every
// record paired.
int test_one_hop_a_gap(const Env& env) {
  auto lane = empty_lane(env);
  CHECK(lane.ok());
  std::shared_ptr<const std::vector<vr_test::ScriptedRecord>> fits[2];
  for (int k = 0; k < 2; ++k) {
    auto parts = logged_session(ten_hz(100), k == 0 ? "a" : "b", &fits[k]);
    CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
  }
  std::size_t ran[2] = {0, 0};    // fits so far, each
  std::size_t together = 0;       // gaps whose two hops were due at once
  std::vector<std::size_t> back;  // whose hop went back, gap by gap
  for (std::size_t i = 0; i < 250; ++i) {  // 10 s of sets at 25 Hz
    std::uint64_t deferred[2];
    for (std::size_t k = 0; k < 2; ++k) {
      deferred[k] = (*lane)->sessions().at(k).counts.deferred;
    }
    CHECK((*lane)->next(env.grid, gap_set(i)).ok());
    const std::size_t now[2] = {fits[0]->size(), fits[1]->size()};
    CHECK(now[0] + now[1] <= ran[0] + ran[1] + 1);  // one hop a gap at most
    for (std::size_t k = 0; k < 2; ++k) {
      if ((*lane)->sessions().at(k).counts.deferred == deferred[k]) continue;
      ++together;
      back.push_back(k);
      // Exactly one ran: the other's.
      CHECK(now[k] == ran[k] && now[1 - k] == ran[1 - k] + 1);
    }
    CHECK(now[0] <= now[1] + 1 && now[1] <= now[0] + 1);
    ran[0] = now[0];
    ran[1] = now[1];
  }
  // A hop each 0.5 s from the first record: 20 each, every one due with the
  // other's, one put back once each time, the second's first and then by
  // turns: 10 each.
  const std::vector<MobileLane::SessionView> views = (*lane)->sessions();
  CHECK(fits[0]->size() == 20 && fits[1]->size() == 20 && together == 20);
  for (std::size_t k = 0; k < back.size(); ++k) CHECK(back[k] == (k + 1) % 2);
  CHECK(views[0].counts.deferred == 10 && views[1].counts.deferred == 10);
  for (const MobileLane::SessionView& v : views) {
    CHECK(v.counts.paired == 100 && v.counts.skipped == 0 &&
          v.counts.over_budget == 0);
  }
  CHECK(has((*lane)->report(),
            "10 deferred to a later gap (one hop a gap; 0 "
            "by the gap's budget)"));
  CHECK((*lane)->close().ok());
  return 0;
}

// A live gap whose end has passed runs no hop and puts its preps back, and
// nothing is lost: once gaps have room again, the loss in between is still
// one split. A file's lane given the same end reads none: it pairs as with
// no budget.
int test_past_budget(const Env& env) {
  auto lane = live_lane(env, {});
  CHECK(lane.ok() && (*lane)->live());
  std::vector<PhoneFrame> script = ten_hz(20);  // 1.0-2.9 s
  for (std::uint64_t i = 30; i < 50; ++i) {
    script.push_back({kS + i * 100 * kMs, i});  // 4.0-5.9 s, 20-29 lost
  }
  std::shared_ptr<const std::vector<vr_test::ScriptedRecord>> fits;
  auto parts = logged_session(script, "phone", &fits);
  CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
  vr_example::GapBudget past;
  past.gap_end = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  // Record k is due at gap ceil(2.5 k): 0-5 paired by gap 14, the hops at
  // 1.0 and 1.5 s run; record 6 (1.6 s, gap 15) is a prep.
  for (std::size_t i = 0; i < 15; ++i) {
    CHECK((*lane)->next(env.grid, gap_set(i)).ok());
  }
  MobileLane::Counts c = (*lane)->sessions().at(0).counts;
  CHECK(fits->size() == 2 && c.paired == 6 && c.deferred == 0);
  // Past the gap's end to 6.2 s of sets, past every record and the loss:
  // the prep at gap 15 goes back, and so does each record due after it,
  // each by the budget; a gap already past runs no probe.
  for (std::size_t i = 15; i < 130; ++i) {
    CHECK((*lane)->next(env.grid, gap_set(i), past).ok());
    c = (*lane)->sessions().at(0).counts;
    if (i == 15) CHECK(c.deferred == 1);
    CHECK(fits->size() == 2 && c.paired == 6 && c.splits == 0 &&
          c.over_budget == c.deferred);
  }
  CHECK(c.deferred > 100 && c.skipped > 0);
  // Room again: the newest record, put back last, pairs, its hop runs, and
  // the loss before it is one split.
  for (std::size_t i = 130; i < 140; ++i) {
    CHECK((*lane)->next(env.grid, gap_set(i)).ok());
  }
  c = (*lane)->sessions().at(0).counts;
  CHECK(fits->size() == 3 && fits->back().pts.back() == kS + 49 * 100 * kMs);
  CHECK(c.paired == 7 && c.splits == 1);
  CHECK(has((*lane)->report(), "0 probe(s)"));
  CHECK((*lane)->close().ok());
  // A file's lane, given that end throughout and given none.
  MobileLane::Counts file[2];
  std::size_t file_fits[2] = {0, 0};
  for (int given = 0; given < 2; ++given) {
    auto f = empty_lane(env);
    CHECK(f.ok() && !(*f)->live());
    auto p = logged_session(script, "phone", &fits);
    CHECK(p.ok() && (*f)->add(std::move(*p)).ok());
    for (std::size_t i = 0; i < 140; ++i) {
      CHECK((*f)->next(env.grid, gap_set(i),
                       given == 1 ? past : vr_example::GapBudget{})
                .ok());
    }
    file[given] = (*f)->sessions().at(0).counts;
    file_fits[given] = fits->size();
    CHECK((*f)->close().ok());
  }
  CHECK(same_counts(file[0], file[1]) && file_fits[0] == file_fits[1]);
  CHECK(file[1].deferred == 0 && file[1].paired == 40 && file[1].skipped == 0 &&
        file[1].splits == 1);
  return 0;
}

// The guard's main branch and its probe, on a live lane whose estimate
// starts at an hour, above every gap's minute of room (so no host work, how
// slow soever, makes a gap late): the hops held back while the preps pair,
// every deferral the budget's; once the budget has held hops back for a
// second of sets with none run, the next gap runs one, a probe that ends
// within its minute and so brings the estimate down; then the hops run as
// due.
int test_guard(const Env& env) {
  LaneOptions options;
  options.hop_seed = std::chrono::hours(1);
  auto lane = live_lane(env, options);
  CHECK(lane.ok());
  std::shared_ptr<const std::vector<vr_test::ScriptedRecord>> fits;
  auto parts = logged_session(ten_hz(60), "phone", &fits);  // 1.0-6.9 s
  CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
  // No budget: records 0-5 paired by gap 14, the hops at 1.0 and 1.5 s run.
  for (std::size_t i = 0; i < 15; ++i) {
    CHECK((*lane)->next(env.grid, gap_set(i)).ok());
  }
  MobileLane::Counts c = (*lane)->sessions().at(0).counts;
  CHECK(fits->size() == 2 && c.paired == 6 && c.deferred == 0);
  CHECK(has((*lane)->report(), "a hop estimated at 3600000.0 ms"));
  const auto minute = [] { return ending_in(std::chrono::minutes(1)); };
  // A minute of room each gap: records 6-9 (gaps 15-23) are preps and pair;
  // record 10's hop (2.0 s, gap 25) goes back, and so does each newer
  // record's that takes its place, until gap 50 (3.04 s of sets, a second
  // after gap 25's 2.04 s), whose probe runs record 20's.
  for (std::size_t i = 15; i < 50; ++i) {
    CHECK((*lane)->next(env.grid, gap_set(i), minute()).ok());
    c = (*lane)->sessions().at(0).counts;
    CHECK(fits->size() == 2);
    // Records 0 to 2i/5 are due by gap i.
    CHECK(c.paired == std::min<std::size_t>(10, 2 * i / 5 + 1));
    CHECK(c.over_budget == c.deferred && c.deferred == (i < 25 ? 0 : i - 24));
  }
  CHECK(c.paired == 10 && c.deferred == 25 && c.skipped == 9);
  CHECK((*lane)->next(env.grid, gap_set(50), minute()).ok());
  CHECK(fits->size() == 3 && fits->back().pts.back() == kS + 20 * 100 * kMs);
  CHECK((*lane)->sessions().at(0).counts.skipped == 10);
  std::string report = (*lane)->report();
  CHECK(has(report, "1 probe(s)") &&
        report.find("estimated at 3600000.0 ms") == std::string::npos);
  // The estimate the probe's: each hop after it runs as due, at records 25,
  // 30, ..., 55, with no deferral more.
  for (std::size_t i = 51; i < 160; ++i) {
    CHECK((*lane)->next(env.grid, gap_set(i), minute()).ok());
  }
  c = (*lane)->sessions().at(0).counts;
  CHECK(fits->size() == 10 && c.deferred == 25 && c.paired == 50 &&
        c.skipped == 10);
  CHECK(has((*lane)->report(), "1 probe(s)"));
  CHECK((*lane)->close().ok());
  return 0;
}

// A live lane's budget that ends far ahead (an hour) defers nothing: the
// same fillers and counts as no budget. And every hop that runs feeds the
// estimate: from an hour, 30 hops with no budget bring it down.
int test_far_budget(const Env& env) {
  std::vector<std::pair<std::uint64_t, tsdf::FillTier>> got[2];
  MobileLane::Counts counts[2];
  for (int far = 0; far < 2; ++far) {
    LaneOptions options;
    options.hop_seed = std::chrono::hours(1);
    auto lane = live_lane(env, far == 1 ? LaneOptions{} : options);
    CHECK(lane.ok());
    std::shared_ptr<const std::vector<vr_test::ScriptedRecord>> fits;
    auto parts = logged_session(ten_hz(150), "phone", &fits);  // 30 hops
    CHECK(parts.ok() && (*lane)->add(std::move(*parts)).ok());
    for (std::size_t i = 0; i < 400; ++i) {
      auto sent = (*lane)->next(env.grid, gap_set(i),
                                far == 1 ? ending_in(std::chrono::hours(1))
                                         : vr_example::GapBudget{});
      CHECK(sent.ok());
      for (const sender::Filler& f : *sent) {
        got[far].emplace_back(f.frame.timestamp_ns, f.tier);
      }
    }
    CHECK(fits->size() == 30);
    counts[far] = (*lane)->sessions().at(0).counts;
    if (far == 0) {
      CHECK(has((*lane)->report(), "0 probe(s)") &&
            (*lane)->report().find("estimated at 3600000.0 ms") ==
                std::string::npos);
    }
    CHECK((*lane)->close().ok());
  }
  CHECK(same_counts(counts[0], counts[1]) && counts[1].deferred == 0);
  CHECK(!got[0].empty() && got[0] == got[1]);
  return 0;
}

// One filler as a send got it, to compare runs by.
struct Got {
  std::size_t send = 0;
  std::uint64_t pts = 0;
  std::uint32_t index = 0;
  tsdf::FillTier tier = tsdf::FillTier::Anchored;
  Mat4f pose{1.0f};

  bool operator==(const Got& o) const {
    return send == o.send && pts == o.pts && index == o.index &&
           tier == o.tier && std::memcmp(&pose, &o.pose, sizeof pose) == 0;
  }
};

// Three phones on one clock, every hop due in the same gaps: the fillers each
// send got and the sessions' counts, one hop a gap throughout.
vkc::Result<std::vector<Got>> three(const Env& env,
                                    std::vector<MobileLane::Counts>* counts) {
  VKC_ASSIGN(std::unique_ptr<MobileLane> lane, empty_lane(env));
  std::shared_ptr<const std::vector<vr_test::ScriptedRecord>> fits[3];
  for (int k = 0; k < 3; ++k) {
    VKC_ASSIGN(MobileLane::SessionParts parts,
               logged_session(ten_hz(60), "p" + std::to_string(k), &fits[k]));
    VKC_TRY(lane->add(std::move(parts)));
  }
  std::vector<Got> out;
  std::size_t ran = 0;
  for (std::size_t i = 0; i < 260; ++i) {
    VKC_ASSIGN(const std::vector<sender::Filler> sent,
               lane->next(env.grid, kS + (i + 1) * 40 * kMs));
    for (const sender::Filler& f : sent) {
      out.push_back({i, f.frame.timestamp_ns, f.index, f.tier,
                     f.frame.depth_camera.cam_to_world});
    }
    const std::size_t now = fits[0]->size() + fits[1]->size() + fits[2]->size();
    if (now > ran + 1) {
      return vkc::Status::invalid_argument("two hops in one gap");
    }
    ran = now;
  }
  counts->clear();
  for (const MobileLane::SessionView& v : lane->sessions()) {
    counts->push_back(v.counts);
  }
  VKC_TRY(lane->close());
  return out;
}

// Three phones replay identically: the same fillers in the same sends, the
// same counts. Their hops always due together, the first place goes round:
// each phone is put back as often as the others, and passes as many records
// by (a hop that waits two gaps where the next record is due two gaps on).
int test_three_replay(const Env& env) {
  std::vector<MobileLane::Counts> counts[2];
  auto first = three(env, &counts[0]);
  auto again = three(env, &counts[1]);
  if (!first.ok())
    std::fprintf(stderr, "%s\n", first.status().message().c_str());
  CHECK(first.ok() && again.ok() && !first->empty() && *first == *again);
  CHECK(counts[0].size() == 3 && counts[1].size() == 3);
  const MobileLane::Counts& first_phone = counts[0][0];
  CHECK(first_phone.deferred > 0 && first_phone.over_budget == 0);
  for (std::size_t k = 0; k < 3; ++k) {
    CHECK(same_counts(counts[0][k], counts[1][k]));
    // Each record paired, or passed by for a newer one due before its hop
    // ran.
    const MobileLane::Counts& c = counts[0][k];
    CHECK(c.paired + c.skipped == 60 && c.fused > 0);
    CHECK(c.deferred == first_phone.deferred &&
          c.skipped == first_phone.skipped);
  }
  return 0;
}

// --- Through open_mobile_lane -----------------------------------------------

// A depth-alone phone's session, its id ending `id`, and its frames.
struct Phone {
  stream::Session session;
  vr_test::PatternFrames frames;
};

Phone phone(std::uint8_t id, std::size_t frames = 10) {
  Phone p{{}, vr_test::pattern_frames(vr_test::phone_info(), frames)};
  p.session = vr_test::session_of(p.frames.info, id);
  return p;
}

// The rig's calibration, its world a sensor (no tag), and `p`'s
// registration in it, in `dir`.
struct Files {
  std::string calibration;
  std::string registration;
};

vkc::Result<Files> write_files(const std::filesystem::path& dir,
                               const Phone& p) {
  std::filesystem::create_directories(dir);
  camera::ArrayCalibration array;
  array.sensors.push_back({"rig-0", camera::Mat4d(1.0), {}, {}, {}, 0.0});
  array.world = {"rig-0", std::nullopt};
  Files f;
  f.calibration = (dir / "array_calibration.json").string();
  VKC_TRY(camera::write_array_calibration(f.calibration, array,
                                          vkc::WriteMode::Replace));
  camera::TrackedRegistration reg;
  reg.sensor = p.frames.info.id;
  reg.session = p.session.id;
  reg.world = array.world;
  reg.rms_px = 0.5;
  reg.views = 6;
  reg.edge_m = 0.15;
  f.registration = (dir / "phone.registration.json").string();
  VKC_TRY(camera::write_tracked_registration(f.registration, reg,
                                             vkc::WriteMode::Replace));
  return f;
}

// A live scenario on 127.0.0.1:`port`, its registration if given, its log
// in `track_log` if given, and `extra` keys at the top level.
vkc::Result<cfg::Config> live_scenario(const Files& files, std::uint16_t port,
                                       bool registered,
                                       const std::string& track_log,
                                       const std::string& extra = {}) {
  std::string mobile =
      "\"mobile\": \"tcp://127.0.0.1:" + std::to_string(port) + "\"";
  if (registered) {
    mobile += ", \"mobile_registration\": \"" + files.registration + "\"";
  }
  std::string text = R"({"version": 1, "operation": "fuse",
      "source": {"kind": "live", "sync_config": "sync.json",
                 "calibration": ")" +
                     files.calibration + "\", " + mobile + "}";
  if (!track_log.empty()) {
    text += ", \"diagnostics\": {\"track_log\": \"" + track_log + "\"}";
  }
  text += extra + "}";
  return cfg::parse(text, cfg::Application::FuseOrbbec);
}

// A writer on a thread of its own: it dials `p`'s session, sends `count` of
// its frames, and holds the connection until let go (`hang_up`); then it
// is gone, as a phone that leaves is.
class Writer {
 public:
  Writer(const Phone& p, std::uint16_t port, std::size_t count)
      : thread_([this, &p, port, count] { run(p, port, count); }) {}
  ~Writer() { hang_up(); }
  Writer(const Writer&) = delete;
  Writer& operator=(const Writer&) = delete;

  void hang_up() {
    done_ = true;
    if (thread_.joinable()) thread_.join();
  }
  bool sent() const { return sent_.load(); }

 private:
  void run(const Phone& p, std::uint16_t port, std::size_t count) {
    auto d = vr_test::Dialler::dial(p.session, port, false,
                                    std::chrono::seconds(10));
    if (!d.ok()) return;
    for (std::size_t i = 0; i < count && !done_.load(); ++i) {
      vr_test::Record r;
      if (!vr_test::depth_record(p.frames.info, p.frames.frames[i], 0, i, &r)
               .ok() ||
          !(*d)->publish(r).ok()) {
        return;
      }
    }
    sent_ = true;
    (void)vr_test::wait_for([&] { return done_.load(); },
                            std::chrono::seconds(30));
  }

  std::atomic<bool> done_{false};
  std::atomic<bool> sent_{false};
  std::thread thread_;  // last: started once the rest is
};

// Gaps 40 ms of set time apart from `*set`, until `done` or 8 s of steady
// time; whether `done` came, every gap OK.
bool run_until(const Env& env, MobileLane& lane, std::uint64_t* set,
               const std::function<bool()>& done) {
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (!done()) {
    if (std::chrono::steady_clock::now() >= until) return false;
    *set += 40 * kMs;
    const vkc::Status ran = lane.next(env.grid, *set).status();
    if (!ran.ok()) {
      std::fprintf(stderr, "next: %s\n", ran.message().c_str());
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}

// Whether a writer of `p` dialling now is refused: closed right after its
// session's chunk.
bool refused_now(const Phone& p, std::uint16_t port) {
  auto d = vr_test::Dialler::dial(p.session, port);
  return d.ok() && (*d)->let_go();
}

// open returns at once; a run no phone dials writes nothing; a phone joins
// while the sets run, and leaves at the close; the layout; a second run into
// the directory takes its phone, unlogged.
int test_open_live(const Env& env, const std::filesystem::path& dir) {
  const Phone a = phone(0x41);
  auto files = write_files(dir, a);
  CHECK(files.ok());
  const std::filesystem::path log = dir / "log";
  const std::uint16_t port = vkc::test::closed_port();
  auto opt = live_scenario(*files, port, true, log.string());
  CHECK(opt.ok());
  // No phone: D made, and nothing in it.
  {
    auto quiet =
        vr_example::open_mobile_lane(*opt, env.gpu.device, env.gpu.allocator);
    CHECK(quiet.ok());
    std::uint64_t at = kS;
    for (std::size_t i = 0; i < 5; ++i) {
      CHECK((*quiet)->next(env.grid, at += 40 * kMs).ok());
    }
    CHECK((*quiet)->close().ok());
    CHECK(std::filesystem::is_directory(log) && std::filesystem::is_empty(log));
  }
  // A dated folder, no session's, is no earlier run.
  std::filesystem::create_directories(log / "2026-11-07");
  std::ofstream(log / "2026-11-07" / "run.csv") << "notes\n";
  const auto t0 = std::chrono::steady_clock::now();
  auto lane =
      vr_example::open_mobile_lane(*opt, env.gpu.device, env.gpu.allocator);
  CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(1));
  CHECK(lane.ok() && (*lane)->enabled() && (*lane)->sessions().empty());
  std::uint64_t set = kS;
  {
    Writer w(a, port, 10);
    CHECK(run_until(env, **lane, &set, [&] {
      return w.sent() && (*lane)->counts().polled == 10;
    }));
  }
  const MobileLane::SessionView v = (*lane)->sessions().at(0);
  const std::string hex = vr_test::hex_of(a.session.id);
  const std::filesystem::path own = log / ("01-ipad-" + hex.substr(0, 8));
  CHECK(v.ordinal == 1 && v.name == "ipad" &&
        v.placement == MobileLane::Placement::Registration);
  CHECK(v.log != nullptr && v.log->directory() == own.string());
  CHECK(has((*lane)->report(),
            "phones: door on tcp://127.0.0.1:" + std::to_string(port) +
                ": 1 accepted, 1 arrived, 1 admitted"));
  CHECK((*lane)->close().ok());
  std::fputs((*lane)->report().c_str(), stdout);
  CHECK(std::filesystem::exists(own / "phone.vrst") &&
        std::filesystem::exists(own / "summary.csv"));
  auto read = vr_example::read_track_log(own.string());
  CHECK(read.ok() && read->source.session_id == hex &&
        read->source.sensor == "ipad" && read->source.ordinal == 1 &&
        read->source.mobile_index == 0 && read->source.joined_set_ns > kS &&
        read->source.peer.rfind("127.0.0.1:", 0) == 0 &&
        read->source.registration == files->registration);
  const std::vector<std::string> rows = lines_of(log / "sessions.csv");
  CHECK(rows.size() == 3 && rows[1].rfind("join,", 0) == 0 &&
        rows[2].rfind("leave,", 0) == 0);
  CHECK(has(rows[2], ",01-ipad-" + hex.substr(0, 8) + ",10,") &&
        has(rows[2], ",the run's end"));
  CHECK(!std::filesystem::exists(log / "door.csv"));  // nothing refused

  // A second run into the directory: its phone joins, and nothing is
  // written beside the first run's.
  auto again =
      vr_example::open_mobile_lane(*opt, env.gpu.device, env.gpu.allocator);
  CHECK(again.ok() && (*again)->enabled());
  {
    Writer w(a, port, 4);
    CHECK(run_until(env, **again, &set, [&] {
      return w.sent() && (*again)->counts().polled == 4;
    }));
  }
  const MobileLane::SessionView unlogged = (*again)->sessions().at(0);
  CHECK(unlogged.ordinal == 1 && unlogged.log != nullptr &&
        unlogged.log->directory().empty());
  CHECK((*again)->close().ok());
  const std::string report = (*again)->report();
  CHECK(has(report, "0 refused (0 turned away)") &&
        has(report, "phones not logged: diagnostics.track_log: ") &&
        has(report, "holds an earlier run's sessions.csv"));
  CHECK(lines_of(log / "sessions.csv") == rows);  // left as it was
  std::size_t entries = 0;
  for (const auto& entry : std::filesystem::directory_iterator(log)) {
    (void)entry;
    ++entries;
  }
  CHECK(entries == 3);  // sessions.csv, 01-ipad-<hex8>, 2026-11-07
  return 0;
}

// An address another socket holds: the run goes on, and the lane binds it
// once it frees.
int test_bind_retry(const Env& env, const std::filesystem::path& dir) {
  const Phone a = phone(0x42);
  auto files = write_files(dir, a);
  CHECK(files.ok());
  vkc::test::LoopbackListener held = vkc::test::bind_loopback();
  CHECK(held.fd.valid());
  auto opt = live_scenario(*files, held.port, true, {});
  CHECK(opt.ok());
  const auto t0 = std::chrono::steady_clock::now();
  auto lane =
      vr_example::open_mobile_lane(*opt, env.gpu.device, env.gpu.allocator);
  CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(1));
  CHECK(lane.ok() && (*lane)->enabled());
  std::uint64_t set = kS;
  for (std::size_t i = 0; i < 10; ++i) {
    CHECK((*lane)->next(env.grid, set += 40 * kMs).ok());
  }
  CHECK(has((*lane)->report(), "phones: not listening: "));
  held.fd.reset();  // the address frees; the lane binds it within 2 s
  {
    Writer w(a, held.port, 3);
    CHECK(run_until(env, **lane, &set,
                    [&] { return (*lane)->counts().polled == 3; }));
  }
  CHECK((*lane)->sessions().size() == 1);
  CHECK(has((*lane)->report(), "phones: door on tcp://127.0.0.1:"));
  CHECK((*lane)->close().ok());
  return 0;
}

// Settings that rule phones out turn them away; a registration places the
// session it names where no tag places another.
int test_turned_away(const Env& env, const std::filesystem::path& dir) {
  const Phone a = phone(0x43);
  const Phone b = phone(0x44);
  auto files = write_files(dir, a);
  CHECK(files.ok());
  struct Case {
    bool registered;
    std::string extra;
    std::string says;  // in the report
  };
  const Case cases[] = {
      {true, R"(, "fusion": {"dynamic": false})", "fusion.dynamic"},
      {false, "", "source.mobile_registration"},
  };
  for (const Case& c : cases) {
    const std::filesystem::path log =
        dir / ("log" + std::to_string(&c - cases));
    const std::uint16_t port = vkc::test::closed_port();
    auto opt = live_scenario(*files, port, c.registered, log.string(), c.extra);
    CHECK(opt.ok());
    CHECK(vr_example::check_mobile(*opt).ok());  // never the run
    auto lane =
        vr_example::open_mobile_lane(*opt, env.gpu.device, env.gpu.allocator);
    CHECK(lane.ok() && (*lane)->enabled());
    CHECK(refused_now(a, port));
    std::uint64_t set = kS;
    for (std::size_t i = 0; i < 5; ++i) {
      CHECK((*lane)->next(env.grid, set += 40 * kMs).ok());
    }
    CHECK((*lane)->sessions().empty());
    const std::string report = (*lane)->report();
    CHECK(has(report, "1 refused (1 turned away)") &&
          has(report, "every phone turned away: ") && has(report, c.says));
    CHECK((*lane)->close().ok());
    const std::vector<std::string> rows = lines_of(log / "door.csv");
    CHECK(rows.size() == 2 && has(rows[1], ",refused,") &&
          has(rows[1], c.says));
  }
  // The registration's session joins; another, which no tag places, is
  // turned away.
  const std::uint16_t port = vkc::test::closed_port();
  auto opt = live_scenario(*files, port, true, {});
  CHECK(opt.ok());
  auto lane =
      vr_example::open_mobile_lane(*opt, env.gpu.device, env.gpu.allocator);
  CHECK(lane.ok());
  CHECK(refused_now(b, port));
  std::uint64_t set = kS;
  {
    Writer w(a, port, 3);
    CHECK(run_until(env, **lane, &set,
                    [&] { return (*lane)->counts().polled == 3; }));
  }
  CHECK((*lane)->sessions().size() == 1);
  CHECK(has((*lane)->report(), "1 refused (1 turned away)"));
  CHECK((*lane)->close().ok());
  return 0;
}

// A phone that leaves, then dials again once it retired: a new session.
int test_rejoin(const Env& env, const std::filesystem::path& dir) {
  const Phone a = phone(0x45);
  auto files = write_files(dir, a);
  CHECK(files.ok());
  const std::filesystem::path log = dir / "log";
  const std::uint16_t port = vkc::test::closed_port();
  auto opt = live_scenario(*files, port, true, log.string());
  CHECK(opt.ok());
  LaneOptions options;
  options.settle_ns = 500 * kMs;
  options.grace_ns = kS;
  auto lane = vr_example::open_mobile_lane(*opt, env.gpu.device,
                                           env.gpu.allocator, options);
  CHECK(lane.ok());
  std::uint64_t set = kS;
  {
    Writer w(a, port, 3);
    CHECK(run_until(env, **lane, &set,
                    [&] { return (*lane)->counts().polled == 3; }));
  }  // gone
  CHECK(run_until(env, **lane, &set, [&] {
    return (*lane)->sessions().at(0).stage == Stage::Left;
  }));
  {
    Writer w(a, port, 2);
    CHECK(run_until(env, **lane, &set,
                    [&] { return (*lane)->counts().polled == 5; }));
  }
  const std::vector<MobileLane::SessionView> views = (*lane)->sessions();
  CHECK(views.size() == 2 && views[1].ordinal == 2 &&
        views[1].stage != Stage::Left);
  CHECK((*lane)->close().ok());
  const std::string hex8 = vr_test::hex_of(a.session.id).substr(0, 8);
  CHECK(std::filesystem::exists(log / ("01-ipad-" + hex8) / "phone.vrst") &&
        std::filesystem::exists(log / ("02-ipad-" + hex8) / "phone.vrst"));
  const std::vector<std::string> rows = lines_of(log / "sessions.csv");
  CHECK(rows.size() == 5);
  CHECK(has(rows[2], "leave,") && has(rows[2], ",away 1 s"));
  CHECK(has(rows[3], "join,") && has(rows[3], ",2,"));
  return 0;
}

// A session that fails of its own keeps its id away for the back-off, which
// doubles with each failure.
int test_back_off(const Env& env, const std::filesystem::path& dir) {
  const Phone a = phone(0x47);
  auto files = write_files(dir, a);
  CHECK(files.ok());
  // The registration names a's session, but another sensor: admit refuses
  // it for what is asked, which the door does not remember.
  auto reg = camera::read_tracked_registration(files->registration);
  CHECK(reg.ok());
  reg->sensor = "another";
  CHECK(camera::write_tracked_registration(files->registration, *reg,
                                           vkc::WriteMode::Replace)
            .ok());
  const std::filesystem::path log = dir / "log";
  const std::uint16_t port = vkc::test::closed_port();
  auto opt = live_scenario(*files, port, true, log.string());
  CHECK(opt.ok());
  LaneOptions options;
  options.error_back_off = std::chrono::milliseconds(1000);
  auto lane = vr_example::open_mobile_lane(*opt, env.gpu.device,
                                           env.gpu.allocator, options);
  CHECK(lane.ok());
  const auto not_opened = [&](int n) {
    return vr_test::wait_for([&] {
      return (*lane)->report().find(std::to_string(n) +
                                    " session(s) not opened") !=
             std::string::npos;
    });
  };
  const auto turned_away = [&](int n) {
    return (*lane)->report().find("(" + std::to_string(n) + " turned away)") !=
           std::string::npos;
  };
  // Opened, failed; at once, turned away.
  CHECK(refused_now(a, port) && not_opened(1));
  const auto failed_at = std::chrono::steady_clock::now();
  CHECK(refused_now(a, port) && turned_away(1));
  CHECK(std::chrono::steady_clock::now() - failed_at <
        std::chrono::milliseconds(1000));
  // Past the back-off, opened again and failed; the next back-off is 2 s.
  std::this_thread::sleep_until(failed_at + std::chrono::milliseconds(1100));
  CHECK(refused_now(a, port) && not_opened(2));
  CHECK(refused_now(a, port) && turned_away(2));
  std::uint64_t set = kS;
  for (std::size_t i = 0; i < 5; ++i) {
    CHECK((*lane)->next(env.grid, set += 40 * kMs).ok());
  }
  CHECK((*lane)->sessions().empty());
  CHECK((*lane)->close().ok());
  const std::vector<std::string> rows = lines_of(log / "door.csv");
  for (const std::string& r : rows) std::printf("door.csv: %s\n", r.c_str());
  CHECK(rows.size() == 5);
  CHECK(has(rows[1], ",refused,") && has(rows[1], "the registration is"));
  CHECK(has(rows[2], "kept away 1.0 s after 1 failure(s)"));
  CHECK(has(rows[3], "the registration is"));
  CHECK(has(rows[4], "kept away 2.0 s after 2 failure(s)"));
  return 0;
}

// An empty diagnostics.track_log writes nothing.
int test_empty_log(const Env& env, const std::filesystem::path& dir) {
  const Phone a = phone(0x46);
  auto files = write_files(dir, a);
  CHECK(files.ok());
  const std::uint16_t port = vkc::test::closed_port();
  auto opt = live_scenario(*files, port, true, {});
  CHECK(opt.ok());
  LaneOptions options;
  options.settle_ns = 500 * kMs;
  options.grace_ns = kS;
  auto lane = vr_example::open_mobile_lane(*opt, env.gpu.device,
                                           env.gpu.allocator, options);
  CHECK(lane.ok());
  std::uint64_t set = kS;
  {
    Writer w(a, port, 3);
    CHECK(run_until(env, **lane, &set,
                    [&] { return (*lane)->counts().polled == 3; }));
  }
  CHECK(run_until(env, **lane, &set, [&] {
    return (*lane)->sessions().at(0).stage == Stage::Left;
  }));
  CHECK((*lane)->close().ok());
  CHECK(has((*lane)->report(), "mobile 01 (ipad): 3 records"));
  std::size_t files_there = 0;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    (void)entry;
    ++files_there;
  }
  CHECK(files_there == 2);  // the calibration and the registration
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::test::Scratch scratch("recon_example_live_phones_");
  auto grid =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, 0.05f, 0.15f, 64);
  CHECK(grid.ok());
  const Env env{gpu, *grid};
  // The pipeline cache warmed, as a run's earlier stages leave it, so the
  // times open_mobile_lane is held to measure no wait for a phone, not a
  // first compile of the fitters it checks the device by.
  {
    auto prep = sensor::GpuFramePrep::create(gpu.device, gpu.allocator);
    auto fitters = track::create_fitters(gpu.device, gpu.allocator,
                                         vr_example::lane_tracking());
    CHECK(prep.ok() && fitters.ok());
  }
  CHECK(test_hop_estimate() == 0);
  CHECK(test_leave(env, scratch.path / "leave") == 0);
  CHECK(test_drains_when_retired(env) == 0);
  CHECK(test_return(env) == 0);
  CHECK(test_containment(env) == 0);
  CHECK(test_one_hop_a_gap(env) == 0);
  CHECK(test_past_budget(env) == 0);
  CHECK(test_guard(env) == 0);
  CHECK(test_far_budget(env) == 0);
  CHECK(test_three_replay(env) == 0);
  CHECK(test_open_live(env, scratch.path / "open") == 0);
  CHECK(test_bind_retry(env, scratch.path / "bind") == 0);
  CHECK(test_turned_away(env, scratch.path / "away") == 0);
  CHECK(test_rejoin(env, scratch.path / "rejoin") == 0);
  CHECK(test_back_off(env, scratch.path / "back_off") == 0);
  CHECK(test_empty_log(env, scratch.path / "empty") == 0);
  std::puts("recon_example_live_phones: OK");
  return 0;
}

}  // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
