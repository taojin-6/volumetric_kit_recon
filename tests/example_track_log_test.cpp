// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A tracking run's log and its one summary (examples/common/track_log.hpp):
// - A write -> read round trip: a run of scripted fitters (two driver chains,
//   arkit, a shadow that refuses at times) through a log to a directory, read
//   back field for field; each chain's TUM has a line `t tx ty tz qx qy qz qw`
//   for each decided frame, in time order; an empty directory writes nothing.
// - The summary against values worked by hand on a run built in place:
//   decisions by kind, the lock share, latency, the distance from arkit, the
//   jitter between consecutive Anchored frames, fits by verdict, the fill by
//   placement and the drops, and arkit_raw rebuilt from a device step, its
//   poses written beside the table.
// - A window recovers what single frames leave held (real fitters, on the
//   device): frames of a room whose model holds only one corner -- a floor and
//   two walls -- each frame seeing a wall and the floor with a third of its
//   view on the model, registered 5 cm / 1 deg off. The `window` driver
//   anchors and its correction undoes the error within 3 mm / 0.1 deg; the
//   single-frame shadow refuses every frame (coverage under 0.4) and keeps
//   the error, as the summary's rows say.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "scripted_fitter.hpp"
#include "track_log.hpp"
#include "track_scenes.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/track/tracker.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace vol = volumetric_kit::grid::volume;
namespace ts = track_scenes;
using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec3f;
using volumetric_kit::recon::Vec4f;
using vr_example::TrackRun;

namespace {

constexpr std::uint64_t kMs = 1'000'000;

Mat4f translate(float x, float y, float z) {
  Mat4f m(1.0f);
  m[3] = Vec4f(x, y, z, 1.0f);
  return m;
}

bool same(const Mat4f& a, const Mat4f& b) {
  return std::memcmp(&a, &b, sizeof(Mat4f)) == 0;
}

track::TrackFit anchored_fit(const Mat4f& correction) {
  track::TrackFit fit;
  fit.correction = correction;
  fit.lambda_max = 1.0f;
  fit.relative_eigenvalues = {0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
  for (int i = 0; i < 6; ++i) fit.directions[std::size_t(7 * i)] = 1.0f;
  fit.held = 0;
  fit.frames = 1;
  fit.attempts = 20000;
  fit.hits = 18000;
  fit.inliers = 17000;
  fit.rms_m = 0.004f;
  fit.origin = Vec3f(0.25f, -1.5f, 0.75f);
  fit.scale_m = 1.75f;
  return fit;
}

// --- 1. The round trip ------------------------------------------------------

int round_trip(vkc::test::GpuContext& gpu, const std::filesystem::path& dir) {
  auto depth = vkc::device_storage_buffer(gpu.allocator, 32 * 24 * 4);
  CHECK(depth.ok());
  const auto buffer = std::make_shared<const vkc::Buffer>(std::move(*depth));
  const vol::AttributeSpec attrs[] = {{vol::kTsdfAttribute, 4},
                                      {vol::kWeightAttribute, 4}};
  auto grid = vol::VoxelBlockGrid::create(gpu.device, gpu.allocator,
                                          ts::grid_params(0.05f, 8), attrs, 2);
  CHECK(grid.ok());
  track::TrackerConfig config;  // window_method on rig and track
  config.drivers = {{"rig", {}}, {"track", {}}};
  track::Method frame = track::frame_method();
  frame.seed = "end, with a comma.json";
  config.shadows = {track::device_method(), frame};
  using vr_test::ScriptedFitter;
  const auto at = [](std::uint64_t t) {
    return translate(0.001f * float(t / (100 * kMs)), 0.0f, 0.002f);
  };
  std::vector<std::unique_ptr<track::IFitter>> fitters;
  fitters.push_back(std::make_unique<ScriptedFitter>(
      [&at](const std::vector<sensor::DeviceFrame>& f,
            const track::PosePrior&) -> vkc::Result<track::TrackFit> {
        return anchored_fit(at(f.back().timestamp_ns));
      }));
  fitters.push_back(std::make_unique<ScriptedFitter>(
      [](const std::vector<sensor::DeviceFrame>&,
         const track::PosePrior&) -> vkc::Result<track::TrackFit> {
        track::TrackFit none;
        return none;
      }));
  fitters.emplace_back();
  fitters.push_back(std::make_unique<ScriptedFitter>(
      [](const std::vector<sensor::DeviceFrame>& f,
         const track::PosePrior&) -> vkc::Result<track::TrackFit> {
        if (f.back().timestamp_ns % (300 * kMs) == 0) {
          return vkc::Status::invalid_argument("scripted\nrefusal, twice");
        }
        return anchored_fit(Mat4f(1.0f));
      }));
  auto tracker = track::Tracker::create(config, std::move(fitters));
  CHECK(tracker.ok());
  vr_example::RunSource source;
  source.session = "phone, take.vrst";
  source.registration = "phone.registration.json";
  source.device = "test GPU";
  source.tracking_to_world = translate(0.5f, 0.25f, 1.0f);
  source.device_steps = {{1200 * kMs, translate(0.0f, 0.05f, 0.0f)}};
  auto log = vr_example::TrackLog::open(dir.string(), **tracker, source);
  CHECK(log.ok());
  for (std::uint64_t i = 0; i < 40; ++i) {
    sensor::DeviceFrame f;
    f.depth = buffer;
    f.depth_camera = {40.0f, 40.0f, 15.5f,
                      11.5f, 0.1f,  5.0f,
                      32,    24,    translate(0.01f * float(i), 0.0f, 1.0f)};
    f.timestamp_ns = (i + 1) * 100 * kMs;
    f.depth_noise = {0.002f, 0.0f, 0.002f};
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    auto p = (*tracker)->record(batch, *grid, f, &(*log)->rows());
    CHECK(p.ok());
    (*log)->rows().add_gpu("frame prep", 0.125 * double(i));
    auto step = (*tracker)->finish(*p);
    CHECK(step.ok());
    CHECK((*log)->write(*step).ok());
  }
  CHECK((*log)->write((*tracker)->flush()).ok());
  CHECK((*log)->close().ok());
  const TrackRun& a = (*log)->run();
  CHECK(!a.frames.empty() && !a.fits.empty() && !a.placements.empty() &&
        !a.rows.empty());
  auto read = vr_example::read_track_log(dir.string());
  if (!read.ok()) std::fprintf(stderr, "%s\n", read.status().message().c_str());
  CHECK(read.ok());
  const TrackRun& b = *read;
  CHECK(b.source.session == a.source.session &&
        b.source.registration == a.source.registration &&
        b.source.device == a.source.device &&
        same(b.source.tracking_to_world, a.source.tracking_to_world));
  CHECK(b.source.device_steps.size() == 1 &&
        b.source.device_steps[0].pts_ns == 1200 * kMs &&
        same(b.source.device_steps[0].raw_from_sent,
             a.source.device_steps[0].raw_from_sent));
  CHECK(b.hold_ns == a.hold_ns && b.chains.size() == 4);
  for (std::size_t c = 0; c < 4; ++c) {
    CHECK(b.chains[c].name == a.chains[c].name &&
          b.chains[c].driver == a.chains[c].driver &&
          b.chains[c].fit == a.chains[c].fit &&
          b.chains[c].window_ns == a.chains[c].window_ns &&
          b.chains[c].hop_ns == a.chains[c].hop_ns &&
          b.chains[c].passes == a.chains[c].passes &&
          b.chains[c].seed == a.chains[c].seed &&
          same(b.chains[c].initial_correction, a.chains[c].initial_correction));
  }
  CHECK(b.chains[3].seed == "end, with a comma.json");
  CHECK(b.frames.size() == a.frames.size());
  for (std::size_t i = 0; i < a.frames.size(); ++i) {
    const auto& x = a.frames[i];
    const auto& y = b.frames[i];
    CHECK(x.pts_ns == y.pts_ns && x.chain == y.chain &&
          x.pose.from == y.pose.from && x.pose.fit == y.pose.fit &&
          x.pose.decided_pts_ns == y.pose.decided_pts_ns &&
          same(x.pose.correction, y.pose.correction) &&
          same(x.device_pose, y.device_pose));
  }
  CHECK(b.fits.size() == a.fits.size());
  std::size_t refused = 0;
  for (std::size_t i = 0; i < a.fits.size(); ++i) {
    const auto& x = a.fits[i];
    const auto& y = b.fits[i];
    CHECK(x.chain == y.chain && x.serial == y.serial &&
          x.first_pts_ns == y.first_pts_ns && x.last_pts_ns == y.last_pts_ns &&
          x.refused == y.refused && x.judged.state == y.judged.state &&
          same(x.judged.correction, y.judged.correction) &&
          same(x.prior.correction, y.prior.correction) &&
          x.prior.sigma_m == y.prior.sigma_m &&
          x.prior.sigma_rad == y.prior.sigma_rad &&
          same(x.fit.correction, y.fit.correction) &&
          x.fit.lambda_max == y.fit.lambda_max &&
          x.fit.relative_eigenvalues == y.fit.relative_eigenvalues &&
          x.fit.held == y.fit.held && x.fit.frames == y.fit.frames &&
          x.fit.passes == y.fit.passes && x.fit.attempts == y.fit.attempts &&
          x.fit.hits == y.fit.hits && x.fit.inliers == y.fit.inliers &&
          x.fit.rms_m == y.fit.rms_m && x.fit.origin == y.fit.origin &&
          x.fit.scale_m == y.fit.scale_m);
    refused += x.refused.empty() ? 0 : 1;
  }
  CHECK(refused > 0);  // the shadow's, its message on one line
  CHECK(b.placements.size() == a.placements.size());
  for (std::size_t i = 0; i < a.placements.size(); ++i) {
    CHECK(a.placements[i].pts_ns == b.placements[i].pts_ns &&
          a.placements[i].dropped == b.placements[i].dropped &&
          (a.placements[i].dropped ||
           (a.placements[i].placement == b.placements[i].placement &&
            a.placements[i].driver == b.placements[i].driver)));
  }
  CHECK(b.rows.size() == a.rows.size());
  for (std::size_t i = 0; i < a.rows.size(); ++i) {
    CHECK(a.rows[i].step == b.rows[i].step &&
          a.rows[i].name == b.rows[i].name &&
          a.rows[i].cpu_ms == b.rows[i].cpu_ms &&
          a.rows[i].gpu_ms == b.rows[i].gpu_ms);
  }
  // The TUMs: a line of eight numbers a decided frame, in time order.
  for (std::size_t c = 0; c < 4; ++c) {
    std::ifstream tum(dir / ("poses_" + a.chains[c].name + ".tum"));
    CHECK(tum.good());
    std::size_t lines = 0;
    double last = -1.0;
    for (std::string line; std::getline(tum, line);) {
      std::istringstream in(line);
      double v[8];
      for (double& x : v) in >> x;
      CHECK(!in.fail());
      std::string extra;
      CHECK(!(in >> extra));
      CHECK(v[0] > last);
      last = v[0];
      CHECK(std::fabs(v[4] * v[4] + v[5] * v[5] + v[6] * v[6] + v[7] * v[7] -
                      1.0) < 1e-6);
      ++lines;
    }
    std::size_t decided = 0;
    for (const auto& f : a.frames) decided += f.chain == c ? 1 : 0;
    CHECK(lines == decided && decided == 40);
  }
  // Another format is refused; an empty directory writes nothing.
  {
    std::ofstream bad(dir / "run.csv");
    bad << "key,value\nformat,something.else\nversion,1\n";
  }
  CHECK(vr_example::read_track_log(dir.string()).status().domain() ==
        vkc::Status::Code::InvalidArgument);
  auto quiet = vr_example::TrackLog::open("", **tracker, source);
  CHECK(quiet.ok() && (*quiet)->close().ok());
  return 0;
}

// --- 2. The summary by hand -------------------------------------------------

// summary.csv's row for `chain`, by column name.
std::map<std::string, std::string> summary_row(const std::filesystem::path& dir,
                                               const std::string& chain) {
  std::ifstream in(dir / "summary.csv");
  std::string header;
  std::getline(in, header);
  std::vector<std::string> names;
  std::stringstream hs(header);
  for (std::string n; std::getline(hs, n, ',');) names.push_back(n);
  for (std::string line; std::getline(in, line);) {
    std::vector<std::string> cells;
    std::stringstream ls(line);
    for (std::string c; std::getline(ls, c, ',');) cells.push_back(c);
    cells.resize(names.size());
    if (cells[0] != chain) continue;
    std::map<std::string, std::string> row;
    for (std::size_t i = 0; i < names.size(); ++i) row[names[i]] = cells[i];
    return row;
  }
  return {};
}

double cell(const std::map<std::string, std::string>& row, const char* name) {
  const auto it = row.find(name);
  return it == row.end() || it->second.empty() ? std::nan("")
                                               : std::stod(it->second);
}

int summary_by_hand(const std::filesystem::path& dir) {
  TrackRun run;
  run.hold_ns = 5'000'000'000;
  vr_example::ChainInfo rig;
  rig.name = "rig";
  rig.driver = true;
  rig.fit = track::FitKind::PointToPlane;
  vr_example::ChainInfo arkit;
  arkit.name = "arkit";
  run.chains = {rig, arkit};
  // Frames at 100 ms, every device pose 1 m in front of the origin. rig:
  // frames 0-3 Anchored at x = 10, 12, 14, 16 mm, decided 2 s later; frame 4
  // Coasting, 5 s later. arkit: each at once.
  const std::uint64_t t0 = 1'000'000'000;
  for (std::uint64_t i = 0; i < 5; ++i) {
    vr_example::FrameRecord f;
    f.pts_ns = t0 + i * 100 * kMs;
    f.device_pose = translate(0.0f, 0.0f, 1.0f);
    f.chain = 0;
    f.pose.from = i < 4 ? track::PoseFrom::Anchored : track::PoseFrom::Coasting;
    f.pose.correction = translate(0.010f + 0.002f * float(i), 0.0f, 0.0f);
    f.pose.decided_pts_ns = f.pts_ns + (i < 4 ? 2000 : 5000) * kMs;
    run.frames.push_back(f);
    f.chain = 1;
    f.pose.from = track::PoseFrom::Device;
    f.pose.correction = Mat4f(1.0f);
    f.pose.decided_pts_ns = f.pts_ns;
    run.frames.push_back(f);
  }
  vr_example::FitRecord fit;
  fit.chain = 0;
  fit.judged.state = track::TrackState::Anchored;
  run.fits = {fit, fit};
  run.fits[1].judged.state = track::TrackState::Rejected;
  run.placements = {{t0, false, track::Placement::Anchored, 0},
                    {t0 + 100 * kMs, false, track::Placement::Interpolated, 0},
                    {t0 + 400 * kMs, true, {}, 0}};
  run.rows = {{0, "track: rig", 1.0, 0.5}, {1, "track: rig", 3.0, 1.5}};
  // One device step at frame 2: the device's own pose 50 mm up in y from
  // there on, in a tracking world rotated 90 deg about z.
  Mat4f rz(1.0f);
  rz[0] = Vec4f(0, 1, 0, 0);
  rz[1] = Vec4f(-1, 0, 0, 0);
  run.source.tracking_to_world = rz;
  run.source.device_steps = {{t0 + 200 * kMs, translate(0.0f, 0.05f, 0.0f)}};
  auto table = vr_example::summarize(run, dir.string());
  CHECK(table.ok());
  std::fputs(table->c_str(), stdout);
  const auto r = summary_row(dir, "rig");
  CHECK(cell(r, "decided") == 5 && cell(r, "anchored") == 4 &&
        cell(r, "coasting") == 1 && cell(r, "device") == 0);
  CHECK(std::fabs(cell(r, "lock_share") - 0.8) < 1e-12);
  CHECK(std::fabs(cell(r, "latency_p50_ms") - 2000.0) < 1e-6);
  CHECK(std::fabs(cell(r, "latency_p95_ms") - 5000.0) < 1e-6);
  // From arkit: 10, 12, 14, 16, 18 mm; p50 14, max 18.
  CHECK(std::fabs(cell(r, "dist_p50_mm") - 14.0) < 1e-3);
  CHECK(std::fabs(cell(r, "dist_max_mm") - 18.0) < 1e-3);
  CHECK(std::fabs(cell(r, "dist_max_deg")) < 1e-6);
  // Jitter: three 2 mm changes between the Anchored frames.
  CHECK(std::fabs(cell(r, "jitter_mm") - 2.0) < 1e-3);
  CHECK(cell(r, "fits") == 2 && cell(r, "fits_anchored") == 1 &&
        cell(r, "fits_rejected") == 1);
  // arkit_raw: 0 for frames 0-1, then the step, seen in the world: 50 mm.
  const auto raw = summary_row(dir, "arkit_raw");
  CHECK(cell(raw, "decided") == 5);
  CHECK(std::fabs(cell(raw, "dist_p50_mm") - 50.0) < 1e-3);
  CHECK(std::fabs(cell(raw, "dist_max_mm") - 50.0) < 1e-3);
  CHECK(std::isnan(cell(raw, "lock_share")));
  std::ifstream tum(dir / "poses_arkit_raw.tum");
  std::size_t lines = 0;
  for (std::string line; std::getline(tum, line);) ++lines;
  CHECK(lines == 5);
  CHECK(table->find("fill: rig anchored 1, interpolated 1; dropped 1") !=
        std::string::npos);
  CHECK(table->find("track: rig") != std::string::npos);
  // No device steps: no arkit_raw row.
  run.source.device_steps.clear();
  auto plain = vr_example::summarize(run);
  CHECK(plain.ok() && plain->find("arkit_raw") == std::string::npos);
  return 0;
}

// --- 3. A window recovers what single frames leave held ---------------------

int window_recovers(vkc::test::GpuContext& gpu) {
  // The room, 6 x 6 x 3 m; the model holds its corner alone: the floor and
  // the walls x = 0 and y = 0, out to 2.4 m (the blocks of [-0.3, 2.2]).
  ts::Scene room{{{{1, 0, 0}, 0},
                  {{0, 1, 0}, 0},
                  {{0, 0, 1}, 0},
                  {{-1, 0, 0}, 6},
                  {{0, -1, 0}, 6},
                  {{0, 0, -1}, 3}},
                 {-0.3f, -0.3f, -0.3f},
                 {6.3f, 6.3f, 3.3f}};
  ts::Scene corner{{{{1, 0, 0}, 0}, {{0, 1, 0}, 0}, {{0, 0, 1}, 0}},
                   {-0.3f, -0.3f, -0.3f},
                   {2.2f, 2.2f, 2.8f}};
  auto model = ts::make_model(gpu, corner, 0.05f, 256);
  CHECK(model.ok());
  // Each frame sees one wall and the floor, a quarter to a third of its view
  // on the model, from the same spot, the walls in turn.
  const Vec3f eye(1.0f, 1.0f, 1.2f);
  const double kRad = ts::kPi / 180.0;
  std::vector<Mat4f> truths;
  const double yaws[] = {85, 0, 90, 5};
  for (std::size_t i = 0; i < 30; ++i) {
    const double yaw = yaws[i % 4] * kRad;
    const double pitch = -10.0 * kRad;
    const Vec3f target = eye + Vec3f(float(std::cos(yaw) * std::cos(pitch)),
                                     float(std::sin(yaw) * std::cos(pitch)),
                                     float(std::sin(pitch)));
    truths.push_back(ts::look_at(eye, target));
  }
  const Mat4f error =
      ts::error_at(ts::Vec3d(eye), {0.03, -0.03, 0.025}, {0.3, 0.5, 1.0}, 1.0);
  ts::Camera cam;
  auto frames = ts::frames_of(gpu, room, cam, truths, error, 100 * kMs);
  CHECK(frames.ok());
  track::TrackerConfig config;
  config.driver = track::window_method();
  config.drivers = {{"rig", tsdf::rig_cast()}};
  config.shadows = {track::device_method(), track::frame_method()};
  auto fitters = track::create_fitters(gpu.device, gpu.allocator, config);
  CHECK(fitters.ok());
  auto tracker = track::Tracker::create(config, std::move(*fitters));
  CHECK(tracker.ok());
  auto log = vr_example::TrackLog::open("", **tracker, {});
  CHECK(log.ok());
  for (const sensor::DeviceFrame& f : *frames) {
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    auto p = (*tracker)->record(batch, *model, f, &(*log)->rows());
    CHECK(p.ok());
    auto done = batch.submit_async({}, {});
    CHECK(done.ok() && done->wait().ok());
    auto step = (*tracker)->finish(*p);
    CHECK(step.ok());
    CHECK((*log)->write(*step).ok());
  }
  CHECK((*log)->write((*tracker)->flush()).ok());
  const TrackRun& run = (*log)->run();
  // The window's last Anchored fit undoes the error; the single frames never
  // anchor, and their chain keeps it.
  const Mat4f truth = ts::inverse(error);
  const vr_example::FitRecord* last = nullptr;
  std::size_t frame_fits = 0;
  for (const vr_example::FitRecord& f : run.fits) {
    if (f.chain == 0 && f.judged.state == track::TrackState::Anchored) {
      last = &f;
    }
    if (f.chain == 2) {
      ++frame_fits;
      if (f.judged.state != track::TrackState::Rejected) {
        std::printf(
            "  frame fit at %llu: state %d, %u attempts, %u hits, %u "
            "inliers, lambda %g, held %u\n",
            static_cast<unsigned long long>(f.last_pts_ns), int(f.judged.state),
            f.fit.attempts, f.fit.hits, f.fit.inliers, double(f.fit.lambda_max),
            unsigned(f.fit.held));
      }
      CHECK(f.judged.state == track::TrackState::Rejected);
      CHECK(double(f.fit.hits) < 0.4 * double(f.fit.attempts));
      CHECK(same(f.judged.correction, Mat4f(1.0f)));
    }
  }
  CHECK(frame_fits == truths.size());
  CHECK(last != nullptr);
  double m = 0.0, deg = 0.0;
  ts::gap(last->judged.correction, truth, eye, m, deg);
  std::printf("  window: %.2f mm / %.3f deg from the truth; frames held at ",
              m * 1e3, deg);
  ts::gap(Mat4f(1.0f), truth, eye, m, deg);
  std::printf("%.1f mm / %.2f deg\n", m * 1e3, deg);
  CHECK(m > 0.045 && deg > 0.9);
  ts::gap(last->judged.correction, truth, eye, m, deg);
  CHECK(m < 0.003 && deg < 0.1);
  auto table = vr_example::summarize(run);
  CHECK(table.ok());
  std::fputs(table->c_str(), stdout);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::test::Scratch scratch("recon_example_track_log_");
  CHECK(round_trip(gpu, scratch.path / "run") == 0);
  CHECK(summary_by_hand(scratch.path / "hand") == 0);
  CHECK(window_recovers(gpu) == 0);
  std::puts("recon_example_track_log: OK");
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
