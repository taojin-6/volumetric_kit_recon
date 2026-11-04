// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The origin tag's watch (examples/common/live_tag.hpp), built with
// VR_WITH_CALIB, on 640 x 480 pictures of OpenCV's 36h11 marker 0 pasted
// face-on, the camera (fx 500) straight above it, put on the device as a
// decoder's NV12 (sensor's testing/nv12_picture.hpp); the phone's world is
// ARKit's, +y up, at a known transform from the tag's. Accuracy is calib's
// own test's; here, the glue:
// - Six steady stills register: the registration validates, names the
//   session and the sensor, and places a still's camera where its picture
//   has it, within 1 cm / 0.5 deg.
// - The gates: no candidate before the settle, while the camera moves or
//   turns too fast, or within 200 ms of the last; no hold from 5 stills or
//   across a split; a still older than the span drops out; two prints in
//   view are no still.
// - A hold with one still's pose 10 cm off is refused (poor_fit) and
//   registers nothing; the next good hold registers, and a good one after it
//   passes, applied by nothing.
// - Without background two runs give the same news, bit for bit. With it,
//   every candidate is searched or replaced, the pictures are let go, and
//   the destructor joins with one boxed.
// - check and create refuse a world with no tag, an empty sensor or session.
// - The lane registered by the tag: frames before the registration are
//   counted unregistered and never tracked; after it they are tracked at the
//   registration's placement; run.csv names registration.json, which reads
//   back, tags.csv holds each hold; a second run into the same directory is
//   refused before any wait.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "live_mobile.hpp"
#include "live_tag.hpp"
#include "scripted_fitter.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/testing/nv12_picture.hpp"

namespace vkc = volumetric_kit::core;
namespace cfg = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace camera = volumetric_kit::sensor::camera;
namespace track = volumetric_kit::recon::track;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace vol = volumetric_kit::grid::volume;
using volumetric_kit::recon::Mat4f;
using vr_example::TagHold;
using vr_example::TagNews;
using vr_example::TagWatch;

namespace {

constexpr std::uint64_t kMs = 1'000'000;
constexpr std::uint64_t kS = 1'000'000'000;
constexpr std::uint32_t kW = 640;
constexpr std::uint32_t kH = 480;
constexpr double kF = 500.0;
constexpr int kSide = 80;       // the tag's pixels
constexpr int kMargin = 20;     // its white margin's
constexpr double kEdge = 0.15;  // the world's tag
// The camera's height over the tag that shows it kSide pixels across.
constexpr double kHeight = kF * kEdge / kSide;
const std::array<std::uint8_t, 16> kSession = {
    0x8f, 0x3c, 0x0a, 0x7e, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

const camera::CameraModel kCamera{{kW, kH}, {kF, kF, 319.5, 239.5}, {}};

std::uint64_t pts(std::size_t i) { return kS + i * 100 * kMs; }

// The phone's world in the tag's: ARKit's +y onto +z, turned 30 deg about
// it, and moved.
camera::Mat4d truth() {
  const camera::Mat4d up = glm::rotate(camera::Mat4d(1.0), glm::radians(90.0),
                                       camera::Vec3d(1.0, 0.0, 0.0));
  const camera::Mat4d yaw = glm::rotate(camera::Mat4d(1.0), glm::radians(30.0),
                                        camera::Vec3d(0.0, 0.0, 1.0));
  return glm::translate(camera::Mat4d(1.0), camera::Vec3d(0.4, -0.3, 0.02)) *
         yaw * up;
}

// The camera in the tag's world, kHeight above (x, y), looking down, the
// image's x along the world's and its rows down the world's y, turned
// `turn_deg` about its axis.
camera::Mat4d above(double x, double y, double turn_deg = 0.0) {
  camera::Mat4d m(1.0);
  m[1] = {0.0, -1.0, 0.0, 0.0};
  m[2] = {0.0, 0.0, -1.0, 0.0};
  m[3] = {x, y, kHeight, 1.0};
  return m * glm::rotate(camera::Mat4d(1.0), glm::radians(turn_deg),
                         camera::Vec3d(0.0, 0.0, 1.0));
}

// A grey picture with the tag pasted at each top-left pixel: the camera
// above the tag's centre sees it at (280, 200).
sensor::testing::HostNv12 tag_picture(
    const std::vector<std::pair<int, int>>& at) {
  static const cv::Mat marker = [] {
    cv::Mat m;
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)
        .generateImageMarker(0, kSide, m, 1);
    return m;
  }();
  sensor::testing::HostNv12 p;
  p.width = kW;
  p.height = kH;
  p.luma.assign(std::size_t{kW} * kH, 128);
  p.chroma.assign(std::size_t{(kW + 1) / 2} * ((kH + 1) / 2) * 2, 128);
  for (const auto& [col, row] : at) {
    for (int r = -kMargin; r < kSide + kMargin; ++r) {
      for (int c = -kMargin; c < kSide + kMargin; ++c) {
        const bool inside = r >= 0 && r < kSide && c >= 0 && c < kSide;
        p.luma[std::size_t(row + r) * kW + std::size_t(col + c)] =
            inside ? marker.at<std::uint8_t>(r, c) : 255;
      }
    }
  }
  return p;
}

vkc::Result<sensor::YuvImage> on_device(vkc::test::GpuContext& gpu,
                                        const sensor::testing::HostNv12& p) {
  VKC_ASSIGN(sensor::YuvImage yuv,
             sensor::testing::device_picture(gpu.device, gpu.allocator, p));
  yuv.full_range = false;  // as a phone's HEVC
  return yuv;
}

// A frame as the lane shows it: its pose in the phone's own world.
sensor::RgbdFrame shown(std::size_t i, const camera::Mat4d& cam_to_world,
                        const sensor::YuvImage& picture) {
  sensor::RgbdFrame f;
  f.color = picture;
  f.color_camera = kCamera;
  f.color_to_world = glm::inverse(truth()) * cam_to_world;
  f.timestamp_ns = pts(i);
  f.sequence = i;
  return f;
}

camera::WorldFrame tag_world() {
  camera::WorldFrame w;
  w.apriltag = camera::AprilTagWorld{"tag36h11", 0, kEdge};
  return w;
}

vkc::Result<std::unique_ptr<TagWatch>> watch(vkc::test::GpuContext& gpu,
                                             bool background) {
  return TagWatch::create(gpu.device, gpu.allocator,
                          {"phone", kSession, tag_world(), background});
}

struct Pictures {
  sensor::YuvImage tag, blank, two;
};

// Frames [first, last] above (0, 0) showing `picture`, each seen.
int show(TagWatch& w, std::size_t first, std::size_t last,
         const sensor::YuvImage& picture,
         const camera::Mat4d& at = above(0.0, 0.0)) {
  for (std::size_t i = first; i <= last; ++i) {
    CHECK(w.see(shown(i, at, picture)).ok());
  }
  return 0;
}

bool same(const camera::Mat4d& a, const camera::Mat4d& b) {
  return std::memcmp(&a, &b, sizeof(a)) == 0;
}

bool same(const TagHold& a, const TagHold& b) {
  return a.first_pts_ns == b.first_pts_ns && a.last_pts_ns == b.last_pts_ns &&
         a.stills == b.stills && a.use == b.use && a.miss == b.miss &&
         a.reason == b.reason && a.rms_px == b.rms_px &&
         a.worst_px == b.worst_px && a.range_m == b.range_m &&
         a.off_normal_deg == b.off_normal_deg &&
         same(a.tracking_to_world, b.tracking_to_world);
}

// --- The cases --------------------------------------------------------------

int test_registers(vkc::test::GpuContext& gpu, const Pictures& pics) {
  auto w = watch(gpu, false);
  CHECK(w.ok());
  CHECK(show(**w, 0, 30, pics.tag) == 0);
  auto news = (*w)->take();
  CHECK(news.ok() && news->failures.empty());
  CHECK(news->holds.size() == 1);
  const TagHold& h = news->holds[0];
  CHECK(h.use == TagHold::Use::Registered && h.stills == 6);
  CHECK(h.first_pts_ns == pts(20) && h.last_pts_ns == pts(30));
  CHECK(h.miss.empty() && h.reason.empty() && h.rms_px < 2.0);
  CHECK(std::abs(h.range_m - kHeight) < 0.01 && h.off_normal_deg < 1.0);
  CHECK(news->registered.has_value());
  const camera::TrackedRegistration& reg = *news->registered;
  CHECK(camera::validate_tracked_registration(reg).ok());
  CHECK(reg.sensor == "phone" && reg.session == kSession &&
        reg.world_epoch == 0 && reg.views == 6 && reg.edge_m == kEdge);
  CHECK(same(reg.tracking_to_world, h.tracking_to_world));
  // A still's camera, placed by the registration, where its picture has it.
  const camera::Mat4d placed =
      reg.tracking_to_world * glm::inverse(truth()) * above(0.0, 0.0);
  const auto [mm, deg] =
      vr_example::pose_gap(Mat4f(placed), Mat4f(above(0.0, 0.0)));
  std::printf("registered: %.2f mm / %.3f deg from the truth\n", mm, deg);
  CHECK(mm < 10.0 && deg < 0.5);
  const TagWatch::Counts& c = (*w)->counts();
  CHECK(c.seen == 31 && c.candidates == 6 && c.stills == 6 && c.holds == 1 &&
        c.refused == 0 && c.replaced == 0 && c.failed == 0);
  return 0;
}

int test_gates(vkc::test::GpuContext& gpu, const Pictures& pics) {
  // None before the settle, then one every 200 ms (above), none from 5.
  auto settle = watch(gpu, false);
  CHECK(settle.ok());
  CHECK(show(**settle, 0, 19, pics.tag) == 0);
  CHECK((*settle)->counts().candidates == 0);
  CHECK(show(**settle, 20, 28, pics.tag) == 0);
  CHECK((*settle)->counts().stills == 5 && (*settle)->counts().holds == 0);
  // Moving at 0.2 m/s, or turning at 20 deg/s: never steady.
  auto moving = watch(gpu, false);
  auto turning = watch(gpu, false);
  CHECK(moving.ok() && turning.ok());
  for (std::size_t i = 0; i <= 30; ++i) {
    CHECK(
        (*moving)->see(shown(i, above(0.02 * double(i), 0.0), pics.tag)).ok());
    CHECK((*turning)
              ->see(shown(i, above(0.0, 0.0, 2.0 * double(i)), pics.tag))
              .ok());
  }
  CHECK((*moving)->counts().seen == 31 && (*moving)->counts().candidates == 0);
  CHECK((*turning)->counts().candidates == 0);
  // A split ends the hold being gathered: stills at 2.0-2.4 s, then from
  // 2.7 s (the frame after it has nothing to be steady against).
  auto split = watch(gpu, false);
  CHECK(split.ok());
  CHECK(show(**split, 0, 25, pics.tag) == 0);
  (*split)->split();
  CHECK(show(**split, 26, 33, pics.tag) == 0);
  CHECK((*split)->counts().stills == 7 && (*split)->counts().holds == 0);
  CHECK(show(**split, 34, 37, pics.tag) == 0);
  auto after = (*split)->take();
  CHECK(after.ok() && after->holds.size() == 1);
  CHECK(after->holds[0].first_pts_ns == pts(27));
  // A still more than 3 s older than the newest drops out.
  auto span = watch(gpu, false);
  CHECK(span.ok());
  CHECK(show(**span, 0, 20, pics.tag) == 0);
  CHECK(show(**span, 21, 55, pics.blank) == 0);
  CHECK(show(**span, 56, 64, pics.tag) == 0);
  CHECK((*span)->counts().stills == 6 && (*span)->counts().holds == 0);
  CHECK(show(**span, 65, 66, pics.tag) == 0);
  auto dropped = (*span)->take();
  CHECK(dropped.ok() && dropped->holds.size() == 1);
  CHECK(dropped->holds[0].first_pts_ns == pts(56));
  // Two prints in view: searched, no still.
  auto two = watch(gpu, false);
  CHECK(two.ok());
  CHECK(show(**two, 0, 30, pics.two) == 0);
  CHECK((*two)->counts().candidates == 6 && (*two)->counts().stills == 0);
  return 0;
}

// One still 10 cm off: refused; then a hold registers, and one passes.
vkc::Result<TagNews> refused_then_passed(vkc::test::GpuContext& gpu,
                                         const Pictures& pics) {
  VKC_ASSIGN(std::unique_ptr<TagWatch> w, watch(gpu, false));
  for (std::size_t i = 0; i <= 54; ++i) {
    const bool off = i == 23 || i == 24;  // 24 is a still, steady after 23
    VKC_TRY(w->see(shown(i, above(off ? 0.1 : 0.0, 0.0), pics.tag)));
  }
  return w->take();
}

int test_refused_then_passed(vkc::test::GpuContext& gpu, const Pictures& pics) {
  auto news = refused_then_passed(gpu, pics);
  CHECK(news.ok() && news->holds.size() == 3);
  const TagHold& bad = news->holds[0];
  std::printf("refused: %s: %s\n", bad.miss.c_str(), bad.reason.c_str());
  CHECK(bad.use == TagHold::Use::Refused && bad.miss == "poor_fit");
  CHECK(!bad.reason.empty() && same(bad.tracking_to_world, camera::Mat4d(1.0)));
  CHECK(news->holds[1].use == TagHold::Use::Registered);
  CHECK(news->holds[1].first_pts_ns == pts(32));
  CHECK(news->holds[2].use == TagHold::Use::Passed);
  CHECK(news->registered && same(news->registered->tracking_to_world,
                                 news->holds[1].tracking_to_world));
  // Bit for bit again.
  auto again = refused_then_passed(gpu, pics);
  CHECK(again.ok() && again->holds.size() == 3 && again->registered);
  for (std::size_t k = 0; k < 3; ++k) {
    CHECK(same(news->holds[k], again->holds[k]));
  }
  CHECK(same(news->registered->tracking_to_world,
             again->registered->tracking_to_world));
  return 0;
}

int test_background(vkc::test::GpuContext& gpu) {
  const sensor::testing::HostNv12 host = tag_picture({{280, 200}});
  std::vector<std::weak_ptr<const vkc::Image>> held;
  {
    auto w = watch(gpu, true);
    CHECK(w.ok());
    for (std::size_t i = 0; i <= 30; ++i) {
      auto picture = on_device(gpu, host);
      CHECK(picture.ok());
      held.push_back(picture->image[0]);
      CHECK((*w)->see(shown(i, above(0.0, 0.0), *picture)).ok());
    }
    // Every candidate is searched or replaced, in the end: no timing.
    const auto until =
        std::chrono::steady_clock::now() + std::chrono::seconds(20);
    std::uint64_t holds = 0;
    for (;;) {
      auto news = (*w)->take();
      CHECK(news.ok() && news->failures.empty());
      holds += news->holds.size();
      const TagWatch::Counts& c = (*w)->counts();
      if (c.candidates + c.replaced == 6) break;
      CHECK(std::chrono::steady_clock::now() < until);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const TagWatch::Counts& c = (*w)->counts();
    std::printf("background: %llu searched, %llu replaced\n",
                static_cast<unsigned long long>(c.candidates),
                static_cast<unsigned long long>(c.replaced));
    CHECK(c.seen == 31 && holds == (c.candidates == 6 ? 1u : 0u));
    for (const auto& p : held) CHECK(p.expired());
  }
  // Destroyed with a candidate boxed, or being searched: joined, let go.
  held.clear();
  {
    auto w = watch(gpu, true);
    CHECK(w.ok());
    for (std::size_t i = 0; i <= 20; ++i) {
      auto picture = on_device(gpu, host);
      CHECK(picture.ok());
      held.push_back(picture->image[0]);
      CHECK((*w)->see(shown(i, above(0.0, 0.0), *picture)).ok());
    }
  }
  for (const auto& p : held) CHECK(p.expired());
  return 0;
}

int test_refusals(vkc::test::GpuContext& gpu) {
  CHECK(TagWatch::check(tag_world()).ok());
  CHECK(TagWatch::check({}).domain() == vkc::Status::Code::InvalidArgument);
  camera::WorldFrame sensor_world;
  sensor_world.sensor = "rig-0";
  CHECK(TagWatch::check(sensor_world).domain() ==
        vkc::Status::Code::InvalidArgument);
  camera::WorldFrame no_edge = tag_world();
  no_edge.apriltag->size_m = 0.0;
  CHECK(TagWatch::check(no_edge).domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(TagWatch::create(gpu.device, gpu.allocator,
                         {"", kSession, tag_world(), false})
            .status()
            .domain() == vkc::Status::Code::InvalidArgument);
  CHECK(TagWatch::create(gpu.device, gpu.allocator,
                         {"phone", {}, tag_world(), false})
            .status()
            .domain() == vkc::Status::Code::InvalidArgument);
  return 0;
}

// --- The lane ---------------------------------------------------------------

constexpr std::uint32_t kDepthW = 32;
constexpr std::uint32_t kDepthH = 24;

// A recorded phone above the tag, steady, at 10 Hz from 1 s: depth and the
// tag's picture, its poses in its own world.
class TagPhone final : public sensor::IRgbdSensor {
 public:
  TagPhone(std::size_t count, sensor::YuvImage picture)
      : count_(count), picture_(std::move(picture)) {
    info_.id = "phone";
    info_.clock = sensor::ClockDomain::Host;
    info_.fps = 10;
  }
  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override { return {}; }
  void stop() noexcept override {}
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    if (next_ >= count_) return std::optional<sensor::RgbdFrame>{};
    struct Pixels {
      std::vector<std::uint16_t> depth;
      std::vector<std::uint8_t> confidence;
    };
    auto pixels = std::make_shared<Pixels>();
    pixels->depth.assign(std::size_t{kDepthW} * kDepthH, 900);
    pixels->confidence.assign(std::size_t{kDepthW} * kDepthH, 2);
    sensor::RgbdFrame f = shown(next_++, above(0.0, 0.0), picture_);
    f.depth = pixels->depth.data();
    f.depth_confidence = pixels->confidence.data();
    f.min_confidence = 2;
    f.depth_camera = {{kDepthW, kDepthH}, {25.0, 25.0, 15.5, 11.5}, {}};
    f.min_depth = 0.1f;
    f.max_depth = 5.0f;
    f.depth_noise = {0.002f, 0.0f, 0.002f};
    f.pixels = std::move(pixels);
    return std::optional<sensor::RgbdFrame>(std::move(f));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    VKC_ASSIGN(std::optional<sensor::RgbdFrame> f, poll());
    if (f) out->push_back(std::move(*f));
    return {};
  }
  bool exhausted() const noexcept override { return next_ >= count_; }
  sensor::SensorStats stats() const noexcept override {
    sensor::SensorStats s{};
    s.received = next_;
    s.delivered = next_;
    return s;
  }

 private:
  std::size_t count_;
  std::size_t next_ = 0;
  sensor::YuvImage picture_;
  sensor::SensorInfo info_;
};

std::vector<std::string> lines(const std::filesystem::path& path) {
  std::ifstream in(path);
  std::vector<std::string> out;
  for (std::string line; std::getline(in, line);) out.push_back(line);
  return out;
}

int test_lane(vkc::test::GpuContext& gpu, const Pictures& pics,
              const std::filesystem::path& dir) {
  auto grid =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, 0.05f, 0.15f, 64);
  CHECK(grid.ok());
  auto rig = std::make_unique<vr_test::ScriptedFitter>(
      vr_test::anchors_when([](std::uint64_t) { return true; }, Mat4f(1.0f)));
  auto rig_log = rig->log();
  std::vector<std::unique_ptr<track::IFitter>> fitters;
  fitters.push_back(std::move(rig));
  fitters.push_back(std::make_unique<vr_test::ScriptedFitter>(
      vr_test::anchors_when([](std::uint64_t) { return false; }, Mat4f(1.0f))));
  fitters.emplace_back();  // arkit
  auto tracker =
      track::Tracker::create(vr_example::lane_tracking(), std::move(fitters));
  CHECK(tracker.ok());
  vr_example::RunSource source;
  source.registration = "tag";
  auto log = vr_example::TrackLog::open(dir.string(), **tracker, source);
  CHECK(log.ok());
  auto tags = watch(gpu, false);  // in see(): the replay repeats
  CHECK(tags.ok());
  auto lane = vr_example::MobileLane::create(
      gpu.device, gpu.allocator, std::make_unique<TagPhone>(60, pics.tag),
      std::move(*tracker), std::move(*log), std::move(*tags));
  CHECK(lane.ok());
  std::size_t fillers = 0;
  for (std::size_t i = 0; i < 200; ++i) {
    auto sent = (*lane)->next(*grid, kS + (i + 1) * 40 * kMs);
    CHECK(sent.ok());
    fillers += sent->size();
  }
  CHECK((*lane)->close().ok());
  const std::string report = (*lane)->report();
  std::fputs(report.c_str(), stdout);
  // Frames 0-30 came before the registration (the hold of 2.0-3.0 s, taken
  // in the gap after frame 30's): never tracked.
  const vr_example::MobileLane::Counts& c = (*lane)->counts();
  CHECK(c.unregistered == 31 && c.paired == 29 && fillers > 0);
  for (const vr_test::ScriptedRecord& r : *rig_log) {
    CHECK(r.pts.front() >= pts(31));
  }
  // Candidates every 200 ms from 2.0 s: holds at 2.0-3.0, 3.2-4.2 and
  // 4.4-5.4 s, two stills left over.
  CHECK(report.find("tag: 20 searched, 20 stills, 3 holds (0 refused)") !=
        std::string::npos);
  CHECK(report.find("registered at 3.0 s, 31 frames before it") !=
        std::string::npos);
  // The registration on disk, and the run placed by it.
  const std::filesystem::path written = dir / "registration.json";
  auto reg = camera::read_tracked_registration(written.string());
  CHECK(reg.ok() && reg->sensor == "phone" && reg->session == kSession);
  const vr_example::TrackRun& run = (*lane)->log()->run();
  CHECK(run.source.registration == written.string());
  const Mat4f a = Mat4f(glm::inverse(truth()) * above(0.0, 0.0));
  std::size_t tracked = 0;
  for (const vr_example::FrameRecord& f : run.frames) {
    if (f.chain != 0) continue;
    const auto [mm, deg] =
        vr_example::pose_gap(f.device_pose, Mat4f(reg->tracking_to_world) * a);
    CHECK(mm < 0.01 && deg < 0.001);
    ++tracked;
  }
  CHECK(tracked == 29);
  auto read = vr_example::read_track_log(dir.string());
  CHECK(read.ok() && read->source.registration == written.string());
  const std::vector<std::string> rows = lines(dir / "tags.csv");
  CHECK(rows.size() == 4);
  CHECK(rows[1].find(",registered,") != std::string::npos);
  CHECK(rows[2].find(",passed,") != std::string::npos &&
        rows[3].find(",passed,") != std::string::npos);
  // A second run into the directory is refused before any wait.
  camera::ArrayCalibration array;
  array.sensors.push_back({"rig-0", camera::Mat4d(1.0), {}, {}, {}, 0.0});
  array.world = tag_world();
  const std::string calibration = (dir / "array_calibration.json").string();
  CHECK(camera::write_array_calibration(calibration, array,
                                        vkc::WriteMode::Replace)
            .ok());
  auto again = cfg::parse(
      R"({"version": 1, "operation": "fuse",
          "source": {"kind": "live", "sync_config": "sync.json",
                     "calibration": ")" +
          calibration + R"(", "mobile": "phone.vrst"},
          "diagnostics": {"track_log": ")" +
          dir.string() + R"("}})",
      cfg::Application::FuseOrbbec);
  CHECK(again.ok());
  const vkc::Status refused =
      vr_example::open_mobile_lane(*again, gpu.device, gpu.allocator).status();
  CHECK(refused.domain() == vkc::Status::Code::InvalidArgument);
  CHECK(refused.message().find("registration.json") != std::string::npos);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::test::Scratch scratch("recon_example_live_tag_");
  auto tag = on_device(gpu, tag_picture({{280, 200}}));
  auto blank = on_device(gpu, tag_picture({}));
  auto two = on_device(gpu, tag_picture({{100, 200}, {460, 200}}));
  CHECK(tag.ok() && blank.ok() && two.ok());
  const Pictures pics{*tag, *blank, *two};
  CHECK(test_registers(gpu, pics) == 0);
  CHECK(test_gates(gpu, pics) == 0);
  CHECK(test_refused_then_passed(gpu, pics) == 0);
  CHECK(test_background(gpu) == 0);
  CHECK(test_refusals(gpu) == 0);
  CHECK(test_lane(gpu, pics, scratch.path / "lane") == 0);
  std::puts("recon_example_live_tag: OK");
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
