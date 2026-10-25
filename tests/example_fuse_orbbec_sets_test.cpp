// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec's sets (examples/common/fuse_orbbec_sets.hpp). The cut: a run
// of N frames keeps a set's first frames in the array's order, past its
// missing ones, and lets the rest go, their pictures with them. The run over
// an array of pushed fake cameras: each set waited for on their
// notifications, counted and cut, the run done at exactly N frames, a set
// missing a secondary counted as it came,
// and each secondary's skew to the primary over every frame it sent, the
// primary's none. The single camera a calibration file names: the one asked
// for if posed, or the file's only posed one, past a camera listed for its
// lenses alone. A camera's report, its lens models included. Host only.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fuse_orbbec_sets.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace vkc = volumetric_kit::core;
namespace camera = volumetric_kit::recon::camera;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// A set with a frame from each camera `present` names, numbered by camera,
// each holding a picture `pictures` watches.
sensor::FrameSet set_of(const std::vector<bool>& present,
                        std::vector<std::weak_ptr<const void>>* pictures) {
  sensor::FrameSet set;
  set.frames.resize(present.size());
  for (std::size_t c = 0; c < present.size(); ++c) {
    if (!present[c]) continue;
    sensor::RgbdFrame frame;
    frame.sequence = c;
    auto picture = std::make_shared<int>(0);
    pictures->push_back(picture);
    frame.pixels = std::move(picture);
    set.frames[c] = std::move(frame);
  }
  return set;
}

int cut() {
  std::vector<std::weak_ptr<const void>> pictures;
  // Room for the whole set: every frame stays.
  sensor::FrameSet set = set_of({true, false, true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 5) == 3 && set.count() == 3);
  CHECK(vr_example::cut_set(set, 3) == 3 && set.count() == 3);

  // The first two, in the array's order, past the missing one; the third
  // goes, and its picture with it.
  pictures.clear();
  set = set_of({true, false, true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 2) == 2 && set.frames.size() == 4);
  CHECK(set.frames[0] && set.frames[0]->sequence == 0 && !set.frames[1]);
  CHECK(set.frames[2] && set.frames[2]->sequence == 2 && !set.frames[3]);
  CHECK(!pictures[0].expired() && !pictures[1].expired());
  CHECK(pictures[2].expired());

  // None left: the set keeps a slot per camera, with no frame.
  pictures.clear();
  set = set_of({true, true}, &pictures);
  CHECK(vr_example::cut_set(set, 0) == 0 && set.frames.size() == 2);
  CHECK(set.count() == 0 && pictures[0].expired() && pictures[1].expired());
  return 0;
}

// A camera on the host's clock at 1 kHz whose frames the test pushes; the
// array's next drain hands them over.
class Pushed final : public sensor::IRgbdSensor {
 public:
  Pushed(std::string id, sensor::SyncRole role) {
    info_.id = std::move(id);
    info_.role = role;
    info_.clock = sensor::ClockDomain::Host;
    info_.fps = 1000;
  }

  void push(std::uint64_t timestamp_ns) {
    sensor::RgbdFrame frame;
    frame.timestamp_ns = timestamp_ns;
    frame.sequence = timestamp_ns;
    frames_.push_back(std::move(frame));
    if (notify_) notify_();
  }

  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override { return {}; }
  void stop() noexcept override { frames_.clear(); }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    if (frames_.empty()) return no_frame();
    sensor::RgbdFrame frame = std::move(frames_.back());
    frames_.clear();
    return some_frame(std::move(frame));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    for (sensor::RgbdFrame& frame : frames_) out->push_back(std::move(frame));
    frames_.clear();
    return {};
  }
  sensor::SensorStats stats() const noexcept override { return {}; }
  vkc::Status set_frame_notify(std::function<void()> notify) override {
    notify_ = std::move(notify);
    return {};
  }

 private:
  sensor::SensorInfo info_;
  std::vector<sensor::RgbdFrame> frames_;
  std::function<void()> notify_;
};

// A run of 4 frames from a primary "P" and its secondary "S", in that order:
// a complete set (2 frames), one missing S (1), and a complete one cut to the
// last frame the run takes, P's.
int run() {
  auto primary = std::make_unique<Pushed>("P", sensor::SyncRole::Primary);
  auto secondary = std::make_unique<Pushed>("S", sensor::SyncRole::Secondary);
  Pushed* p = primary.get();
  Pushed* s = secondary.get();
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensors.push_back(std::move(primary));
  sensors.push_back(std::move(secondary));
  sensor::SensorArray::Options options;
  options.tolerance_us = 200;  // under half of 1 kHz's period
  auto opened = sensor::SensorArray::open(std::move(sensors), options);
  CHECK(opened.ok());
  sensor::SensorArray array = std::move(opened).value();
  CHECK(array.start().ok() && array.primary() == 0);

  vr_example::RigSets sets(array, 4);
  vkc::StageMetrics stages;
  CHECK(!sets.done() && sets.sets() == 0 && sets.frames() == 0);
  const std::uint64_t base = 5'000'000'000;
  // Trigger `t`: P's frame, and S's `lag_ns` after it unless S is silent.
  const auto trigger = [&](std::uint64_t t,
                           std::optional<std::int64_t> lag_ns) {
    const std::uint64_t ts = base + t * 1'000'000;  // 1 kHz
    p->push(ts);
    if (lag_ns) s->push(ts + static_cast<std::uint64_t>(*lag_ns));
  };

  trigger(0, 80'000);
  vkc::Result<sensor::FrameSet> set = sets.next(&stages);
  CHECK(set.ok() && set.value().complete());
  CHECK(!sets.done() && sets.sets() == 1 && sets.frames() == 2);

  // S silent: the set comes once the array stops waiting for it, and counts
  // the one frame it holds.
  trigger(1, std::nullopt);
  set = sets.next(&stages);
  CHECK(set.ok() && set.value().count() == 1 && set.value().frames[0]);
  CHECK(!sets.done() && sets.sets() == 2 && sets.frames() == 3);

  // One frame left to take: the set is cut to P's, the first in the array's
  // order, and the run is done at exactly 4.
  trigger(2, -120'000);
  set = sets.next(&stages);
  CHECK(set.ok() && set.value().count() == 1 && set.value().frames[0]);
  CHECK(sets.done() && sets.sets() == 3 && sets.frames() == 4);

  // S's skew over the two frames it sent, the one the cut let go included;
  // the primary has none.
  const vr_example::RigSets::Skew& lag = sets.skew(1);
  CHECK(lag.frames == 2);
  CHECK(std::fabs(lag.sum_ms - (0.08 - 0.12)) < 1e-9);
  CHECK(std::fabs(lag.worst_ms - 0.12) < 1e-9);
  CHECK(sets.skew(0).frames == 0 && sets.skew(0).sum_ms == 0.0);

  // The polls, and the wait for the silent secondary, blocked on the
  // cameras' notifications until the array stopped waiting for it.
  CHECK(stages.rows().size() == 2);
  CHECK(std::string(stages.rows()[0].name) == "poll");
  CHECK(std::string(stages.rows()[1].name) == "wait: cameras");
  CHECK(stages.rows()[1].wait_ms > 0.0);
  return 0;
}

int posed() {
  using Code = vkc::Status::Code;
  const std::string path = "rig.json";
  const auto code = [&](const camera::ArrayCalibration& c,
                        const std::string& serial) {
    return vr_example::posed_camera(c, serial, path).status().domain();
  };
  // No file: the camera asked for, or the first found.
  CHECK(vr_example::posed_camera({}, "", path).value().empty());
  CHECK(vr_example::posed_camera({}, "A", path).value() == "A");

  // A posed, B listed for its lenses alone (a factory record).
  camera::ArrayCalibration c;
  camera::SensorCalibration a;
  a.id = "A";
  a.color_to_world = camera::Mat4d(1.0);
  camera::SensorCalibration b;
  b.id = "B";
  c.sensors = {a, b};
  // The only posed one.
  CHECK(vr_example::posed_camera(c, "", path).value() == "A");
  CHECK(vr_example::posed_camera(c, "A", path).value() == "A");
  CHECK(code(c, "B") == Code::NotFound);  // listed, unposed
  CHECK(code(c, "Z") == Code::NotFound);  // not listed
  const vkc::Status s = vr_example::posed_camera(c, "B", path).status();
  CHECK(s.message() == "rig.json does not pose camera B");

  // None posed, or two: the run must name one.
  c.sensors = {b};
  CHECK(code(c, "") == Code::NotFound);
  b.color_to_world = camera::Mat4d(1.0);
  c.sensors = {a, b};
  CHECK(code(c, "") == Code::NotFound);
  CHECK(vr_example::posed_camera(c, "B", path).value() == "B");
  return 0;
}

// A camera's report: each camera's size, lens model and intrinsics, and the
// colour camera where the calibration poses it.
int report() {
  sensor::OrbbecDeviceInfo info;
  info.name = "Orbbec Femto Mega";
  info.serial = "CL2A141000N";
  info.firmware_version = "1.2.9";
  info.connection_type = "Ethernet";
  info.ip_address = "192.168.1.10";
  info.sync_mode = sensor::OrbbecSyncMode::Primary;
  info.color_lens_model = "brown-conrady-k6";
  info.depth_lens_model = "brown-conrady";
  sensor::SensorInfo cameras;
  cameras.clock = sensor::ClockDomain::Host;
  camera::CameraModel color;
  color.size = {3840, 2160};
  color.intrinsics = {2240.5, 2241.37, 1920.0, 1080.6};
  cameras.color = color;
  camera::CameraModel depth;
  depth.size = {640, 576};
  depth.intrinsics = {504.0, 504.5, 320.3, 288.0};
  cameras.depth = depth;
  camera::ArrayCalibration calibration;
  camera::SensorCalibration calibrated;
  calibrated.id = info.serial;
  camera::Mat4d pose(1.0);
  pose[3] = {1.0, -0.5, 2.25, 1.0};
  calibrated.color_to_world = pose;
  calibration.sensors.push_back(std::move(calibrated));

  const std::string text =
      vr_example::orbbec_camera_report(info, cameras, calibration);
  const auto has = [&](const char* line) {
    return text.find(line) != std::string::npos;
  };
  CHECK(
      has("  colour 3840x2160 brown-conrady-k6 @ fx=2240.5 fy=2241.4 "
          "cx=1920.0 cy=1080.6, at (1.000, -0.500, 2.250) m\n"));
  CHECK(
      has("  depth 640x576 brown-conrady @ fx=504.0 fy=504.5 cx=320.3 "
          "cy=288.0\n"));
  CHECK(text.rfind("camera: Orbbec Femto Mega CL2A141000N, firmware 1.2.9, "
                   "Ethernet 192.168.1.10, sync ",
                   0) == 0);

  // Not in the calibration: at the origin.
  const std::string origin =
      vr_example::orbbec_camera_report(info, cameras, {});
  CHECK(origin.find(", at (0.000, 0.000, 0.000) m\n") != std::string::npos);

  // Listed for its lenses alone: no pose, so the report uses the origin too.
  calibration.sensors[0].color_to_world.reset();
  CHECK(vr_example::orbbec_camera_report(info, cameras, calibration) == origin);
  return 0;
}

}  // namespace

int main() {
  CHECK(cut() == 0);
  CHECK(run() == 0);
  CHECK(posed() == 0);
  CHECK(report() == 0);
  std::puts("example_fuse_orbbec_sets: OK");
  return 0;
}
