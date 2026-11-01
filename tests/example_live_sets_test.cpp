// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The live examples' array (examples/common/live_sets.hpp), which fuse_orbbec,
// fuse_realsense and fuse_decklink report through: the array's counters and
// each camera's line, a camera model's part of it and its lens's name, and the
// array's clock; fuse_decklink's camera report and depth noise
// (examples/common/decklink_camera.hpp); and, where the RealSense driver is
// built, fuse_realsense's camera report (examples/common/realsense_camera.hpp).
// Host only: the sensors are fakes, the array opened with no device.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "decklink_camera.hpp"
#include "live_sets.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/frame/depth_noise.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/stereo/stereo_sensor.hpp"
#ifdef VR_TEST_REALSENSE_CAMERA
#include "realsense_camera.hpp"
#endif

namespace vkc = volumetric_kit::core;
namespace camera = volumetric_kit::sensor::camera;
namespace sensor = volumetric_kit::sensor;

namespace {

// A sensor that streams nothing and reports fixed counters, on @p clock.
class FakeSensor final : public sensor::IRgbdSensor {
 public:
  FakeSensor(std::string id, sensor::ClockDomain clock, sensor::SensorStats s)
      : stats_(s) {
    info_.id = std::move(id);
    info_.clock = clock;
  }
  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override { return {}; }
  void stop() noexcept override {}
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    return no_frame();
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>*) override { return {}; }
  sensor::SensorStats stats() const noexcept override { return stats_; }

 private:
  sensor::SensorInfo info_;
  sensor::SensorStats stats_;
};

// An array of one fake sensor, opened with no device.
vkc::Result<sensor::SensorArray> array_of(std::string id,
                                          sensor::ClockDomain clock,
                                          sensor::SensorStats stats) {
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensors.push_back(std::make_unique<FakeSensor>(std::move(id), clock, stats));
  return sensor::SensorArray::open(std::move(sensors), {});
}

std::string marked(const sensor::IRgbdSensor& camera) {
  return "counters of " + camera.info().id;
}

int report() {
  auto array =
      array_of("215122251234", sensor::ClockDomain::Host, {7, 5, 1, 2});
  CHECK(array);
  // The array's sets, then the camera's line: its id and the interface's
  // counters, in their order.
  const std::string expected =
      "0 sets, 0 missing a camera, 0 frames unmatched\n"
      "  215122251234: 7 received, 5 delivered, 1 dropped, 2 failed\n";
  const std::string got =
      vr_example::array_report(*array, nullptr, vr_example::sensor_counters);
  if (got != expected) std::fprintf(stderr, "report:\n%s", got.c_str());
  CHECK(got == expected);
  // A driver's own counters take the line's place after the id.
  CHECK(vr_example::array_report(*array, nullptr, marked) ==
        "0 sets, 0 missing a camera, 0 frames unmatched\n"
        "  215122251234: counters of 215122251234\n");
  return 0;
}

int host_clock() {
  // A set's time is the system clock's only where its camera stamps on the
  // host's.
  auto host = array_of("A", sensor::ClockDomain::Host, sensor::SensorStats{});
  auto own = array_of("B", sensor::ClockDomain::Device, sensor::SensorStats{});
  CHECK(host && own);
  CHECK(vr_example::on_host_clock(*host));
  CHECK(!vr_example::on_host_clock(*own));
  return 0;
}

int summary() {
  camera::CameraModel model;
  model.size = {1280, 720};
  model.intrinsics = {640.24, 641.0, 639.5, 359.76};
  CHECK(vr_example::camera_summary(model, "") ==
        " 1280x720 @ fx=640.2 fy=641.0 cx=639.5 cy=359.8");
  CHECK(vr_example::camera_summary(model, "brown-conrady") ==
        " 1280x720 brown-conrady @ fx=640.2 fy=641.0 cx=639.5 cy=359.8");
  // Each lens named by the coefficients it has.
  camera::RationalDistortion lens;
  CHECK(std::string(vr_example::lens_name(lens)) == "pinhole");
  lens.p2 = 1e-4;
  CHECK(std::string(vr_example::lens_name(lens)) == "brown-conrady");
  lens.k6 = 1e-3;
  CHECK(std::string(vr_example::lens_name(lens)) == "rational");
  lens = {};
  lens.k5 = -2e-3;
  CHECK(std::string(vr_example::lens_name(lens)) == "rational");
  return 0;
}

int decklink() {
  // Two 4K cameras 0.5 m apart, the right one mostly along the left one's +x
  // (a translation off the axis, whose length is the baseline), rectified to
  // 960x544 at f 384: FFS's near limit is 384 * 0.5 / 192 = 1 m. The depth
  // FFS keeps is the scenario's fusion gate.
  const auto scenario = volumetric_kit::core::config::parse(
      R"({
    "version": 1, "operation": "fuse",
    "source": {"kind": "live", "driver": "decklink",
               "cameras": [{"id": "bm-2", "input": "SDI 1"},
                           {"id": "bm-1", "input": "SDI 2"}],
               "pairs": [{"left": "bm-1", "right": "bm-2"}],
               "calibration": "rig.json", "ffs_model": "ffs.onnx",
               "ffs_plan_cache": "plans"},
    "fusion": {"min_depth_m": 0.8, "max_depth_m": 4.5}
  })",
      volumetric_kit::core::config::Application::FuseDecklink);
  CHECK(scenario);
  sensor::StereoCalibration pair;
  pair.left.size = pair.right.size = {3840, 2160};
  pair.left.intrinsics = {1702.41, 1702.12, 1917.38, 1108.27};
  pair.left.distortion.k1 = 0.76;
  pair.left.distortion.k4 = 1.13;
  pair.right = pair.left;
  pair.left_to_right[3] = {-0.48, 0.0, 0.14, 1.0};
  camera::CameraModel rectified;
  rectified.size = {960, 544};
  rectified.intrinsics = {384.0, 384.0, 479.5, 271.5};
  sensor::SensorInfo depth;
  depth.clock = sensor::ClockDomain::Host;
  depth.color = pair.left;
  depth.depth = rectified;

  // The depth noise every frame carries: c = 0.07 / (384 * 0.5), quadratic
  // alone, so Dynamic fusion has a model.
  CHECK(vr_example::stereo_baseline(pair) > 0.5 - 1e-12 &&
        vr_example::stereo_baseline(pair) < 0.5 + 1e-12);
  const sensor::DepthNoise noise =
      vr_example::decklink_depth_noise(pair, rectified);
  CHECK(noise.known() && noise.a == 0.0f && noise.b == 0.0f);
  CHECK(std::fabs(noise.c - 0.07f / (384.0f * 0.5f)) < 1e-9f);

  const std::string report =
      vr_example::decklink_camera_report(scenario.value(), pair, depth);
  const std::string expected =
      "camera: DeckLink bm-1 on SDI 2 (left) and bm-2 on SDI 1 (right), "
      "3840x2160 at 30 fps, host clock\n"
      "  colour 3840x2160 rational @ fx=1702.4 fy=1702.1 cx=1917.4 "
      "cy=1108.3\n"
      "  depth 960x544 pinhole @ fx=384.0 fy=384.0 cx=479.5 cy=271.5, FFS in "
      "the rectified left camera\n"
      "  baseline 500.0 mm, left and right at most 16.666 ms apart\n"
      "  depth kept 0.80-4.50 m; FFS's near limit 1.00 m, gated\n"
      "  depth noise 1.5 mm at 2 m, 5.8 mm at 4 m (disparity 0.07 px)\n";
  if (report != expected) {
    std::fprintf(stderr, "decklink report:\n%s", report.c_str());
  }
  CHECK(report == expected);
  return 0;
}

#ifdef VR_TEST_REALSENSE_CAMERA
int realsense() {
  // What the camera reported, its two cameras and where the depth one sits
  // in the colour one's frame: depth_to_color's translation, as it is.
  sensor::RealSenseDeviceInfo info;
  info.name = "RealSense D455";
  info.serial = "215122251234";
  info.firmware_version = "5.17.0.10";
  info.usb_type = "3.2";
  sensor::SensorInfo cameras;
  cameras.clock = sensor::ClockDomain::Host;
  camera::CameraModel color;
  color.size = {1280, 800};
  color.intrinsics = {640.51, 640.0, 645.24, 401.51};
  color.distortion.k1 = -0.05;
  camera::CameraModel depth;
  depth.size = {1280, 720};
  depth.intrinsics = {657.27, 657.27, 651.38, 359.03};
  cameras.color = color;
  cameras.depth = depth;
  cameras.depth_to_color[3] = {-0.05904, -0.00007, 0.00036, 1.0};
  const std::string auto_exposure =
      vr_example::realsense_camera_report(info, cameras);
  const std::string expected =
      "camera: RealSense D455 215122251234, firmware 5.17.0.10, USB 3.2, "
      "host clock\n"
      "  colour 1280x800 brown-conrady @ fx=640.5 fy=640.0 cx=645.2 cy=401.5, "
      "auto exposure\n"
      "  depth 1280x720 pinhole @ fx=657.3 fy=657.3 cx=651.4 cy=359.0, "
      "at (-59.0, -0.1, 0.4) mm in the colour camera's frame\n";
  if (auto_exposure != expected) {
    std::fprintf(stderr, "realsense report:\n%s", auto_exposure.c_str());
  }
  CHECK(auto_exposure == expected);
  cameras.color_controls = camera::ColorControls{167, 60};
  CHECK(vr_example::realsense_camera_report(info, cameras)
            .find(", exposure 167 gain 60\n") != std::string::npos);
  return 0;
}
#endif

}  // namespace

int main() {
  CHECK(report() == 0);
  CHECK(host_clock() == 0);
  CHECK(summary() == 0);
  CHECK(decklink() == 0);
#ifdef VR_TEST_REALSENSE_CAMERA
  CHECK(realsense() == 0);
#endif
  std::printf("live sets: ok\n");
  return 0;
}
