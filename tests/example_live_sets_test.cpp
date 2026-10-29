// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The live examples' array (examples/common/live_sets.hpp), which fuse_orbbec
// and fuse_realsense report through: the array's counters and each camera's
// line, a camera model's part of it, and the array's clock; and, where the
// RealSense driver is built, fuse_realsense's camera report
// (examples/common/realsense_camera.hpp). Host only: the sensors are fakes,
// the array opened with no device.

#include <cstddef>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "live_sets.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
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
  return 0;
}

#ifdef VR_TEST_REALSENSE_CAMERA
int realsense() {
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

  // What the camera reported, its two cameras and where the colour one sits.
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
      "colour at (-59.0, -0.1, 0.4) mm\n";
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
#ifdef VR_TEST_REALSENSE_CAMERA
  CHECK(realsense() == 0);
#endif
  std::printf("live sets: ok\n");
  return 0;
}
