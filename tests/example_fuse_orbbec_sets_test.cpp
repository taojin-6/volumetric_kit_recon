// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec's cameras (examples/common/fuse_orbbec_sets.hpp). The single
// camera a calibration file names: the one asked for if posed, or the file's
// only posed one, past a camera listed for its lenses alone. A camera's
// report, its lens models included. Host only; the run's sets are the
// library's (sensor_array_rig_sets_test.cpp).

#include <cstdio>
#include <string>
#include <utility>

#include "fuse_orbbec_sets.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/camera/camera_model.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_stream.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace vkc = volumetric_kit::core;
namespace camera = volumetric_kit::recon::camera;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

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
  CHECK(posed() == 0);
  CHECK(report() == 0);
  std::puts("example_fuse_orbbec_sets: OK");
  return 0;
}
