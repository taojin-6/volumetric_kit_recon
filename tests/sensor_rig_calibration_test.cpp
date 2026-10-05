// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The rig calibration file: the family's config layout read as calib writes it
// -- a world->camera pose turned into this repo's camera->world, Rodrigues at
// its edge cases, the refusals -- and a written file read back. Host-only.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/sensor/rig_calibration.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::recon::sensor;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// The family's config layout (config_nunga.json), trimmed: an unrelated
// section, a non-object entry, and a camera turned 90 degrees about +Y (rvec
// (0, pi/2, 0)) with tvec (0, 0, 2).
const char* kConfig = R"({
  "kinect_config": {"device_count": 2, "master_serial": "A"},
  "device_calibration": {
    "pose_calib_file_path": "/somewhere/all_extrinsics.npy",
    "A": {
      "intrinsics": {"fx": 1822.1, "fy": 1820.1, "cx": 1930.0, "cy": 1107.7},
      "distortion": {"k1": -0.175, "k2": -2.855, "p1": 0.0023, "p2": 0.0006,
                     "k3": 2.184, "k4": -0.282, "k5": -2.634, "k6": 2.063},
      "optimal_intrinsics": {"fx": 1892.1, "fy": 1861.0, "cx": 1933.5,
                             "cy": 1111.1},
      "pose": {"rvec": [0.0, 0.0, 0.0], "tvec": [0.0, 0.0, 0.0]}
    },
    "B": {"pose": {"rvec": [0.0, 1.5707963267948966, 0.0],
                   "tvec": [0.0, 0.0, 2.0]}}
  },
  "SenderToUnity": {"host": "0.0.0.0", "port": 33669}
})";

bool near(float a, float b, float eps = 1e-5f) {
  return std::fabs(a - b) < eps;
}

bool near(const vr::Mat4f& a, const vr::Mat4f& b, float eps) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!near(a[c][r], b[c][r], eps)) return false;
    }
  }
  return true;
}

bool refused(const std::string& json) {
  const auto r = sensor::parse_rig_calibration(json);
  if (r.ok()) return false;
  std::printf("  refused as expected: %s\n", r.status().message().c_str());
  return r.status().domain() == vkc::Status::Code::InvalidArgument;
}

// A rotation of `angle` about unit `axis`, with a translation, as
// camera->world.
vr::Mat4f pose(vr::Vec3f axis, float angle, vr::Vec3f t) {
  axis = axis / std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  const float c = std::cos(angle), s = std::sin(angle), k = 1.0f - c;
  vr::Mat4f m(1.0f);
  m[0] = vr::Vec4f(k * axis.x * axis.x + c, k * axis.x * axis.y + s * axis.z,
                   k * axis.x * axis.z - s * axis.y, 0.0f);
  m[1] = vr::Vec4f(k * axis.x * axis.y - s * axis.z, k * axis.y * axis.y + c,
                   k * axis.y * axis.z + s * axis.x, 0.0f);
  m[2] = vr::Vec4f(k * axis.x * axis.z + s * axis.y,
                   k * axis.y * axis.z - s * axis.x, k * axis.z * axis.z + c,
                   0.0f);
  m[3] = vr::Vec4f(t, 1.0f);
  return m;
}

int test_parse() {
  const auto r = sensor::parse_rig_calibration(kConfig);
  CHECK(r.ok());
  CHECK(r.value().size() == 2);
  const sensor::RigCameraCalibration& a = r.value()[0];
  const sensor::RigCameraCalibration& b = r.value()[1];
  CHECK(a.serial == "A" && b.serial == "B");
  CHECK(a.cam_to_world == vr::Mat4f(1.0f));
  CHECK(a.intrinsics && near(a.intrinsics->fx, 1822.1f) &&
        near(a.intrinsics->cy, 1107.7f));
  CHECK(a.distortion && near(a.distortion->k2, -2.855f) &&
        near(a.distortion->k6, 2.063f));
  CHECK(a.optimal_intrinsics && near(a.optimal_intrinsics->cx, 1933.5f));
  CHECK(!b.intrinsics && !b.distortion && !b.optimal_intrinsics);

  // B's pose is world->camera: x_cam = R x_world + t. So the camera centre is
  // -R^T t, and a point 1 m ahead of it (0, 0, 1 in its frame) is R^T((0, 0,
  // 1) - t) in the world. R turns +X into -Z, so R^T (0, 0, -1) = (1, 0, 0).
  const vr::Mat4f& m = b.cam_to_world;
  CHECK(near(m[3][0], 2.0f) && near(m[3][1], 0.0f) && near(m[3][2], 0.0f));
  const vr::Vec4f p = m * vr::Vec4f(0.0f, 0.0f, 1.0f, 1.0f);
  CHECK(near(p.x, 1.0f) && near(p.y, 0.0f) && near(p.z, 0.0f));
  return 0;
}

int test_refusals() {
  CHECK(refused("{not json"));
  CHECK(refused(R"({"kinect_config": {}})"));
  CHECK(refused(R"({"device_calibration": {"pose_calib_file_path": "x"}})"));
  CHECK(refused(R"({"device_calibration": {"A": {"intrinsics":
      {"fx": 1, "fy": 1, "cx": 0, "cy": 0}}}})"));  // no pose
  CHECK(refused(R"({"device_calibration": {"A": {"pose":
      {"rvec": [0, 0], "tvec": [0, 0, 0]}}}})"));
  CHECK(refused(R"({"device_calibration": {"A": {"pose":
      {"rvec": [0, 0, 0], "tvec": [0, "0", 0]}}}})"));
  CHECK(refused(R"({"device_calibration": {"A": {"pose":
      {"rvec": [0, 0, 0], "tvec": [0, 0, 0]},
      "intrinsics": {"fx": 0, "fy": 1, "cx": 0, "cy": 0}}}})"));
  CHECK(refused(R"({"device_calibration": {"A": {"pose":
      {"rvec": [0, 0, 0], "tvec": [0, 0, 0]},
      "distortion": {"k1": 0, "k2": 0, "p1": 0, "p2": 0, "k3": 0}}}})"));
  return 0;
}

int test_round_trip() {
  // Rodrigues' edge cases: no rotation, a tiny one, a general one, and turns
  // of pi and just under it, where sin(theta) vanishes.
  const std::vector<sensor::RigCameraCalibration> written = {
      {"id", vr::Mat4f(1.0f), {}, {}, {}},
      {"tiny", pose({0.3f, 0.5f, 0.8f}, 1e-7f, {0.1f, 0.2f, 0.3f}), {}, {}, {}},
      {"general", pose({0.3f, -0.8f, 0.52f}, 0.7f, {1.25f, -2.5f, 3.75e-3f}),
       sensor::PinholeIntrinsics{746.494f, 746.319f, 630.403f, 345.615f},
       sensor::LensDistortion{0.07536f, -0.10528f, -0.000272f, 0.000335f,
                              0.04364f, 0, 0, 0},
       sensor::PinholeIntrinsics{740.1f, 741.2f, 631.0f, 346.0f}},
      {"pi_x", pose({1, 0, 0}, 3.14159265f, {0, 0, 1}), {}, {}, {}},
      {"pi_xy", pose({1, 1, 0}, 3.14159265f, {0.5f, 0, 0}), {}, {}, {}},
      {"near_pi", pose({0.2f, 0.9f, -0.4f}, 3.1415f, {0, 1, 0}), {}, {}, {}}};
  const std::string path =
      std::string(VR_TEST_SCRATCH_DIR) + "/rig_calibration.json";
  CHECK(sensor::write_rig_calibration(path, written).ok());
  const auto read = sensor::read_rig_calibration(path);
  CHECK(read.ok());
  CHECK(read.value().size() == written.size());
  for (const auto& w : written) {
    const auto it =
        std::find_if(read.value().begin(), read.value().end(),
                     [&](const auto& c) { return c.serial == w.serial; });
    CHECK(it != read.value().end());
    CHECK(near(it->cam_to_world, w.cam_to_world, 5e-7f));  // float round-off
    CHECK(it->intrinsics.has_value() == w.intrinsics.has_value());
    if (w.intrinsics) CHECK(it->intrinsics->fx == w.intrinsics->fx);
    if (w.distortion) CHECK(it->distortion->p1 == w.distortion->p1);
  }

  // The writer refuses what the reader would.
  CHECK(!sensor::write_rig_calibration(path, {}).ok());
  CHECK(
      !sensor::write_rig_calibration(path, {{"A", vr::Mat4f(1.0f), {}, {}, {}},
                                            {"A", vr::Mat4f(1.0f), {}, {}, {}}})
           .ok());
  CHECK(
      !sensor::write_rig_calibration(path, {{"A", vr::Mat4f(2.0f), {}, {}, {}}})
           .ok());
  const auto missing = sensor::read_rig_calibration(path + ".missing");
  CHECK(!missing.ok() &&
        missing.status().domain() == vkc::Status::Code::IoError);
  return 0;
}

}  // namespace

int main() {
  if (test_parse() != 0) return 1;
  if (test_refusals() != 0) return 1;
  if (test_round_trip() != 0) return 1;
  std::printf("rig calibration tests passed\n");
  return 0;
}
