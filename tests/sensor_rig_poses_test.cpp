// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The rig pose file: what calib writes and a capture reads. The row-major
// matrix lands in GLM's column-major one the right way round, the declarations
// that make a wrong rig look valid (units, camera axes, which sensor) are
// required and checked, a matrix that is not a rigid transform is refused, and
// a written file reads back exactly. Host-only.

#include <clocale>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/sensor/rig_poses.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::recon::sensor;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// A camera 2 m along +X, turned 90 degrees about +Y (its +Z looks along -X),
// written row-major the way numpy's tolist() writes it.
const char* kCameraRow =
    R"({"serial": "CL2A141000G", "sensor": "color", "cam_to_world": [
         [0, 0, -1, 2], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]})";

std::string document(
    const std::string& cameras,
    const std::string& header = R"("format": "volumetric_kit/rig_poses",
       "version": 1, "units": "m", "camera_axes": "opencv")") {
  return "{" + header + ", \"cameras\": [" + cameras + "]}";
}

const char* kIdentity =
    R"({"serial": "CL2A141000N", "sensor": "color", "cam_to_world": [
         [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]})";

bool refused(const std::string& json) {
  const auto r = sensor::parse_rig_poses(json);
  if (r.ok()) return false;
  std::printf("  refused as expected: %s\n", r.status().message().c_str());
  return r.status().domain() == vr::Status::Code::InvalidArgument;
}

bool near(float a, float b) { return std::fabs(a - b) < 1e-6f; }

int test_parse() {
  const auto r = sensor::parse_rig_poses(
      document(std::string(kIdentity) + "," + kCameraRow));
  CHECK(r.ok());
  const std::vector<sensor::RigCameraPose>& poses = r.value();
  CHECK(poses.size() == 2);
  CHECK(poses[0].serial == "CL2A141000N");
  CHECK(poses[0].cam_to_world == vr::Mat4f(1.0f));
  CHECK(poses[1].serial == "CL2A141000G");
  const vr::Mat4f& m = poses[1].cam_to_world;  // m[column][row]
  // The translation is the last *column* of the row-major text.
  CHECK(near(m[3][0], 2.0f) && near(m[3][1], 0.0f) && near(m[3][2], 0.0f));
  // The camera's +Z axis (third column) points along world -X.
  CHECK(near(m[2][0], -1.0f) && near(m[2][1], 0.0f) && near(m[2][2], 0.0f));
  // So a point 1 m in front of the camera is at world (1, 0, 0).
  const vr::Vec4f p = m * vr::Vec4f(0.0f, 0.0f, 1.0f, 1.0f);
  CHECK(near(p.x, 1.0f) && near(p.y, 0.0f) && near(p.z, 0.0f));

  // Keys this reader does not know are ignored, top level and per camera.
  CHECK(sensor::parse_rig_poses(document(
                                    R"({"serial": "A", "sensor": "color",
          "intrinsics": {"fx": 1}, "cam_to_world": [[1, 0, 0, 0],
          [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]})",
                                    R"("format": "volumetric_kit/rig_poses",
          "version": 1, "units": "m", "camera_axes": "opencv",
          "calibrated": "2026-09-27")"))
            .ok());
  return 0;
}

int test_refusals() {
  const std::string good_header =
      R"("format": "volumetric_kit/rig_poses", "version": 1, "units": "m",
         "camera_axes": "opencv")";
  CHECK(refused("{not json"));
  CHECK(refused("[1, 2]"));
  CHECK(refused(document(kIdentity, R"("format": "something/else",
      "version": 1, "units": "m", "camera_axes": "opencv")")));
  CHECK(refused(document(kIdentity, R"("format": "volumetric_kit/rig_poses",
      "version": 2, "units": "m", "camera_axes": "opencv")")));
  // The declarations are required, and version 1 accepts one value of each.
  CHECK(refused(document(kIdentity, R"("format": "volumetric_kit/rig_poses",
      "version": 1, "camera_axes": "opencv")")));
  CHECK(refused(document(kIdentity, R"("format": "volumetric_kit/rig_poses",
      "version": 1, "units": "mm", "camera_axes": "opencv")")));
  CHECK(refused(document(kIdentity, R"("format": "volumetric_kit/rig_poses",
      "version": 1, "units": "m")")));
  CHECK(refused(document(kIdentity, R"("format": "volumetric_kit/rig_poses",
      "version": 1, "units": "m", "camera_axes": "opengl")")));
  CHECK(refused(document(R"({"serial": "A", "cam_to_world": [[1, 0, 0, 0],
      [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]})")));
  CHECK(refused(document(R"({"serial": "A", "sensor": "depth",
      "cam_to_world": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],
      [0, 0, 0, 1]]})")));
  // Cameras: at least one, each named once.
  CHECK(refused(document("")));
  CHECK(refused(document(std::string(kIdentity) + "," + kIdentity)));
  CHECK(refused(document(R"({"serial": "", "sensor": "color",
      "cam_to_world": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],
      [0, 0, 0, 1]]})")));
  // The matrix: 4x4 numbers, a rigid transform.
  CHECK(refused(document(R"({"serial": "A", "sensor": "color",
      "cam_to_world": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]})")));
  CHECK(refused(document(R"({"serial": "A", "sensor": "color",
      "cam_to_world": [[1, 0, 0, "0"], [0, 1, 0, 0], [0, 0, 1, 0],
      [0, 0, 0, 1]]})")));
  CHECK(refused(document(R"({"serial": "A", "sensor": "color",
      "cam_to_world": [[2, 0, 0, 0], [0, 2, 0, 0], [0, 0, 2, 0],
      [0, 0, 0, 1]]})")));  // scaled
  CHECK(refused(document(R"({"serial": "A", "sensor": "color",
      "cam_to_world": [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],
      [0, 0, 0, 1]]})")));  // a reflection
  CHECK(refused(document(R"({"serial": "A", "sensor": "color",
      "cam_to_world": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],
      [0, 0, 1, 1]]})")));  // projective bottom row
  // Metres, not millimetres: a translation in mm is still rigid, which is why
  // the units must be declared rather than guessed from the numbers.
  CHECK(sensor::parse_rig_poses(document(R"({"serial": "A", "sensor": "color",
      "cam_to_world": [[1, 0, 0, 1500], [0, 1, 0, 0], [0, 0, 1, 0],
      [0, 0, 0, 1]]})",
                                         good_header))
            .ok());
  return 0;
}

int test_round_trip() {
  vr::Mat4f pose(1.0f);
  // An arbitrary rotation (about a tilted axis) and translation, so every
  // element is a non-trivial float.
  const float a = 0.7f;
  const vr::Vec3f axis =
      vr::Vec3f(0.3f, -0.8f, 0.52f) / std::sqrt(0.09f + 0.64f + 0.2704f);
  const float c = std::cos(a), s = std::sin(a), t = 1.0f - c;
  pose[0] = vr::Vec4f(t * axis.x * axis.x + c, t * axis.x * axis.y + s * axis.z,
                      t * axis.x * axis.z - s * axis.y, 0.0f);
  pose[1] = vr::Vec4f(t * axis.x * axis.y - s * axis.z, t * axis.y * axis.y + c,
                      t * axis.y * axis.z + s * axis.x, 0.0f);
  pose[2] = vr::Vec4f(t * axis.x * axis.z + s * axis.y,
                      t * axis.y * axis.z - s * axis.x, t * axis.z * axis.z + c,
                      0.0f);
  pose[3] = vr::Vec4f(0.1234567f, -2.5f, 3.75e-3f, 1.0f);
  const std::vector<sensor::RigCameraPose> written = {
      {"CL2A141000N", vr::Mat4f(1.0f)}, {"CL2A141006G", pose}};

  const std::string path = std::string(VR_TEST_SCRATCH_DIR) + "/rig.json";
  CHECK(sensor::write_rig_poses(path, written).ok());
  const auto read = sensor::read_rig_poses(path);
  CHECK(read.ok());
  CHECK(read.value().size() == 2);
  for (std::size_t i = 0; i < 2; ++i) {
    CHECK(read.value()[i].serial == written[i].serial);
    // Exactly: nine significant digits round-trip every float.
    CHECK(read.value()[i].cam_to_world == written[i].cam_to_world);
  }

  // The writer refuses what the reader would.
  CHECK(!sensor::write_rig_poses(path, {}).ok());
  CHECK(!sensor::write_rig_poses(
             path, {{"A", vr::Mat4f(1.0f)}, {"A", vr::Mat4f(1.0f)}})
             .ok());
  CHECK(!sensor::write_rig_poses(path, {{"A", vr::Mat4f(2.0f)}}).ok());
  // A serial JSON cannot carry is refused, not thrown.
  const vr::Status not_utf8 =
      sensor::write_rig_poses(path, {{"CL\xff\xfe", vr::Mat4f(1.0f)}});
  CHECK(!not_utf8.ok() &&
        not_utf8.domain() == vr::Status::Code::InvalidArgument);
  // The same checks, for poses that never touch a file.
  CHECK(sensor::validate_rig_poses(written).ok());
  CHECK(!sensor::validate_rig_poses({{"A", vr::Mat4f(2.0f)}}).ok());

  const auto missing = sensor::read_rig_poses(path + ".missing");
  CHECK(!missing.ok() &&
        missing.status().domain() == vr::Status::Code::IoError);
  // A path that opens but cannot be read is an I/O failure, not bad JSON.
  const auto directory = sensor::read_rig_poses(VR_TEST_SCRATCH_DIR);
  CHECK(!directory.ok() &&
        directory.status().domain() == vr::Status::Code::IoError);
  return 0;
}

int test_decimal_comma_locale() {
  // A host app that adopts the user's locale must not get "0,5" in the file.
  const char* chosen = nullptr;
  for (const char* name :
       {"de_DE.UTF-8", "de_DE.utf8", "fr_FR.UTF-8", "fr_FR.utf8"}) {
    if (std::setlocale(LC_ALL, name) != nullptr &&
        std::strcmp(std::localeconv()->decimal_point, ",") == 0) {
      chosen = name;
      break;
    }
  }
  if (chosen == nullptr) {
    std::setlocale(LC_ALL, "C");
    std::printf("  skipped: no decimal-comma locale installed\n");
    return 0;
  }
  vr::Mat4f pose(1.0f);
  pose[3] = vr::Vec4f(0.5f, -1.25f, 2.0f, 1.0f);
  const std::string path =
      std::string(VR_TEST_SCRATCH_DIR) + "/rig_locale.json";
  const vr::Status written = sensor::write_rig_poses(path, {{"A", pose}});
  const auto read = sensor::read_rig_poses(path);
  std::setlocale(LC_ALL, "C");
  CHECK(written.ok());
  CHECK(read.ok());
  CHECK(read.value()[0].cam_to_world == pose);
  return 0;
}

}  // namespace

int main() {
  if (test_parse() != 0) return 1;
  if (test_refusals() != 0) return 1;
  if (test_round_trip() != 0) return 1;
  if (test_decimal_comma_locale() != 0) return 1;
  std::printf("rig pose tests passed\n");
  return 0;
}
