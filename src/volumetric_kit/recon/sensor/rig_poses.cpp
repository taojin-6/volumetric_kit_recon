// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/rig_poses.hpp"

#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace volumetric_kit::recon::sensor {

namespace {

using nlohmann::json;

constexpr const char* kFormat = "volumetric_kit/rig_poses";
constexpr int kVersion = 1;
// How far a rotation's columns may be from orthonormal. A calibration's
// rotation is orthonormal to ~1e-7 in float; this catches a matrix that is
// not a rotation (scaled, sheared, transposed into a projective mess) without
// refusing honest round-off.
constexpr float kOrthonormalTolerance = 1e-4f;
constexpr float kBottomRowTolerance = 1e-6f;

Status bad(const std::string& what) {
  return Status::invalid_argument("rig poses: " + what);
}

std::string camera_label(std::size_t index, const std::string& serial) {
  return "camera " + std::to_string(index) +
         (serial.empty() ? std::string() : " (" + serial + ")");
}

// The checks a pose must pass to be written or read.
Status check_pose(const RigCameraPose& pose, std::size_t index) {
  const std::string who = camera_label(index, pose.serial);
  if (pose.serial.empty()) return bad(who + ": empty serial");
  const Mat4f& m = pose.cam_to_world;  // m[column][row]
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(m[c][r])) {
        return bad(who + ": cam_to_world is not finite");
      }
    }
  }
  if (std::fabs(m[0][3]) > kBottomRowTolerance ||
      std::fabs(m[1][3]) > kBottomRowTolerance ||
      std::fabs(m[2][3]) > kBottomRowTolerance ||
      std::fabs(m[3][3] - 1.0f) > kBottomRowTolerance) {
    return bad(who + ": cam_to_world's bottom row is not [0, 0, 0, 1]");
  }
  // Columns 0..2 of the upper 3x3 must be orthonormal: R^T R = I.
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      float dot = 0.0f;
      for (int r = 0; r < 3; ++r) dot += m[a][r] * m[b][r];
      if (std::fabs(dot - (a == b ? 1.0f : 0.0f)) > kOrthonormalTolerance) {
        return bad(who + ": cam_to_world's rotation is not orthonormal");
      }
    }
  }
  const float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                    m[1][0] * (m[0][1] * m[2][2] - m[2][1] * m[0][2]) +
                    m[2][0] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]);
  if (!(det > 0.0f)) {
    return bad(who + ": cam_to_world's rotation is a reflection");
  }
  return {};
}

Status check_poses(const std::vector<RigCameraPose>& poses) {
  if (poses.empty()) return bad("no cameras");
  for (std::size_t i = 0; i < poses.size(); ++i) {
    VR_TRY(check_pose(poses[i], i));
    for (std::size_t j = 0; j < i; ++j) {
      if (poses[j].serial == poses[i].serial) {
        return bad("serial " + poses[i].serial + " appears twice");
      }
    }
  }
  return {};
}

// A required string member, compared against the one value version 1 accepts.
Status expect_string(const json& object, const char* key,
                     const std::string& accepted, const std::string& where) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_string()) {
    return bad(where + "missing \"" + key + "\" (version " +
               std::to_string(kVersion) + " accepts \"" + accepted + "\")");
  }
  const std::string value = it->get<std::string>();
  if (value != accepted) {
    return bad(where + "\"" + key + "\" is \"" + value + "\"; version " +
               std::to_string(kVersion) + " accepts only \"" + accepted + "\"");
  }
  return {};
}

// Nine significant digits round-trip every float.
std::string number(float v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
  return buf;
}

}  // namespace

Result<std::vector<RigCameraPose>> parse_rig_poses(const std::string& text) {
  json doc;
  try {
    doc = json::parse(text);
  } catch (const std::exception& e) {  // json::parse_error names the byte
    return bad(std::string("not JSON: ") + e.what());
  }
  if (!doc.is_object()) return bad("the document is not a JSON object");
  VR_TRY(expect_string(doc, "format", kFormat, ""));
  const auto version = doc.find("version");
  if (version == doc.end() || !version->is_number_integer()) {
    return bad("missing integer \"version\"");
  }
  if (version->get<long long>() != kVersion) {
    return bad("version " + std::to_string(version->get<long long>()) +
               "; this reader knows version " + std::to_string(kVersion));
  }
  VR_TRY(expect_string(doc, "units", "m", ""));
  VR_TRY(expect_string(doc, "camera_axes", "opencv", ""));

  const auto cameras = doc.find("cameras");
  if (cameras == doc.end() || !cameras->is_array()) {
    return bad("missing \"cameras\" array");
  }
  std::vector<RigCameraPose> poses;
  poses.reserve(cameras->size());
  for (std::size_t i = 0; i < cameras->size(); ++i) {
    const json& camera = (*cameras)[i];
    const std::string where = "camera " + std::to_string(i) + ": ";
    if (!camera.is_object()) return bad(where + "not a JSON object");
    RigCameraPose pose;
    const auto serial = camera.find("serial");
    if (serial == camera.end() || !serial->is_string()) {
      return bad(where + "missing string \"serial\"");
    }
    pose.serial = serial->get<std::string>();
    const std::string who = camera_label(i, pose.serial) + ": ";
    VR_TRY(expect_string(camera, "sensor", "color", who));
    const auto matrix = camera.find("cam_to_world");
    if (matrix == camera.end() || !matrix->is_array() || matrix->size() != 4) {
      return bad(who + "\"cam_to_world\" is not 4 rows");
    }
    for (int r = 0; r < 4; ++r) {
      const json& row = (*matrix)[static_cast<std::size_t>(r)];
      if (!row.is_array() || row.size() != 4) {
        return bad(who + "\"cam_to_world\" row " + std::to_string(r) +
                   " is not 4 numbers");
      }
      for (int c = 0; c < 4; ++c) {
        const json& v = row[static_cast<std::size_t>(c)];
        if (!v.is_number()) {
          return bad(who + "\"cam_to_world\" row " + std::to_string(r) +
                     " holds something other than a number");
        }
        pose.cam_to_world[c][r] = v.get<float>();  // row-major in, GLM out
      }
    }
    poses.push_back(std::move(pose));
  }
  VR_TRY(check_poses(poses));
  return poses;
}

Result<std::vector<RigCameraPose>> read_rig_poses(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return Status::io_error("rig poses: cannot open " + path);
  const std::string text((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
  if (in.bad()) return Status::io_error("rig poses: cannot read " + path);
  auto poses = parse_rig_poses(text);
  if (!poses.ok()) {
    return Status::invalid_argument(path + ": " + poses.status().message());
  }
  return poses;
}

Status write_rig_poses(const std::string& path,
                       const std::vector<RigCameraPose>& poses) {
  VR_TRY(check_poses(poses));
  // Written by hand rather than dumped, so each matrix row stays on one line.
  std::ostringstream out;
  out << "{\n  \"format\": \"" << kFormat << "\",\n  \"version\": " << kVersion
      << ",\n  \"units\": \"m\",\n  \"camera_axes\": \"opencv\",\n"
      << "  \"cameras\": [\n";
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const Mat4f& m = poses[i].cam_to_world;
    out << "    {\n      \"serial\": " << json(poses[i].serial).dump()
        << ",\n      \"sensor\": \"color\",\n      \"cam_to_world\": [\n";
    for (int r = 0; r < 4; ++r) {
      out << "        [" << number(m[0][r]) << ", " << number(m[1][r]) << ", "
          << number(m[2][r]) << ", " << number(m[3][r]) << "]"
          << (r < 3 ? ",\n" : "\n");
    }
    out << "      ]\n    }" << (i + 1 < poses.size() ? ",\n" : "\n");
  }
  out << "  ]\n}\n";

  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) return Status::io_error("rig poses: cannot create " + path);
  file << out.str();
  file.close();
  if (!file) return Status::io_error("rig poses: cannot write " + path);
  return {};
}

}  // namespace volumetric_kit::recon::sensor
