// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/rig_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace volumetric_kit::recon::sensor {

namespace {

using nlohmann::json;

constexpr double kPi = 3.14159265358979323846;
// How far a rotation may be from orthonormal: catches a matrix that is not
// one without refusing float round-off.
constexpr float kOrthonormalTolerance = 1e-4f;
constexpr float kBottomRowTolerance = 1e-6f;

core::Status bad(const std::string& what) {
  return core::Status::invalid_argument("rig calibration: " + what);
}

// Row-major 3x3, in double: the conversions below are where precision goes.
struct Mat3 {
  double m[3][3];
};

Mat3 rotation_from_rodrigues(const double r[3]) {
  const double theta = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  Mat3 R{};
  if (theta < 1e-12) {  // first order: I + [r]x
    R = {{{1, -r[2], r[1]}, {r[2], 1, -r[0]}, {-r[1], r[0], 1}}};
    return R;
  }
  const double k[3] = {r[0] / theta, r[1] / theta, r[2] / theta};
  const double c = std::cos(theta), s = std::sin(theta), t = 1.0 - c;
  const double kx[3][3] = {
      {0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R.m[i][j] = (i == j ? c : 0.0) + t * k[i] * k[j] + s * kx[i][j];
    }
  }
  return R;
}

void rodrigues_from_rotation(const Mat3& R, double r[3]) {
  const double vee[3] = {R.m[2][1] - R.m[1][2], R.m[0][2] - R.m[2][0],
                         R.m[1][0] - R.m[0][1]};
  // atan2, not acos: near pi, acos turns a float matrix's round-off into an
  // angle error of order 1e-7 / sin(theta).
  const double cos_theta =
      std::clamp((R.m[0][0] + R.m[1][1] + R.m[2][2] - 1.0) / 2.0, -1.0, 1.0);
  const double sin_theta =
      0.5 * std::sqrt(vee[0] * vee[0] + vee[1] * vee[1] + vee[2] * vee[2]);
  const double theta = std::atan2(sin_theta, cos_theta);
  if (theta < 1e-6) {  // first order
    for (int i = 0; i < 3; ++i) r[i] = 0.5 * vee[i];
    return;
  }
  if (theta < kPi / 2) {
    const double scale = theta / (2.0 * sin_theta);
    for (int i = 0; i < 3; ++i) r[i] = scale * vee[i];
    return;
  }
  // Past pi/2, sin(theta) shrinks toward pi, so take the axis from the
  // symmetric part, (R + R^T)/2 = cos(theta) I + (1 - cos(theta)) k k^T, off
  // its largest diagonal, and only its sign from vee.
  double kk[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      kk[i][j] = ((R.m[i][j] + R.m[j][i]) / 2.0 - (i == j ? cos_theta : 0.0)) /
                 (1.0 - cos_theta);
    }
  }
  int a = 0;
  for (int i = 1; i < 3; ++i) {
    if (kk[i][i] > kk[a][a]) a = i;
  }
  double k[3];
  k[a] = std::sqrt(std::max(0.0, kk[a][a]));
  for (int i = 0; i < 3; ++i) {
    if (i != a) k[i] = kk[a][i] / k[a];
  }
  const double dot = k[0] * vee[0] + k[1] * vee[1] + k[2] * vee[2];
  const double sign = dot < 0.0 ? -1.0 : 1.0;
  for (int i = 0; i < 3; ++i) r[i] = sign * theta * k[i];
}

// The file's world->camera extrinsic to this repo's camera->world.
Mat4f cam_to_world_from(const double rvec[3], const double tvec[3]) {
  const Mat3 R = rotation_from_rodrigues(rvec);
  Mat4f m(1.0f);  // m[column][row]
  for (int i = 0; i < 3; ++i) {
    double t = 0.0;
    for (int j = 0; j < 3; ++j) {
      m[j][i] = static_cast<float>(R.m[j][i]);  // R^T
      t -= R.m[j][i] * tvec[j];                 // -R^T t
    }
    m[3][i] = static_cast<float>(t);
  }
  return m;
}

void extrinsic_from(const Mat4f& cam_to_world, double rvec[3], double tvec[3]) {
  Mat3 R{};  // world->camera = transpose of camera->world's rotation
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) R.m[i][j] = cam_to_world[i][j];
  }
  for (int i = 0; i < 3; ++i) {
    tvec[i] = 0.0;
    for (int j = 0; j < 3; ++j) tvec[i] -= R.m[i][j] * cam_to_world[3][j];
  }
  rodrigues_from_rotation(R, rvec);
}

core::Status check_camera(const RigCameraCalibration& camera) {
  const std::string who = "camera " + camera.serial + ": ";
  const Mat4f& m = camera.cam_to_world;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(m[c][r])) return bad(who + "pose is not finite");
    }
  }
  if (std::fabs(m[0][3]) > kBottomRowTolerance ||
      std::fabs(m[1][3]) > kBottomRowTolerance ||
      std::fabs(m[2][3]) > kBottomRowTolerance ||
      std::fabs(m[3][3] - 1.0f) > kBottomRowTolerance) {
    return bad(who + "pose's bottom row is not [0, 0, 0, 1]");
  }
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      float dot = 0.0f;
      for (int r = 0; r < 3; ++r) dot += m[a][r] * m[b][r];
      if (std::fabs(dot - (a == b ? 1.0f : 0.0f)) > kOrthonormalTolerance) {
        return bad(who + "pose's rotation is not orthonormal");
      }
    }
  }
  const float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                    m[1][0] * (m[0][1] * m[2][2] - m[2][1] * m[0][2]) +
                    m[2][0] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]);
  if (!(det > 0.0f)) return bad(who + "pose's rotation is a reflection");
  for (const auto* k : {&camera.intrinsics, &camera.optimal_intrinsics}) {
    if (!*k) continue;
    const PinholeIntrinsics& p = **k;
    if (!(std::isfinite(p.fx) && p.fx > 0.0f && std::isfinite(p.fy) &&
          p.fy > 0.0f && std::isfinite(p.cx) && std::isfinite(p.cy))) {
      return bad(who + "intrinsics need finite, positive focal lengths");
    }
  }
  if (camera.distortion) {
    const LensDistortion& d = *camera.distortion;
    for (const float v : {d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6}) {
      if (!std::isfinite(v)) return bad(who + "distortion is not finite");
    }
  }
  return {};
}

core::Result<float> number(const json& object, const char* key,
                           const std::string& where) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_number()) {
    return bad(where + "missing number \"" + key + "\"");
  }
  return it->get<float>();
}

core::Result<PinholeIntrinsics> pinhole(const json& j,
                                        const std::string& where) {
  if (!j.is_object()) return bad(where + "not an object");
  PinholeIntrinsics p;
  VKC_ASSIGN(p.fx, number(j, "fx", where));
  VKC_ASSIGN(p.fy, number(j, "fy", where));
  VKC_ASSIGN(p.cx, number(j, "cx", where));
  VKC_ASSIGN(p.cy, number(j, "cy", where));
  return p;
}

core::Status vector3(const json& object, const char* key, double out[3],
                     const std::string& where) {
  const auto it = object.find(key);
  if (it == object.end() || !it->is_array() || it->size() != 3) {
    return bad(where + "\"" + key + "\" is not 3 numbers");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (!(*it)[i].is_number() || !std::isfinite((*it)[i].get<double>())) {
      return bad(where + "\"" + key + "\" is not 3 finite numbers");
    }
    out[i] = (*it)[i].get<double>();
  }
  return {};
}

}  // namespace

core::Status validate_rig_calibration(
    const std::vector<RigCameraCalibration>& cameras) {
  if (cameras.empty()) return bad("no cameras");
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    const std::string& serial = cameras[i].serial;
    if (serial.empty())
      return bad("camera " + std::to_string(i) + ": no serial");
    try {  // JSON text is UTF-8
      (void)json(serial).dump();
    } catch (const json::exception&) {
      return bad("camera " + std::to_string(i) + ": serial is not UTF-8");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (cameras[j].serial == serial) {
        return bad("camera " + serial + " appears twice");
      }
    }
    VKC_TRY(check_camera(cameras[i]));
  }
  return {};
}

core::Result<std::vector<RigCameraCalibration>> parse_rig_calibration(
    const std::string& text) {
  json doc;
  try {
    doc = json::parse(text);
  } catch (const std::exception& e) {  // json::parse_error names the byte
    return bad(std::string("not JSON: ") + e.what());
  }
  const auto section =
      doc.is_object() ? doc.find("device_calibration") : doc.end();
  if (!doc.is_object() || section == doc.end() || !section->is_object()) {
    return bad("no \"device_calibration\" object");
  }
  std::vector<RigCameraCalibration> cameras;
  for (auto it = section->begin(); it != section->end(); ++it) {
    if (!it->is_object()) continue;  // e.g. "pose_calib_file_path"
    RigCameraCalibration camera;
    camera.serial = it.key();
    const std::string where = "camera " + camera.serial + ": ";
    const auto pose = it->find("pose");
    if (pose == it->end() || !pose->is_object()) {
      return bad(where + "no \"pose\" object");
    }
    double rvec[3], tvec[3];
    VKC_TRY(vector3(*pose, "rvec", rvec, where + "pose "));
    VKC_TRY(vector3(*pose, "tvec", tvec, where + "pose "));
    camera.cam_to_world = cam_to_world_from(rvec, tvec);
    for (const auto& [key, field] :
         {std::pair{"intrinsics", &camera.intrinsics},
          std::pair{"optimal_intrinsics", &camera.optimal_intrinsics}}) {
      const auto j = it->find(key);
      if (j == it->end()) continue;
      VKC_ASSIGN(*field, pinhole(*j, where + key + " "));
    }
    const auto distortion = it->find("distortion");
    if (distortion != it->end()) {
      const std::string w = where + "distortion ";
      if (!distortion->is_object()) return bad(w + "not an object");
      LensDistortion d;
      VKC_ASSIGN(d.k1, number(*distortion, "k1", w));
      VKC_ASSIGN(d.k2, number(*distortion, "k2", w));
      VKC_ASSIGN(d.p1, number(*distortion, "p1", w));
      VKC_ASSIGN(d.p2, number(*distortion, "p2", w));
      VKC_ASSIGN(d.k3, number(*distortion, "k3", w));
      VKC_ASSIGN(d.k4, number(*distortion, "k4", w));
      VKC_ASSIGN(d.k5, number(*distortion, "k5", w));
      VKC_ASSIGN(d.k6, number(*distortion, "k6", w));
      camera.distortion = d;
    }
    cameras.push_back(std::move(camera));
  }
  VKC_TRY(validate_rig_calibration(cameras));
  return cameras;
}

core::Result<std::vector<RigCameraCalibration>> read_rig_calibration(
    const std::string& path) {
  // stdio rather than a stream: a stream reports a failed read -- a directory,
  // say -- as the end of the file.
  std::FILE* in = std::fopen(path.c_str(), "rb");
  if (in == nullptr) {
    return core::Status::io_error("rig calibration: cannot open " + path);
  }
  std::string text;
  char buf[4096];
  std::size_t n = 0;
  while ((n = std::fread(buf, 1, sizeof(buf), in)) > 0) text.append(buf, n);
  const bool failed = std::ferror(in) != 0;
  std::fclose(in);
  if (failed)
    return core::Status::io_error("rig calibration: cannot read " + path);
  auto cameras = parse_rig_calibration(text);
  if (!cameras.ok()) {
    return core::Status::invalid_argument(path + ": " +
                                          cameras.status().message());
  }
  return cameras;
}

core::Status write_rig_calibration(
    const std::string& path, const std::vector<RigCameraCalibration>& cameras) {
  VKC_TRY(validate_rig_calibration(cameras));
  // Written by hand so each vector stays on one line. The classic locale keeps
  // JSON's decimal point; 9 digits round-trip a float, 17 a double.
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(9);
  const auto pinhole_out = [&out](const PinholeIntrinsics& p) {
    out << "{\"fx\": " << p.fx << ", \"fy\": " << p.fy << ", \"cx\": " << p.cx
        << ", \"cy\": " << p.cy << "}";
  };
  out << "{\n  \"device_calibration\": {\n";
  for (std::size_t i = 0; i < cameras.size(); ++i) {
    const RigCameraCalibration& c = cameras[i];
    out << "    " << json(c.serial).dump() << ": {\n";
    if (c.intrinsics) {
      out << "      \"intrinsics\": ";
      pinhole_out(*c.intrinsics);
      out << ",\n";
    }
    if (c.distortion) {
      const LensDistortion& d = *c.distortion;
      out << "      \"distortion\": {\"k1\": " << d.k1 << ", \"k2\": " << d.k2
          << ", \"p1\": " << d.p1 << ", \"p2\": " << d.p2
          << ", \"k3\": " << d.k3 << ", \"k4\": " << d.k4
          << ", \"k5\": " << d.k5 << ", \"k6\": " << d.k6 << "},\n";
    }
    if (c.optimal_intrinsics) {
      out << "      \"optimal_intrinsics\": ";
      pinhole_out(*c.optimal_intrinsics);
      out << ",\n";
    }
    double rvec[3], tvec[3];
    extrinsic_from(c.cam_to_world, rvec, tvec);
    out << std::setprecision(17);
    out << "      \"pose\": {\"rvec\": [" << rvec[0] << ", " << rvec[1] << ", "
        << rvec[2] << "], \"tvec\": [" << tvec[0] << ", " << tvec[1] << ", "
        << tvec[2] << "]}\n    }" << (i + 1 < cameras.size() ? ",\n" : "\n");
    out << std::setprecision(9);
  }
  out << "  }\n}\n";

  std::FILE* file = std::fopen(path.c_str(), "wb");
  if (file == nullptr) {
    return core::Status::io_error("rig calibration: cannot create " + path);
  }
  const std::string text = out.str();
  const bool ok = std::fwrite(text.data(), 1, text.size(), file) == text.size();
  if (std::fclose(file) != 0 || !ok) {
    return core::Status::io_error("rig calibration: cannot write " + path);
  }
  return {};
}

}  // namespace volumetric_kit::recon::sensor
