// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The tier's host rigid-motion helpers: what the judge and the Tracker do with
// a 4x4 correction. Internal. Arithmetic in double, results in float, so a
// chain of them loses nothing a fit could see.

#include <cmath>

#include <glm/glm.hpp>

#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::track::detail {

using Mat3d = glm::dmat3;
using Vec3d = glm::dvec3;

// A world-from-world motion as a twist about an origin o: X -> exp(w) (X - o)
// + o + v.
struct Twist {
  Vec3d v{0.0};
  Vec3d w{0.0};
};

// m's rotation and translation, in double.
inline Mat3d rotation_of(const Mat4f& m) {
  Mat3d r(1.0);
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) r[c][k] = double(m[c][k]);
  }
  return r;
}
inline Vec3d translation_of(const Mat4f& m) {
  return Vec3d(double(m[3][0]), double(m[3][1]), double(m[3][2]));
}

inline bool finite(const Mat4f& m) noexcept {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(m[c][r])) return false;
    }
  }
  return true;
}

// Whether m is a rigid motion: finite, last row (0 0 0 1), an orthonormal
// right-handed rotation to 1e-4.
inline bool rigid(const Mat4f& m) noexcept {
  if (!finite(m)) return false;
  if (m[0][3] != 0.0f || m[1][3] != 0.0f || m[2][3] != 0.0f ||
      m[3][3] != 1.0f) {
    return false;
  }
  const Mat3d r = rotation_of(m);
  const Mat3d e = glm::transpose(r) * r;
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) {
      if (std::fabs(e[c][k] - (c == k ? 1.0 : 0.0)) > 1e-4) return false;
    }
  }
  return glm::determinant(r) > 0.0;
}

inline Mat3d skew(const Vec3d& w) {
  // Column-major: skew(w) * x = cross(w, x).
  return Mat3d(Vec3d(0.0, w.z, -w.y), Vec3d(-w.z, 0.0, w.x),
               Vec3d(w.y, -w.x, 0.0));
}

inline Mat3d so3_exp(const Vec3d& w) {
  const double th = glm::length(w);
  const Mat3d k = skew(w);
  if (th < 1e-12) return Mat3d(1.0) + k;
  const Mat3d kn = k * (1.0 / th);
  return Mat3d(1.0) + std::sin(th) * kn + (1.0 - std::cos(th)) * (kn * kn);
}

inline Vec3d so3_log(const Mat3d& r) {
  const double tr = (r[0][0] + r[1][1] + r[2][2] - 1.0) * 0.5;
  const double th = std::acos(std::fmax(-1.0, std::fmin(1.0, tr)));
  // r[c][row]: element (row, col).
  const Vec3d v(r[1][2] - r[2][1], r[2][0] - r[0][2], r[0][1] - r[1][0]);
  const double s = std::sin(th);
  if (s > 1e-9) return v * (th / (2.0 * s));
  if (th < 1e-6) return v * 0.5;
  // Near pi: the axis from the symmetric part.
  const Mat3d b = (r + glm::transpose(r)) * 0.5;
  Vec3d axis(std::sqrt(std::fmax(0.0, (b[0][0] + 1.0) * 0.5)),
             std::sqrt(std::fmax(0.0, (b[1][1] + 1.0) * 0.5)),
             std::sqrt(std::fmax(0.0, (b[2][2] + 1.0) * 0.5)));
  if (axis.x >= axis.y && axis.x >= axis.z) {
    axis.y = std::copysign(axis.y, b[0][1]);
    axis.z = std::copysign(axis.z, b[0][2]);
  } else if (axis.y >= axis.z) {
    axis.x = std::copysign(axis.x, b[0][1]);
    axis.z = std::copysign(axis.z, b[1][2]);
  } else {
    axis.x = std::copysign(axis.x, b[0][2]);
    axis.y = std::copysign(axis.y, b[1][2]);
  }
  return glm::normalize(axis) * th;
}

inline Mat4f to_f(const Mat3d& r, const Vec3d& t) {
  Mat4f m(1.0f);
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) m[c][k] = static_cast<float>(r[c][k]);
  }
  m[3] = Vec4f(float(t.x), float(t.y), float(t.z), 1.0f);
  return m;
}

// m with its rotation made orthonormal again (the exponential of its log),
// its translation kept. A rotation composed in float drifts off orthonormal a
// little at each step; fed back fit after fit, it would reach rigid()'s 1e-4.
inline Mat4f orthonormalised(const Mat4f& m) {
  return to_f(so3_exp(so3_log(rotation_of(m))), translation_of(m));
}

// The twist about o that takes `from` to `to`: to = exp(twist; o) * from.
inline Twist twist_between(const Mat4f& from, const Mat4f& to, const Vec3d& o) {
  const Mat3d rd = rotation_of(to) * glm::transpose(rotation_of(from));
  // Delta(X) = rd X + td, td = t_to - rd t_from; and Delta(X) = rd (X - o) +
  // o + v.
  const Vec3d td = translation_of(to) - rd * translation_of(from);
  Twist out;
  out.w = so3_log(rd);
  out.v = td - o + rd * o;
  return out;
}

// exp(twist; o) * m.
inline Mat4f apply_twist(const Twist& x, const Vec3d& o, const Mat4f& m) {
  const Mat3d rd = so3_exp(x.w);
  const Vec3d td = o - rd * o + x.v;
  return to_f(rd * rotation_of(m), rd * translation_of(m) + td);
}

// How far `to` is from `from` at the point o: metres and degrees.
inline void motion_at(const Mat4f& from, const Mat4f& to, const Vec3d& o,
                      double& metres, double& degrees) {
  const Twist x = twist_between(from, to, o);
  // Delta(o) - o = v.
  metres = glm::length(x.v);
  degrees = glm::length(x.w) * (180.0 / 3.14159265358979323846);
}

// SE(3) by time: the rotation slerped and the translation lerped, a at 0 and
// b at 1.
inline Mat4f interpolate(const Mat4f& a, const Mat4f& b, double s) {
  const Mat3d ra = rotation_of(a);
  const Vec3d w = so3_log(glm::transpose(ra) * rotation_of(b));
  const Vec3d t = translation_of(a) * (1.0 - s) + translation_of(b) * s;
  return to_f(ra * so3_exp(w * s), t);
}

}  // namespace volumetric_kit::recon::track::detail
