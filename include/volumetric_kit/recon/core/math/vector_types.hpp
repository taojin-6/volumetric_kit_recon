// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file math/vector_types.hpp
/// @brief Portable POD vector/matrix types shared by host code and GPU kernels.
///
/// Deliberately self-contained (no glm): the renderer side uses glm, but this
/// compute backend keeps its own host/device-portable POD types so the same
/// layouts compile under nvcc and the Metal toolchain. Matrices are
/// column-major to match glm / Metal / Vulkan conventions.

#include <cmath>
#include <cstdint>

#include "volumetric_kit/recon/core/device_macros.hpp"

namespace volumetric_kit::recon {

/// 3-component float vector.
struct Vec3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

VR_DEVICE_HOST inline Vec3f operator+(Vec3f a, Vec3f b) {
  return Vec3f{a.x + b.x, a.y + b.y, a.z + b.z};
}
VR_DEVICE_HOST inline Vec3f operator-(Vec3f a, Vec3f b) {
  return Vec3f{a.x - b.x, a.y - b.y, a.z - b.z};
}
VR_DEVICE_HOST inline Vec3f operator*(Vec3f v, float s) {
  return Vec3f{v.x * s, v.y * s, v.z * s};
}
VR_DEVICE_HOST inline Vec3f operator*(float s, Vec3f v) { return v * s; }
VR_DEVICE_HOST inline float dot(Vec3f a, Vec3f b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
VR_DEVICE_HOST inline Vec3f cross(Vec3f a, Vec3f b) {
  return Vec3f{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
               a.x * b.y - a.y * b.x};
}
VR_DEVICE_HOST inline float length(Vec3f a) { return std::sqrt(dot(a, a)); }
VR_DEVICE_HOST inline Vec3f normalize(Vec3f a) {
  const float len = length(a);
  return len > 0.0f ? a * (1.0f / len) : a;
}

/// 3-component signed-integer vector -- e.g. voxel-block coordinates.
struct Vec3i {
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::int32_t z = 0;
};

VR_DEVICE_HOST inline bool operator==(Vec3i a, Vec3i b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
VR_DEVICE_HOST inline bool operator!=(Vec3i a, Vec3i b) { return !(a == b); }

/// 4-component float vector.
struct Vec4f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 0.0f;
};

/// 4x4 matrix, column-major: element (row, col) is `m[col * 4 + row]`.
struct Mat4f {
  float m[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

  /// @return The 4x4 identity matrix.
  VR_DEVICE_HOST static Mat4f identity() { return Mat4f{}; }
};

VR_DEVICE_HOST inline Mat4f operator*(const Mat4f& a, const Mat4f& b) {
  Mat4f r;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      float sum = 0.0f;
      for (int k = 0; k < 4; ++k) {
        sum += a.m[k * 4 + row] * b.m[col * 4 + k];
      }
      r.m[col * 4 + row] = sum;
    }
  }
  return r;
}

VR_DEVICE_HOST inline Vec4f operator*(const Mat4f& a, Vec4f v) {
  return Vec4f{a.m[0] * v.x + a.m[4] * v.y + a.m[8] * v.z + a.m[12] * v.w,
               a.m[1] * v.x + a.m[5] * v.y + a.m[9] * v.z + a.m[13] * v.w,
               a.m[2] * v.x + a.m[6] * v.y + a.m[10] * v.z + a.m[14] * v.w,
               a.m[3] * v.x + a.m[7] * v.y + a.m[11] * v.z + a.m[15] * v.w};
}

}  // namespace volumetric_kit::recon
