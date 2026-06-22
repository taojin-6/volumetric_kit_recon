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

/// @brief 3-component float vector.
struct Vec3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/// @brief Component-wise sum.
/// @return `{a.x+b.x, a.y+b.y, a.z+b.z}`.
VR_DEVICE_HOST inline Vec3f operator+(Vec3f a, Vec3f b) {
  return Vec3f{a.x + b.x, a.y + b.y, a.z + b.z};
}
/// @brief Component-wise difference.
/// @return `{a.x-b.x, a.y-b.y, a.z-b.z}`.
VR_DEVICE_HOST inline Vec3f operator-(Vec3f a, Vec3f b) {
  return Vec3f{a.x - b.x, a.y - b.y, a.z - b.z};
}
/// @brief Scale a vector by a scalar.
/// @param v  The vector.
/// @param s  The scalar factor.
/// @return `{v.x*s, v.y*s, v.z*s}`.
VR_DEVICE_HOST inline Vec3f operator*(Vec3f v, float s) {
  return Vec3f{v.x * s, v.y * s, v.z * s};
}
/// @copydoc operator*(Vec3f,float)
VR_DEVICE_HOST inline Vec3f operator*(float s, Vec3f v) { return v * s; }
/// @brief Dot (inner) product.
/// @return `a.x*b.x + a.y*b.y + a.z*b.z`.
VR_DEVICE_HOST inline float dot(Vec3f a, Vec3f b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
/// @brief Cross product, right-handed (`cross(+x, +y) == +z`).
/// @return The vector perpendicular to both @p a and @p b.
VR_DEVICE_HOST inline Vec3f cross(Vec3f a, Vec3f b) {
  return Vec3f{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
               a.x * b.y - a.y * b.x};
}
/// @brief Euclidean length (L2 norm).
/// @return `sqrt(dot(a, a))`.
VR_DEVICE_HOST inline float length(Vec3f a) { return std::sqrt(dot(a, a)); }
/// @brief Unit vector in the direction of @p a.
/// @return @p a scaled to unit length, or @p a unchanged when its length is
///         zero -- never divides by zero.
VR_DEVICE_HOST inline Vec3f normalize(Vec3f a) {
  const float len = length(a);
  return len > 0.0f ? a * (1.0f / len) : a;
}

/// @brief 3-component signed-integer vector (e.g. voxel-block coordinates).
struct Vec3i {
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::int32_t z = 0;
};

/// @brief Equality: true when all three components are equal.
VR_DEVICE_HOST inline bool operator==(Vec3i a, Vec3i b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
/// @brief Inequality: the negation of @ref operator==(Vec3i,Vec3i).
VR_DEVICE_HOST inline bool operator!=(Vec3i a, Vec3i b) { return !(a == b); }

/// @brief 4-component float vector.
struct Vec4f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 0.0f;
};

/// @brief 4x4 matrix in column-major storage: element (row, col) is
///        `m[col * 4 + row]`, matching glm / Metal / Vulkan conventions.
///
/// A default-constructed Mat4f is the identity. The @ref Uninitialized
/// constructor leaves the elements indeterminate; it exists for hot-path
/// producers (e.g. `operator*`) that overwrite every element themselves, so
/// they pay nothing for a throwaway initial fill.
struct Mat4f {
  float m[16];

  /// Tag type selecting the indeterminate-element constructor.
  struct Uninitialized {};

  /// @brief Construct the identity matrix.
  VR_DEVICE_HOST Mat4f()
      : m{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f} {}

  /// @brief Construct with indeterminate elements (no initialization).
  VR_DEVICE_HOST explicit Mat4f(Uninitialized) {}

  /// @return The 4x4 identity matrix.
  VR_DEVICE_HOST static Mat4f identity() { return Mat4f{}; }
};

/// @brief Matrix product `a * b` (column-major).
/// @return The 4x4 product matrix.
VR_DEVICE_HOST inline Mat4f operator*(const Mat4f& a, const Mat4f& b) {
  Mat4f r{Mat4f::Uninitialized{}};  // every element is written below
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

/// @brief Transform a 4-vector by a matrix: `a * v`.
/// @return The transformed vector.
VR_DEVICE_HOST inline Vec4f operator*(const Mat4f& a, Vec4f v) {
  return Vec4f{a.m[0] * v.x + a.m[4] * v.y + a.m[8] * v.z + a.m[12] * v.w,
               a.m[1] * v.x + a.m[5] * v.y + a.m[9] * v.z + a.m[13] * v.w,
               a.m[2] * v.x + a.m[6] * v.y + a.m[10] * v.z + a.m[14] * v.w,
               a.m[3] * v.x + a.m[7] * v.y + a.m[11] * v.z + a.m[15] * v.w};
}

}  // namespace volumetric_kit::recon
