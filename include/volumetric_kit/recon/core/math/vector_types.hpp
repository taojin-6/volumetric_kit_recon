// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file math/vector_types.hpp
/// @brief Vector/matrix vocabulary types shared by host code and GPU kernels.
///
/// These are thin aliases over GLM rather than hand-rolled structs, for three
/// reasons. GLM is the renderer's (`volumetric_kit_gfx`) math library, so
/// sharing it keeps the interop seam free of vector-type conversions. Its types
/// are trivially-copyable, standard-layout PODs whose packed layout (`vec3` =
/// 12 B, `vec4` = 16 B, `mat4` = 64 B, column-major) mirrors a GLSL `std430`
/// block byte-for-byte -- the host side of the buffer ABI the Vulkan compute
/// shaders read (the shader keeps its own `std430` definition in lockstep; see
/// the gotchas in CLAUDE.md). And GLM qualifies its operators
/// `__host__ __device__` under nvcc, so the same types and math are callable
/// inside the native-CUDA accelerator's kernels (locked decision, 2026-07-04)
/// -- exactly what the earlier hand-rolled POD types reproduced by hand.
///
/// The `vr::` names are kept as the project's vocabulary so call sites don't
/// bind to `glm::` directly and the backing type stays swappable. Matrices are
/// column-major, matching GLM / Vulkan conventions; a default-constructed
/// `glm::mat4` is indeterminate -- spell the identity `Mat4f(1.0f)`.

#include <glm/glm.hpp>
#include <glm/gtc/type_precision.hpp>

#include "volumetric_kit/recon/core/device_macros.hpp"

namespace volumetric_kit::recon {

/// @brief 3-component float vector (packed 12 B; positions, normals, ...).
using Vec3f = glm::vec3;
/// @brief 3-component signed-integer vector (e.g. voxel-block coordinates).
using Vec3i = glm::ivec3;
/// @brief 3-component unsigned 8-bit vector (packed 3 B; per-voxel RGB color).
using Vec3u8 = glm::u8vec3;
/// @brief 4-component float vector (packed 16 B; homogeneous points, ...).
using Vec4f = glm::vec4;
/// @brief 4x4 column-major matrix (64 B; element (row, col) is `m[col][row]`).
using Mat4f = glm::mat4;

// Re-export the GLM free functions used across the codebase into `vr::` so
// `vr::dot(a, b)` and friends resolve without every call site reaching into
// `glm::`. The arithmetic operators need no re-export -- argument-dependent
// lookup finds them through the GLM types' own namespace.
using glm::cross;
using glm::dot;
using glm::length;

/// @brief Unit vector in the direction of @p v, or @p v unchanged when its
///        length is zero -- never divides by zero.
///
/// Deliberately *not* a re-export of `glm::normalize`, which yields NaNs for a
/// zero-length input. Degenerate (zero-area) geometry produces zero normals in
/// fusion and meshing, so this guard preserves the prior engine's safe
/// behaviour. Marked @c VR_DEVICE_HOST so it stays callable from CUDA device
/// code alongside the GLM operators.
/// @param v  The vector to normalize.
/// @return @p v scaled to unit length, or @p v unchanged when its length is 0.
VR_DEVICE_HOST inline Vec3f normalize(Vec3f v) {
  const float len = glm::length(v);
  return len > 0.0f ? v / len : v;
}

}  // namespace volumetric_kit::recon
