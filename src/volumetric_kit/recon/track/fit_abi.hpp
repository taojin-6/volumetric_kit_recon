// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The host's mirrors of the fitter's device state (track_common.glsl's
// FitView and FitState) and of the reduce's push constants, byte for byte
// (scalar layout). Internal: point_to_plane.cpp records with them, and the
// reduce's kernel test (tests/track_reduce_tie_test.cpp) dispatches the
// kernel alone with them.

#include <cstddef>
#include <cstdint>

#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::track::detail {

// Mirrors FitView in track_common.glsl.
struct FitView {
  Mat4f cam_to_world{1.0f};
  Vec4f k{0.0f};
  Vec3f noise{0.0f};
  std::uint32_t entry = 0;
  float min_depth = 0.0f;
  float max_depth = 0.0f;
  std::uint32_t pad0 = 0;
  std::uint32_t pad1 = 0;
};
static_assert(sizeof(FitView) == 112, "FitView layout drift");
static_assert(offsetof(FitView, entry) == 92, "FitView layout drift");

// Mirrors FitState in track_common.glsl.
struct FitState {
  Mat4f correction{1.0f};
  Mat4f prior{1.0f};
  Mat4f pass_start{1.0f};
  Vec3f origin{0.0f};
  float scale = 1.0f;
  Vec2f info{0.0f};
  float n_eff = 0.0f;
  std::uint32_t frames = 0;
  std::uint32_t serial_lo = 0;
  std::uint32_t serial_hi = 0;
  std::uint32_t stop = 0;
  std::uint32_t diverged = 0;
  float attempts = 0.0f;
  float hits = 0.0f;
  float inliers = 0.0f;
  float in_front = 0.0f;
  float behind = 0.0f;
  float raw2 = 0.0f;
  std::uint32_t evaluated = 0;
  float lambda_max = 0.0f;
  float rel[6] = {};
  float dirs[36] = {};
  std::uint32_t held = 0x3f;
};
static_assert(sizeof(FitState) == 444, "FitState layout drift");
static_assert(offsetof(FitState, origin) == 192, "FitState layout drift");
static_assert(offsetof(FitState, attempts) == 240, "FitState layout drift");
static_assert(offsetof(FitState, held) == 440, "FitState layout drift");

// Mirrors track_reduce_point_to_plane.comp's push constants.
struct ReducePush {
  std::uint32_t pixels, groups_per_view, model_width, model_height, finest;
  float gate, gate_sigmas, normal_cos, huber_sigmas, model_sigma, see_through;
};
static_assert(sizeof(ReducePush) == 44, "ReducePush layout drift");

// What a reduce workgroup leaves for the solve (track_common.glsl's partial
// layout): kPartial floats, H's upper triangle row by row first (21), then
// g, the inliers' squared residuals and the counts.
inline constexpr std::uint32_t kPartial = 33;
inline constexpr std::uint32_t kPartialAttempts = 28;
inline constexpr std::uint32_t kPartialHits = 29;
inline constexpr std::uint32_t kPartialInliers = 30;

}  // namespace volumetric_kit::recon::track::detail
