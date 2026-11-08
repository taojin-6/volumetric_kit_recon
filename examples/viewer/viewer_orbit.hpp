// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_orbit.hpp
/// @brief `rig_viewer`'s view: gfx's `OrbitCamera` about the world's up, its
///        home around where the cameras look, from the primary's side, and a
///        light that follows the eye. CPU only, on gfx_camera and glm.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <glm/glm.hpp>

#include "volumetric_kit/gfx/camera/orbit_camera.hpp"

namespace fuse_viewer {

namespace vgc = volumetric_kit::gfx::camera;

/// @return The point nearest every camera's optical axis in the least-squares
///         sense: where a rig built around a subject is looking. @p fallback
///         when the axes are (nearly) parallel, as one camera's always is, or
///         meet behind any camera, or less than 0.1 m in front of one.
/// @param poses     Each camera's colour-to-world pose (OpenCV axes: +z the
///                  optical axis).
/// @param fallback  The point to use where the axes name none.
inline glm::vec3 axes_meet(const std::vector<glm::mat4>& poses,
                           glm::vec3 fallback) {
  glm::mat3 a(0.0f);
  glm::vec3 b(0.0f);
  for (const glm::mat4& c2w : poses) {
    const glm::vec3 d = glm::normalize(glm::vec3(c2w[2]));
    const glm::mat3 away = glm::mat3(1.0f) - glm::outerProduct(d, d);
    a += away;
    b += away * glm::vec3(c2w[3]);
  }
  if (poses.size() < 2 || std::fabs(glm::determinant(a)) < 1e-6f) {
    return fallback;
  }
  const glm::vec3 p = glm::inverse(a) * b;
  for (const glm::mat4& c2w : poses) {
    if (glm::dot(p - glm::vec3(c2w[3]), glm::normalize(glm::vec3(c2w[2]))) <
        0.1f) {
      return fallback;
    }
  }
  return p;
}

/// @return @p stated (`vr_example::calibration_up`), or where that is unknown
///         (all zero) @p primary's image up, -y, as before.
/// @param stated   The up the rig's calibration states.
/// @param primary  The primary camera's colour-to-world pose.
inline glm::vec3 rig_up(const std::array<float, 3>& stated,
                        const glm::mat4& primary) {
  if (stated != std::array<float, 3>{}) {
    return {stated[0], stated[1], stated[2]};
  }
  return -glm::vec3(primary[1]);
}

/// @brief The home view: about @p up, around where the cameras look
///        (@ref axes_meet), the eye behind that point along the primary's
///        optical axis at the primary's distance (at least 0.3 m) -- where the
///        primary looks from when it looks at it, turned upright. Within about
///        2.6 degrees of a pole, where no picture is upright, the primary's own
///        picture, its image x the right, the eye as near its axis as that
///        allows.
/// @param poses      Each camera's colour-to-world pose, as @ref axes_meet.
/// @param primary    The primary camera's index into @p poses.
/// @param up         The world's up (@ref rig_up); any length.
/// @param depth_mid  How far along the primary's axis to look where the axes
///                   name no point.
/// @return The orbit at home; assign it back to reset the view.
inline vgc::OrbitCamera home_view(const std::vector<glm::mat4>& poses,
                                  std::size_t primary, glm::vec3 up,
                                  float depth_mid) {
  const glm::mat4& c2w = poses[primary];
  const glm::vec3 eye(c2w[3]);
  const glm::vec3 forward = glm::normalize(glm::vec3(c2w[2]));
  const glm::vec3 target = axes_meet(poses, eye + forward * depth_mid);
  vgc::OrbitCamera home;
  home.set_world_up(up);
  home.set_target(target);
  home.set_distance(std::max(0.3f, glm::length(target - eye)));
  // World into the camera's own level frame: its view's rows at azimuth =
  // elevation = 0 (right, up, back). gfx's heading rule is read here, not
  // restated. `back` is the eye's direction from the target.
  const glm::mat3 level(home.view());
  const glm::vec3 back = level * -forward;
  // The eye's heading; at a pole, where that is noise, the one whose right,
  // (cos az, 0, -sin az), is the primary's image x.
  constexpr float kPole = 0.999f;
  const glm::vec3 right = level * glm::vec3(c2w[0]);
  const float azimuth = std::fabs(back.y) < kPole
                            ? std::atan2(back.x, back.z)
                            : std::atan2(-right.z, right.x);
  home.set_azimuth(azimuth);
  // The eye's pitch on that heading's meridian: on the primary's axis off a
  // pole, nearest it at one (past the pole when the axis leans the other way).
  home.set_elevation(std::atan2(
      back.y, back.x * std::sin(azimuth) + back.z * std::cos(azimuth)));
  return home;
}

/// @return The world direction to a light that follows the eye (the player's
///         headlight, `HybridMeshFrame::light_dir`): toward it, a little up
///         and to the left, (-0.3, 0.4, 1) in view space, normalized. Lit from
///         the eye's side wherever it orbits, whatever the world's up.
/// @param view  The camera's world-to-view matrix, a rotation and a
///              translation, upside down too (`OrbitCamera::view`).
// TODO: gfx derives a view-relative light (the player's TODO); use it then.
inline glm::vec3 headlight(const glm::mat4& view) {
  const glm::vec3 in_view = glm::normalize(glm::vec3(-0.3f, 0.4f, 1.0f));
  return glm::normalize(glm::transpose(glm::mat3(view)) * in_view);
}

}  // namespace fuse_viewer
