// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// rig_viewer's view (examples/viewer/viewer_orbit.hpp): the home view about
// any up -- the eye behind where the cameras look along the primary's optical
// axis, the picture upright, or at a pole the primary's own -- and the same
// view as before for a rig with no calibration; where the cameras' axes meet,
// or the fallback; the up the calibration states, or the primary's image up;
// and the light that follows the eye. Host only.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <glm/glm.hpp>

#include "viewer_orbit.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/gfx/camera/camera.hpp"
#include "volumetric_kit/gfx/camera/orbit_camera.hpp"

namespace vgc = volumetric_kit::gfx::camera;

namespace {

bool near(float a, float b, float tolerance = 1e-4f) {
  return std::fabs(a - b) <= tolerance;
}

bool near(const glm::vec3& a, const glm::vec3& b, float tolerance = 1e-4f) {
  return glm::length(a - b) <= tolerance;
}

bool near(const glm::mat4& a, const glm::mat4& b, float tolerance) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!near(a[c][r], b[c][r], tolerance)) return false;
    }
  }
  return true;
}

// A view matrix's row: its right (0), up (1) and back (2) in world axes.
glm::vec3 row(const glm::mat4& view, int r) {
  return {view[0][r], view[1][r], view[2][r]};
}

// A colour camera's pose (OpenCV axes: x right, y down, z forward) at `eye`,
// looking at `at`, the right way up about `up`.
glm::mat4 looking(glm::vec3 eye, glm::vec3 at, glm::vec3 up) {
  const glm::vec3 z = glm::normalize(at - eye);
  const glm::vec3 x = glm::normalize(glm::cross(z, up));
  const glm::vec3 y = glm::cross(z, x);
  glm::mat4 c2w(1.0f);
  c2w[0] = glm::vec4(x, 0.0f);
  c2w[1] = glm::vec4(y, 0.0f);
  c2w[2] = glm::vec4(z, 0.0f);
  c2w[3] = glm::vec4(eye, 1.0f);
  return c2w;
}

// The same camera turned a quarter about its optical axis, onto its side: its
// image up across the world's up, as the lab rig's Femto cameras are mounted.
glm::mat4 on_side(glm::mat4 c2w) {
  const glm::vec4 x = c2w[0];
  c2w[0] = c2w[1];
  c2w[1] = -x;
  return c2w;
}

constexpr glm::vec3 kPlusZ(0.0f, 0.0f, 1.0f);
constexpr glm::vec3 kMinusY(0.0f, -1.0f, 0.0f);

// Four cameras 2 m around (0.1, 0.2, 1.0) in a tag's world, 1.6 m up, looking
// at it, the second on its side.
std::vector<glm::mat4> ring() {
  const glm::vec3 subject(0.1f, 0.2f, 1.0f);
  std::vector<glm::mat4> poses;
  for (int i = 0; i < 4; ++i) {
    const float a = 1.5707964f * static_cast<float>(i);
    const glm::vec3 eye =
        subject + glm::vec3(2.0f * std::cos(a), 2.0f * std::sin(a), 0.6f);
    poses.push_back(looking(eye, subject, kPlusZ));
  }
  poses[1] = on_side(poses[1]);
  return poses;
}

// Where the cameras' axes meet: a ring's subject, whichever way each camera is
// turned about its axis; the fallback for one camera, for parallel axes, and
// for axes that meet behind a camera.
int axes_meets() {
  const glm::vec3 fallback(9.0f, 9.0f, 9.0f);
  CHECK(near(fuse_viewer::axes_meet(ring(), fallback),
             glm::vec3(0.1f, 0.2f, 1.0f)));
  CHECK(fuse_viewer::axes_meet({ring()[0]}, fallback) == fallback);
  CHECK(fuse_viewer::axes_meet({}, fallback) == fallback);
  const std::vector<glm::mat4> parallel = {
      looking({0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, kPlusZ),
      looking({1.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, kPlusZ)};
  CHECK(fuse_viewer::axes_meet(parallel, fallback) == fallback);
  // They meet at (-1, 0, 0), behind the first.
  const std::vector<glm::mat4> behind = {
      looking({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, kPlusZ),
      looking({-1.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, kPlusZ)};
  CHECK(fuse_viewer::axes_meet(behind, fallback) == fallback);
  return 0;
}

// The up the calibration states, passed through as given; where it states
// none, the primary's image up, its -y.
int rig_ups() {
  const glm::mat4 primary =
      on_side(looking({0.0f, -2.0f, 1.6f}, {0.0f, 0.0f, 1.0f}, kPlusZ));
  CHECK(fuse_viewer::rig_up({0.0f, 0.0f, 1.0f}, primary) == kPlusZ);
  CHECK(fuse_viewer::rig_up({0.0f, -2.0f, 0.0f}, primary) ==
        glm::vec3(0.0f, -2.0f, 0.0f));
  CHECK(fuse_viewer::rig_up({}, primary) == -glm::vec3(primary[1]));
  CHECK(fuse_viewer::rig_up({}, glm::mat4(1.0f)) == kMinusY);
  return 0;
}

// The same camera pitched `radians` about its image x, its axis toward its
// image up for a positive pitch.
glm::mat4 pitched(glm::mat4 c2w, float radians) {
  const glm::vec4 y = c2w[1];
  const glm::vec4 z = c2w[2];
  c2w[1] = std::cos(radians) * y + std::sin(radians) * z;
  c2w[2] = std::cos(radians) * z - std::sin(radians) * y;
  return c2w;
}

// The home view about +z, -y and an oblique up of any length, for a lone
// camera on its side pitched 30 degrees down (the lab rig's, in a tag's
// world), one looking straight down, the image's right along +y, and pitched
// 1e-4 rad either way, the identity and a ring whose primary is on its side:
// the target where the axes meet, the eye behind it along the primary's
// optical axis at the primary's distance, at least 0.3 m, looking along that
// axis; the picture upright about the up, its right across the axis and the
// up, the eye's pitch the primary's; and at a pole, where no picture is
// upright, the primary's own, its right the image's, however little the
// camera leans off the pole.
int home_views() {
  const float pitch = 0.5235988f;  // 30 degrees
  const glm::vec3 lab_eye(0.3f, -1.5f, 1.6f);
  const glm::mat4 lab = on_side(looking(
      lab_eye, lab_eye + glm::vec3(0.0f, std::cos(pitch), -std::sin(pitch)),
      kPlusZ));
  glm::mat4 down(1.0f);  // x along +y, y down the image, z straight down
  down[0] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
  down[1] = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  down[2] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
  down[3] = glm::vec4(0.2f, 0.1f, 2.0f, 1.0f);
  struct Rig {
    std::vector<glm::mat4> poses;
    std::size_t primary;
  };
  const std::vector<Rig> rigs = {{{lab}, 0},
                                 {{down}, 0},
                                 {{pitched(down, 1e-4f)}, 0},
                                 {{pitched(down, -1e-4f)}, 0},
                                 {{glm::mat4(1.0f)}, 0},
                                 {ring(), 1}};
  const std::vector<glm::vec3> ups = {kPlusZ, kMinusY,
                                      glm::vec3(0.6f, -1.6f, 1.0f)};
  const float depth_mid = 1.5f;
  for (const glm::vec3& up : ups) {
    const glm::vec3 unit = glm::normalize(up);
    for (const Rig& rig : rigs) {
      const glm::mat4& c2w = rig.poses[rig.primary];
      const glm::vec3 eye(c2w[3]);
      const glm::vec3 forward = glm::normalize(glm::vec3(c2w[2]));
      const vgc::OrbitCamera home =
          fuse_viewer::home_view(rig.poses, rig.primary, up, depth_mid);
      CHECK(near(home.world_up(), unit, 1e-6f));
      CHECK(near(home.target(),
                 fuse_viewer::axes_meet(rig.poses, eye + forward * depth_mid),
                 1e-6f));
      CHECK(near(home.distance(),
                 std::max(0.3f, glm::length(home.target() - eye)), 1e-6f));
      CHECK(near(home.eye(), home.target() - forward * home.distance()));
      const glm::mat4 view = home.view();
      CHECK(near(row(view, 2), -forward));
      if (std::fabs(glm::dot(forward, unit)) < 0.999f) {
        CHECK(near(row(view, 0), glm::normalize(glm::cross(forward, unit))));
        CHECK(near(home.elevation(), std::asin(glm::dot(-forward, unit))));
      } else {
        CHECK(near(row(view, 0), glm::vec3(c2w[0])));
      }
    }
  }
  return 0;
}

// With no calibration file every camera sits at the origin, the identity, and
// the up is -y: the home view is the one before gfx's OrbitCamera, a look-at
// from behind the target along +z, image up -y, at either distance.
int no_calibration() {
  const std::vector<glm::mat4> poses(3, glm::mat4(1.0f));
  const float vfov = 0.9f;
  const float aspect = 16.0f / 9.0f;
  for (const float depth_mid : {1.4f, 0.2f}) {
    const vgc::OrbitCamera home =
        fuse_viewer::home_view(poses, 0, kMinusY, depth_mid);
    const glm::vec3 target(0.0f, 0.0f, depth_mid);
    const float distance = std::max(0.3f, depth_mid);
    const glm::mat4 before = vgc::Camera::look_at_perspective(
                                 target - glm::vec3(0.0f, 0.0f, distance),
                                 target, kMinusY, vfov, aspect, 0.05f, 10.0f)
                                 .view_proj();
    CHECK(near(home.to_camera(vfov, aspect, 0.05f, 10.0f).view_proj(), before,
               1e-5f));
  }
  return 0;
}

// The light follows the eye: (-0.3, 0.4, 1) in view space, normalized,
// whatever the view's position, at any angle, upside down too.
int headlights() {
  const glm::vec3 in_view = glm::normalize(glm::vec3(-0.3f, 0.4f, 1.0f));
  CHECK(near(fuse_viewer::headlight(glm::mat4(1.0f)), in_view, 1e-6f));
  vgc::OrbitCamera orbit;
  orbit.set_world_up(kPlusZ);
  orbit.set_target({0.4f, -0.2f, 1.0f});
  orbit.set_distance(2.5f);
  orbit.set_azimuth(0.7f);
  for (const float elevation : {0.3f, 2.5f}) {  // upright, then past the pole
    orbit.set_elevation(elevation);
    const glm::mat4 view = orbit.view();
    const glm::vec3 light = fuse_viewer::headlight(view);
    CHECK(near(glm::mat3(view) * light, in_view, 1e-5f));
    CHECK(near(light, glm::transpose(glm::mat3(view)) * in_view, 1e-5f));
    CHECK(
        near(fuse_viewer::headlight(glm::mat4(glm::mat3(view))), light, 1e-6f));
  }
  return 0;
}

}  // namespace

int main() {
  CHECK(axes_meets() == 0);
  CHECK(rig_ups() == 0);
  CHECK(home_views() == 0);
  CHECK(no_calibration() == 0);
  CHECK(headlights() == 0);
  std::puts("example_viewer_orbit: OK");
  return 0;
}
