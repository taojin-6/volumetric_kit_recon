// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The fitter's shared device state (point_to_plane.cpp mirrors every struct
// here byte for byte, scalar layout). A fit of K frames: view i is frame i,
// whose pyramid sits in entry views[i].entry of the fitter's stacked levels and
// whose model view is view i of the ray caster's stack.
//
// Twists are (v, w) about the frames' mean camera centre o, in world axes: a
// motion X -> exp(w) (X - o) + o + v. The solve's unknowns scale v by 1 / s
// (s the points' median distance from o), so both halves are angles.

#ifndef VR_TRACK_COMMON_GLSL
#define VR_TRACK_COMMON_GLSL

#extension GL_EXT_scalar_block_layout : require

#include "volumetric_kit/sensor/frame/shaders/pinhole.glsl"

// The frame's depth noise, sigma(z) = a + b z + c z^2 metres (sensor's
// DepthNoise::sigma).
float vr_track_sigma(vec3 abc, float z) {
  return abc.x + z * (abc.y + z * abc.z);
}

// One frame of a fit. Mirrors FitView (112 bytes).
struct FitView {
  mat4 cam_to_world;  // the frame's own pose: the device's
  vec4 k;             // level-0 fx, fy, cx, cy
  vec3 noise;         // its DepthNoise a, b, c
  uint entry;         // where its pyramid is
  float min_depth;
  float max_depth;
  uint pad0;
  uint pad1;
};

// grid's DepthPinhole (96 bytes): the cast cameras.
struct CastCamera {
  float fx;
  float fy;
  float cx;
  float cy;
  float min_depth;
  float max_depth;
  uint width;
  uint height;
  mat4 cam_to_world;
};

// The fit's state. Mirrors FitState (444 bytes).
struct FitState {
  mat4 correction;  // the estimate
  mat4 prior;       // the prior's correction: the prior term's mean
  mat4 pass_start;  // the estimate when the current pass began
  vec3 origin;      // o
  float scale;      // s (metres)
  vec2 info;        // the prior's 1 / sigma_m^2, 1 / sigma_rad^2; 0: none
  float n_eff;      // points a frame with hits counts for against the prior
  uint frames;
  uint serial_lo;   // the record's serial, which the readback carries
  uint serial_hi;
  uint stop;        // a pass moved under the bar: no more solves
  uint diverged;    // a step over the guard: the estimate is the last good
  // The finest level's last evaluation.
  float attempts;
  float hits;
  float inliers;
  float in_front;
  float behind;
  float raw2;       // the inliers' squared residuals, summed
  uint evaluated;   // 1 once one ran
  float lambda_max;
  float rel[6];     // ascending
  float dirs[36];   // column-major, in rel's order
  uint held;
};

// What a reduce workgroup leaves for the solve (track_reduce_common.glsl).
const uint kPartial = 33u;
const uint kH = 0u;          // 21: H's upper triangle, row by row
const uint kG = 21u;         // 6: J^T W r
const uint kRaw2 = 27u;      // sum r^2 over the inliers
const uint kAttempts = 28u;  // valid frame pixels
const uint kHits = 29u;      // of them, the model answered
const uint kInliers = 30u;   // of the hits, inside every gate
const uint kInFront = 31u;   // of the hits, in front of the surface
const uint kBehind = 32u;    // of the hits, behind it

// The world-from-world motion of a twist about o: exp(w) (X - o) + o + v.
mat3 vr_track_so3_exp(vec3 w) {
  float th = length(w);
  // Column-major: K * x = cross(w, x).
  mat3 k = mat3(0.0, w.z, -w.y, -w.z, 0.0, w.x, w.y, -w.x, 0.0);
  if (th < 1e-8) {
    return mat3(1.0) + k;
  }
  k /= th;
  return mat3(1.0) + sin(th) * k + (1.0 - cos(th)) * (k * k);
}

vec3 vr_track_so3_log(mat3 r) {
  float c = clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) * 0.5, -1.0, 1.0);
  float th = acos(c);
  vec3 v = vec3(r[1][2] - r[2][1], r[2][0] - r[0][2], r[0][1] - r[1][0]);
  float s = sin(th);
  if (s > 1e-6) {
    return v * (th / (2.0 * s));
  }
  return v * 0.5;  // a fit's motions are far from pi
}

mat4 vr_track_apply(vec3 v, vec3 w, vec3 o, mat4 m) {
  mat3 rd = vr_track_so3_exp(w);
  vec3 td = o - rd * o + v;
  mat3 rm = mat3(m);
  return mat4(vec4(rd * rm[0], 0.0), vec4(rd * rm[1], 0.0),
              vec4(rd * rm[2], 0.0), vec4(rd * m[3].xyz + td, 1.0));
}

// The twist about o taking `from` to `to`.
void vr_track_between(mat4 from, mat4 to, vec3 o, out vec3 v, out vec3 w) {
  mat3 rd = mat3(to) * transpose(mat3(from));
  vec3 td = to[3].xyz - rd * from[3].xyz;
  w = vr_track_so3_log(rd);
  v = td - o + rd * o;
}

#endif  // VR_TRACK_COMMON_GLSL
