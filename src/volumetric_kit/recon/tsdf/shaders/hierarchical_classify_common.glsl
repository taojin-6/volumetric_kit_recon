// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#extension GL_GOOGLE_include_directive : require
#define VR_HASH_ENTRIES_BINDING 4
#define VR_HIERARCHY_NODES_BINDING 3
#include "volumetric_kit/recon/volume/shaders/hierarchical_lookup.glsl"

layout(set = 0, binding = 0, scalar) readonly buffer Depth { float depth[]; };
layout(set = 0, binding = 1, scalar) readonly buffer Camera {
  float fx;
  float fy;
  float cx;
  float cy;
  float min_depth;
  float max_depth;
  uint width;
  uint height;
  mat4 cam_to_world;
} cam;
layout(push_constant, scalar) uniform Push {
  VoxelGridParams root_grid;
  float finest_voxel_size;
  float surface_error;
  float noise_floor;
  uint max_level;
  uint node_capacity;
  uint pixel_stride;
  uint patch_radius;
  uint columns;
  uint sample_count;
  uint first_leaf;
} pc;

bool insideImage(ivec2 pixel) {
  return all(greaterThanEqual(pixel, ivec2(0))) &&
      pixel.x < int(cam.width) && pixel.y < int(cam.height);
}

bool pointAt(ivec2 pixel, out vec3 point) {
  if (!insideImage(pixel)) return false;
  float d = depth[uint(pixel.y) * cam.width + uint(pixel.x)];
  if (!(d > 0.0 && d >= cam.min_depth && d <= cam.max_depth)) return false;
  point = vec3((float(pixel.x) + 0.5 - cam.cx) * d / cam.fx,
               (float(pixel.y) + 0.5 - cam.cy) * d / cam.fy, d);
  return true;
}

// A valid center beside missing depth returns a finest request but does not
// supply a smooth patch suitable for coarsening.
bool patchLevel(ivec2 pixel, out uint desired, out bool is_smooth) {
  is_smooth = false;
  vec3 center;
  if (!pointAt(pixel, center)) return false;
  int radius = int(pc.patch_radius);
  vec3 points[8];
  const ivec2 shifts[8] = ivec2[8](ivec2(-1, 0), ivec2(1, 0),
      ivec2(0, -1), ivec2(0, 1), ivec2(-1, -1), ivec2(1, -1),
      ivec2(-1, 1), ivec2(1, 1));
  // An image boundary supplies no surrounding evidence. An invalid depth
  // inside the image next to a valid foreground point is a silhouette and
  // must retain detail, even though a tangent plane cannot be estimated.
  for (int i = 0; i < 8; ++i) {
    if (!insideImage(pixel + radius * shifts[i])) return false;
  }
  bool discontinuity = false;
  for (int i = 0; i < 8; ++i) {
    if (!pointAt(pixel + radius * shifts[i], points[i])) {
      desired = 0u;
      return true;
    }
    discontinuity = discontinuity ||
        abs(points[i].z - center.z) > pc.root_grid.trunc_dist;
  }
  vec3 normal = cross(points[1] - points[0], points[3] - points[2]);
  float magnitude = length(normal);
  if (!(magnitude > 1e-10)) return false;
  normal /= magnitude;
  float residual = 0.0;
  float radius2 = 0.0;
  for (int i = 0; i < 8; ++i) {
    vec3 delta = points[i] - center;
    residual = max(residual, abs(dot(normal, delta)));
    radius2 = max(radius2, dot(delta, delta));
  }
  residual = max(0.0, residual - pc.noise_floor);
  // Large camera-Z slope alone also occurs on a grazing plane. Require
  // measured nonplanarity before treating it as a depth discontinuity.
  discontinuity = discontinuity && residual > pc.surface_error;
  desired = 0u;
  is_smooth = !discontinuity;
  if (!discontinuity) {
    // A smooth patch's departure from its tangent plane is quadratic in
    // length. Choose the coarsest candidate that fits this local estimate.
    // A flat patch can still lose its negative samples at grazing incidence:
    // the camera-Z truncation band becomes narrow in the surface-normal
    // direction. Bound one cell's normal span by that observed band width.
    // Level zero is the available fallback even when it cannot meet the bound.
    vec3 world_normal = mat3(cam.cam_to_world) * normal;
    float normal_span = dot(abs(world_normal), vec3(1.0));
    float observed_band = pc.root_grid.trunc_dist *
        abs(dot(normal, center / center.z));
    for (uint level = 1u; level <= pc.max_level; ++level) {
      float spacing = pc.finest_voxel_size * float(1u << level);
      float estimate = residual * spacing * spacing / max(radius2, 1e-12);
      if (estimate <= pc.surface_error && spacing * normal_span <= observed_band) desired = level;
    }
  }
  return true;
}
