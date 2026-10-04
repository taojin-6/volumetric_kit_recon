// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Shared by AdaptiveGrid's residual and mask kernels: the unprojection
// integration agrees with. The includer declares the Depth binding (`depth[]`)
// before including this, and binds a DepthCameraParams.

#ifndef ADAPTIVE_COMMON_GLSL
#define ADAPTIVE_COMMON_GLSL

#include "depth_camera.glsl"

// Camera-space point of pixel (u, v); false for invalid depth. Zero is a
// missing measurement, even when the near bound is 0. Pixel u's depth sits at
// u + 0.5, where integration samples it.
bool unproject(DepthCameraParams c, uint u, uint v, out vec3 p) {
  p = vec3(0.0);
  if (u >= c.width || v >= c.height) return false;
  float d = depth[v * c.width + u];
  if (!(d > 0.0 && d >= c.min_depth && d <= c.max_depth)) return false;
  p = vec3((float(u) + 0.5 - c.cx) * d / c.fx,
           (float(v) + 0.5 - c.cy) * d / c.fy, d);
  return true;
}

// The block whose marching-cubes cells hold world point w: block b meshes
// voxels [8b, 8b + 8), so this floors rather than rounding to a node.
ivec3 region_block(vec3 w, float voxel_size) {
  return ivec3(floor(w / (8.0 * voxel_size)));
}

#endif
