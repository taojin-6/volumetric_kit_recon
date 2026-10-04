// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Shared by the adaptive viewer's residual and mask kernels: the depth camera
// and the unprojection integration agrees with. The includer declares the
// Depth binding (`depth[]`) and the Camera binding before including this.

#ifndef ADAPTIVE_COMMON_GLSL
#define ADAPTIVE_COMMON_GLSL

// Mirrors DepthCameraParams (96 bytes, scalar).
struct AdaptiveCamera {
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

// Camera-space point of pixel (u, v); false for invalid depth. Pixel u's depth
// sits at u + 0.5, where integration samples it.
bool unproject(AdaptiveCamera c, uint u, uint v, out vec3 p) {
  p = vec3(0.0);
  if (u >= c.width || v >= c.height) return false;
  float d = depth[v * c.width + u];
  if (!(d >= c.min_depth && d <= c.max_depth)) return false;
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
