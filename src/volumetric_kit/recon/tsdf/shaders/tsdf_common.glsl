// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Shared definitions for the TSDF compute kernels: the device struct layouts
// (scalar block layout, byte-identical to the host POD structs) and the
// push-constant block. #included by tsdf_integrate.glsl, the body of the
// integrate and clear kernels.
//
// VoxelGridParams / BlockIndex mirror volume/{voxel_grid,hash_types}.hpp (the
// same scalar-layout structs the volume kernels use in hash_common.glsl);
// IntegrateParams mirrors the host struct in tsdf_integrator.cpp. Under scalar
// block layout every field lands at its host offset -- no std430 vec padding.

#extension GL_EXT_scalar_block_layout : require

#include "volumetric_kit/recon/core/shaders/pinhole.glsl"

struct VoxelGridParams {
  float voxel_size;
  int block_size;
  int voxels_per_block;
  float trunc_dist;
  int bucket_size;
  int num_buckets;
  int num_blocks;
  int max_chain;
};

struct BlockIndex {
  ivec3 coord;
  int ptr;
};

// Camera intrinsics + pose + depth range (mirrors DepthCameraParams
// byte-for-byte: scalars at their 4-byte offsets, the cam_to_world mat4 at 32).
// The integrate kernel derives world -> camera from the rigid cam_to_world, so
// the pose is passed straight through -- no separate pre-inverted struct.
struct DepthCameraParams {
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

// The (separate) color camera, mirroring ColorCameraParams byte-for-byte:
// the color-camera analogue of DepthCameraParams without the (color-irrelevant)
// depth-range fields, so width/height sit at offset 16 and the mat4 at 24
// (scalar layout, 88 bytes).
struct ColorCameraParams {
  float fx;
  float fy;
  float cx;
  float cy;
  uint width;
  uint height;
  mat4 cam_to_world;
};

// Project a world point into a pinhole camera given its intrinsics + rigid
// cam_to_world pose: core/shaders/pinhole.glsl's vr_world_to_camera, then its
// vr_project_pinhole, under its rules. Also sets `zc`, the camera-space depth
// (metres). The two project_to_image overloads adapt the depth and color
// camera structs onto it.
bool project_pinhole(mat4 cam_to_world, float fx, float fy, float cx, float cy,
                     uint width, uint height, vec3 world, out vec2 px,
                     out float zc) {
  vec3 p_cam = vr_world_to_camera(cam_to_world, world);
  zc = p_cam.z;
  return vr_project_pinhole(p_cam, vec4(fx, fy, cx, cy), uvec2(width, height),
                            px);
}

// vr_nearest_pixel for a camera's width and height.
uvec2 nearest_pixel(vec2 px, uint width, uint height) {
  return vr_nearest_pixel(px, uvec2(width, height));
}

bool project_to_image(DepthCameraParams c, vec3 world, out vec2 px,
                      out float zc) {
  return project_pinhole(c.cam_to_world, c.fx, c.fy, c.cx, c.cy, c.width,
                         c.height, world, px, zc);
}

bool project_to_image(ColorCameraParams c, vec3 world, out vec2 px,
                      out float zc) {
  return project_pinhole(c.cam_to_world, c.fx, c.fy, c.cx, c.cy, c.width,
                         c.height, world, px, zc);
}

// Integration mode (mirrors tsdf::IntegrationMode): for free space ahead of the
// surface (sdf > trunc_dist), classic keeps the voxel (clamped to +trunc) while
// dynamic clears any stale geometry there.
const uint kModeClassic = 0u;
const uint kModeDynamic = 1u;

layout(push_constant, scalar) uniform PushConstants {
  VoxelGridParams grid;
  uint num_active_blocks;
  float max_weight;
  uint mode;
  uint has_color;       // 0 = depth only; 1 = also fuse the color frame
  uint has_color_attr;  // 1 = binding 6 is the grid's color attribute (clearable)
  uint tick;            // the map's tick, which a changed block is stamped with
  uint coverage_in_alpha;  // 1 = a colour word with a zero high byte is no colour
  // 1 = a Dynamic set's fuse, whose clears tsdf_clear.comp ran for every frame
  // of the set first: free space past the band is left alone.
  uint fuse_only;
} pc;
