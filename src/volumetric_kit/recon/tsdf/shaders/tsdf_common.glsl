// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Shared definitions for the TSDF compute kernels: the device struct layouts
// (scalar block layout, byte-identical to the host POD structs) and the
// push-constant block. #included by tsdf_integrate.comp.
//
// VoxelGridParams / BlockIndex mirror volume/{voxel_grid,hash_types}.hpp (the
// same scalar-layout structs the volume kernels use in hash_common.glsl);
// IntegrateParams mirrors the host struct in tsdf_integrator.cpp. Under scalar
// block layout every field lands at its host offset -- no std430 vec padding.

#extension GL_EXT_scalar_block_layout : require

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

// Camera intrinsics + pose + depth range (mirrors volume::DepthCameraParams
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

layout(push_constant, scalar) uniform PushConstants {
  VoxelGridParams grid;
  uint num_active_blocks;
  float max_weight;
} pc;
