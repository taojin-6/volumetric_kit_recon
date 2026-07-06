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

// Per-dispatch camera + fusion parameters (mirrors IntegrateParams host struct:
// scalars at their 4-byte offsets, the mat4 at offset 36).
struct IntegrateParams {
  float fx;
  float fy;
  float cx;
  float cy;
  float min_depth;
  float max_depth;
  float max_weight;
  uint width;
  uint height;
  mat4 world_to_cam;
};

layout(push_constant, scalar) uniform PushConstants {
  VoxelGridParams grid;
  uint num_active_blocks;
} pc;
