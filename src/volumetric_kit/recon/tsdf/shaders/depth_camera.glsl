// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The depth camera every tsdf kernel reads: intrinsics + pose + depth range,
// mirroring DepthCameraParams byte-for-byte under scalar block layout (scalars
// at their 4-byte offsets, the cam_to_world mat4 at 32; 96 bytes). One
// definition for the tier's kernels, so none can drift from the others.

#ifndef VR_TSDF_DEPTH_CAMERA_GLSL
#define VR_TSDF_DEPTH_CAMERA_GLSL

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

#endif
