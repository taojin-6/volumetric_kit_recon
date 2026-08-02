// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Shared definitions for the projective-texturing compute kernel: the device
// struct layouts (scalar block layout, byte-identical to the host POD structs),
// the pinhole projection, and the push-constant block. #included by
// texture_score.comp.
//
// DepthCameraParams mirrors DepthCameraParams byte-for-byte (the same
// scalar-layout camera the volume/tsdf kernels use); Vertex mirrors
// mesh::Vertex. project_to_image is the same world -> camera -> pixel projection
// tsdf_common.glsl's project_pinhole computes -- kept a self-contained copy here
// so this tier's shaders vendor no cross-tier include, matching how each tier's
// GLSL restates the small structs/helpers it needs (hash_common.glsl /
// tsdf_common.glsl). Under scalar block layout every field lands at its host
// offset (no std430 vec padding).

#extension GL_EXT_scalar_block_layout : require

// Camera intrinsics + pose + depth range (mirrors DepthCameraParams:
// scalars at their 4-byte offsets, the cam_to_world mat4 at 32). world -> camera
// is derived from the rigid cam_to_world, so the pose is passed straight through.
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

// One mesh vertex (mirrors mesh::Vertex byte-for-byte: position@0, normal@12,
// color@24, uv0@40, 48 B). The kernel reads `position` and overwrites `uv0`.
struct Vertex {
  vec3 position;
  vec3 normal;
  vec4 tangent;  // renderer's slot; meshing has no parameterisation for one
  vec2 uv0;
  vec4 color;
};

// Project a world point into a pinhole camera given its intrinsics + rigid
// cam_to_world pose. Returns true and sets `px` (pixel coords, in
// [0,width)x[0,height)) and `zc` (camera-space depth, metres) when the point is
// in front of the camera and inside the image; false otherwise. world -> camera
// is R^T (world - t) -- the rigid inverse, no explicit mat4 inverse -- which
// equals the prior engine's precomputed extrinsics_inv * (world, 1). Pinhole
// u = fx*x/z + cx, v = fy*y/z + cy (OpenCV +X right / +Y down / +Z forward; no
// y-flip), matching the prior engine's project_to_pixel and tsdf_common.glsl.
bool project_to_image(DepthCameraParams c, vec3 world, out vec2 px,
                      out float zc) {
  mat3 rot = mat3(c.cam_to_world);
  vec3 t = c.cam_to_world[3].xyz;
  vec3 p_cam = transpose(rot) * (world - t);
  zc = p_cam.z;
  if (zc <= 0.0) {
    return false;  // behind the camera
  }
  float u = c.fx * (p_cam.x / zc) + c.cx;
  float v = c.fy * (p_cam.y / zc) + c.cy;
  if (u < 0.0 || u >= float(c.width) || v < 0.0 || v >= float(c.height)) {
    return false;  // outside the image
  }
  px = vec2(u, v);
  return true;
}

layout(push_constant, scalar) uniform PushConstants {
  uint num_triangles;
  float occlusion_threshold;  // max |projected depth - sensor depth|, metres
  uint num_vertices;          // vertices[] length; bounds the index -> vertex read
} pc;
