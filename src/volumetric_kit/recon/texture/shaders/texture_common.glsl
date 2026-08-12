// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Shared definitions for the projective-texturing compute kernel: the device
// struct layouts (scalar block layout, byte-identical to the host POD structs),
// the pinhole projection, and the push-constant block. #included by
// texture_score.comp.
//
// DepthCameraParams mirrors DepthCameraParams byte-for-byte (the same
// scalar-layout camera the volume/tsdf kernels use); Vertex mirrors
// mesh::Vertex. Under scalar block layout every field lands at its host offset
// (no std430 vec padding).
//
// project_to_image computes the same world -> camera -> pixel arithmetic as
// tsdf_common.glsl's project_pinhole -- a self-contained copy, so this tier's
// shaders vendor no cross-tier include, matching how each tier's GLSL restates
// the small structs/helpers it needs (hash_common.glsl / tsdf_common.glsl).
//
// It is NOT interchangeable with that one, and the difference is deliberate:
// this copy accepts a point in front of the camera whose pixel lands outside
// the image and returns the extrapolated coordinate, where project_pinhole
// rejects it. See the contract on the function. Do not reconcile the two by
// copying either into the other -- tsdf's version here restores the
// frustum-edge smear this tier's encoding exists to prevent, and this version
// there makes integration fuse depth at pixels the sampler never validated.
//
// This header holds STRUCTS AND PURE FUNCTIONS ONLY -- no bindings, and no
// push-constant block. Each kernel declares its own `pc`, because the two in
// this tier need different fields and a block name may not be reused within one
// interface: a shared `pc` here made the second kernel that included it fail to
// compile with an error naming the FIRST one's members.

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
// tangent@24, uv0@40, color@48, 64 B -- the renderer's layout since the
// 2026-08-02 decision). The kernel reads `position` and overwrites `uv0`.
struct Vertex {
  vec3 position;
  vec3 normal;
  vec4 tangent;  // renderer's slot; meshing has no parameterisation for one
  vec2 uv0;
  vec4 color;
};

// Project a world point into a pinhole camera given its intrinsics + rigid
// cam_to_world pose, setting `px` (pixel coords) and `zc` (camera-space depth,
// metres). world -> camera is R^T (world - t) -- the rigid inverse, no explicit
// mat4 inverse -- which equals the prior engine's precomputed
// extrinsics_inv * (world, 1). Pinhole u = fx*x/z + cx, v = fy*y/z + cy (OpenCV
// +X right / +Y down / +Z forward; no y-flip), matching the prior engine's
// project_to_pixel and tsdf_common.glsl.
///
/// Returns false when the point is not strictly in front of the camera, where
/// the pinhole divide has no meaning, or when the pixel it lands on is not a
/// number -- so a caller that gets `true` holds a usable coordinate. Both
/// tests are written so a non-finite input FAILS them rather than slipping
/// through: every comparison with NaN is false, so the direct forms would have
/// accepted one. A point in front whose pixel lands outside the image still
/// returns true, with `px` extrapolated past the border.
///
/// That is deliberate, and it is what keeps a triangle straddling the frustum
/// edge from sweeping the whole atlas. The caller needs a *coordinate* for such
/// a vertex even though it will not be textured: it classes it as vertex-colour
/// either way, but the uv it carries is interpolated across any triangle whose
/// provoking vertex IS textured. Refusing to project here left those vertices
/// carrying the bare sentinel, which the renderer reads as (0, 0) -- so a
/// boundary triangle interpolated from a real uv near the image edge all the
/// way to the atlas origin, drawing the entire camera image inside one
/// triangle, repeated along the whole frustum boundary.
///
/// The bounds test does not disappear; it moves to the callers that need it.
/// `occluded_ok` rejects a pixel within one texel of the border (its bilinear
/// taps would straddle it), so visibility is still refused outside the image --
/// and `pixel_to_atlas_uv` clamps the extrapolated coordinate half a texel
/// inside, so what a boundary triangle actually samples is the edge of the
/// image stretched, which is what projective texturing is expected to do there.
bool project_to_image(DepthCameraParams c, vec3 world, out vec2 px,
                      out float zc) {
  mat3 rot = mat3(c.cam_to_world);
  vec3 t = c.cam_to_world[3].xyz;
  vec3 p_cam = transpose(rot) * (world - t);
  zc = p_cam.z;
  // Negated rather than `zc <= 0.0`, so a NaN fails it: every comparison with
  // NaN is false, so the direct form would ACCEPT a non-finite depth and hand
  // the caller a NaN pixel. That matters more than it used to. The sentinel is
  // computed from `px` now (`-uv - 1`) instead of written as a literal, and a
  // NaN uv is not negative, so `uv0.x < 0` -- the renderer's whole class test
  // -- would read it as textured. It would also reach `occluded_ok`, whose
  // bounds test NaN passes for the same reason, and `sample_depth` indexes
  // depth[] off a NaN pixel with no bound left to catch it.
  if (!(zc > 0.0)) {
    return false;  // behind the camera or non-finite: no projection exists
  }
  px = vec2(c.fx * (p_cam.x / zc) + c.cx, c.fy * (p_cam.y / zc) + c.cy);
  // A finite `zc` does not make the pixel finite -- a non-finite x or y in the
  // position survives the divide with the depth intact (an identity pose puts
  // a NaN x straight through to u while z stays 1). An infinity is fine and
  // deliberately allowed: `pixel_to_atlas_uv` clamps it to the border like any
  // far-outside projection. A NaN is not, for the reason above.
  return !isnan(px.x) && !isnan(px.y);
}
