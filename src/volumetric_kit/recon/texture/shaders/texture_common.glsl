// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Shared definitions for the projective-texturing compute kernels: the device
// struct layouts (scalar block layout, byte-identical to the host POD structs),
// the pinhole projection, the depth binding and the occlusion test that reads
// it. #included by texture_score.comp (one camera, per vertex) and
// texture_multiview.comp (several, per triangle), each of which declares its
// own push-constant block and its other bindings.
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

// A world point in camera space: R^T (world - t), the rigid inverse of
// cam_to_world.
vec3 world_to_camera(DepthCameraParams c, vec3 world) {
  return transpose(mat3(c.cam_to_world)) * (world - c.cam_to_world[3].xyz);
}

// project_to_image's divide, for a point already in camera space -- which is
// how texture_multiview.comp calls it, having transformed each vertex once for
// both the facing test and the projection.
bool camera_to_image(DepthCameraParams c, vec3 p_cam, out vec2 px) {
  // Negated rather than `p_cam.z <= 0.0`, so a NaN fails it: every comparison
  // with NaN is false, so the direct form would ACCEPT a non-finite depth and
  // hand the caller a NaN pixel. That matters more than it used to. The
  // sentinel is computed from `px` now (`-uv - 1`) instead of written as a
  // literal, and a NaN uv is not negative, so `uv0.x < 0` -- the renderer's
  // whole class test -- would read it as textured. It would also reach
  // `occluded_ok`, whose bounds test NaN passes for the same reason, and
  // `sample_depth` indexes depth[] off a NaN pixel with no bound left to catch
  // it.
  if (!(p_cam.z > 0.0)) {
    return false;  // behind the camera or non-finite: no projection exists
  }
  px = vec2(c.fx * (p_cam.x / p_cam.z) + c.cx,
            c.fy * (p_cam.y / p_cam.z) + c.cy);
  // A finite depth does not make the pixel finite -- a non-finite x or y in
  // the position survives the divide with the depth intact (an identity pose
  // puts a NaN x straight through to u while z stays 1). An infinity is fine
  // and deliberately allowed: the atlas-coordinate helpers clamp it to the
  // border like any far-outside projection. A NaN is not, for the reason
  // above.
  return !isnan(px.x) && !isnan(px.y);
}

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
  vec3 p_cam = world_to_camera(c, world);
  zc = p_cam.z;
  return camera_to_image(c, p_cam, px);
}

// Binding 1 in both kernels: the depth the occlusion test reads -- the one
// camera's map, or every view's end to end with each view's `base` offset.
// Declared here, beside the sampler that reads it, so the two kernels share
// one sampler rather than keeping two copies in step.
layout(set = 0, binding = 1, scalar) readonly buffer Depth { float depth[]; };

// Bilinear depth sample at pixel `px` of camera `c`'s map (starting at
// depth[base]), in metres, excluding invalid (<= 0) taps from the blend and
// renormalizing by the surviving weight -- so a hole (sensor no-return, 0)
// next to a valid reading does not pull the result toward 0 and cause a false
// occlusion. When the valid taps straddle a depth discontinuity (their range
// exceeds `threshold`, the occlusion tolerance), the 2x2 spans a
// foreground/background surface edge and blending would yield a phantom
// mid-depth that fails the occlusion test on a genuinely visible foreground
// vertex; fall back to the nearest (max-weight) valid tap instead, the same
// intent as tsdf_integrate.comp's discontinuity guard (keyed here to the
// occlusion tolerance -- the scale at which the blend must be trustworthy --
// since this tier carries no trunc_dist). Sampling is integer-centred,
// matching this tier's integer-centred projection (u = fx*x/z + cx) and
// half-texel atlas UV; it deliberately does NOT copy the tsdf sampler's -0.5
// texture-centred tap shift, which is self-consistent only with that tier's
// texture-centred convention (DECISIONS.md, the 2026-07-06 depth-sampling
// decision). Ported in spirit from the prior engine's sample_depth_bilinear_m
// (our depth is already float metres, so there is no uint16 depth-scale
// divide). Returns 0.0 when all four taps are invalid; occluded_ok's range
// check then rejects it.
float sample_depth(DepthCameraParams c, uint base, vec2 px, float threshold) {
  int w = int(c.width);
  int h = int(c.height);
  float cu = clamp(px.x, 0.0, float(w - 1));
  float cv = clamp(px.y, 0.0, float(h - 1));
  int x0 = int(floor(cu));
  int y0 = int(floor(cv));
  int x1 = min(x0 + 1, w - 1);
  int y1 = min(y0 + 1, h - 1);
  float tx = cu - float(x0);
  float ty = cv - float(y0);

  float sm[4] = float[4](
      depth[base + uint(y0 * w + x0)], depth[base + uint(y0 * w + x1)],
      depth[base + uint(y1 * w + x0)], depth[base + uint(y1 * w + x1)]);
  float wts[4] = float[4]((1.0 - tx) * (1.0 - ty), tx * (1.0 - ty),
                          (1.0 - tx) * ty, tx * ty);

  // One pass: blend the valid (> 0) taps, and track their range [lo, hi] and
  // the nearest (max-weight) valid tap for the discontinuity fallback.
  float total = 0.0;
  float result = 0.0;
  float lo = 3.4e38;  // ~FLT_MAX; first valid tap lowers it
  float hi = 0.0;     // depths are positive; first valid tap raises it
  float d_near = 0.0;
  float best_w = -1.0;
  for (int i = 0; i < 4; i++) {
    if (sm[i] > 0.0) {  // a comparison with NaN is false, so NaN taps drop too
      result += wts[i] * sm[i];
      total += wts[i];
      lo = min(lo, sm[i]);
      hi = max(hi, sm[i]);
      if (wts[i] > best_w) {
        best_w = wts[i];
        d_near = sm[i];
      }
    }
  }
  if (total <= 0.0) {
    return 0.0;  // no valid tap; occluded_ok's range check rejects this
  }
  if ((hi - lo) > threshold) {
    return d_near;  // depth discontinuity: nearest tap, do not blend fg + bg
  }
  return result / total;
}

// True when the sensor surface at `px` sits within `threshold` of the point's
// camera-space depth `zc` -- i.e. the point is on the visible surface, not
// hidden behind nearer geometry -- with `diff` the disagreement, which the
// several-view score weighs. Uses a 1-px inset so the bilinear taps never
// straddle the image border (the prior engine's occlusion-bounds margin), and
// rejects a hole / NaN / inf / out-of-range depth via the sensor range (the
// negated compare drops non-finite depth).
bool occluded_ok(DepthCameraParams c, uint base, vec2 px, float zc,
                 float threshold, out float diff) {
  diff = 0.0;
  // Negated rather than `px.x < 1.0 || ...`, which is the same work and
  // rejects a NaN instead of admitting one: every comparison with NaN is
  // false, so the direct form falls THROUGH to sample_depth, whose clamp and
  // floor are undefined on a NaN and whose depth[] index has nothing further
  // to bound it. project_to_image refuses a non-finite pixel before this is
  // reached, so the form is what keeps the predicate true on its own terms
  // rather than only on its caller's.
  if (!(px.x >= 1.0 && px.x < float(int(c.width) - 1) && px.y >= 1.0 &&
        px.y < float(int(c.height) - 1))) {
    return false;
  }
  float d = sample_depth(c, base, px, threshold);
  if (!(d >= c.min_depth && d <= c.max_depth)) {
    return false;  // hole / non-finite / out-of-range: no line of sight proof
  }
  diff = abs(d - zc);
  return diff <= threshold;
}
