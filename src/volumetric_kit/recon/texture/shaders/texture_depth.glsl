// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The line-of-sight test both kernels in this tier decide visibility with:
// bilinear depth sampling with a discontinuity fallback, and the occlusion
// comparison over it.
//
// INCLUDED AFTER THE BINDINGS, not before, because these read `depth[]` and
// `cam` directly -- so a kernel declares its Depth and Camera bindings first
// and then pulls this in. That is the opposite order from texture_common.glsl,
// which declares structs and a pure projection and so goes at the top.
//
// The tolerance is a PARAMETER rather than a read of `pc`, which is what lets
// one copy serve two kernels whose push blocks are laid out differently. It is
// the one number deciding what "the camera can see it" means, and both callers
// pass their own.

// Bilinear depth sample at pixel (u, v), in metres, excluding invalid (<= 0)
// taps from the blend and renormalizing by the surviving weight -- so a hole
// (sensor no-return, 0) next to a valid reading does not pull the result toward
// 0 and cause a false occlusion. When the valid taps straddle a depth
// discontinuity (their range exceeds `threshold`), the 2x2 spans a
// foreground/background surface edge and blending would yield a phantom
// mid-depth that fails the occlusion test on a genuinely visible foreground
// point; fall back to the nearest (max-weight) valid tap instead, the same
// intent as tsdf_integrate.comp's discontinuity guard (keyed here to the
// occlusion tolerance -- the scale at which the blend must be trustworthy --
// since this tier carries no trunc_dist). Sampling is integer-centred, matching
// this tier's integer-centred projection (u = fx*x/z + cx) and half-texel atlas
// UV; it deliberately does NOT copy the tsdf sampler's -0.5 texture-centred tap
// shift, which is self-consistent only with that tier's texture-centred
// convention (DECISIONS.md, the 2026-07-06 depth-sampling decision). Ported in
// spirit from the prior engine's sample_depth_bilinear_m (our depth is already
// float metres, so there is no uint16 depth-scale divide). Returns 0.0 when all
// four taps are invalid; vrOccludedOk's range check then rejects it.
float vrSampleDepth(float u, float v, float threshold) {
  int w = int(cam.width);
  int h = int(cam.height);
  float cu = clamp(u, 0.0, float(w - 1));
  float cv = clamp(v, 0.0, float(h - 1));
  int x0 = int(floor(cu));
  int y0 = int(floor(cv));
  int x1 = min(x0 + 1, w - 1);
  int y1 = min(y0 + 1, h - 1);
  float tx = cu - float(x0);
  float ty = cv - float(y0);

  float d00 = depth[y0 * w + x0];
  float d10 = depth[y0 * w + x1];
  float d01 = depth[y1 * w + x0];
  float d11 = depth[y1 * w + x1];
  float wts[4] = float[4]((1.0 - tx) * (1.0 - ty), tx * (1.0 - ty),
                          (1.0 - tx) * ty, tx * ty);
  float sm[4] = float[4](d00, d10, d01, d11);

  // One pass: blend the valid (> 0) taps, and track their range [lo, hi] and the
  // nearest (max-weight) valid tap for the discontinuity fallback.
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
    return 0.0;  // no valid tap; vrOccludedOk's range check rejects this
  }
  if ((hi - lo) > threshold) {
    return d_near;  // depth discontinuity: nearest tap, do not blend fg + bg
  }
  return result / total;
}

// True when the sensor surface at `px` sits within `threshold` of the point's
// camera-space depth `zc` -- i.e. the point is on the visible surface, not
// hidden behind nearer geometry. Uses a 1-px inset so the bilinear taps never
// straddle the image border (the prior engine's occlusion-bounds margin), and
// rejects a hole / NaN / inf / out-of-range depth via the sensor range (the
// negated compare drops non-finite depth).
bool vrOccludedOk(vec2 px, float zc, float threshold) {
  // Negated rather than `px.x < 1.0 || ...`, which is the same work and rejects
  // a NaN instead of admitting one: every comparison with NaN is false, so the
  // direct form falls THROUGH to vrSampleDepth, whose clamp and floor are
  // undefined on a NaN and whose depth[] index has nothing further to bound it.
  // project_to_image refuses a non-finite pixel before this is reached, so the
  // form is what keeps the predicate true on its own terms rather than only on
  // its caller's.
  if (!(px.x >= 1.0 && px.x < float(int(cam.width) - 1) && px.y >= 1.0 &&
        px.y < float(int(cam.height) - 1))) {
    return false;
  }
  float d = vrSampleDepth(px.x, px.y, threshold);
  if (!(d >= cam.min_depth && d <= cam.max_depth)) {
    return false;  // hole / non-finite / out-of-range: no line of sight proof
  }
  return abs(d - zc) <= threshold;
}
