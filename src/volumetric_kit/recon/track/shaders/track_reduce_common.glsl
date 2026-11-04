// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// What every residual's reduce shares: a thread's running sums of one
// iteration's linear system (track_common.glsl's partial layout), folded per
// workgroup in a fixed tree, so a fit's sums are the same on every run.
// A residual's .comp includes this, calls vr_track_add for each inlier and
// vr_track_fold at the end, and writes thread 0's column (g_shared[i *
// kReduceLocal]) as its group's partial. Its workgroup is 64 threads: 33 x 64
// floats of shared memory, inside the 16 KiB every device has.

#ifndef VR_TRACK_REDUCE_COMMON_GLSL
#define VR_TRACK_REDUCE_COMMON_GLSL

#include "track_common.glsl"

const uint kReduceLocal = 64u;

float g_acc[kPartial];

void vr_track_clear() {
  for (uint i = 0u; i < kPartial; ++i) {
    g_acc[i] = 0.0;
  }
}

// One inlier: its scaled Jacobian row j, residual r and weight w.
void vr_track_add(float j[6], float r, float w) {
  uint k = 0u;
  for (uint a = 0u; a < 6u; ++a) {
    float wa = w * j[a];
    for (uint b = a; b < 6u; ++b) {
      g_acc[kH + k] += wa * j[b];
      ++k;
    }
    g_acc[kG + a] += wa * r;
  }
  g_acc[kRaw2] += r * r;
  g_acc[kInliers] += 1.0;
}

shared float g_shared[kPartial * kReduceLocal];

// Fold the workgroup's sums and write them to partial[group * kPartial ..].
void vr_track_fold(uint lid) {
  for (uint i = 0u; i < kPartial; ++i) {
    g_shared[i * kReduceLocal + lid] = g_acc[i];
  }
  barrier();
  for (uint s = kReduceLocal / 2u; s > 0u; s >>= 1u) {
    if (lid < s) {
      for (uint i = 0u; i < kPartial; ++i) {
        g_shared[i * kReduceLocal + lid] += g_shared[i * kReduceLocal + lid + s];
      }
    }
    barrier();
  }
}

#endif  // VR_TRACK_REDUCE_COMMON_GLSL
