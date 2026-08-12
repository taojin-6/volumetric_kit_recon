// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The patch-atlas addressing ABI, mirrored from texture/patch_atlas.hpp.
//
// A patch is a RIGHT TRIANGLE of `leg` texels along each leg -- rows
// j = 0 .. leg-1, row j holding leg - j texels -- laid out row-major inside one
// contiguous run of the atlas buffer. Patch `t` occupies
// [t * count, (t+1) * count), where t is the mesh arena's TRIANGLE SLOT, so the
// atlas needs no allocator of its own: the arena already hands slots out, keeps
// a block's range contiguous, and (since the 2026-08-12 scanned-offset
// decision) puts a cell's triangles at a fixed offset inside it.
//
// These two functions are the whole of that ABI and are what any consumer --
// this tier's fusion kernel, and a renderer sampling the result -- has to agree
// with. The host constexpr versions are patch_texel_count / patch_texel_index;
// keep the pair in lockstep, the way the scalar-layout struct mirrors are.

// Texels one patch holds: the triangular number of `leg`. A square patch of the
// same edge would waste just under half its area, which at room scale is the
// difference between a ~400 MB atlas and a ~800 MB one.
uint vrPatchTexelCount(uint leg) { return leg * (leg + 1u) / 2u; }

// Where texel (i, j) lands inside its patch. Row j starts at
// j*leg - j*(j-1)/2 and runs for leg - j texels, so i + j < leg. A bijection
// onto [0, vrPatchTexelCount(leg)), which is what makes a patch addressable
// with no lookup table on either side.
uint vrPatchTexelIndex(uint leg, uint i, uint j) {
  return j * leg - j * (j - 1u) / 2u + i;
}

// The barycentric coordinate texel (i, j) stands for, as (b1, b2) against the
// triangle's second and third vertices -- b0 is 1 - b1 - b2 and is >= 0 exactly
// because i + j < leg. Corner (0,0) is vertex 0, (leg-1,0) is vertex 1 and
// (0,leg-1) is vertex 2, so a patch spans its triangle whole.
//
// Divides by leg-1 rather than leg so the extreme texels land ON the corners
// instead of short of them; a leg of 1 has no span at all, which is why the
// host refuses one.
vec2 vrPatchBarycentric(uint leg, uint i, uint j) {
  float d = float(leg - 1u);
  return vec2(float(i) / d, float(j) / d);
}
