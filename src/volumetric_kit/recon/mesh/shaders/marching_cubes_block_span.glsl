// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#ifndef VR_MARCHING_CUBES_BLOCK_SPAN_GLSL
#define VR_MARCHING_CUBES_BLOCK_SPAN_GLSL

// Where one block's output landed, keyed by block SLOT (ptr / voxels_per_block)
// -- the mirror of mesh::BlockSpan in marching_cubes.hpp.
//
// The mapping each kernel's per-block reservation computes and used to drop. It
// is not derivable on the host: the atomic hands ranges out in workgroup ARRIVAL
// order, not block order, so nothing outside the dispatch knows which range
// belongs to which block. Publishing it is what lets a later extract leave a
// clean block's geometry in place and re-mesh only what a fuse changed.
//
// Keyed by slot because a slot is a dense index into a fixed table where a coord
// would need a device-side hash. A slot only means something against a
// particular grid and topology epoch -- the host anchors it, exactly as the tsdf
// tier anchors its dirty flags.
//
// DECLARED ONCE, for both kernels, and that is the point of the file rather
// than a tidiness preference: four same-typed fields make every permutation 16
// bytes, so neither the host's static_asserts nor SPIR-V validation can see a
// transposition. Two copies of this struct could disagree on field ORDER and
// both compile, and the two kernels would then write the same buffer to two
// different ABIs. The buffer block itself stays in each kernel, because the
// binding number differs (8 by default; 9 with sharing, which spends 8 on the
// index run it writes itself), and this header must be included ahead of it.
//
// Counted in vertices and in TRIANGLES, not indices: a triangle is what a block
// owns and what a re-mesh replaces. Both kernels assign these BY NAME for the
// reason above -- `s_ibase` is an index base only until it is divided by
// kIndicesPerTriangle, and a positional write cannot say which it holds.
struct BlockSpan {
  uint vertex_base;
  uint vertex_count;
  uint triangle_base;
  uint triangle_count;
};

#endif  // VR_MARCHING_CUBES_BLOCK_SPAN_GLSL
