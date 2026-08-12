#ifndef VR_MARCHING_CUBES_SPARSE_COMMON_GLSL
#define VR_MARCHING_CUBES_SPARSE_COMMON_GLSL

// What both sparse marching-cubes kernels do identically: address a block's
// voxels, resolve its 2x2x2 +neighbourhood on-device, gather a cell's eight
// corners through it, and scan the per-cell triangle counts into the per-cell
// offsets each emitter writes at.
//
// TWO kernels, because vertex sharing is a pipeline choice rather than a
// runtime branch. `marching_cubes_sparse.comp` gives every triangle three
// private vertices (what this tier always did); `marching_cubes_sparse_shared.comp`
// shares a vertex between the cells that meet on an edge. Sharing needs ~8 KiB
// of `shared` arrays BEYOND what the default path uses, and a `shared` array is
// reserved at pipeline creation whatever a push constant later says -- so
// folding the two into one kernel would make the DEFAULT path pay sharing's
// threadgroup budget plus its per-corner vertex atomic, for a feature it does
// not use. MarchingCubes::create builds exactly one of the two.
//
// The figures are SPIR-V-verified and both move as the kernels grow, so treat
// the GAP rather than the endpoints as the decision -- it has narrowed sharply
// and the reason is worth stating. The default path was 44 B when this note was
// written and had drifted to 60 B by the time incremental extraction gave it a
// block reservation, against sharing's 8 476 B: a ~140x gap, which is why the
// default kernel's per-cell triangle count was kept in a private register. The
// 2026-08-12 ordering decision spent ~2 KiB on `s_cell_off` below, which BOTH
// kernels carry, so the measured endpoints are now **2 108 B and 10 524 B** and
// the gap is ~5x. What still separates them is the ~8 KiB of edge and case
// tables sharing alone needs, and it is still worth a kernel: 15 resident
// workgroups against 3 on Apple's 32 KiB, 7 against 1 at Vulkan's guaranteed
// 16 KiB floor. But it is no longer the difference between a rounding error and
// a budget, and an argument that assumes it is has stopped being true.
//
// The cost of two kernels is two copies of the code that is genuinely common,
// which is what this header exists to prevent: the neighbour probe, the corner
// gather and the offset scan are the correctness-critical parts, and a fix to
// any of them must not be appliable to only one path. That is not hypothetical
// -- the scan shipped as a near-copy per kernel and the sharing one was
// converted a revision late, which the slot-for-slot test caught at 5 373
// differing slots. Mirrors marching_cubes_common.glsl, which plays the same
// role for the dense/sparse pair.
//
// The includer must, *before* the include, declare the `pc` push block (fields
// `block_size`, `voxels_per_block`, `iso`, `weight_threshold`, `has_color`,
// `num_buckets`, `bucket_size`, `max_chain`), the `active_blocks` / `tsdf` /
// `weight` / `color_attr` buffers, and have included hash_lookup.glsl (for
// `BlockIndex` + `vrFindBlockPtr`), color_common.glsl and
// marching_cubes_common.glsl (for `mcCornerShift`).

// The workgroup's thread count, as a COMPILE-TIME constant.
//
// The full dimension product rather than gl_WorkGroupSize.x, because the two
// agree only while local_size_y/z are 1. That is true today (the emitted SPIR-V
// is `LocalSize 256 1 1`) and nothing states it, so a later y/z dimension --
// to cut the idle lanes the emitters note below block_size 8 -- would silently
// race 8*Ny*Nz writers onto the 8 s_neighbour slots and re-mesh every voxel
// Ny*Nz times. Both forms compile; only this one stays correct.
//
// Constant rather than a local read off gl_WorkGroupSize, because the scan
// below SIZES AN ARRAY by it. A hardcoded 256 there was correct only while the
// layout said 256, and the TODO(mesh) in each emitter books exactly the change
// that would break it: a smaller group would leave the top of s_cell_off
// unscanned, which reads back as an unsigned underflow in a cell's budget
// rather than as anything a test would notice.
const uint kGroupThreads =
    gl_WorkGroupSize.x * gl_WorkGroupSize.y * gl_WorkGroupSize.z;

// Local voxel index within a block (x-fastest), matching the attribute layout
// BlockIndex::ptr + local the tsdf integrator writes.
int localIndex(ivec3 lc, int bs) {
  return lc.x + bs * (lc.y + bs * lc.z);
}

// The cell base voxel for a local index, in this block's frame (x-fastest, as
// tsdf wrote).
ivec3 mcLocalBase(uint local, int bs) {
  return ivec3(int(local % uint(bs)), int((local / uint(bs)) % uint(bs)),
               int(local / uint(bs * bs)));
}

// This block's 2x2x2 +neighbourhood: s_neighbour[ox + 2*oy + 4*oz] is the voxel-
// array base of the neighbour that owns corners spilling in that direction, or -1
// when it is unallocated. Resolved once per workgroup and read by every voxel,
// which is the whole point of one-workgroup-per-block: 8 probes serve
// voxels_per_block cells.
shared int s_neighbour[8];

// Fill s_neighbour. Threads 0..7 each resolve one octant; octant 0 is this
// block, already known. The caller must `barrier()` before any cell reads it.
//
// TODO(mesh): this prologue, not an occupancy change, is what the on-device
// probe costs -- 8 lanes each walking a bucket while the rest wait at the
// barrier, serialised ahead of every cell in the block. The dispatch measured
// +25% (25.9 -> 32.2 ms) on the M5 iPad Pro when the host table went away, and
// the flat grid it replaced was NOT more densely packed: at block_size 8,
// voxels_per_block is 512 = 2*256, so `gid / 512` was already constant across
// each 256-thread group and every workgroup belonged to one block. Only the
// group count changed (2 per block -> 1). So the levers are the probe --
// spreading the 8 across subgroups, or one lane per bucket slot with a
// reduction -- not the cell or block count. Left unmeasured rather than guessed
// at; the repo's own lesson is to profile the phase before optimising it. Note
// that an early exit on the first free slot is NOT one of the levers: see the
// primary-scan note on vrFindBlockPtr.
void mcResolveNeighbourhood(BlockIndex block, uint tid) {
  if (tid < 8u) {
    uint oi = tid;
    if (oi == 0u) {
      s_neighbour[0] = block.ptr;
    } else {
      ivec3 off = ivec3(int(oi & 1u), int((oi >> 1) & 1u), int((oi >> 2) & 1u));
      s_neighbour[oi] = vrFindBlockPtr(block.coord + off, pc.num_buckets,
                                       pc.bucket_size, pc.max_chain);
    }
  }
}

// Resolve one corner voxel to its slot in the per-voxel attribute arrays.
//
// A corner at local coordinate bs on an axis spills into the +neighbour block on
// that axis: octant selects which of the 2x2x2 neighbourhood holds it, and the
// residual indexes within it. False when that neighbour is unallocated (ptr < 0)
// -- a corner with no block behind it invalidates the whole cell, no
// extrapolation -- and `si` is then 0 rather than undefined.
//
// `c` may be the +1 layer (a component == bs) when a cell reaches for an edge
// whose owner lies outside the block, so corners reach bs + 1. That still lands
// inside the 2x2x2 neighbourhood -- octant is `c / bs`, which is 1 for anything
// in [bs, 2*bs), and bs + 1 < 2*bs for every legal block size.
//
// THE cross-block addressing, and one copy of it: both gathers below resolve a
// corner through here, so the sign-only count and the full gather cannot land on
// different voxels. They must not, because the count reserves the span the
// gather writes into.
bool mcCornerStorage(int bs, ivec3 c, out uint si) {
  si = 0u;
  ivec3 octant = c / bs;
  ivec3 lc = c - octant * bs;
  int nptr = s_neighbour[octant.x + 2 * octant.y + 4 * octant.z];
  if (nptr < 0) {
    return false;
  }
  si = uint(nptr) + uint(localIndex(lc, bs));
  return true;
}

// Gather a cell's eight corners through the shared neighbourhood: sdf values,
// linear colours, and the cube index. False when a corner is unavailable --
// its block unallocated, or the voxel unintegrated (weight below threshold).
//
// On that failure `cube_index` comes back 0, NOT the partial mask the loop had
// accumulated over the corners it did reach. That is load-bearing rather than
// tidy: mcCellTriangleCount reads 0 as "no triangles", so a caller that ignores
// the bool still cannot count, reserve, or emit for a cell whose corners were
// never all read.
bool mcGather(int bs, ivec3 base, out float sdf[8], out vec3 corner_color[8],
              out int cube_index) {
  cube_index = 0;
  for (int i = 0; i < 8; ++i) {
    uint si;
    if (!mcCornerStorage(bs, base + mcCornerShift(i), si) ||
        weight[si] < pc.weight_threshold) {
      cube_index = 0;  // drop the partial mask -- see the note above
      return false;
    }
    sdf[i] = tsdf[si];
    if (pc.has_color != 0u) {
      // color_attr == 0 is the tsdf integrator's "colour never observed"
      // sentinel: a fused colour is packed with alpha 0xFF, so any observed
      // colour is non-zero. A depth-integrated but colour-unobserved corner
      // falls back to opaque white rather than dragging the interpolated vertex
      // toward black, mirroring the integrator's first-observation-assigns
      // anti-darkening rule. Decoded to LINEAR working values here, because the
      // edge interpolation is an average (the 2026-08-02 decision); white is
      // 1.0 in either space, so the sentinel branch is unaffected.
      uint packed = color_attr[si];
      corner_color[i] = packed != 0u ? vrUnpackSrgbToLinear(packed) : vec3(1.0);
    }
    if (sdf[i] < pc.iso) {
      cube_index |= (1 << i);
    }
  }
  return true;
}

// The cube index ALONE -- what a caller that only needs a cell's TRIANGLE COUNT
// has to read. 0 for a cell whose corners are unavailable, exactly as mcGather
// returns on failure, so an invalid cell and an empty one need no separate
// encoding.
//
// Separate from mcGather rather than a flag on it because what it drops is the
// expensive part, and the sparse kernel's counting phase runs over 100% of
// cells: no `out float[8]` / `out vec3[8]` copy-out (GLSL passes arrays by
// value, so each is a real 128-byte copy at every call), and no colour decode --
// vrUnpackSrgbToLinear is a transfer curve per corner, up to 24 of them per
// cell, for values a count cannot use.
//
// The sign test below is the one line this shares with mcGather by convention
// rather than by construction, and the two must agree exactly: the count taken
// here reserves the span mcGather's caller then writes into. Both read
// `tsdf[si] < pc.iso` over the same quiescent buffer through the same
// mcCornerStorage, so they can only diverge if the two lines are edited apart --
// and the emitter bounds its writes to the reserved span, so even that drops
// triangles rather than writing over the next block's.
int mcCellSigns(int bs, ivec3 base) {
  int cube_index = 0;
  for (int i = 0; i < 8; ++i) {
    uint si;
    if (!mcCornerStorage(bs, base + mcCornerShift(i), si) ||
        weight[si] < pc.weight_threshold) {
      return 0;
    }
    if (tsdf[si] < pc.iso) {
      cube_index |= (1 << i);
    }
  }
  return cube_index;
}

// Cells the per-cell offset array holds at once. Mirrored by kMaxScanCells in
// marching_cubes.cpp, which REPORTS a block with more cells than this rather
// than refusing it (the default emitter walks such a block a chunk at a time).
// The sharing kernel refuses one anyway, on its own kMaxSharedCells, because
// its edge table would run off the end.
const uint kMaxScanCells = 512u;

// Per-cell TRIANGLE OFFSET within the block's reserved triangle range: the
// exclusive prefix sum of every cell's count, stored shifted by one so entry
// `local` is where cell `local` starts and entry `local + 1` is one past its
// last triangle. The difference of two adjacent entries is therefore that
// cell's count, which is why nothing else stores one.
//
// This replaced a shared cursor -- `s_cursor` here, `s_icursor` in the sharing
// kernel -- that handed runs out in whatever order cells reached it. Correct
// either way, since every slot landed inside the block's own reservation, but
// the ORDER was not reproducible: a block re-meshed to identical geometry could
// permute its own triangles inside its own unchanged range. Invisible to a
// consumer reading the mesh; fatal to one binding per-triangle state to the
// arena slot, which is what the projective texture atlas does.
//
// VERTEX slots are deliberately left on their arrival-order atomic. Under
// sharing a vertex is reached only through the index run the kernel writes
// itself, so permuting them renames nothing a consumer can key on -- and giving
// them fixed offsets would need a second scan over a count that is not known
// until the edges are walked.
//
// Entry 0 is the fixed zero an exclusive prefix starts from, and the only entry
// a counting pass never writes: it writes `local + 1` for each of the block's
// cells. A count pass must therefore write UNCONDITIONALLY -- including the
// zero -- so no entry carries a previous chunk's value, and entry 0 must be
// zeroed once. The scan never touches it (every round writes `i >= d >= 1`), so
// once is once per dispatch and not once per chunk.
shared uint s_cell_off[kMaxScanCells + 1u];

// Turn the per-cell counts in s_cell_off[1 .. count] into those exclusive
// prefix sums, in place. Hillis-Steele over the block's own cell count rather
// than the array's capacity, so a smaller block pays fewer rounds.
//
// ONE copy for both kernels, which is the whole reason it lives here: a fix to
// the barrier placement, to the read-before-write ordering, or to the `i >= d`
// guard has to reach both emitters or it mis-slots triangles in the one it
// misses, silently and with exact counters.
//
// Every thread must reach every barrier, so the bounds tests gate the memory
// access and never the iteration. `tmp` is read before the write barrier and
// applied after it, which is what keeps a round from reading a neighbour's
// already-updated value. `kScanSlots` covers the array at the workgroup's
// actual width -- see kGroupThreads.
void mcScanCellOffsets(uint tid, uint count) {
  const uint kScanSlots = (kMaxScanCells + kGroupThreads) / kGroupThreads;
  uint tmp[kScanSlots];
  for (uint d = 1u; d <= count; d <<= 1u) {
    for (uint s = 0u; s < kScanSlots; ++s) {
      uint i = tid + s * kGroupThreads;
      tmp[s] = (i <= count && i >= d) ? s_cell_off[i - d] : 0u;
    }
    barrier();
    for (uint s = 0u; s < kScanSlots; ++s) {
      uint i = tid + s * kGroupThreads;
      if (i <= count && i >= d) {
        s_cell_off[i] += tmp[s];
      }
    }
    barrier();
  }
}

#endif  // VR_MARCHING_CUBES_SPARSE_COMMON_GLSL
