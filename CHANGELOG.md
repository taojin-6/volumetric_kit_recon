# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to
follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial repository scaffolding: tiered layout, MIT license, `.clang-format`,
  `.cmake-format.yaml`, `.pre-commit-config.yaml`, `.gitignore`.
- `CLAUDE.md` — the living source of truth (objective, locked decisions, tier
  architecture, interop contract, salvage + exclusion policy, gotchas).
- `core` tier foundation: backend-neutral `Status` / `Result<T>` error handling
  (`VR_TRY` / `VR_ASSIGN`), `VR_CHECK` contract checks, a pluggable log handler,
  the version API, and portable POD math types.
- Tiered CMake with install/export and package config
  (`find_package(volumetric_kit_recon)` / `FetchContent`), warnings-as-errors,
  and sanitizer support.
- Direction set to a single **Vulkan compute** path (MoltenVK on Apple),
  mirroring `volumetric_kit_gfx`, so the reconstruction backend and renderer
  share one cross-platform API and a trivial same-device interop seam.
- `core` Vulkan **compute foundation**: a VMA `Allocator` (VMA v3.3.0 vendored
  via pinned FetchContent, built in one TU), a RAII `Buffer`, a SPIR-V
  `ShaderModule`, `DescriptorSetLayout` / `DescriptorPool` / `DescriptorSet`
  (storage-buffer binding), a `ComputePipeline` (explicit descriptor layout +
  push constants, no SPIR-V reflection), the `UniqueHandle` owner for
  device-scoped Vulkan handles, and a shared-queue-safe
  `Device::submit_single_time` / `queue_submit` dispatch primitive.
- GLSL → SPIR-V build step: `vr_compile_shaders()` (`cmake/vr_shaders.cmake`),
  targeting Vulkan 1.2 (the device floor; scalar block layout is 1.2 core).
- Vulkan **compute smoke** (`tests/compute_smoke_test.cpp` + `tests/shaders/
  fill.comp`) proving the end-to-end path — allocate → bind → dispatch → read
  back — on MoltenVK, plus move-only RAII tests (`tests/compute_raii_test.cpp`)
  the sanitizer job turns into leak/double-free detectors.
- `volume` tier host math: `VoxelGridParams` (grid + hash-table shape, the
  scalar-layout shader ABI), world/voxel/block coordinate transforms
  (`volume/voxel_coords.hpp`), and the Teschner spatial hash + slot sentinels
  (`volume/hash.hpp`), with CPU tests.
- `volume` **sparse voxel hash map** (first GPU slice): the host `VoxelHashMap`
  owns the device buffers + compute pipelines and drives init /
  allocate-from-coords / compact via GLSL kernels (`volume/shaders/hash_*.comp`,
  scalar-block-layout `HashEntry`/`BlockIndex`), embedded into the now-compiled
  STATIC `recon_volume` via `vr_embed_shaders` (`cmake/vr_embed.cmake` +
  `cmake/embed_spirv.cmake`). A GPU test (`tests/volume_hash_map_test.cpp`) proves
  allocate→compact + the on-device layout round-trip on MoltenVK.
- `core`: `Device` now enables `scalarBlockLayout` (Vulkan 1.2 core) — the
  compute-shader buffer ABI — on create, and `adopt` requires the creator did.
- `volume`: `VoxelHashMap::remove` + the `hash_delete_coords` kernel — delete
  blocks by coordinate (collision-chain splice + successor pull-up) and return
  each freed block to the heap. The delete searches authoritatively under the
  bucket lock — no lock-free existence pre-check, since (unlike allocate) a
  delete cannot trust a lock-free "absent" read — retries and counts a
  found-but-not-deleted coord so a lost lock race is never silently dropped, and
  flags a block that cannot be returned to the heap. A GPU test
  (`tests/volume_delete_test.cpp`) forces overflow chains and a tight heap to
  prove the chain splice, successor pull-up, and real heap reuse on MoltenVK.
- `volume`: `VoxelHashMap::resize` — grow the hash table to more buckets,
  **preserving each block's index** so per-voxel data survives. Snapshots the
  active set, re-inits the larger table, then a `hash_rehash.comp` kernel
  re-inserts each block with its ORIGINAL pointer (the shared `insert_block`'s
  new `preset_ptr` — reused across normal allocation and rehash, so the tested
  insert path stays single-sourced, instead of drawing a fresh block off the
  heap), and the host rebuilds the free-block heap to exclude those live indices.
  The grow is failure-atomic (the larger buffers are built off to the side and
  swapped in together) and re-drives the snapshot to absorb transient lock
  contention. `VoxelBlockGrid::resize` grows every attribute array first (copying
  the old contents forward), so a block keeps its `ptr` and its
  `tsdf`/`weight`/`color` data at the same offset. GPU tests
  (`tests/volume_resize_test.cpp`, `tests/volume_block_grid_test.cpp`) prove
  256 → 1024 growth with block indices preserved, the heap rebuilt to exclude the
  live set, per-voxel attribute data surviving the grow, and allocation past the
  old capacity still working on MoltenVK.
- `volume`: `VoxelHashMap::diagnostics` — host-side occupancy + health stats
  (active / overflow / collision-chain length, load factor, heap utilization)
  from the entries + heap counter; a GPU test
  (`tests/volume_diagnostics_test.cpp`) forces a collision chain and verifies the
  counts. The shared coord-kernel path (allocate + remove) now re-dispatches on
  failure to converge under same-bucket contention (a GPU spin-lock livelock); a
  non-zero return means a genuine capacity limit.
- `volume`: `VoxelBlockGrid` — a structure-of-arrays voxel attribute store
  (Open3D-style) composing a `VoxelHashMap` block index with named, independently
  allocated per-voxel attribute arrays (`tsdf`, `weight`, `color`, …), each
  `num_blocks * voxels_per_block` and keyed by `BlockIndex::ptr`. A consumer
  declares only the channels it needs, so a `volume`-only user allocates no
  per-voxel memory; the TSDF / colour integrators and meshing bind the attribute
  buffers they touch. A GPU test (`tests/volume_block_grid_test.cpp`) proves
  distinct, correctly-sized attribute storage, the composed map, and the error /
  move paths on MoltenVK.
- `tsdf`: new **`tsdf` tier** with `TsdfIntegrator` — classic projective TSDF
  integration of a posed depth frame (float metres, reusing `DepthCameraParams`)
  into a `VoxelBlockGrid`'s `tsdf` + `weight` attributes. One GLSL dispatch
  (`tsdf_integrate.comp`) runs a thread per voxel of each active block: project
  the node-centred voxel into the camera, `sdf = depth - Zc`, truncate at
  `±trunc_dist`, weight by inverse-square with a behind-surface dropoff, and fuse
  by a running average capped at `max_weight` — faithful to the prior engine's
  classic kernel (neural/triplane channels excluded). A GPU test
  (`tests/tsdf_integrate_test.cpp`) fuses a constant-depth plane and checks the
  per-voxel sdf/weight, the truncation boundary, and the two-frame weight cap on
  MoltenVK.
- `tsdf`: `TsdfIntegrator::integrate` gains an `IntegrationMode` (classic /
  dynamic). Dynamic integration adds one kernel branch — a voxel that projects
  into free space past the truncation band (`sdf > trunc_dist`) is cleared if it
  held prior weight, so a receded surface leaves no ghost geometry (classic keeps
  it clamped): the prior engine's stale-free-space clearing. The GPU test
  re-integrates a receding plane and asserts dynamic clears the stale voxel while
  classic keeps it. Bilinear depth sampling and colour follow.
- `tsdf`: the integrate kernel now samples depth **bilinearly** (was
  nearest-neighbour), falling back to the nearest sample when a tap is out of
  bounds, invalid, or the 2x2 taps straddle a depth discontinuity
  (`max - min > trunc_dist`) that would blend across a surface edge — the prior
  engine's `sampleDepthBilinear`. The GPU test projects an on-axis voxel onto a
  half-pixel tap boundary and checks a sub-`trunc` step blends (distinct from the
  nearest sample) while an over-`trunc` step falls back. Colour follows.
- `tsdf`: `TsdfIntegrator::integrate` gains an optional `ColorFrame` — fuse a
  posed color image into the grid's `color` attribute alongside depth. Color uses
  a **separate color camera** — its own `ColorCameraParams` (pinhole intrinsics +
  pose + dimensions, the color analogue of `DepthCameraParams` without the
  depth-range fields), projected per voxel and running-averaged with the same
  weights as the SDF. A voxel's **first color
  observation assigns** the sampled RGB — keyed on whether color was seen
  (`color_attr == 0`), not on the depth weight, so an unregistered color camera
  (or a depth-only warmup) does not blend the first color toward black. Dynamic
  mode clears a receded voxel's color whenever the grid carries the attribute,
  **including on a depth-only frame**, so no color ghost survives a recede. RGB
  is packed in a `uint`'s low three bytes, matching the mesh tier's `color`
  layout so meshing reads it directly. The GPU test fuses a constant color and
  checks the packed RGB per voxel, that an occluded voxel keeps zero, that a
  color camera shifted out of frame skips color while depth still fuses, that the
  first color after a depth-only warmup assigns the full RGB, and that a
  depth-only dynamic recede clears the color ghost.
- `mesh`: a second `MarchingCubes::extract` overload meshes straight off a sparse
  `volume::VoxelBlockGrid` — the real `tsdf`/`weight`/`color` blocks — via
  `mesh/shaders/marching_cubes_sparse.comp`. One invocation per voxel of each
  active block (the tsdf integrator's iteration); a cell on a block's `+face`
  resolves its cross-block corners through a **host-built 2×2×2 neighbour table**
  (each active block plus its seven `+x/+y/+z` neighbours, from the compacted
  active set), so the kernel needs no device-side hash probe and no access to the
  hash table's internal buffers. *(Superseded — the kernel resolves the
  neighbourhood on-device now; see the `Changed` entry below.)* When the grid
  carries a `uint32` `color`
  attribute each vertex's color is interpolated from it, else opaque white; a
  corner whose color is the integrator's `0` "colour unobserved" sentinel (a
  written colour carries alpha `0xFF`) also falls back to white rather than
  dragging the vertex toward black; `uv0` stays the `(-1,-1)` sentinel. Only the
  corner sampling differs from the dense kernel — the shared per-cell body (cube
  index, gradient normal, reversed winding, independent triangles) is factored
  into `mesh/shaders/marching_cubes_common.glsl`, `#include`d by both kernels.
  The host rejects a worst-case vertex arena larger than `maxStorageBufferRange`
  with a clean `Status` instead of an opaque allocation failure. The GPU test
  writes an analytic sphere into a real 6³-block grid so the surface crosses
  interior block boundaries, then proves the sparse mesh matches the dense path
  **triangle-for-triangle** (plus cross-block color, that an unobserved colour
  meshes white, and that a sub-threshold weight gates every cell out) — the
  exact-count equivalence being the cross-block-addressing proof — and checks the
  empty / moved-from / missing-attribute paths.
- `examples`: `fuse_replica` — the first example and the vertical slice running
  end-to-end on real data. Reads a posed Replica-SLAM RGB-D sequence (nvblox's
  `fuse_replica` layout: `results/frameNNNNNN.jpg` + `depthNNNNNN.png`, row-major
  `traj.txt` camera-to-world poses, `cam_params.json` intrinsics/scale), fuses
  each frame into a sparse TSDF+colour volume (`allocate_from_depth`, growing the
  map via the block-index-preserving `resize` on overflow, then `integrate`
  depth+colour), extracts a marching-cubes mesh, and writes a coloured binary
  PLY. A small `examples/common` reader (stb_image colour/depth decode + a
  tinyply PLY writer, via pinned examples-only FetchContent, plus the poses)
  activates the reserved `examples/` slot. Verified on Replica room0 (400
  frames): a coherent
  4.0×4.4×2.8 m room, unit normals, plausible surface colour, triangle count
  converging, at ~60 fps on MoltenVK.
- `examples`: gfx-linked **viewer examples** (`examples/viewer/`, behind an
  off-by-default `VR_BUILD_VIEWER` that FetchContents `volumetric_kit_gfx` + finds
  GLFW — the only place the recon tree touches the renderer; the tiers + default
  build + CI stay renderer-independent). `fuse_render` fuses a Replica sequence
  and renders the coloured reconstruction to a **PNG** headlessly through gfx's
  `HybridMeshPipeline` + an `OffscreenTarget` (per-vertex colour on the `uv0`
  sentinel; optional `--follow N` renders from a sensor pose). `fuse_viewer` opens
  a **live window** — the nvblox `FuserVisualizer` analogue — fusing on a
  background thread (load/decode/integrate/extract off the render thread) while
  the render thread draws the growing mesh each frame following the capture
  trajectory (a host-mesh handoff, interop seam A, two devices).
  `recon_gfx_bridge.hpp` converts `mesh::Vertex` → `gfx::assets::Vertex`
  (synthesizing `tangent`, keeping the `uv0` sentinel). Verified on Replica room0:
  a correct first-person coloured room render.

### Changed

- `mesh`: the sparse `MarchingCubes` kernel emits **per block contiguously**.
  It counts a block's triangles, reserves one span for all of them with a single
  global `atomicAdd`, and only then writes — where every triangle used to claim
  its own slot and a block's output interleaved with every other block's in
  flight. Per-block ranges are what a dirty-only dispatch needs to leave a clean
  block's geometry in place, so nothing downstream of incremental extraction can
  start without this. Applies to the **default** sparse kernel;
  `share_vertices` selects a kernel that still appends per triangle, and is
  therefore the one path incremental extraction cannot use (a `TODO(mesh)` in
  `marching_cubes_sparse_shared.comp` records what restructuring it would take).
  Geometry is **byte-identical**: room0 at 120 frames matches
  `main` triangle-for-triangle at `--voxel 0.02` (277 506 triangles) and at
  `--voxel 0.012` (766 117), by a canonical hash over the sorted triangle set.
  **It costs ~10%** on the extract dispatch (1.17 → 1.28 ms at `--voxel 0.012`,
  Release, samples interleaved to cancel thermal drift) and buys **no**
  coalescing win —
  a triangle's three vertices were already written consecutively at `tri * 3`;
  the interleaving that cost coalescing is per-*vertex* and lives in the sharing
  kernel, which this does not touch. Nor is the index run monotonic within a
  block: slots are still handed out in whatever order cells reach the cursor.
  Justified as the precondition, not as a speedup; see the 2026-08-09
  incremental-extraction decision in `DECISIONS.md` (**PR #60 — merge it
  first**, or this reference dangles).
  A cell is visited twice but gathered ~1.08 times, and the two visits are
  asymmetric: the counting phase runs over 100% of cells and gathers **signs
  only** (`mcCellSigns` — no `sdf[8]`/colour array copy-out, no sRGB decode, for
  values a count cannot use), caching each cell's triangle count in one byte,
  four to a uint — 512 B of `shared` at the default block size, against the
  ~8 KiB the sharing kernel needs, which the 2026-08-08 two-kernel decision
  exists to keep off this path. The emitting phase then gathers in full, but
  only the ~8% of cells that emit; the rest are rejected on a shared-memory
  byte. A grid whose block outgrows that cache still meshes correctly and now
  says so, through `ExtractTimings::uncached_cells_per_block`.
  `mcEmitCell` splits into `mcCellTriangleCount` + `mcWriteTriangle`, so the
  per-block emitter and the dense kernel's per-triangle append still write
  through one body and cannot drift, and the cross-block corner addressing
  splits into `mcCornerStorage` so the two gathers resolve a corner through one
  copy of it.

- docs: split the locked-decision record out of `CLAUDE.md` into a new
  `DECISIONS.md`, moved verbatim — same 33 entries, same order, byte-identical
  text. `CLAUDE.md` keeps every decision as a one-line rule linking to its full
  entry, and its "Where to start" tour is trimmed to a tier map plus what has
  landed and what is next; the measured-lesson narrative it carried (the
  `ExtractTimings` breakdown that overturned the bottleneck guess) moves to
  `DECISIONS.md` → "Measured lessons". `CLAUDE.md` goes 2 339 → 438 lines, which
  is what an agent loads on every session; nothing is lost, only relocated.

- `mesh`: the sparse `MarchingCubes::extract` kernel resolves its 2×2×2
  neighbourhood **on-device** instead of being handed a host-built table. One
  workgroup per active block (not a flat voxel grid — shared memory is per
  workgroup), threads 0–7 each probing one octant through the new
  `volume/shaders/hash_lookup.glsl` into `shared int s_neighbour[8]`, then every
  thread strides over the block's voxels. The host pass it replaces was an
  `O(active·8)` serial `unordered_map` build measured at **102.2 ms of a 132.7 ms
  extract** at 107 k blocks on an M5 iPad Pro, against 25.9 ms for the dispatch it
  fed; extract drops to **42.0 ms**. Both prior CUDA and Metal implementations of
  this pipeline resolved neighbours the same way.
  `volume::VoxelHashMap::entries_buffer` / `entries_buffer_size` are published for
  it (bound with the real range and checked against `maxStorageBufferRange`, since
  `resize` doubles the table), and `hash_lookup.glsl` shares `hash_common.glsl`'s
  struct layouts and hash constants rather than mirroring them — the `pc`
  push-constant block is opt-out via `VR_HASH_COMMON_NO_PUSH_CONSTANTS`, and both
  headers gained include guards.
  **The probe requires a quiescent table** (no `allocate`/`remove`/`clear`/`resize`
  dispatch in flight), which is stated on `MarchingCubes::extract_device` as well
  as on the accessor: it holds by construction on one thread, and a consumer that
  fuses and meshes concurrently must serialise them. See the 2026-08-08 decision in
  `DECISIONS.md` for what the `mesh`→`volume` coupling costs.
- `mesh`: **removed** `ExtractTimings::neighbour_lut_ms` — the phase it measured no
  longer exists, and a permanently-zero row in the viewer overlay is worse than an
  absent one. `total_ms()` and `fuse_viewer`'s stage table drop it with the field.

### Fixed

- `volume`: `VoxelHashMap::entries_buffer_size()` reported the full table size on a
  **moved-from** map while `entries_buffer()` correctly reported `VK_NULL_HANDLE`,
  so the descriptor write the two accessors exist to spell would pair a null handle
  with a non-zero range — invalid usage, and undefined with layers off, which this
  repo neither enables `nullDescriptor` nor `robustBufferAccess` to survive. Both
  accessors now gate on `valid()`, and `entries_buffer()` documents that a completed
  `resize` destroys the handle (both shipped examples resize mid-scan).
