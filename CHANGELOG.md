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
- `mesh`: `MarchingCubes::block_spans()` publishes **where each block's geometry
  landed** — vertex base/count and triangle base/count per block slot
  (`BlockIndex::ptr / voxels_per_block`), written by both sparse kernels. This is
  the mapping the per-block reservation computes and used to drop, and it is
  **not derivable on the host**: the atomics hand ranges out in workgroup arrival
  order, not block order, so nothing outside the dispatch knows which range
  belongs to which block. It is what stage 3 re-meshes against. Counted in
  vertices and **triangles**, not indices, because a triangle is what a block
  owns and what a re-mesh replaces; the four numbers are independent under
  `share_vertices` and locked at `v = 3t` without it.
  **Opt-in** behind `MarchingCubesConfig::track_block_spans`, off by default: the
  table is sized by the grid rather than the surface (`num_blocks` entries — 24 MB
  at `VoxelGridParams::defaults()`, doubling with every `VoxelHashMap::resize`),
  so a caller who does not read it allocates nothing and the kernel is told not to
  write it, the bargain `TsdfIntegratorConfig::track_dirty_blocks` strikes for the
  same table shape. It is counted in `ExtractTimings::arena_bytes`, and grown
  beside the arena so the allocation lands in `arena_alloc_ms` rather than in the
  descriptor row.
  **Borrowed and retired by generation.** The accessor returns mapped device
  memory, which a grow frees, so `block_spans_generation()` carries the same
  counter as `DeviceMesh::generation`: a consumer compares the two rather than
  being told in prose not to cache the pointer. It returns `nullptr` until an
  extract has left a table describing its own output — both kernels publish a
  span *before* their capacity guard (deliberately: the counters must carry each
  block's full total for the host's refit), so a failed, empty or dense extract
  leaves spans naming triangles the arena never held. **Not per slot**, unlike the
  arena and index run: one table describes the current dispatch, and the
  generation is what makes a mismatch against a held mesh visible.
  A grow carries the existing spans forward and zeroes only the new tail
  (`VoxelHashMap::resize` preserves block indices, which is why
  `topology_epoch()` does not move across one), mirroring
  `TsdfIntegrator::prepare_dirty_flags`.
  The host/device mirror is pinned per field: `BlockSpan` is four same-typed
  `uint32`s, so `sizeof` alone cannot see a transposition — it carries `offsetof`
  asserts like every other mirror in the repo, is declared once in
  `shaders/marching_cubes_block_span.glsl` rather than copied into two kernels
  that could disagree on field order, and both kernels assign it by name.
  A slot is meaningful only against the grid and topology epoch that produced it,
  and only for the blocks the *published* extract meshed — nothing is cleared on
  the way past, so every other entry reads as a well-formed span belonging to an
  earlier one. `block_span_valid` is what separates them; there is no value in
  the table that means "not mine".
- `mesh`: `MarchingCubes::block_span_valid(grid, slot)` — the **anchor** that
  makes a published span mean something on a later extract, and the per-slot half
  of the question `block_spans_generation()` answers for the table as a whole. A
  span is keyed by block slot, and a slot names a block only against a particular
  grid at a particular topology: the block heap is LIFO, so after a `remove()` a
  reused slot names a *different* block and its span reads as that block's
  geometry under `Status::ok`. Anchored on `topology_epoch` alone, which now
  identifies both — see the `volume` entry below.
  It takes the **grid** rather than trusting the caller to re-extract first:
  between a `remove()` and the next extract the per-slot stamps are still set, so
  a query that re-checked nothing would call a stale span live — a staleness the
  caller cannot see, which makes it this tier's to check (the 2026-08-04 rule).
  The stamp is compared for **equality** with the extract that published the
  table, not merely for being set: a block that drops out of the active set keeps
  the stamp its last extract wrote, so "ever meshed" would hand back a span whose
  bases index an arena since rewritten — which is what dirty-only dispatch, where
  meshing a strict subset is the normal case, would make the steady state.
  False whenever the whole table has been retired, so it can never report a slot
  live beside a `block_spans()` of `nullptr`; false too when
  `track_block_spans` is off, on a moved-from extractor, and for a moved-from
  grid, which owns no blocks however its token reads.
  It is **not** the whole answer at `slot_count > 1`: one table serves the whole
  ring, so compare `block_spans_generation()` against your own
  `DeviceMesh::generation` first.
  A `resize()` does not break it: resizing preserves block indices, so spans stay
  true and the table simply grows, its new entries unstamped.
- `volume`: `topology_epoch` moved from `VoxelBlockGrid` down to
  `VoxelHashMap` — the object that hands block indices out and takes them back —
  and became a **globally unique token** drawn from a process-wide counter at
  `create` and at every `remove()` / `clear()`, rather than a per-grid count.
  Two consequences, both of which closed a hole an anchor built on it could not
  see. It can no longer be dodged: `VoxelHashMap::remove` reached through
  `VoxelBlockGrid::map()` moves it exactly as the wrapper does, where a counter
  owned one tier up was moved only by the wrapper. And no two grids, or two
  topologies of one grid, ever share a value, so an anchor holding a dead grid's
  token cannot be revived by a new grid built in the same storage — the ABA a
  raw pointer comparison has no way to detect. `MarchingCubes` consequently
  anchors on the token alone and holds no grid pointer at all.
  `resize()` still does not move it, which is what lets a slot-keyed cache
  survive a grow. It counts nothing now; compare it for equality only.

### Changed

- `mesh`: the **vertex-sharing** sparse kernel emits per block, like the default
  one. It counts a block's vertices and triangles, reserves one range of each
  with a single `atomicAdd`, and only then writes — where it previously appended
  per vertex *and* per triangle through the global counters, so its output
  interleaved across blocks and no per-block range described it. `share_vertices`
  was therefore the one path a dirty-only dispatch could not use, which matters
  because it is the path the iOS scanner runs (memory: an in-block-shared mesh is
  ~3x smaller, and an iPad is where the arena ceiling is real). Two ranges rather
  than the default kernel's one, since sharing breaks `v = 3t`.
  **No measurable cost**: three interleaved samples each of `main` and of this
  kernel span 1.77–1.82 ms on the extract dispatch, and their means differ by
  less than that spread (room0 at 120 frames, `--voxel 0.012 --share-vertices
  --device-extract --preload`, **Release** — the build type belongs with the
  figure, since a bare configure measures `-O0`). The kernel already ran two
  passes over a cached classification, so the count phases reuse it — and
  counting reads the eight corner *signs* alone (`mcCellSigns`), no `sdf`/colour
  arrays copied out of a gather and no sRGB decode, for the one pass that runs
  over 100% of the cells. The default kernel paid ~10% for the same property.
  Geometry is **unchanged as a triangle set and as a vertex count** — not
  byte-identical, since the arena layout is exactly what this changes: the same
  766 117 triangles over the same 668 792 vertices as `main`, in a PLY of
  identical length and different bytes.
  Counting duplicates is a **second** pass, not folded into the first: a
  duplicate is decided by the *owner* cell's state, which pass one is still
  writing. Reading it there races, and under-reserves the block's range so the
  cursor walks off the end of it — invisible on the sphere fixture, a
  6 062-vertex error on room0, which is how it was caught.
  Each cursor is **bounded by the block's own reservation**, the fail-safe the
  default kernel already pays for on the same mechanism: an over-consuming block
  drops geometry rather than writing over the next block's range with exact
  counters and no error anywhere.
  `kVertexDropped` is **removed**: it existed so a dropped vertex would not be
  re-claimed and double-counted, and per-block reservation makes every slot
  deterministic whether or not it lands inside the arena, so the totals are exact
  by construction rather than by bookkeeping.

- `mesh`: the sparse `MarchingCubes` kernel emits **per block contiguously**.
  It counts a block's triangles, reserves one span for all of them with a single
  global `atomicAdd`, and only then writes — where every triangle used to claim
  its own slot and a block's output interleaved with every other block's in
  flight. Per-block ranges are what a dirty-only dispatch needs to leave a clean
  block's geometry in place, so nothing downstream of incremental extraction can
  start without this. Applies to the **default** sparse kernel; the
  `share_vertices` one is the entry above, which reserves two ranges rather than
  one.
  Geometry is **unchanged as a triangle set** — not byte-identical, since the
  arena layout is exactly what this changes: room0 at 120 frames matches `main`
  triangle-for-triangle at `--voxel 0.02` (277 506 triangles) and at
  `--voxel 0.012` (766 117), by a canonical hash over the sorted triangle set.
  **It costs ~10%** on the extract dispatch (1.17 → 1.28 ms at `--voxel 0.012`,
  Release, samples interleaved to cancel thermal drift) and buys **no**
  coalescing win —
  a triangle's three vertices were already written consecutively at `tri * 3`;
  the interleaving that cost coalescing is per-*vertex* and lives in the sharing
  kernel, which this does not touch. Nor is the index run monotonic within a
  block: slots are still handed out in whatever order cells reach the cursor.
  Justified as the precondition, not as a speedup; see the 2026-08-09
  incremental-extraction decision in `DECISIONS.md`, which records what did and
  did not move that number.
  A cell is visited twice but gathered ~1.08 times, and the two visits are
  asymmetric: the counting phase runs over 100% of cells and gathers **signs
  only** (`mcCellSigns` — no `sdf[8]`/colour array copy-out, no sRGB decode, for
  values a count cannot use), caching each cell's triangle count in one byte,
  four to a **private uint**. Private, not `shared`: both phases stride the
  block identically from the same thread id, so the writer is the cache's only
  reader — which costs the default path **zero** bytes of `shared` (44 B in
  total, against the sharing kernel's 8 428 B that the 2026-08-08 two-kernel
  decision exists to keep off this path), and drops an `atomicOr` and a zeroing
  pass with it. The emitting phase then gathers in full, but only the ~8% of
  cells that emit; the rest are rejected on a register byte. A grid whose block
  outgrows the four slots still meshes correctly and now says so, through
  `ExtractTimings::uncached_cells_per_block`.
  Contiguity is asserted directly, and exactly: triangles are attributed to
  blocks by centroid and the owning block must change exactly `distinct - 1`
  times walking the index buffer — on a clean extract, and on one that refit
  against an arena too small, over a 27-block run where the arena boundary falls
  inside one block's span and past others entirely. The golden sparse-vs-dense
  equivalence cannot see this: it compares triangles as a **set**.
  `mcEmitCell` splits into `mcCellTriangleCount` + `mcWriteTriangle`, so the
  per-block emitter and the dense kernel's per-triangle append still write
  through one body and cannot drift, and the cross-block corner addressing
  splits into `mcCornerStorage` so the two gathers resolve a corner through one
  copy of it. Both walks of a `tri_table` row are now bounded by the row
  (`kMaxTrianglesPerCell`) as well as by its `-1` terminator: the terminator is
  data the host uploads, and that count went from a loop trip to the size of a
  block's arena reservation.

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
