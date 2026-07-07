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
  hash table's internal buffers. When the grid carries a `uint32` `color`
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
