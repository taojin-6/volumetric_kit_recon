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
  preserving the active block set by re-inserting through the proven init /
  allocate / compact kernels (no new shaders). The grow is failure-atomic (the
  larger buffers are built off to the side and swapped in together), and the
  re-insert re-drives the snapshot to absorb transient lock contention. A GPU
  test (`tests/volume_resize_test.cpp`) proves 256 → 1024 growth, that the
  active set survives, and that allocation past the old capacity then works on
  MoltenVK. The block-index-preserving GPU rehash is deferred until blocks carry
  SDF data.
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
