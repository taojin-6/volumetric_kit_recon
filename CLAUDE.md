# volumetric_kit_recon

The production-grade **reconstruction + compression** compute backend of the
`volumetric_kit` family. It turns posed depth/RGB-D frames into a sparse TSDF
volume, extracts geometry, compresses it, and hands the result to the renderer
— all as a **Vulkan compute** workload, cross-platform, mirroring the renderer.

This file is the **living source of truth** for *why* and *how* this repo is
built. Decisions live here, not in chat history or commit messages — update this
file in the same change that alters a decision.

## The family

- **`volumetric_kit_recon`** (this repo) — capture → fusion → meshing →
  compression as **Vulkan compute** (GLSL → SPIR-V; MoltenVK on Apple), one code
  path across Linux / Android / macOS / iOS / Windows. Produces meshes / volumes
  / compressed TSDF bitstreams.
- **`volumetric_kit_gfx`** — the Vulkan/MoltenVK renderer. Consumes meshes /
  point clouds / glTF.

Both speak **Vulkan**, and both are **independent siblings**: each builds and
ships on its own. They meet at the interop seam (below) — and because both speak
Vulkan, that seam is the *easy* same-API case: they can share a `VkDevice` and
pass `VkBuffer`/`VkImage` directly, with none of the cross-API external-memory
machinery. `volumetric_kit_gfx` is the template this repo mirrors for both
conventions and Vulkan setup.

## Naming conventions (use these consistently)

- Package/repo: `volumetric_kit_recon`
- Namespace: `volumetric_kit::recon`. Internally and in docs, `vr::` abbreviates
  `volumetric_kit::recon::`.
- Headers: `include/volumetric_kit/recon/<tier>/…`.
- Macros: `VR_` prefix (`VR_TRY`, `VR_ASSIGN`, `VR_CHECK`, `VR_CORE_API`).
  Deliberately *not* the `VK_` prefix — that belongs to Vulkan. (The prior
  engine's `VK_DEVICE_HOST`-style macros are renamed `VR_*` on port.)
- CMake: `find_package(volumetric_kit_recon)`; component targets
  `volumetric_kit::recon_core`, `…_volume`, `…_tsdf`, `…_mesh`, `…_interop`
  (+ later `…_compress`, `…_sensor`, `…_track`, `…_codec`, `…_stream`); umbrella
  alias `volumetric_kit::recon`.

## Architecture (tiered)

Strict left-to-right dependency rule: a tier may depend only on tiers to its
left. No upward includes.

`core` → `volume` → `tsdf` → `mesh` → `interop` (later: `compress`, `sensor`,
`track`, `codec`, `stream`).

- **`core`** — the Vulkan foundation, mirroring `volumetric_kit_gfx`'s core:
  instance, device (compute + transfer queues), VMA allocator, RAII buffer/image,
  compute-pipeline + descriptor-set wrappers (and the `ComputeKernel` bundle +
  `KernelSetBuilder` that groups a kernel's layout/pipeline/set behind one
  shared pool), sync (fences, timeline semaphores), the `Status`/`Result` idiom,
  a pluggable log handler, and
  the GLM-backed vector/matrix math. Vulkan is reached through one umbrella header
  (`core/vulkan.hpp`), as in gfx — no other code includes `<vulkan/...>`
  directly.
- **`volume`** — the sparse voxel hash map in Vulkan buffers; allocate / compact
  / rehash as compute shaders. (POD layouts already landed in `volume/hash_types.hpp`.)
- **`tsdf`** — TSDF integration compute shaders (classic + dynamic).
- **`mesh`** — marching-cubes compute shaders, host mesh containers, and
  OBJ/PLY + glTF/GLB export.
- **`interop`** — the handoff to `volumetric_kit_gfx` (below).

## Locked decisions

Each dated; newest context wins. Change the decision *and* this list together.

- **2026-06-21 — Single Vulkan path (MoltenVK on Apple), like gfx.** Compute is
  Vulkan compute (GLSL → SPIR-V), one path across Linux / Android / macOS / iOS /
  Windows — chosen over a Metal + CUDA split for cross-platform reach and a
  single shader source. *Supersedes* the earlier "Metal-first / CUDA-later"
  decision. Native Metal is not pursued (MoltenVK runs our GLSL); native CUDA is
  now a planned NVIDIA accelerator, *not* the baseline — see 2026-07-04.
- **2026-06-21 — Trivial interop (same Vulkan API).** Because the renderer is
  also Vulkan, recon and gfx share **one** `VkDevice` and pass `VkBuffer`/`VkImage`
  directly (zero-copy needs a *single* shared device — see the 2026-07-04 interop
  refinement below; UUID-matched *separate* devices would still need an import).
  The cross-API external-memory
  machinery (CUDA↔Vulkan UUID import, Metal-objects, the MoltenVK shared-event
  export) is **not needed** for the recon→gfx path. This is the main reason for
  the Vulkan choice. (Exception: when the native-CUDA accelerator [2026-07-04]
  produces the geometry, the CUDA→gfx handoff *does* need the CUDA↔Vulkan
  external-memory import — or an extra copy back into a `VkBuffer` — a cost borne
  only on the CUDA path.)
- **2026-06-21 — Independent siblings; gfx untouched.** recon and gfx stay
  standalone. Now that both are Vulkan, a shared `volumetric_kit_core` is
  attractive and may be revisited — but for now each mirrors the Vulkan core
  independently so neither repo's release is coupled to the other. We keep the
  `Status`/`Result` shape close to gfx's so a later extraction stays cheap.
- **2026-06-21 — Vertical slice first.** v1 = `core` → `volume` → `tsdf` →
  `mesh` → `interop`. Compression, sensor capture, tracking, codecs are later
  tiers, added once the spine renders end-to-end.
- **2026-06-21 — Codec ships DCT-only.** KLT (trained basis) and the iQuantizer
  refinement are excluded as research.
- **2026-07-04 — Native CUDA accelerator, under the Vulkan baseline.** Vulkan
  compute (GLSL → SPIR-V) stays the cross-platform baseline — it is the only path
  that reaches every target (Linux, Android, macOS, iOS/iPadOS via MoltenVK);
  CUDA reaches none of Android/Apple and only NVIDIA on Linux/Windows. On top of
  that baseline we add a **native CUDA backend as an NVIDIA performance
  accelerator**, layered under the Vulkan path and ported per-kernel where
  profiling justifies it — *prove the gain on a hot kernel (e.g. TSDF integrate)
  before porting the rest*, since these fusion kernels are bandwidth/atomic-bound
  and the CUDA-vs-Vulkan gap is often small. Cost carried knowingly: two kernel
  sources (GLSL + CUDA) kept numerically in lockstep, a backend seam in the
  compute tiers, and the CUDA↔Vulkan interop exception noted above. *Amends* the
  2026-06-21 single-Vulkan-path decision (CUDA: "possible" → "planned"); it does
  **not** demote Vulkan.
- **2026-07-04 — GLM for host/device math (dropped the hand-rolled POD types).**
  The `vr::Vec3f/Vec3i/Vec4f/Mat4f` vocabulary aliases GLM instead of hand-rolled
  structs. GLM gives tested math, byte-for-byte packed layouts for the Vulkan
  buffer ABI (via scalar block layout, since `std430` 16-byte-aligns a `vec3` --
  see 2026-07-05), `__host__ __device__` operators for the CUDA accelerator, and
  parity with gfx (which also uses GLM) so the interop seam needs no vector
  conversions. `core` therefore takes one header-only external
  dependency (GLM); Eigen was rejected (its alignment + expression templates fight
  a GPU-upload POD contract). The `vr::` names stay so the backing type is
  swappable, and `vr::normalize` keeps a zero-length guard GLM lacks.
- **2026-07-04 — Zero-copy interop = one shared `VkDevice` + a create/adopt seam
  (refines "Trivial interop" above).** Being both-Vulkan removes the *cross-API*
  machinery, but not all of it: a `VkBuffer` is valid only on the `VkDevice` that
  created it, so zero-copy requires a **single shared `VkDevice`** (one process) —
  *not* "UUID-matched compatible devices," which would still need external-memory
  FD import (the same cost the native-CUDA→gfx handoff pays; see the CUDA
  accelerator decision above). Each `core` therefore exposes `Device::create`
  (owns) *and* `Device::adopt` (borrows, verifying against a published
  `DeviceRequirements`); a neutral app-side bootstrap builds one device from the
  union of both libraries' requirements and hands it to each, so both stay
  standalone *and* compose. The live textured mesh is the target: variable
  topology via `vkCmdDrawIndexedIndirect`, a ring of mesh/atlas slots, an
  intra-device timeline-semaphore handoff (the MoltenVK external-semaphore caveat
  does not apply on one device). Authoritative detail: DESIGN.md → "The interop
  seam".
- **2026-07-05 — Shader buffer ABI is scalar block layout, not `std430`.** The
  host POD structs (`HashEntry`, …) embed `Vec3i` voxel-block coords, which
  `std430` 16-byte-aligns — so a naive `std430` shader mirror does *not* match the
  packed C/CUDA layout the host uploads (`HashEntry`: host `pos` at 4 in 20 B;
  `std430` `pos` at 16 in 32 B). The compute shaders therefore declare
  `layout(scalar)` (`GL_EXT_scalar_block_layout`; core in Vulkan 1.2, supported by
  MoltenVK), under which the GLSL struct is byte-identical to the host struct for
  every vector-bearing type — no per-field scalarization, one POD definition
  across CPU / GLSL / CUDA. *Refines* the 2026-07-04 GLM decision
  ("`std430`-compatible" was imprecise for `vec3`-bearing structs); the host-side
  `static_assert`s guard only the host packing.
- **2026-07-05 — Compute core is explicit, not reflected; dispatch via
  `submit_single_time`.** The Vulkan compute foundation mirrors gfx's core (VMA
  `Allocator`, RAII `Buffer`, `ShaderModule`, descriptor + `ComputePipeline`
  wrappers, a `UniqueHandle` owner for pure-Vulkan handles) with two deliberate,
  lean divergences. (1) **No SPIR-V reflection** — `ComputePipeline::create`
  takes explicit descriptor-set layouts + push-constant ranges rather than
  reflecting them from the shader, so the tier vendors only VMA (gfx pulls in
  spirv-cross); the caller declares bindings matching the shader's
  `layout(set=, binding=)`. (2) **No standalone `CommandBuffer` / `Fence` /
  `CommandPool` types yet** — a one-shot `Device::submit_single_time(record_fn)`,
  shared-queue-safe via `Device::queue_submit` + `submit_mutex`, is the dispatch
  primitive; reusable command buffers and timeline-semaphore sync land when a
  fusion tier actually batches dispatches. VMA is a private dependency (only
  `allocator.cpp` and the one `VMA_IMPLEMENTATION` TU, `vma_impl.cpp`, include
  `<vk_mem_alloc.h>`; `SYSTEM` include). Revisit reflection if the volume/tsdf
  binding boilerplate grows painful.
- **2026-07-05 — Per-voxel storage is a structure-of-arrays attribute store
  (`VoxelBlockGrid`), not the prior engine's AoS `Voxel`-in-the-hashmap.**
  Following Open3D's `VoxelBlockGrid`, `VoxelHashMap` keys only a block *index*;
  each per-voxel channel (`tsdf`, `weight`, `color`, …) is its own flat device
  array of `num_blocks·voxels_per_block` elements, keyed by `BlockIndex::ptr`,
  declared and allocated independently. *Diverges* from the monolithic prior
  engine (which bundled a single AoS `Voxel{sdf,weight}` buffer inside the
  hash-table struct): the win is à-la-carte + tiered — a `volume`-only user
  (pure spatial hashing) allocates **zero** per-voxel memory, and each consumer
  materialises only the channels it needs (TSDF writes `tsdf`+`weight`, a colour
  pass writes `color`, meshing reads `tsdf`) instead of forcing the full SDF
  volume (~6 GB at prod defaults) onto every map. The store lives in `volume`
  (below every consumer tier); the AoS `Voxel`/`VoxelData` PODs in
  `hash_types.hpp` become host-side read views.
- **2026-07-06 — Per-kernel resources are bundled (`core/compute_kernel.hpp`),
  still not reflected.** The volume tier had grown parallel `*_layout_` /
  `*_pipeline_` / `*_set_` members (21 fields for 7 kernels) plus a hand-summed
  descriptor-pool size — the boilerplate the 2026-07-05 decision flagged.
  Resolved *without* SPIR-V reflection, in `core` because every compute tier
  repeats the shape: a `ComputeKernel` struct bundles one kernel's
  `{layout, pipeline, set}`; a `KernelSetBuilder` registers each kernel (its
  SPIR-V + storage-buffer binding count + optional push range), auto-sizes the
  shared pool from the exact descriptor total, and allocates every set (resolving
  the pool chicken-and-egg the tier used to hand-sum); and a free
  `dispatch(Device&, const ComputeKernel&, push, push_size, groups, max)` carries
  the workgroup-limit guard + the COMPUTE→COMPUTE/HOST barrier. Seven kernels'
  setup drops from ~80 lines to seven `kb.add(...)` calls. The **mechanism**
  (kernel bundle / pool builder / dispatch) is generic and lives in `core`; the
  **policy** — which buffers bind to which slot — stays in `volume`
  (`write_persistent_bindings`). A reusable buffer-slot table is deliberately
  *not* built yet: it waits until `tsdf` is a second consumer whose buffer shape
  can be generalized against, rather than over-fitting one now. The caller still
  declares each shader's binding count by hand, so this answers the 2026-07-05
  "revisit reflection" note with a builder, not reflection — the tier still
  vendors only VMA.
- **2026-07-06 — Hybrid color renders through a gfx pipeline (amends "interop
  seam A needs zero gfx changes").** The renderer's shipped PBR path is
  texture-only and ignores per-vertex `color`, so `volumetric_kit_gfx` grew a
  **`HybridMeshPipeline`** (its long-anticipated "mesh pipeline"): per fragment
  it samples the projective-texturing atlas where a triangle carries a valid
  `uv0`, else uses the per-vertex `color` (the `uv0 = (-1,-1)` sentinel). A
  general vertex-color/atlas pipeline is a broadly-useful renderer feature, so
  the siblings stay independent — gfx gained a capability, not a dependency on
  recon. Recon's `mesh::Vertex` accordingly carries `{position, normal, color,
  uv0}`; marching cubes interpolates per-voxel `color` (the RGB the `tsdf` tier
  fuses into the volume) to `Vertex::color` and leaves `uv0` at the sentinel, and
  the projective-texturing pass (a later slice) fills real atlas UVs. The
  interop-seam converter still maps recon's vertex to gfx's `{…, tangent, uv0,
  color}` shape (synthesizing `tangent`). *Amends* the 2026-06-21 "zero gfx
  changes" stance in the interop seam below.

## Provenance & salvage policy

The algorithms here are a clean re-implementation of the proven core of the
prior reconstruction engine (on-disk directory `implicit_world_reconstruction`).
Its *implementation* is good and is what we port; what we leave behind is its
name and its prototype-grade packaging, not its numerics. The older research
codec (directory `implicit_surface_compression`) is **reference-only**. Both
source repos are left untouched on disk — never build or write in them.

- **Port (with refactor) from the prior engine:** `core/{math,types}`,
  `voxel_hashing`, `tsdf`, `mesh_extraction`, `mesh_io`. Its Metal/CUDA kernels
  are **re-implemented as Vulkan compute shaders (GLSL → SPIR-V)** for the
  baseline, with an **optional native-CUDA port as the NVIDIA accelerator**
  (2026-07-04) — the algorithms transfer, the kernel source does not.
- **Hardening applied on port:** add install/export + package config; replace
  glog/fmt with the pluggable log handler; adopt `Status`/`Result` repo-wide;
  rename `VK_*` device macros → `VR_*`; break the `ReconstructionPipeline`
  god-object so each tier has a standalone API; drop committed build artifacts,
  hardcoded paths, and machine-specific config.
- **Drop:** the prior engine's own renderer (gfx replaces it) and its SwiftUI
  demo app (reference only). Its sensor driver framework is rebuilt behind a
  clean `ICameraCapture` in the later `sensor` tier.

## Excluded from the public release (experimental — never ship)

Production-only is a **hard rule**. Dropped completely — not ported, not
deferred, not stubbed. When in doubt, it stays out.

- **Triplane neural fields** (the prior engine's `triplane/` and the Python
  blockwise-triplane NeRF trainer/viewer) and their **tiny-cuda-nn** dependency.
- **KLT transform** and the **iQuantizer** post-quant refinement (research-grade;
  codec is DCT-only).
- **Python research/eval harnesses** and any learned/neural or paper-experiment
  code. These remain in the prior repos for research; none enter this repo.

## The interop seam (pairing with gfx)

Two contracts — both simpler now that recon and gfx are both Vulkan.

- **A — Data handoff (v1).** The `mesh` tier emits geometry in gfx's exact
  ingestion shape (interleaved `Vertex{position,normal,tangent,uv0,color}` +
  `uint32_t` indices) or serializes **glTF/GLB**, which gfx's `load_gltf` already
  ingests. Needs zero gfx changes. The converter reconciles the impedance
  mismatches: merge the prior engine's two-stream vertex into one interleaved
  layout, synthesize `tangent`, widen `color` to vec4, `int32`→`uint32` indices,
  bake a per-triangle atlas into per-vertex `uv0` + a `Material` texture (gfx has
  no per-triangle UV). The triangle-mesh path is the first milestone — a
  `PointCloud` handoff waits on gfx growing a point-splat pipeline (it renders
  only meshes today).
- **B — Shared Vulkan resources (zero-copy; the live target).** recon writes the
  mesh/atlas into a `VkBuffer`/`VkImage` on a **single `VkDevice` shared with gfx**
  (one process) and gfx draws it directly — no external-memory import. Realized by
  the create/adopt device seam: a neutral bootstrap builds one device from both
  libraries' merged `DeviceRequirements`; two queues from one graphics+compute
  family avoid any queue-family ownership transfer; the handoff is an intra-device
  timeline semaphore over a ring of mesh/atlas slots, variable topology drawn
  indirectly. See DESIGN.md → "The interop seam".

## Key gotchas (verified)

- **MoltenVK is the Vulkan driver on Apple** — there is no other. Validate
  MoltenVK *compute* on the target Apple GPU early (prove the path before
  building on it, the gfx playbook). Metal supports compute; MoltenVK translates
  Vulkan compute → Metal compute.
- **Vulkan via the link-time loader through one umbrella header**
  (`core/vulkan.hpp`), exactly as gfx — never `#include <vulkan/...>` directly,
  so adopting volk later for the iOS/Android loader stays a one-header change.
- **Host buffer layout must match the shader.** Host POD structs (`HashEntry`,
  `Voxel`, …) and their GLSL mirrors must agree byte-for-byte, so the shaders
  read them through **scalar block layout** (`GL_EXT_scalar_block_layout`; Vulkan
  1.2 core, MoltenVK-supported), *not* `std430` — `std430` 16-byte-aligns a
  three-component vector, so an `std430` `HashEntry` puts `pos` at offset 16 in
  32 B where the host packs it at offset 4 in 20 B (see the 2026-07-05 ABI
  decision). Scalar layout is the C/CUDA layout, so one struct maps 1:1 across
  CPU / GLSL / CUDA. The `static_assert`s on the C++ side guard only the host
  packing; keep the `layout(scalar)` GLSL definitions in lockstep.
  `Device::create` enables the `scalarBlockLayout` feature (and `adopt` requires
  the creator did); `vr_compile_shaders` validates the emitted SPIR-V with
  `--scalar-block-layout`.
- **GLSL compute is the baseline; CUDA is the optional accelerator.** In the
  Vulkan path, warp/wave tricks become Vulkan subgroup ops and device atomics use
  GLSL atomics; the prior engine's kernels are a reference for the *algorithm*,
  rewritten in GLSL. The native-CUDA accelerator (2026-07-04) keeps its warp
  intrinsics/atomics but must stay numerically in lockstep with the GLSL path.

## RAII resource types (handle/deleter wrappers)

Every type that owns a Vulkan/VMA handle — or a deleter that frees one — follows
the same shape (gfx's rules; the mistakes reviews keep catching):

- **Move-only.** `= delete` copy ctor/assign; `= default` (or hand-write) the
  move pair. A copyable wrapper double-frees.
- **Reset *every* owned member on each ownership transfer** — move ctor, move
  assignment, *and* `destroy()`. Null the handle *and* zero the metadata and the
  deleter, so a moved-from/destroyed object is fully empty and its accessors stay
  consistent with `valid()`.
- **`operator=` guards self-move** and runs `destroy()` on the current state
  before adopting the source's.
- **Type-erase the backend via a `std::function<void()>` deleter** so VMA stays
  out of public headers. Reset the moved-from deleter to `nullptr`.
- **Validate before creating** — reject zero size/extent, `usage == 0`, etc.,
  with a non-OK `Status` before touching Vulkan/VMA.
- **Tests for every move-only type:** move-construct (assert the *source* is
  empty), move-assign over a live object, and self-move (launder through a
  pointer to dodge `-Wself-move` under `-Werror`). The sanitizer CI job turns
  those into real leak/double-free detectors.

## Working preferences

- Implement every change in a dedicated **git worktree** under `.worktrees/`
  (e.g. `git worktree add .worktrees/<branch> -b <branch>`); remove it when
  merged. **All work happens in this repo only** — never build or write in the
  prior engine's tree.
- Use **absolute paths** for `cmake -S/-B`, `git -C`, and file ops; the
  workspace holds several sibling repos and a leaked `cd` lands you in the wrong
  one.
- Conventional Commits (`feat(volume): …`, `build: …`, `refactor(core): …`).
- When opening a PR, **assign yourself** (`gh pr create --assignee @me`) so it
  lands on your board and ownership is unambiguous.
- Mark deferred work inline with a greppable `TODO:` comment.
- Prefer plain, behavior-level tests over friend-class backdoors.
- Full Doxygen on public classes/functions, matching
  `include/volumetric_kit/recon/core/result.hpp`. Don't write "move-only" in
  prose — the deleted-copy/defaulted-move declarations convey it.

## Where to start

Current state: the `core` foundation + the full Vulkan **compute foundation**
(VMA `Allocator`, RAII `Buffer`, `ShaderModule`, descriptor + `ComputePipeline`,
the shared-queue-safe `Device::submit_single_time` dispatch; `Device` also
enables `scalarBlockLayout`), proven by the compute smoke on MoltenVK. On top of
it, the first **`volume` tier** slice — the sparse voxel hash map: the host
`VoxelHashMap` (`volume/voxel_hash_map.hpp`) owns the device buffers + pipelines
and drives **init / allocate-from-coords / -depth / -points / remove / compact /
compact-in-frustum / resize** via GLSL kernels (`volume/shaders/hash_*.comp`) that read
`HashEntry`/`BlockIndex` through the scalar-block-layout ABI (2026-07-05), plus a
host-side **diagnostics** scan (occupancy + collision-chain health).
Depth/point allocation unprojects a posed depth frame (via `DepthCameraParams`
intrinsics+pose, uploaded through the same scalar ABI) or takes world points,
and dilates each surface block into the `(2·tb+1)³` truncation band — the prior
engine's solid block cube, *not* a ray march; the shared insert +
band-dilation live in `hash_allocate_common.glsl`, the coord transforms in
`hash_common.glsl`. Frustum-culled compaction (`make_frustum_planes` in
`volume/frustum.hpp` + an on-device p-vertex AABB cull) filters the active set to
a camera view — the per-frame streamed working set. The kernels are embedded into
`recon_volume` (a compiled STATIC tier) via `cmake/vr_embed.cmake`, and GPU tests
(`tests/volume_{hash_map,allocate,frustum,delete,resize,diagnostics}_test.cpp`)
prove each op + the on-device `HashEntry` layout round-trip on MoltenVK. Host
coord/hash math + POD layouts are in
`volume/{voxel_coords,hash,frustum,voxel_grid,hash_types}.hpp`.
On top of the map, **`VoxelBlockGrid`** (`volume/voxel_block_grid.hpp`) composes
it with a set of independently-allocated, named per-voxel **attribute** arrays
(SoA — `tsdf`, `weight`, `color`, …), each `num_blocks·voxels_per_block` and
keyed by `BlockIndex::ptr`, so a consumer materialises only the channels it needs
(the 2026-07-05 SoA decision); `tests/volume_block_grid_test.cpp` proves
independent, correctly-sized attribute storage on MoltenVK.

The first **`tsdf` tier** slice then lands on top: `TsdfIntegrator`
(`tsdf/tsdf_integrator.hpp`) fuses a posed depth frame (float metres, reusing
`volume::DepthCameraParams`) into a `VoxelBlockGrid`'s `tsdf`/`weight` attributes
via `tsdf/shaders/tsdf_integrate.comp` — one thread per voxel of each active
block, classic projective `sdf = depth − Zc` with `±trunc_dist` truncation, an
inverse-square-with-behind-dropoff weight, and a running average capped at
`max_weight` (faithful to the prior engine's classic kernel; node-centred voxels
matching `voxel_to_world`). An `IntegrationMode` selects **classic** (keep free
space ahead of the surface, a smooth field) or **dynamic** (clear stale geometry
there so a receded surface leaves no ghost) — one kernel branch, the prior
engine's stale-free-space clearing. `tests/tsdf_integrate_test.cpp` fuses a
constant-depth plane, checks the per-voxel numerics under a rotated pose, and
proves dynamic clears a receded-surface voxel that classic keeps, on MoltenVK.
Depth is sampled **bilinearly** (nearest fallback at image edges and across depth
discontinuities `> trunc_dist`), the prior engine's `sampleDepthBilinear`.

The first **`mesh` tier** slice — GPU marching cubes — lands alongside it. The
host `MarchingCubes` (`mesh/marching_cubes.hpp`) owns the compute pipeline and
drives a GLSL kernel (`mesh/shaders/marching_cubes.comp`) that turns a **dense**
`volume::Voxel` SDF grid into a triangle `Mesh` (`mesh/mesh.hpp`): one invocation
per cell, cube index from the eight corner signs, independent triangles appended
via an atomic counter, per-cell outward gradient normals, winding flipped so the
front face (CCW) points outward for gfx. Each vertex also carries the hybrid
appearance the renderer's `HybridMeshPipeline` consumes — a per-vertex `color`
interpolated from an optional per-voxel color input (opaque white when absent)
and a `uv0` at the `(-1,-1)` sentinel until projective texturing fills it. The
256-case tables (`mesh/marching_cubes_tables.hpp`, ported verbatim) are uploaded
once as an SSBO — one definition across CPU/GLSL, mirroring the volume ABI
discipline. A GPU test (`tests/marching_cubes_test.cpp`) extracts an analytic
sphere and verifies radius, outward normals, winding, and the interpolated
vertex color on MoltenVK. `mesh` depends only on the
`volume` voxel payload (a tier to its left, so the strict dependency rule holds)
and is proven against a dense analytic SDF until it reads `tsdf`'s real blocks.

Next: **colour** (a `color` attribute).
A block-index-preserving GPU **rehash** + heap-rebuild then preserves per-voxel
data across a `resize` (which currently reassigns block indices, discarding the
`tsdf`/`weight` a block held). On the `mesh` side (greppable `TODO`s): extraction
straight off the sparse `VoxelHashMap` with cross-block neighbour sampling,
shared-vertex dedup + the incremental block-mesh pool, and OBJ/PLY + glTF/GLB
export + the gfx-vertex converter (interop seam A).
