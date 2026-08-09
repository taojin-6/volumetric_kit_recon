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
  `volumetric_kit::recon_core`, `…_volume`, `…_tsdf`, `…_mesh`, `…_texture`,
  `…_interop` (+ later `…_compress`, `…_sensor`, `…_track`, `…_codec`,
  `…_stream`); umbrella
  alias `volumetric_kit::recon`.

## Architecture (tiered)

Strict left-to-right dependency rule: a tier may depend only on tiers to its
left. No upward includes.

`core` → `volume` → `tsdf` → `mesh` → `texture` → `interop`, with `sensor`
branching off **`core`** (later: `compress`, `track`, `codec`, `stream`).

- **`core`** — the Vulkan foundation *and* the vocabulary every tier trades in
  (`Status`/`Result`, the GLM math aliases, and the posed pinhole
  `DepthCameraParams`/`ColorCameraParams` of `core/camera_params.hpp`),
  mirroring `volumetric_kit_gfx`'s core:
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
- **`texture`** — projective texturing: fills the mesh's per-vertex `uv0` with a
  posed camera's image coordinates where it has line of sight (per-vertex-color
  fallback elsewhere), a compute pass.
- **`sensor`** — the capture *contract*: `ICameraCapture`, the `CapturedFrame`
  view the fusion tiers consume, and the boundary conversions a capture
  integration gets silently wrong — camera conventions (pose handedness,
  registered-depth intrinsics) and colour (`to_canonical`). Reads
  `DepthCameraParams` + `ColorCameraParams` from `core/camera_params.hpp` and
  `ColorEncoding` from `core/color_space.hpp`, so it
  depends on **`core` alone** — it sits beside the fusion tiers, not on top of
  them — and bundles **no drivers**: one ships here only if this repo can build
  *and* test it (the 2026-08-02 decision).
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
  vendors only VMA. (2026-07-08: the other per-tier duplicates the same decision
  flagged — the dispatch group-count ceil-divide and the host-visible
  storage-buffer create/upload helpers, copied verbatim into every tier — moved
  to `core/compute_util.hpp` (`group_count` / `storage_buffer` /
  `upload_storage_buffer`) once the `texture` tier would have been the fourth
  copy; `texture` and `mesh` consume them now — `mesh` migrated on 2026-08-02
  rather than fork its copy to carry the `MarchingCubesConfig` usage bits, which
  is why `storage_buffer` grew its `extra_usage` parameter — and the `volume` /
  `tsdf` copies migrated on 2026-08-04 — forced rather than chosen: `volume` and
  `tsdf` had to include `compute_util.hpp` for the new range guard, and their
  private `storage_buffer` copies then made every unqualified call **ambiguous**
  by ADL against the `core` one, since `Allocator` names `recon` as an
  associated namespace. Each tier keeps only its one-argument `group_count`
  wrapper, which bakes in that tier's `local_size` and cannot collide with
  `core`'s two-argument overload.)
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
  changes" stance in the interop seam below. (*Superseded in part on 2026-08-02*:
  `mesh::Vertex` now **is** gfx's layout, so there is no shape to map — see that
  decision.)
- **2026-07-06 — Depth sampling is texture-centred (pixel centres at i+0.5), a
  deliberate ~½-pixel convention.** The `tsdf` bilinear depth sampler
  (`tsdf_integrate.comp::sample_depth`) shifts its 2×2 taps by −0.5 and takes the
  containing pixel (`floor(u)`, `floor(v)`) as its nearest-neighbour fallback —
  the GPU-texture convention, self-consistent across interpolation *and* fallback.
  recon's projection (`u = fx·x/z + cx`) and the `volume` unprojection
  (`hash_allocate_depth.comp`: `x = (u − cx)·d/fx` on integer pixels) are instead
  integer-centred, so an on-axis voxel (`u = cx`) samples the 50/50 blend of the
  two pixels straddling `cx` rather than the principal-point pixel. We keep the
  offset knowingly: it faithfully ports the prior engine's `sampleDepthBilinear`
  taps, and we further make the *fallback* texture-centred (`floor`) where the
  prior engine inconsistently *rounded* to an integer centre while shifting its
  taps. Revisit only if the depth intrinsics are ever calibrated integer-centred —
  then drop the −0.5 shift and round the fallback so the sampler matches the
  projection.
- **2026-07-07 — The viewer example opts into gfx behind `VR_BUILD_VIEWER`
  (amends "Independent siblings; gfx untouched").** The gfx-linked reconstruction
  viewer examples (`examples/viewer/`: `fuse_render`, a headless colour-PNG
  render, and `fuse_viewer`, a live window) pull `volumetric_kit_gfx` via
  FetchContent + system GLFW — the **only** place the recon tree touches the
  renderer, behind an **off-by-default** `VR_BUILD_VIEWER`. The library tiers and
  the default build stay renderer-independent (verified: a default configure
  fetches no gfx and builds the gfx-free `fuse_replica`). *Amended 2026-08-02:*
  **one** CI leg does opt in — `.github/workflows/viewer.yml`, build-only, macOS,
  scoped to `fuse_viewer` + `fuse_render`. Leaving it unbuilt was worse: this
  directory is the single place recon's code meets gfx's API, and it silently
  drifted out of build (`fuse_viewer.cpp` declared `Result<Mesh>` against
  `extract_device`'s `Result<DeviceMesh>`, so `VR_BUILD_VIEWER=ON` failed at
  `8a439fc` with nothing reporting it). Every *other* leg stays gfx-free, so the
  renderer is still absent from everything that gates the library itself. This
  amends the independence stance *for an opt-in example only* — recon's
  **release** is still never coupled to gfx; a developer who wants the live
  viewer opts in and pays the gfx fetch. The alternative (a standalone neutral
  repo) was weighed and
  set aside for discoverability: the example lives with the pipeline it demos.
  *Amended 2026-08-02 (below):* the device-adoption proof that used to live in
  that standalone repo now lives here too — `fuse_viewer` runs on **one shared
  `VkDevice`**. The mesh **handoff** was still a host mesh (interop seam A); the
  shared device is the precondition seam B needs, not seam B itself.
  *Amended 2026-08-08 (below):* `fuse_viewer` now draws recon's buffers
  directly — seam B — so the precondition has been collected on.
  `fuse_render` deliberately stays two-device seam A.

- **2026-07-07 — Projective texturing is a new `texture` tier; live
  single-camera first.** Filling the mesh `Vertex::uv0` (the atlas coordinate the
  2026-07-06 hybrid-colour decision reserved) lands as a **new `texture` tier**
  between `mesh` and `interop`, not a `mesh` add-on: it is a distinct pass over a
  mesh + a posed camera that depends on `volume` (`DepthCameraParams` + the
  `project_to_image` ABI) as well as `mesh`, mirroring the prior engine's
  separate `projective_texturing` module. The first slice is **live,
  single-camera** (chosen over a post-scan multi-keyframe atlas): one thread per
  triangle projects its three vertices into the current frame's camera and keeps
  the triangle only when all three are in front, in-frame, and **unoccluded** —
  their projected depth agrees with the frame's depth map within an occlusion
  threshold (default 0.02 m), the *line-of-sight* test. A kept triangle's
  vertices get `uv0 = (pixel + 0.5)/size` (the prior engine's half-texel atlas
  UV, no y-flip); the rest get the `(-1,-1)` sentinel, so gfx's
  `HybridMeshPipeline` textures where a camera saw the surface and falls back to
  the fused voxel colour elsewhere — and every call **overwrites** `uv0`, so a
  triangle that leaves the view reverts to the fallback. The atlas is just the
  camera image the caller binds (single-camera: no packing, no per-texel baking,
  full sensor resolution), so **keyframe retention is unneeded** — the current
  frame *is* the atlas. The prior engine's multi-camera score + winner-take-all
  vertex atomic are **dropped here**: one camera needs no ranking, and the mesh
  tier's independent triangles (no shared vertices) let each thread own its three
  `uv0` writes. A later slice restores both for a post-scan multi-keyframe atlas
  (best-of-N view, packed atlas, the Metal two-step 32-bit atomic once a dedup
  slice shares vertices). Ported faithfully from
  `implicit_world_reconstruction`'s `projective_texturing` (the Metal kernel
  shape); `texture_common.glsl`'s projection mirrors `tsdf_common.glsl`'s
  `project_pinhole`. The occlusion depth sampler shares the tsdf sampler's
  *intent* — bilinear with hole renormalisation, plus a **depth-discontinuity
  fallback** to the nearest tap (keyed to `occlusion_threshold`, since this tier
  carries no `trunc_dist`) so a foreground vertex on a silhouette is not rejected
  by a blended foreground/background depth — but is **integer-centred** (no −0.5
  tap shift), matching this tier's integer-centred projection and half-texel
  atlas UV rather than the tsdf sampler's texture-centred convention (the
  2026-07-06 depth-sampling decision); the two are self-consistent within their
  own tier. The kernel bounds its `indices → vertices` addressing with a
  `num_vertices` push constant (a malformed/loaded mesh whose index is out of
  range is skipped, not an out-of-bounds SSBO write), and the host rejects a
  vertex/index/depth buffer past `maxStorageBufferRange` with a clean `Status`
  (the mesh tier's arena guard).
- **2026-08-01 — Perf instrumentation starts in the viewer example, not a shared
  contract package.** The question left open when a private
  `volumetric_kit_interop` repo was stood up to hold a shared `FrameMetrics`
  (and judged "too thin") is settled the lean way: **the viewer example owns the
  instrumentation**. `fuse_viewer` already links both siblings, so it fills gfx's
  plain-data `gfx::FrameMetrics` directly from recon-side timings and draws it
  with gfx's shipped `ui::ImGuiOverlay` + `ui::draw_metrics_panel` — no shared
  package, no metrics contract in recon, and *no recon→gfx dependency* (the
  coupling stays inside the already-opt-in `VR_BUILD_VIEWER` example, per the
  2026-07-07 decision). The compute tiers stay profiler-free. `core` gains only
  `Allocator::memory_stats()` (per-heap usage/budget over `vmaGetHeapBudgets`,
  mirroring gfx's identically-named API) — a genuine library capability, not a
  metrics framework. Promoting a shared contract waits for a **second** consumer
  (a headless recon benchmark, or production telemetry), the same
  wait-for-the-second-consumer rule the 2026-07-06 kernel-registry decision
  applied. Two consequences carried knowingly: (1) recon's stage rows are
  **wall-clock CPU**, not a CPU/GPU split — every recon dispatch goes through
  `submit_single_time`, which blocks on a fence, so the span around a call is
  that stage's true end-to-end cost (host record *plus* device execution) and is
  reported with `has_gpu` false rather than as a fabricated device measurement;
  the renderer's own rows *do* carry real GPU spans, since gfx's `Profiler`
  writes timestamp queries and this GPU reports 64 `timestampValidBits` through
  MoltenVK. Splitting recon's host from device time needs a query pool plumbed
  through `submit_single_time` — the greppable follow-up. (2) VMA's `usage`
  counts allocated *blocks*, not live sub-allocations, and VMA pools an emptied
  block, so the reported figure is a **high-water mark that does not fall when a
  buffer is freed** (`tests/core_memory_stats_test.cpp` pins exactly that); the
  numbers are VMA estimates until `VK_EXT_memory_budget` is enabled, a
  `TODO(core)` on the device seam.
  *Amended the same day:* "the compute tiers stay profiler-free" holds for
  *frameworks*, not for measurement — a tier may report its own phase breakdown
  through an **explicit, opt-in out-param**, and `mesh::MarchingCubes::extract`
  is the first (`ExtractTimings*`, defaulting to `nullptr`). The bar it must
  clear: no global sink, no timing state retained between calls, no dependency
  added, and nothing measured when the caller passes `nullptr`. That is a
  by-value struct the caller owns, not a profiler in the tier — the aggregation,
  display, and history stay in the consumer (the viewer's overlay). It earns its
  place because the phases are *invisible from outside*: whole-volume meshing,
  a neighbour-resolution pass, and a worst-case arena allocation all hide
  inside one call, and picking between interop seam B and the incremental
  block-mesh pool needs their split. (The neighbour pass was a host table when
  this was written; the 2026-08-08 neighbour-probe decision moved it into the
  kernel on the
  strength of exactly this breakdown, and `neighbour_lut_ms` went with it.) Other tiers follow the same shape only
  when they have the same problem.

- **2026-08-01 — iOS is a downstream concern; recon cross-compiles to it
  unchanged.** The iOS app shell lives in a **separate sibling repo**,
  `volumetric_kit_ios` — an Xcode project, a bundle identifier, a signing team,
  and a provisioning profile have no business in a library that also ships on
  Linux and Windows. This is the *stronger* form of the 2026-07-07 viewer
  decision (which admitted an opt-in gfx example into this tree for
  discoverability): an app target is heavier than an example, so it goes fully
  outboard and recon keeps the "independent siblings" rule intact. Verified: the
  five library tiers build for `ios-arm64` (`platform 2`, minos 16.0) with **zero
  source changes here**. The whole platform port is one toolchain file
  downstream, because iOS ships **no ICD loader and no `libvulkan`** — MoltenVK's
  static library *is* the Vulkan implementation, linked directly (so: no
  validation layers on device), and seeding `Vulkan_LIBRARY` /
  `Vulkan_INCLUDE_DIR` with MoltenVK's iOS xcframework is enough for CMake's
  `FindVulkan` to build a `Vulkan::Vulkan` that satisfies every
  `find_package(Vulkan)` in this repo. This is the 2026-06-21 single-Vulkan-path
  decision paying off exactly as argued — MoltenVK carries recon to iOS with no
  second code path. Proven on an iPad Pro M5 (Apple M5 GPU, MoltenVK 1.4.2 /
  Vulkan 1.4.357) by an on-device four-stage smoke: device capabilities, a
  compute dispatch, a **scalar-block-layout ABI round-trip** (`Vec3i` block
  coords survive host → GLSL → host, so the 2026-07-05 ABI decision holds through
  MoltenVK's SPIR-V → MSL translation — the one genuinely at-risk assumption),
  and the full `allocate_from_depth` → TSDF integrate → marching-cubes spine. Two
  findings that shape the later tiers: the GPU reports a **host-visible
  `DEVICE_LOCAL` heap** (unified memory), so an ARKit `CVPixelBuffer` upload
  copies straight into GPU-visible memory with no staging blit; and
  `scalarBlockLayout` / `timelineSemaphore` / `dynamicRendering` are all present,
  so the shared-`VkDevice` interop seam (2026-07-04) is reachable on iOS. One
  packaging wrinkle stays downstream by design: Homebrew's glm 1.0.3 defines
  `glm::glm` as a *macOS* dylib, and `core` links that canonical name
  deliberately (older packagings lack `glm::glm-header-only`), so the iOS
  consumer overrides glm with a header-only copy rather than this repo changing a
  correct choice. The **`sensor` tier is where iOS actually lands in this tree**:
  an ARKit `ICameraCapture` source feeding `sceneDepth` (256×192 float metres),
  `capturedImage`, and `camera.transform` — note ARKit is +Y up / −Z forward
  while recon projects +Z forward, so poses convert as
  `T_world_cv = T_world_arkit · diag(1, −1, −1, 1)`, and ARKit's differing depth
  and colour resolutions are already modelled by the separate
  `DepthCameraParams` / `ColorCameraParams` the 2026-07-06 decision introduced.

- **2026-08-02 — `mesh::Vertex` *is* the renderer's vertex layout.** recon's
  vertex was `{position, normal, color, uv0}` (48 B) and gfx's is
  `{position, normal, tangent, uv0, color}` (64 B), so every mesh crossing the
  seam was rebuilt field by field. recon now emits gfx's layout directly:
  `mesh::Vertex` is byte-for-byte `gfx::assets::Vertex`, pinned by
  `static_assert`s on both sides of the seam (`mesh/mesh.hpp` and the bridge),
  and the marching-cubes kernels + the texturing kernel declare the matching
  `layout(scalar)` mirror. The host converter collapses to a bulk copy, and —
  the actual point — the bytes the kernel writes are **already in the shape the
  renderer binds**, which is what interop seam B needs. gfx's vertex-input
  description reads position/normal/uv0/color at exactly these offsets with this
  stride; it does not bind `tangent` at all. *Necessary, not yet sufficient*:
  the arena also has to be **bindable** (the usage bits — settled by the
  2026-08-02 `MarchingCubesConfig` decision below) and then actually
  **drawable** (lifetime, sharing mode, barrier scope, indirect counter — the
  seam-B `TODO(mesh)` on that struct).
  **The cost is paid knowingly, and it is not free.** Every vertex grows 48 → 64
  bytes (+33%) for a `tangent` slot marching cubes cannot produce — it has no
  surface parameterisation to derive one from, so the kernel writes the same
  `(1, 0, 0, 1)` placeholder the host converter used to synthesize. A
  recon-only consumer (`fuse_replica`, the codec tiers, a headless exporter)
  pays that for a field it never reads: measured on the 400-frame room0,
  **peak memory +11% and throughput −10%**, against a converter saving that
  only a gfx-linked consumer sees. It also moves a limit: the arena is
  `capacity · 3 · sizeof(Vertex)`, so the triangle count that fits under the
  `maxStorageBufferRange` guard drops by a quarter. The
  alternative — teaching gfx a
  vertex-input variant for recon's tighter layout — keeps recon's bytes but
  needs a change in the sibling repo plus a pin bump here, and leaves the
  seam-B buffer still un-bindable without one. We chose the renderer's
  convention because the seam is the point: the whole reason `uv0` and `color`
  live on this vertex at all is the 2026-07-06 hybrid-colour path. Revisit if a
  non-renderer consumer ever dominates the mesh tier's traffic — the tangent is
  then 16 bytes of dead weight per vertex with no offsetting win.
  (*2026-08-02:* that first consumer has arrived — the shared-device bootstrap
  below — and the usage half is now settled by the `MarchingCubesConfig`
  decision below.)

- **2026-08-02 — The neutral shared-`VkDevice` bootstrap lands in the viewer
  example, and prefers two families over a shared queue.** The 2026-07-04
  interop decision's "neutral app-side bootstrap" is now real and in-tree:
  `examples/viewer/shared_device.hpp` merges what each library publishes through
  `Device::requirements` (API floor, extension union, core-feature union, the
  `timelineSemaphore`/`scalarBlockLayout`/`dynamicRendering` chain, plus gfx's
  opaque `feature_chain`), creates one instance + device + surface, and hands the
  same handles to `vr::Device::adopt` and `vg::app::WindowedApp::adopt`. Neither
  library owns it; the bootstrap outlives both wrappers and destroys the device
  last. It lives beside the example it serves rather than in the standalone repo,
  for the same discoverability reason as the 2026-07-07 viewer decision.
  **Taking the merged API floor is load-bearing**: MoltenVK caps a physical
  device's advertised `apiVersion` to whatever the instance requested, so an
  instance at recon's 1.2 floor makes a 1.3-capable device fail gfx's check —
  the gotcha below, hit for real here.
  **The queue plan is a three-tier preference, and the order is the decision.**
  (1) One family with ≥ 2 queues: concurrent submission *and* no queue-family
  ownership transfer on a buffer recon writes and gfx reads — what the seam-B
  plan assumes. (2) Two families, one queue each: still concurrent, but a shared
  buffer will need `VK_SHARING_MODE_CONCURRENT` or an explicit release/acquire
  pair. (3) One family, one queue behind a mutex: fusion and rendering
  *serialize*. Measured on an Apple M5 Max (MoltenVK 1.4.2): the driver reports
  **four graphics+compute+present families of exactly one queue each**, so tier 1
  is **unreachable on Apple** and tier 2 is what actually runs
  (`family 0 (gfx) + family 1 (recon)`). Preferring tier 2 over tier 3 is the
  whole point — the pre-shared-device viewer got concurrency for free from two
  separate `VkDevice`s, and dropping to one shared queue would have handed that
  back to buy an ownership-transfer saving that **nothing yet collects**, since
  the mesh handoff is still a host copy. The cost is booked, not avoided: when
  seam B lands, its buffer is cross-family here and pays one of the two sharing
  mechanisms. Tier 3 remains for hardware that offers no second compute family,
  and says so on stdout when taken.
  Both `adopt` calls are fed from what `vkCreateDevice` actually saw — the
  enabled extension list, the merged `VkPhysicalDeviceFeatures`, and the three
  feature booleans are *recorded* by the bootstrap, never restated by hand in the
  payload builders: Vulkan cannot be asked what a device enabled, so those
  declarations are the only thing the `adopt` verification has to work with, and
  a hardcoded `true` would quietly turn the check into a no-op. Extensions and
  features are checked against the device before creation so a shortfall names
  itself instead of collapsing into "vkCreateDevice failed"; instance creation
  retries without `VK_KHR_portability_enumeration` (the iOS direct-MoltenVK case,
  mirroring `Instance::create`); and `--validation` is the embedder's own knob,
  since `WindowedApp::adopt` ignores gfx's `enable_validation` once the embedder
  owns the instance. Verified on room0: two families, 30/30 frames fused while
  the window renders, and **zero validation errors** with the layer on.
- **2026-08-02 — The `sensor` tier is a *contract*, not a driver collection: a
  capture driver lives here only if this repo can build **and test** it.** The
  tier's first slice is platform-neutral C++ — an `ICameraCapture` interface, a
  `CapturedFrame` POD, and the coordinate/intrinsics conversion helpers — with
  **no** driver implementations bundled in. The first real source, **ARKit,
  lives in `volumetric_kit_ios`**, not here. The rule that places it: a driver
  belongs in this repo when this repo's CI can compile and exercise it (a
  cross-platform C++ SDK — an Orbbec driver would qualify); otherwise it belongs
  with the platform that can. ARKit is iOS-only Objective-C, buildable only in
  an iOS cross-compile recon's CI does not run and verifiable only on LiDAR
  hardware, so hosting it here would mean an Objective-C++ TU that **no CI
  compiles and no test exercises** — unbuilt code that rots silently. *Refines*
  the provenance policy's "its sensor driver framework is rebuilt behind a clean
  `ICameraCapture`": the **interface** is rebuilt here; drivers are placed by the
  buildable-and-testable rule. This mirrors the sibling precedent exactly —
  gfx's windowing tier is *window-system* agnostic (`Surface` adopts a raw
  `VkSurfaceKHR`, `WindowedApp` takes a consumer-supplied `SurfaceFactory`),
  which is precisely why gfx ported to iOS with zero source changes and why the
  app, not the library, owns the `CAMetalLayer`. The `sensor` tier is
  *capture-system* agnostic for the same reason and buys the same portability.
  What deliberately **does** stay here is the conversion math, because it is the
  part most likely to silently ruin a reconstruction and it needs no hardware to
  test: (1) ARKit's camera is +Y **up** / −Z **forward** while recon projects +Z
  forward, so poses convert as `T_world_cv = T_world_arkit · diag(1, −1, −1, 1)`
  — get this wrong and the mesh is smeared or doubled, never an error; and (2)
  ARKit's depth (`sceneDepth`, 256×192 float metres) and colour
  (`capturedImage`, 1920×1440) come from **one physical camera**, so depth is
  *registered* to colour — the two share a pose and differ only by an intrinsics
  scale, which lands fusion in the registered case and avoids the
  occlusion/partial-colouring caveats `TsdfIntegrator::integrate` documents for
  an unregistered colour camera. (`sceneDepth.confidenceMap` gates low-confidence
  pixels to depth 0, so the integrator skips them.) Both are pinned by plain
  host-side unit tests before any Objective-C touches them. Revisit the
  placement if ARKit capture ever gains a second consumer — a headless capture
  tool, a visionOS target — since that is reuse across consumers rather than one
  app's platform glue; moving it down stays cheap precisely because the contract
  already lives here.
  **Two consequences of the implementer being out of tree, both found by review
  and fixed on the same PR.** (1) *The camera-parameter structs are `core`
  vocabulary, and the contract depends on `core` alone.* They had been placed by
  which tier first needed one — `DepthCameraParams` in `volume` (block
  allocation unprojects a depth frame), `ColorCameraParams` in `tsdf` (the first
  tier that fuses colour) — which split a matched pair across two tiers and, far
  worse, buried both inside `voxel_hash_map.hpp` / `tsdf_integrator.hpp`, which
  reach `core/vulkan.hpp` through the VMA allocator and the pipeline wrappers.
  Including the sensor contract therefore preprocessed **105 k lines with 1 412
  Vulkan handle references**, all of it paid by an ARKit Objective-C++ TU that
  only wants to fill in an intrinsics struct. Both now live in
  **`core/camera_params.hpp`** as `vr::DepthCameraParams` / `vr::ColorCameraParams`
  — a posed pinhole camera is pure math over `Mat4f`, the same kind of vocabulary
  as `Status` and the GLM aliases, and four tiers take one. `recon_sensor`
  consequently links **`recon_core` and nothing else**: 0 Vulkan references,
  77 k lines, the remainder GLM, which `Mat4f` requires. They stay two *types*
  (88 vs 96 bytes — colour carries no depth range), and each still pins its host
  packing against the GLSL mirrors in `volume/`, `tsdf/` and `texture/`
  shaders.
  (2) *`Result<std::optional<T>>` has no natural spelling*, and a contract whose
  only implementers are out of tree cannot afford that. `Result` converts
  implicitly from exactly its value type, so `return std::nullopt;` and
  `return frame;` are both two user-defined conversions and neither compiles;
  `return {};` is ambiguous between the value and `Status` constructors; and
  `return Status{};` compiles and then **aborts**, since an OK `Status` is not a
  failure. `ICameraCapture` therefore ships `no_frame()` and `some_frame(f)` and
  documents why. That trap was invisible until something implemented the
  interface: the tier had shipped an interface **nothing in the repo
  implemented**, so `tests/sensor_conventions_test.cpp` now carries a
  `FakeCapture` driven through a base-class reference — it caught the
  success-path half of this on its first compile.

- **2026-08-02 — The mesh arena's extra buffer usage is declared by the
  *consumer*, not named by this tier — and usage alone does not reach seam B.**
  `MarchingCubes::create` takes a `MarchingCubesConfig{extra_vertex_usage,
  extra_index_usage}`, OR-ed onto `STORAGE_BUFFER` for the vertex arena and the
  index run on **every** grow, not just the first. The alternative — hardcoding
  `VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | INDEX_BUFFER` in the `mesh` tier — was
  rejected because it makes this tier guess at a sibling's vertex-input
  requirements that it is deliberately not compiled against; the app knows both
  and passes the union, the same shape as the create/adopt device seam. A
  recon-only consumer (`fuse_replica`, the codec tiers, an exporter) defaults to
  zero and its buffers are bit-identical to before.
  **The verification half is what makes it a seam rather than a hope.** Vulkan
  cannot be asked what a `VkBuffer` was created with, so `core::Buffer` records
  its `usage()` (the reason `AdoptedDevice` carries its enabled extension list),
  and `DeviceMesh` carries `vertex_usage` / `index_usage` — a consumer *checks*
  that the binding it is about to make is permitted instead of trusting that
  the flags it published reached the producer. Without it the app restates the
  same flags independently at both ends and a mismatch is a validation-layer-only
  diagnostic: undefined behaviour with layers off, which is the shipping
  configuration and the only one on iOS. It is also what lets a test assert the
  config *landed* — `marching_cubes_config_test.cpp` extracts through a
  configured extractor and reads the bits back off the `DeviceMesh`, so dropping
  `config_` from the grow path fails rather than passing silently for want of an
  in-tree consumer.
  **Bindable is not drawable, and the gap is recorded rather than implied.**
  Review of the first cut found prose asserting that the usage bit was "the
  whole of what stands between it and binding this arena in place." It is not.
  Four things a create-time flag cannot supply remain, enumerated in a greppable
  seam-B `TODO(mesh)` on the struct: (1) **lifetime** — one grow-only arena,
  reused in place and freed *synchronously* on grow (`Buffer::operator=` runs
  `vmaDestroyBuffer` with no fence), so the next extract overwrites or frees
  memory an in-flight draw is reading; `DeviceMesh::generation` guards only the
  host `download()` path, and the fix is the ring of slots + timeline semaphore
  DESIGN.md's seam B already specifies. (*Superseded by the 2026-08-03 output-
  ring decision below*, which takes the ring and deliberately declines the
  semaphore.) (2) **sharing** —
  `Allocator::create_buffer` hardcodes `VK_SHARING_MODE_EXCLUSIVE` and
  `BufferDesc` has no knob, while on Apple recon and gfx sit on *different queue
  families* (the bootstrap decision above), so a cross-family read needs
  `CONCURRENT` or a release/acquire pair — and Metal has no ownership concept,
  so this appears to work on the only platform CI runs. (3) **visibility** —
  `core`'s shared `dispatch()` barrier reaches `COMPUTE_SHADER|HOST`, never
  `VERTEX_INPUT`/`VERTEX_ATTRIBUTE_READ`/`INDEX_READ`; widening it was weighed
  and declined, since every tier's dispatch would pay a broader scope for one
  consumer, and a cross-queue handoff needs a semaphore regardless.
  (*Both (2) and (3) are superseded by the 2026-08-03 decision below*, which
  reverses that declining: the scope is a destination mask, so it costs an
  execution dependency the driver already owed — the real cost turned out to be
  a capability check, not a performance one.) (4)
  **indirect draw** — `counter_` carries no `INDIRECT_BUFFER` usage and is not
  on `DeviceMesh`, so the `vkCmdDrawIndexedIndirect` path seam B is specified
  around cannot be expressed and the count still round-trips to the host.
  None are implemented here: each needs a consumer to verify against, and
  building them blind is how a two-field config grows four more fields nothing
  passes. `create` does reject `SHADER_DEVICE_ADDRESS` (this repo's `Device`
  never enables `bufferDeviceAddress`, so VMA asserts in debug and silently
  drops the allocate flag in NDEBUG) — validated where the caller supplies it,
  not inside the first extract's arena grow.

- **2026-08-02 — One color-space rule, and a *named* working space: 8-bit color
  is encoded, float color is linear, converted once at the sensor boundary and
  encoded once at presentation.** Color arrives from a sensor encoded for a
  display and is then **averaged** four times — the TSDF running mean
  (`tsdf_integrate.comp`), the marching-cubes edge lerp
  (`marching_cubes_common.glsl`), texture filtering, and gfx's shading multiply
  — before a `_SRGB` swapchain encodes it a second time. Averaging is linear and
  a display encoding deliberately is not, so each is wrong by construction, and
  wrong *quietly*: linear `0.0` and `1.0` fuse to `0.214` rather than `0.5` (a
  plausibly darker surface, not a visible error), a few percent between similar
  samples, worst across high-contrast pairs — so it concentrates on silhouettes
  and depth discontinuities, exactly where a reconstruction gets inspected.
  Surfaced by the iOS scanner, where only the double-encode is visible.
  The rule decides the integrator, marching cubes, the atlas format, the vertex
  color and the render targets with no judgment left over, and it is not
  arbitrary: eight bits suffice for color *because* a display curve spends them
  perceptually, so 8-bit storage stays encoded (naively linearizing trades a
  blending error for banding in the darks); a float has the range to be linear,
  so it is. It says *color* deliberately — a future 8-bit confidence or
  occupancy channel is a number and carries no curve.
  **"Linear" alone does not name a space, so one is named**: the working space
  is **linear BT.709/D65** (the linear half of sRGB), its canonical encoded form
  is that space through the *exact piecewise* sRGB transfer, full range, and
  every 8-bit color below the sensor boundary is in that one form. Without this
  the rule is under-determined — a driver can declare four transfers and three
  primaries, so "encoded" would name twelve different things — and, worse,
  `Primaries` would be a label nothing consumes: two sensors with different
  primaries produce linear values in *different RGB bases*, and averaging those
  is wrong the same way. The boundary conversion is therefore two steps, decode
  the transfer then 3×3 into the working basis, both identity for a BT.709
  source (so the ARKit path is a genuine no-op). Cost booked: BT.709 is the
  narrowest declarable gamut, so a P3/BT.2020 source clips — widening the
  working space means widening the storage with it, both named constants rather
  than assumptions spread through the kernels. Presentation is the symmetric
  half: the surface declares its space (`VkFormat` + `VkColorSpaceKHR`) exactly
  as a driver does, so a P3/HDR display is reached by that declaration changing
  and the presentation step gaining the primaries half — nothing else moves.
  **Placement follows the 2026-08-02 camera-params precedent, not the sensor
  tier.** `ColorEncoding` and the curve live in `core` (`core/color_space.hpp` +
  a `core/shaders/color_common.glsl` mirror), because `sensor` branches off
  `core` *beside* the fusion tiers — so `tsdf` and `mesh`, which do the decoding
  in GLSL, cannot include from it — and because a curve four tiers need is
  vocabulary. `sensor/color_conventions.hpp` keeps what is genuinely the
  boundary's — `to_canonical`, which walks a frame. *Implementing this moved
  the line*: the `ColorEncoding` type **and** `is_canonical` went to `core` too,
  because `TsdfIntegrator::integrate` **refuses** a non-canonical colour frame
  rather than fusing it through the wrong curve, and `tsdf` cannot include from
  `sensor` — a property of the type belongs with the type. That refusal is what
  makes "convert once at the sensor boundary" a contract rather than a hope.
  The cross-tier GLSL include (the repo's first) is spelled like the C++ header
  path so its provenance is visible at the include site; `vr_compile_shaders`
  passes `src/` as the shader include root.
  The declaration rides **beside** the camera
  (a field on `sensor::CapturedFrame` / `tsdf::ColorFrame`), never *inside*
  `ColorCameraParams`: that struct is uploaded verbatim under scalar block
  layout, pinned at 88 bytes with GLSL mirrors in `tsdf/` and `texture/`, so a
  field there would spend a cross-tier shader ABI change to carry what the
  kernel needs at most as a push constant. No `Range` field — the contract
  requires full-range packed R'G'B', so the YCbCr matrix and range expansion are
  the driver's, applied before the contract; `Primaries` earns its place because
  the working space gives it a consumer, and a field nothing consumes is a label
  free to drift. ARKit declares `{Bt709, Bt709}` (its YCbCr→R'G'B' uses a BT.601
  *matrix*, independent of the transfer function — conflating the two is its own
  silent error), and `Transfer::Bt709` 8-bit content is **accepted as sRGB
  rather than converted**: they differ by a couple of codes in the toe, which is
  what display pipelines assume anyway (BT.1886), and converting would cost a
  per-frame pass over 1920×1440 to buy nothing.
  Two consequences worth stating. (1) **Presentation is three sites, not one** —
  the swapchain (already `_SRGB`), `fuse_render`'s offscreen target (`_UNORM`
  today; it renders straight to PNG, so linear values would simply come out
  dark, and it is the CI-visible leg), and host export, where PLY `uchar` colors
  are read as sRGB by every external viewer and must be encoded on write while
  glTF `COLOR_0` is linear and passes through — the exporters cannot share one
  path. (2) **`volumetric_kit_gfx` needs no change at all**: every format is
  caller-set, the swapchain already prefers `_SRGB`, and `hybrid_mesh.frag`'s
  `albedo *= ambient + diffuse` was never the bug, only its operands. Both the
  host curve and its GLSL mirror must be the *exact piecewise* sRGB function,
  not `pow(x, 2.2)` — hardware `_SRGB` sampling uses the exact curve and that
  shader selects atlas-vs-vertex-color per triangle across one surface, so an
  approximation shows as a seam exactly where texturing stops. Storage stays
  `uint32` (convert in the shader) and is measured; the escalation trigger is
  **not** banding but the running mean *latching* — re-quantized to 8 bits it
  stops moving once the per-frame delta falls under half a code. **Measured**
  (`tests/core_color_space_test.cpp`) at the default `max_weight = 5` and a 2 m
  observation: the mean stops **~10 codes short, uniformly across the range**
  (0→64 settles at 55, 0→255 at 245), and a gap narrower than ~10 codes never
  moves the voxel at all — the residual is range-independent because the sRGB
  curve turns a fixed fraction of the linear gap into a roughly fixed number of
  codes. So fused colour accuracy is ceilinged at ~4%, a *convergence* limit
  rather than a precision one, which is precisely why banding was the wrong
  thing to watch. Authoritative detail: DESIGN.md → "Color space".

- **2026-08-03 — A buffer names the *families* that will read it, and the
  dispatch barrier widens only as far as its queue family may.** Seam-B
  blockers (2) sharing and (3) visibility from the `MarchingCubesConfig`
  decision above, both the same shape: the buffers were already correct for
  recon reading its own results and **silently** wrong for anyone else.
  (2) `BufferDesc` grows `queue_families` / `queue_family_count`, and
  `create_buffer` picks the mode from their *distinct* count — two or more give
  `CONCURRENT`, one or zero give `EXCLUSIVE`. Naming the families rather than
  the mode is what lets a consumer pass its compute and render indices
  **unconditionally**: Vulkan requires `CONCURRENT` to name at least two and
  requires them unique, so a caller restating the mode itself would be
  malformed on every platform where the two collapse to one family — the common
  case off Apple, and where `CONCURRENT` would cost access performance for
  nothing. Over `kMaxQueueFamilies` (4) distinct is an **error**, not a
  truncation: a partial list names fewer families than actually touch the
  buffer, which is the original bug in a different hat.
  (3) The shared `dispatch()` barrier's destination scope adds
  `VERTEX_INPUT`/`VERTEX_ATTRIBUTE_READ`/`INDEX_READ` and
  `DRAW_INDIRECT`/`INDIRECT_COMMAND_READ`. Unconditional in the *stage* sense —
  a destination mask costs an execution dependency the driver already owed the
  host and compute cases, and a per-dispatch knob would have to be threaded
  through every kernel this helper exists to keep uniform — but **not** in the
  *capability* sense, which is the correction the first cut needed:
  `VK_PIPELINE_STAGE_VERTEX_INPUT_BIT` requires `VK_QUEUE_GRAPHICS_BIT`, and
  recon requires only *compute* of the family it is handed, so it can sit on a
  compute-only one. That is not hypothetical — the bootstrap's two-families
  plan matches on `queue_flags` alone (`VK_QUEUE_COMPUTE_BIT`), so on a discrete
  GPU it picks the async-compute family, and naming `VERTEX_INPUT` there is
  invalid usage in *every* dispatch in *every* tier. Invisible on Apple, where
  all four MoltenVK families are graphics+compute — the same
  correct-on-the-machine-that-ran-it hazard this decision exists to close, hit
  while closing it. `Device` therefore records its family's `queueFlags` at
  create/adopt (**read from the driver**, unlike the enabled extensions and
  features, because capabilities *are* queryable and so nothing here is a
  hand-written declaration that can drift), and `dispatch` consults it.
  `DRAW_INDIRECT` needs graphics *or* compute, so it is always available. A
  cross-queue handoff needs its semaphore regardless, and a semaphore's
  signal/wait carries availability and visibility on its own — so nothing is
  lost on a compute-only family.
  **Verification is what makes both a seam rather than a hope**, and the first
  cut got it wrong in an instructive way. Vulkan cannot be asked what a
  `VkBuffer` was created with, so `Buffer` records its `sharing_mode()` beside
  its `usage()` — with more at stake, since reading an `EXCLUSIVE` buffer from a
  family that does not own it is *undefined* where a missing usage bit is at
  least a validation diagnostic. The mode alone, not the family list: the mode
  is what decides whether an ownership transfer is needed. That accessor is also
  what gives the *test* teeth. The first cut asserted the sharing rule by
  creating deliberately-malformed buffers and checking the call **succeeded**,
  reasoning that a validation layer would catch the rest — but recon's debug
  callback returns `VK_FALSE` (it must; aborting is a debugger's job),
  `vmaCreateBuffer` still returns `VK_SUCCESS`, and nothing turned the
  diagnostic into a failure, so those cases asserted **nothing on any machine**,
  layer or no layer. The suite now pins three independent layers: the reduction
  through the internal header (pure, fails everywhere), the mode read back off
  `Buffer::sharing_mode`, and an Error-count delta through `set_log_handler`
  around the device cases. Sabotaging the mode selection fails layer 2;
  sabotaging the dedup fails layers 2 *and* 3 (the layer is installed on the dev
  machine and in CI, and the error now reaches the exit code). The test also
  reads the device's real family count instead of hardcoding `{0, 1}`, which on
  a single-family driver — lavapipe, which both Linux CI legs run — is itself
  the malformed shape it is trying to detect.
  `distinct_queue_families` lives in a **non-installed** `src/` header
  (`core/queue_family_set.hpp`, mirroring `vk_physical_device.hpp`), not in a
  `detail` namespace in the public one: its location is the access control, so
  the tier commits nothing to the exported ABI for a helper that exists to be
  host-testable, and the test reaches it by include path. Still open at the time
  of writing: seam-B blockers (1) lifetime and (4) indirect draw — both settled
  by the two decisions below, which close the set.

- **2026-08-03 — The mesh arena is a ring of slots released by the *host*, not
  a timeline semaphore.** Seam-B blocker (1) lifetime. `MarchingCubesConfig`
  grows `slot_count`: one (the default) is byte-identical to the old single
  grow-only arena, and more gives each outstanding extract its own arena and
  index run. The consumer calls `MarchingCubes::release_through(generation)` as
  its frames retire, and an extract only ever writes, grows, or **frees** a slot
  that has been released — which is what makes the tier's existing synchronous
  `vmaDestroyBuffer` (no fence wait; its own work is fence-blocked inside
  `submit_single_time`) safe again for an external reader.
  **This takes the ring DESIGN.md's seam B specifies and deliberately declines
  its timeline semaphore.** A cross-library GPU wait is the one thing the
  shared-queue arrangement forbids: a command buffer waiting on a value the
  sibling has not signalled deadlocks against a swapchain rebuild, which drains
  the queue while holding the submit mutex. Reporting completion *after the
  fact* cannot deadlock, needs no fence queue in the library, and costs a
  `std::uint64_t` store. The price is that the contract is now the consumer's to
  honour: extracting with every slot outstanding is **refused**, not silently
  serviced by overwriting a live draw.
  Three things this got wrong before review caught them, all in the same place —
  *the refusal path must cost the caller nothing*. (Not a closed set: six more
  of the same family surfaced once a ring actually ran on device, including a
  capacity plan that compounded 1.5× per extract. See the 2026-08-03
  slot-independence decision below, which also moves the refusal above the
  active-set compact — this decision's own "costs nothing" was still paying a
  GPU round trip.) (1) The slot claim ran inside
  `ensure_output_buffers`, i.e. **after** `++generation_`, so a refused extract —
  the path whose entire purpose is to protect a slot the consumer is still
  reading — advanced the generation and made `download()` retire the very mesh
  it had just protected. Verified by probe before the fix (`download(gen2)` OK
  before the refused extract, REFUSED after) and now pinned by a test that a
  revert fails. The claim is its own `claim_output_slot()`, called once at the
  top of each extract and before the bump; that also makes "once per extract,
  not once per call" structural rather than conditional on a
  generation-equality trick, which matters because a call that plans its
  capacity low re-enters `ensure_output_buffers` to refit. (2) `release_through`
  is **not atomic and not internally synchronized**, while the consumer it is
  written for is on another thread — `fuse_viewer` fuses on a background thread
  and retires frames on the render thread. Documented as a caller obligation
  rather than made `std::atomic`, because an atomic is not movable and this
  class's rule-of-zero defaulted moves are load-bearing; the consumer already
  holds a mutex to hand the `DeviceMesh` across, and a `uint64_t` store under it
  is free. (3) `download()` reads the *current* slot unqualified, which is
  correct only because the generation check and the slot claim together make
  "current generation" and "current slot" the same statement — the comment
  there had been justifying the check with the single-arena argument, which a
  ring **inverts** (a superseded view now names a *different* buffer). Widening
  `download` to older generations therefore means indexing `slots_` by
  generation, not relaxing a comparison.
  Two further notes carried knowingly. The slots are a **fixed array, not a
  `std::vector`**: this class promises a self-move leaves it intact and the
  sparse test exercises `mc = std::move(mc)` directly, but `std::vector`'s
  self-move-assignment is valid-but-unspecified and libc++ empties it, so the
  extractor passed `valid()` and then indexed nothing (it segfaulted on the
  first run). And the empty-mesh early return used to be the one path that
  advanced the generation **without** claiming a slot, on the reasoning that an
  empty mesh owns no bytes to protect. *Corrected by the decision below*: that
  is true of the empty mesh and false of the **ring**, because
  `released_through_` is a high-water mark — a consumer that retires the empty
  generation (instantly, since it draws nothing) thereby marks every older slot
  released too, including the one holding the mesh it is still drawing, which is
  exactly what a viewer does on an empty frame. The empty path now claims and
  stamps a slot like any other extract, so "every generation lives in exactly
  one slot" **is** total. Still open for seam B at the time of writing: blocker
  (4), the indirect command on `DeviceMesh`.

- **2026-08-03 — The draw command is written by the kernel that counts it, and
  the counter's unit changes from triangles to indices.** Seam-B blocker (4),
  the last one. The marching-cubes append atomic no longer bumps a bare
  `uint` triangle counter; it bumps `indexCount` of a real
  `VkDrawIndexedIndirectCommand` by `kIndicesPerTriangle` per triangle, so the
  bytes the kernel writes *are* the command `vkCmdDrawIndexedIndirect` reads.
  The buffer carries `INDIRECT_BUFFER` beside the `STORAGE_BUFFER` the kernel
  counts through, it is per **slot** rather than per extractor (it is part of
  the mesh — a renderer reading slot N's command while N+1 is extracted is the
  whole point of the ring), and `DeviceMesh` publishes it. The count still
  round-trips to the host, but only because *this tier* needs it to refit an
  undersized arena; a consumer no longer does, which is what closes the blocker.
  **Changing what an atomic counts moves every site that reads it, and the first
  cut moved only some.** The three that mattered, all found by review: (1) the
  `uint32` overflow guard still bounded *triangles* while the counter had become
  *indices*, leaving it **3× too loose** — and worse than a plain wrap, since
  `2^32 ≡ 1 (mod 3)` desynchronises the `/3` that recovers the slot index, so a
  wrapped dispatch overwrites triangles it already emitted and returns
  `Status::ok` with a corrupt surface. (2) The safety clamp that bounds
  `indexCount` by the arena ran only on the success path — where the loop's own
  exit condition already makes it a **provable no-op** — and was skipped on the
  two failure returns that genuinely leave the command naming more indices than
  exist. The guarantee was exactly inverted; it now lives on the failure paths,
  where at `slot_count == 1` an outstanding `DeviceMesh` names that very buffer.
  (3) The GLSL contract sentence ("`index_count` always ends as the field's true
  total") became false in the same edit, and it is not decoration — it is the
  specification the refit protocol and the `continue`-not-`return` fix rest on,
  and the CUDA port is required to stay numerically in lockstep with it. The
  open-coded `3` is now `kIndicesPerTriangle`, named on both sides of the ABI,
  because it is the conversion between the two units this tier trades in.
  **Two adjacent bugs the same review turned up, neither about the command.**
  The host `extract` overloads claim and stamp a ring slot but return
  `Result<Mesh>` — no generation — and `download` does not release, so a
  consumer at `slot_count > 1` exhausted the ring in `slot_count` calls and then
  failed **permanently**, quoting generations the API never handed out. They now
  release their own slot once the host copy is taken, on success *and* failure,
  which keeps a host-only caller off the release contract entirely. And the
  empty-extract path is corrected as described above.
  **Verification is the point, and it was measured rather than assumed.**
  Nothing pinned the command's contents: mutating `emitted * 3` to `emitted`
  passed every mesh and texture test, and reverting the atomic to count
  triangles passed the *sparse* suite — because dense and sparse share
  `mcEmitCell`, so a unit error moves both sides of the equivalence equally. The
  suite now reads the command back through a real `vkCmdCopyBuffer` (which makes
  the test a genuine consumer of `extra_indirect_usage` — the readback and the
  flag prove each other) and asserts `indexCount == triangle_count · 3` plus the
  four fields that make it *drawable*; and the sparse test pins the counter's
  **units** against the fixture's analytic surface area (`4πr²`), which no
  counter participates in, so a 3× undercount fails by a mile. Both mutations
  now fail. `DeviceMesh::valid()` covers all three buffers rather than two.
  **Sharing and placement, the two remaining ways to get this wrong quietly.**
  `MarchingCubesConfig` grows `queue_families` (held **by value** — the config
  is stored and re-read on every arena grow, so a `BufferDesc`-style pointer
  would dangle) applying to arena, index run *and* command alike, since a
  renderer reads all three; `DeviceMesh` publishes the resulting
  `sharing_mode`, which matters more than the usage bits it sits beside because
  reading an `EXCLUSIVE` buffer from a non-owning family is *undefined* where a
  missing usage bit is at least a validation diagnostic. Memory placement is
  **recorded rather than fixed**: the command is host-visible because the refit
  protocol reads it every extract, which is free on Apple's unified memory and
  costs a PCIe fetch per draw on a discrete GPU — booked as a greppable
  `TODO(mesh)` awaiting a discrete-GPU consumer to measure it, because the
  hazard is not the cost but that the cost is invisible on the only hardware CI
  runs. With this, **all four seam-B blockers are settled**; what remains is a
  consumer that actually draws it.

- **2026-08-03 — Nothing in the mesh extractor reads "the current slot" except
  the code that writes it: the capacity plan is slot-independent, and a slot is
  marked outstanding only where a `DeviceMesh` is handed out.** Found on an iPad
  Pro, where a ring reached a **1.1 GB arena for 36 904 triangles (0.59% full)**
  in a few hundred remeshes and then lost the device. The cause was one line:
  `plan_capacity` floored its estimate at `arena_capacity()` — right for a
  single arena reused in place, and a **ratchet** across a ring, because it read
  the slot just *written* while a different slot was about to be *grown*. Every
  extract then asked for 1.5× the last, geometrically, with the measured
  triangle density never getting a say.
  **The floor is deleted, not repositioned.** Reordering the claim above the
  plan fixes the symptom and leaves a purely positional invariant inside a
  340-line function; the floor is also **provably inert** either way, since
  `capacity` reaches only `ensure_output_buffers`, which keeps whatever the slot
  holds and grows only on a larger request, and the dispatch is pushed
  `arena_capacity()` rather than `capacity`. So the plan now reads *no slot at
  all* — this call's active set and the last call's density — and cannot
  compound in any call order. Confirmed by mutation: restoring the floor alone
  changes nothing measurable; restoring it *together with* the old claim
  position reproduces the runaway.
  **Fixing that exposed the rest of the family, all the same shape — state that
  was correct for recon reading its own results and silently wrong across the
  ring.** (1) *The stamp says a consumer holds a mesh here, so only a path that
  hands one out may write it.* It lived in `ensure_output_buffers`, so every
  failure after it — the dispatch, the refit, the `out_of_memory` return, the
  `maxStorageBufferRange` rejection — left a slot marked with a generation **no
  `DeviceMesh` ever carried**, which nothing could then release: `slot_count`
  such failures and the extractor refuses *permanently*, quoting generations the
  API never handed out. It now sits beside `device_mesh.generation` on each
  publishing return, which makes "every generation handed out lives in exactly
  one slot" total by construction rather than by audit. (2) *`claim_output_slot`
  scans instead of assuming.* "The next slot is always the oldest" holds only
  while every claim goes on to publish; a failed extract and the host overloads
  both leave a free slot behind the cursor, and assuming would refuse with one
  in hand. (3) *The host overloads give back their own slot, not everything
  below it.* They must hand it back — a `Result<Mesh>` carries no generation, so
  a host-only caller cannot — but doing it through `release_through`, the
  **consumer's** high-water mark, also retired every older slot, including one
  an `extract_device` caller on the same extractor was still drawing out of;
  the next grow then ran `vmaDestroyBuffer` under a live draw, undefined with
  layers off, which is the shipping configuration and the only one on iOS.
  `free_slot_of(generation)` clears one stamp and moves no mark. (4) *The claim
  and `++generation_` are adjacent, and both move above the active-set compact.*
  `download()` reads the current slot on the strength of a *generation*
  comparison, so anything fallible between the two leaves a window where an
  outstanding mesh passes the currency check and is copied out of a slot that
  may never have been sized — a `memcpy` from a null mapping. Above the compact
  because a refusal must cost the caller *nothing*, and the compact is a
  dispatch, a fence wait and a full active-set readback that a consumer one
  frame behind would pay on every frame. (5) *`ExtractTimings::arena_bytes` sums
  the ring.* It reported the claimed slot alone against a header that promised
  what the extractor holds, so at `slot_count = N` it under-reported resident
  memory N-fold — and it is the instrument this bug was diagnosed with. (6)
  *`download` gates on `valid()`* like both extract entry points; the defaulted
  moves reset every owned member but copy the scalar `generation_`/`slot_`, so a
  moved-from extractor passed the check and then read a null mapping.
  **The regression test asserts a magnitude, not just a shape.** Pinning
  per-slot periodicity alone stays green under a wholesale revert to worst-case
  sizing — which is perfectly flat, just ~19× too big — so the ratchet block now
  sandwiches resident arena bytes between 1× and 3× the emitted surface *per
  slot* and holds them flat across the last two turns of the ring, with its
  stride derived from `slot_count` rather than hardcoded beside it. Both halves
  were confirmed to fail against a re-introduced ratchet, against worst-case
  planning, and against reporting one slot instead of the sum. A separate case
  holds a live `DeviceMesh` while a host extract runs on the same extractor and
  asserts the ring then refuses — the handle-independent half of (3), which no
  dedicated-extractor test could see.

- **2026-08-04 — A limit, a lifetime, or a staleness the caller cannot see is
  the library's to check, not to document.** A whole-repo review found fifteen
  defects and thirteen were one shape: *correct for recon reading its own
  results on the machine CI runs, silently wrong elsewhere* — the same family
  the seam-B decisions above kept turning up, now swept across every tier. What
  unifies the fixes is **where** the check went, so the rule is stated once
  here rather than re-argued per site: when a caller cannot obtain the fact it
  is required to honour, an obligation in prose is not a contract, and the
  library asks the question itself.
  **(1) `maxStorageBufferRange` is a real ceiling, and it is lowest exactly
  where nothing tests it.** Vulkan guarantees only 2^27 (128 MiB) — what
  Android-class drivers report — while MoltenVK and desktop drivers report far
  more, so an over-large binding is invisible on every machine this repo runs
  on and undefined on a stated target. `mesh` and `texture` guarded their
  buffers; `volume` and `tsdf` did not, and their `VoxelBlockGrid` attribute
  arrays are the **largest allocations in the repo** — 256 MiB *per attribute*
  at every example's default grid, twice the floor before a single resize, and
  bound `VK_WHOLE_SIZE` with `robustBufferAccess` enabled nowhere. The guard is
  now `core`'s (`compute_util.hpp`: `max_storage_buffer_range` /
  `check_storage_buffer_range`), which is what forced the `volume`/`tsdf`
  helper migration noted above, and it is applied where the size is *chosen*
  (attribute create + every grow, the depth/colour frame uploads) rather than
  at each bind, since a consumer could only re-derive the same number.
  **(2) A lifetime rule that cannot describe move-assignment is not a rule.**
  `Allocator` documented "destroy Buffers before their Allocator", which a
  reader satisfies by keeping the object alive — and `a = std::move(b)` then
  ends the resource's life while the wrapper visibly lives on, so every
  outstanding `Buffer` freed through a destroyed `VmaAllocator`. Made
  structural instead: the pImpl is a `shared_ptr` and each `Buffer`'s deleter
  holds a reference, so the ordering is unconstrained and the `@warning`
  becomes a `@note`. `Allocator` stays move-only; the cost is one atomic per
  buffer create/destroy, against a device allocation on the same path.
  **(3) Staleness must be *askable* by whoever acts on it.** `DeviceMesh`
  carried a `generation`, but only `MarchingCubes::download` could compare it —
  so the *reading* path was guarded and the *writing* path,
  `ProjectiveTexturer::texture`, was not, while its header stated the
  obligation and handed the texturer no means to check it. Binding a superseded
  view is a use-after-free when the intervening extract grew the arena
  (`Buffer::operator=` runs `vmaDestroyBuffer` with no fence), and silently
  wrong when it did not. `DeviceMesh` now carries a non-owning pointer to its
  producer's live counter and answers `is_current()`; the texturer refuses.
  Handle comparison cannot substitute — a grow-only arena reused in place makes
  a superseded view name the very same `VkBuffer`. The pointer borrows the
  producer's address as the handles already borrow its storage, so a view still
  must not outlive or be held across a move of its producer.
  **(4) A predicted quantity is clamped, never rejected.** The mesh arena's
  range check tested `plan_capacity`'s *guess* — last call's density scaled by
  this call's active set — and `tris_per_block_` is written only on success
  while a scan's active set only grows, so a single over-estimate failed the
  extractor **permanently**, quoting a limit the real surface fitted inside. It
  clamps now, and the rejection moved onto the *measured* count the refit
  protocol already produces, which is what the error message always claimed.
  **(5) A validator must not be defeated by its own arithmetic.**
  `VoxelGridParams::validate` compared `num_blocks` against a *narrow* product
  that wrapped exactly as the `uint32` multiply in `VoxelHashMap::resize` had,
  so both sides agreed on the wrapped value and the guard could never fire —
  and cubed `block_size` unbounded, which is signed-overflow UB inside the
  function whose job is rejecting bad input (2048³ wraps to exactly 0, so
  `voxels_per_block == 0` passed and every block aliased pointer 0). Bounded
  before the cube, widened for the product. `bucket_size == 1` is now rejected
  outright: the last entry of each bucket is its chain anchor, so at one entry
  per bucket *every* slot is an anchor and `allocate_in_overflow` — which skips
  anchors — is a guaranteed no-op, failing the first colliding pair forever,
  which a caller reads as capacity pressure and answers by growing until it is
  out of memory.
  **(6) A retry loop may only erase a failure it can actually resolve.**
  `dispatch_with_retry` re-dispatches while the failure count drops, which is
  right for allocation (the element is still unprocessed) and wrong for
  deletion: the heap append runs *after* the table entry is cleared, so a
  coord whose block could not be returned to the free heap is absent next round
  and counts nothing — `remove()` reported a clean delete over a permanently
  leaked block index. The tally gains a **`kFailTerminal` slot** the host
  accumulates across rounds instead of reading from the last one, and the slot
  layout moved to `hash_common.glsl` because it is a host/device ABI two
  kernels share, not one header's private business.
  **(7) The aggregate the API returned could not answer the question the caller
  had to ask.** Every allocate kernel already split its failures by reason and
  nothing exposed them, so `fuse_replica` read a residue of lost bucket-lock
  races — transient contention, worst for depth, whose adjacent pixels dilate
  into the same block — as map overflow and **doubled the volume**: at the
  example defaults 768 MiB → 1536 MiB per attribute, with both old and new
  buffers live across the swap, for a table that was never full. `AllocFailures`
  is an opt-in `nullptr`-defaulted out-param (the `ExtractTimings` shape the
  2026-08-01 amendment blessed, and the same bar: no global sink, no retained
  state, nothing measured when null) carrying `capacity_limited()` — the
  predicate `resize` actually answers. All three examples consult it.
  **(8) The tier that owns both halves does the clearing.** The delete kernel
  justified not clearing a freed block's per-voxel data with "there is no voxel
  data to clear yet", false since the `tsdf` tier landed. It genuinely cannot:
  under the SoA decision the block index does not know what attributes a
  consumer declared. But the heap is **LIFO**, so the next allocation re-draws
  the freed index onto the identical range and the removed surface's
  `tsdf`/`weight` resurrect at full fused weight — bypassing the integrator's
  `color_attr == 0` first-observation gate, which exists to stop exactly a
  wrong first colour. So `VoxelBlockGrid` grew `remove` / `clear`, which zero
  the ranges first (resolved from the compacted active set — no new kernel, and
  the buffers are host-mapped); `map().remove()` remains the raw path and says
  so. Its sibling hazard, `map().resize()` desyncing the attribute arrays, was
  documented as safe because "a downstream bounds check stays sound" via
  `AttributeView::element_count` — which **nothing read**. `attribute()` is the
  funnel every consumer passes through, so it now refuses an array that no
  longer covers the live grid, and the promise is true.
  **Verification followed the repo's own standard, and it was measured.** Each
  fix that could be pinned host-side or on MoltenVK is, and each new assertion
  was confirmed to **fail against the pre-fix code** by mutation: reverting the
  `validate` widening, dropping the attribute clearing, dropping the
  `attribute()` desync check, and dropping the texturer's currency check each
  fail a named line. `fuse_replica` on Replica room0 produces a mesh identical
  to `main` triangle-for-triangle (832 518 vertices / 277 506 triangles over
  120 frames), which is the point: none of this changes fusion numerics on the
  hardware that already worked — it changes what happens on the hardware that
  did not. Two example-level fixes carry no test because they are in the
  gfx-linked examples, which only the build-only viewer CI leg covers:
  `fuse_render` / `fuse_viewer` discriminating a decode failure from
  end-of-sequence (both swallowed it, and the CI-visible render leg wrote a
  partial-room PNG and exited 0), and `StageTimes::total_ms` skipping the
  `"  .."`-prefixed breakdown rows, which re-state time already counted by the
  stage they decompose and so reported every extract twice. Both viewers also
  gained `--trunc`, defaulted to `4 * voxel` as `fuse_replica` does, since a
  band hardcoded in metres made `--voxel` silently change the band's width *in
  voxels* — 1.6 voxels at `--voxel 0.05`, 16 at `0.005` — and made the same
  flag value reconstruct differently across the examples the A/B exists for.
  **Second pass, the nine the review cut for its report cap.** Same rule, and
  three were the same *shape* as the thirteen above. (a) `pack_linear_to_srgb`
  clamped with `e < 0 ? 0 : (e > 1 ? 1 : e)`, which **NaN slips through** —
  it compares false to both bounds — into a float-to-unsigned conversion that
  is undefined for a value it cannot represent, in a header this library
  *installs*. Rewritten as "keep only what is provably in range"; confirmed by
  reverting it under the UBSan leg, which reports `nan is outside the range of
  representable values of type 'unsigned int'` at that exact line. (b) The
  frustum cull's block AABB was the occupied volume shifted **+½ voxel** on
  every axis (it took the origin voxel's *centre* as `bmin` and added a whole
  edge, where node-centred voxels extend a half-voxel past each end). The
  ~10% widening does not absorb it: `make_frustum_planes` scales `fx`/`fy`, so
  near and far are exact, and the shift lands on them asymmetrically —
  the near test compares `bmax` and is merely loosened, while the far test
  compares `bmin` and is *tightened*, culling blocks whose voxels are within
  `far_z` while their surface is in view. (c) An overflow-chain insert that
  failed because the **table** was full reported `kFailHeap` — a reason that is
  *provably impossible* on the rehash path, which presets each pointer and
  never touches the heap. Now `kFailTable`, and the two allocate helpers return
  a reason rather than a bool, which is what makes the distinction expressible;
  `AllocFailures` grows the field and `capacity_limited()` covers it.
  The rest were smaller. `ExtractTimings::arena_bytes` reported 0 on the
  empty-extract path — "the extractor released its memory", the opposite of
  true, on the instrument the ring runaway was diagnosed with.
  `VoxelBlockGrid::resize` now validates the grown grid before allocating,
  though honestly: it *overlaps* the `maxStorageBufferRange` guard added above,
  which fires first on any real device, and is kept only because it states the
  grow is illegal arithmetically rather than too large for this hardware. Three
  GLSL headers still documented the superseded 48-byte vertex (`color@24` —
  the offset `tangent` has occupied since the 2026-08-02 layout decision), a
  stale comment that read as current because `uv0@40` and the number 48 both
  still appear in the real layout. `fuse_render`'s `--follow` computed its FOV
  as `2*atan(360/600)`, wrong twice — 340 is the half-height of a 680-tall
  sensor, and the hardcoded `fy` ignored `--cam-params` — so the sensor's vfov
  is now carried out of `fuse()` from its own intrinsics, split about the
  principal point. `--fuse-per-tick` was parsed, clamped and documented in the
  usage string while **nothing read it**; removed rather than given an invented
  meaning.
  **Three assertions that could not fail** are the last group, and they are the
  reason this pass exists at all — a test that cannot fail is worse than no
  test, because it is counted. `marching_cubes_config_test` asserted that a
  stale `release_through(0)` still *refused* an extract, which held with or
  without the monotonic guard because the ring had no free slot either way; it
  now releases the whole ring first, so a dropped `std::max` turns the mark to
  0 and stalls the ring — a failure. `compute_raii_test` seeded the
  move-assignment's destination from the source (`c.set = b.set`), so
  `c.set.handle() == raw` was true *before* the assignment ran; the destination
  is now a second real pool + set with its own handle to lose. Both were
  confirmed to fail against the mutation they exist to catch.
  `sensor_conventions_test`'s was vacuous for a third reason — `dst` still held
  the previous call's output — and is now zeroed and checked in full, but it is
  recorded here that it *still* cannot discriminate the `is_canonical` mutation
  the review named: for 8-bit input the decode/encode round trip is the
  identity, which is precisely why `Transfer::Bt709` is accepted as sRGB rather
  than converted. That predicate is pinned directly where it lives, in
  `core_color_space_test`, and the mutation fails there.

- **2026-08-08 — The sparse mesh kernel resolves its own 2×2×2 neighbourhood by
  probing the hash table on-device, and `mesh` therefore reads a `volume` buffer
  — a coupling paid for, not stumbled into.** *Reverses* the "host-built
  neighbour table … no device-side hash probe, and no coupling to the hash
  table's internal buffers" that the sparse-extract description above asserted
  since the tier landed. The host pass hashed every active block coordinate into
  an `unordered_map` and did eight lookups per block: an `O(active·8)` serial
  pass rebuilt every extract whether or not anything moved, measured through
  `volumetric_kit_ios` on an M5 iPad Pro at 107 k blocks as **102.2 ms of a
  132.7 ms extract** — against 25.9 ms for the marching-cubes dispatch it existed
  to feed. The kernel now runs one **workgroup per block** (not a flat voxel
  grid, since shared memory is per workgroup) and threads 0–7 each probe one
  octant into `shared int s_neighbour[8]`; eight probes amortise over
  `voxels_per_block` cells. Extract goes 132.7 → 42.0 ms. Both prior
  implementations of this pipeline did it this way — the CUDA and Metal marching
  cubes in `implicit_world_reconstruction` each fill a threadgroup
  `s_neighbor_cache` from device-side `findVoxelBlockByCoord` probes — so the
  host table was the outlier, not the design.
  **What the coupling actually costs, stated because the reversed sentence used
  to deny it.** `VoxelHashMap::entries_buffer` / `entries_buffer_size` are now
  public, and `mesh` tracks `volume`'s entry layout, bucket geometry and chain
  protocol. Three consequences are carried rather than waved at. (1) *The probe
  is quiescent-only.* It takes neither the bucket lock nor the acquire
  `memoryBarrierBuffer()` `block_exists` splits its `ptr`/`pos` test around, and
  no barrier could fix that anyway — a walk can meet a chain mid-splice, which is
  structural, not an ordering problem. So the precondition is stated on
  `MarchingCubes::extract_device`, where a caller meets it, not only on the
  `volume` accessor: a consumer that fuses and meshes on different threads must
  serialise them. Within one thread it holds by construction, every `volume`
  dispatch being fence-blocked. Per the 2026-08-04 rule this would be the
  library's to *check* rather than document, and it is documented instead
  knowingly: `VoxelHashMap` tracks no in-flight mutation, and a flag that a
  second thread could race is not a check. (2) *The entries buffer is
  host-visible* (`volume` books device-local + staging as a follow-up), so the
  win is measured on unified memory only; on a discrete GPU each workgroup's
  seven probes become up to `bucket_size + max_chain` dependent scattered reads
  across PCIe — a greppable `TODO(mesh)` at the binding, the same shape as the
  host-visible indirect command's. (3) *The binding names its real size and is
  range-checked*, because this is the one buffer in the extract that **another
  tier** sized and `resize` doubles: `VK_WHOLE_SIZE` would have shipped the
  accessor added to avoid exactly that with zero call sites.
  **One definition of the hash ABI, not a fourth mirror.** The first cut
  restated `HashEntry`, the free/no-offset sentinels and the three hash primes
  in `hash_lookup.glsl` under a `Vr` prefix, reasoning that the prefix would
  force a compile error on drift. It does the **opposite**, and this was checked
  against `glslc`: a duplicate `const int kFreeEntry` is a hard `redefinition`
  error, so *identical* names are the compile-time guard and a prefix is
  precisely what lets two divergent sets coexist in silence — with a divergence
  resolving neighbours from the wrong bucket and dropping surface at block seams
  under `Status::ok`. The stated blocker to sharing (`hash_common.glsl` declaring
  its own `pc` block, which collides with any kernel that has push constants) is
  now opt-out via `VR_HASH_COMMON_NO_PUSH_CONSTANTS`; only the table *shape* had
  to arrive as arguments. `hash_lookup.glsl` and `hash_common.glsl` both carry
  include guards, and the sparse kernel's own `BlockIndex` mirror is gone too.
  **The chain half of the probe was covered by nothing, and that is the finding
  that mattered.** Every sparse fixture used `bucket_size 8 / num_buckets 128`,
  whose 216 block coords peak at bucket occupancy 4 — `allocate_in_overflow` is
  never entered, so no entry is ever reachable only through a chain. Replacing
  the chain walk with `return -1;` left **every suite green**. The suite now
  meshes the identical field through `bucket_size 2 / num_buckets 512`, asserts
  via `HashDiagnostics` that it *does* spill before trusting what it proves, and
  requires the mesh to match the reference triangle for triangle; the mutation
  fails it. Production is the overflow case — `VoxelGridParams::defaults()` is
  50×30000 with `max_chain 128`, and `overflow_count` exists because buckets
  fill.
  **Two things deliberately not done.** The primary-bucket scan does **not**
  early-exit on the first free slot, though a miss then costs `bucket_size`
  scattered reads and misses are common (the truncation band dilates a shell, so
  most outer blocks have unallocated `+neighbours`). Insertion takes the first
  empty slot but `delete_primary` clears an arbitrary one *in place*, so a bucket
  may hold a free slot ahead of an occupied one — `block_exists` scans in full
  for that reason and so must this. And the workgroup stays a fixed 256: below
  `block_size` 8 that idles most lanes *and* multiplies the group count by
  `256/voxels_per_block`, lowering the reachable active-block count against
  `maxComputeWorkGroupCount[0]` (floor 65535) by the same factor. Sizing it from
  `voxels_per_block` needs a specialization constant through
  `ComputePipeline::create`; every in-tree caller uses 8, where the shape is a
  win, so both costs are booked in the kernel rather than paid.
  **The +25% dispatch regression was misattributed, and the correction moves the
  roadmap.** It was read as lost occupancy — "a workgroup whose cells all bail
  early now idles where the flat dispatch packed work densely". At `block_size`
  8, `voxels_per_block` is 512 = 2·256, so the old `gid / 512` was already
  constant across each 256-thread group: **every workgroup already belonged to
  one block**, and only the group count changed (2 per block → 1). No packing
  existed to lose. The cost is the new prologue — 8 lanes walking buckets while
  the other 248 wait at the `barrier()`, serialised ahead of every cell. That
  matters because `dispatch` is now 77% of extract, and the levers are the probe
  (spread the 8 across subgroups, or one lane per bucket slot with a reduction),
  **not** the cell count (coarser voxels) or the block count (incremental
  extraction) the first analysis pointed at. Left unmeasured rather than guessed
  at, per this file's own lesson about profiling the phase before optimising it.
  `ExtractTimings::neighbour_lut_ms` is **removed**, not retained as a zeroed
  phase: a permanently-0.00 row in the overlay that diagnosed this is worse than
  an absent one.

- **2026-08-08 — The overflow scan stays exhaustive; what gets bounded is its
  cost per slot, not its length.** An M5 iPad Pro lost the GPU inside
  `allocate_from_depth` at **31 480 of 32 768 blocks** —
  `kIOGPUCommandBufferCallbackErrorHang` on recon's compute queue, taking the
  renderer sharing the device down as collateral and reaching the app one frame
  later as an unrelated `VK_ERROR_DEVICE_LOST`, with nothing in the chain naming
  the kernel. `allocate_in_overflow` was the one loop that did not honour
  `hash_common.glsl`'s own rule that a contended dispatch must not be able to
  hang the GPU: it scanned all `num_buckets · bucket_size` entries for a free
  non-anchor slot.
  **The obvious fix — cap the scan — was tried and rejected**, because the
  length was never the expensive part. Each slot cost a contended
  `atomicCompSwap` **and two device-scope `memoryBarrierBuffer()`s taken before
  the slot had even been read**: ~430 of each per attempt at 96% occupancy,
  ×5 `insert_block` attempts, ×27 band blocks per depth pixel — ~58 k atomics
  for one pixel, still watchdog territory under any cap worth having. A cap also
  spends the only thing `kFailTable` is good for. `hash.hpp` tells a caller to
  answer it by growing and `fuse_replica` answers by doubling every attribute
  array (768 → 1536 MiB each, ~2.3 GiB transient), so a reason that can *also*
  mean "I stopped looking" converts lock contention and chain depth into an
  unbounded memory spend over a table that was never full. And `resize`'s rehash
  runs the same insert under the highest concurrency in the repo, so the
  prescribed remedy for `kFailTable` could itself fail with `kFailTable` and
  roll back to an equally full map — a permanent stall.
  **Three changes, and the order of operations in the first is the whole fix.**
  (1) The scan reads a candidate's `ptr` **unlocked first** and skips it when
  occupied, so only a slot that looks free pays the atomic — ~25× fewer of them
  at 96% occupancy. That is safe rather than lucky: alloc and free run in
  separate dispatches, so within one dispatch a slot moves only free →
  occupied, which makes the unlocked read conservative in the one direction that
  matters — a stale "free" costs a wasted lock and is caught by the
  authoritative re-test under it, and a stale "occupied" cannot happen at all,
  so the filter never skips a genuinely free slot. Lock-then-look also
  *manufactured* the contention it went on to report, seizing buckets it had no
  use for. (2) An empty heap short-circuits the scan with **one atomic load**,
  which is the state the iPad was actually in: `validate()` forces
  `num_blocks == bucket_size · num_buckets == total_entries` and every occupied
  slot on this path consumed exactly one heap block, so "no block left" and "no
  slot left" are the same statement, and the sweep was running to exhaustion to
  discover nothing. It fixes attribution too — that state now reports
  `kFailHeap`, which is what `allocate_in_primary` already calls it, rather than
  one physical condition getting two names according to which helper hit it.
  Guarded to the heap path, so `kFailHeap` stays *provably impossible* on the
  rehash preset. (3) `insert_block` stops re-running the scan on a reason it
  cannot resolve: within a dispatch `kFailHeap` / `kFailTable` / `kFailChain`
  are monotone and no other thread can insert the coord either, so only
  `kFailLock` earns another of the five attempts.
  **`kFailTable` therefore comes out *stronger*, not weaker**, which is the
  point: a sweep that skipped a free-looking slot only because another thread
  held its bucket now reports `kFailLock` — the retryable reason — so
  `kFailTable` is reached only by a clean sweep and is genuine proof the table
  is out of usable slots. It is still not the same as "every slot is taken":
  free *anchor* slots are invisible to an overflow insert by design, and the
  test fixture pins exactly that case.
  **Growth also stops being purely reactive.** The occupancy signal existed but
  was reachable only through `diagnostics()`, which scans every slot on the host
  (1.5 M at the example defaults) and carries its own `TODO(volume)` — so no
  caller could afford it per frame, every fuse loop grew only *after*
  allocations had already failed, and that is how a device reached 96% in the
  first place. `VoxelHashMap::load_factor()` is a 4-byte read of the
  host-mapped heap counter, the same quantity `HashDiagnostics::load_factor`
  reports without the scan. `fuse_replica` prints it beside the overflow reason;
  adopting an actual grow-at-threshold policy is deliberately **not** taken
  here, because it would move the peak-memory and throughput figures this file
  quotes and deserves its own measurement.
  **Verified by mutation, which the first cut of this work was not** — its test
  passed against the unbounded original *and* against a cap half the size, so it
  pinned the change in neither direction. Three independent assertions now, each
  confirmed to fail against the code it exists to catch: re-introducing a
  256-slot cap fails the 300-deep chain fixture (its 292nd overflow node sits
  333 iterations out, chosen to exceed any window worth having, on a table that
  stays ~85% free so nothing there is a capacity limit); deleting the heap
  early-out fails the pair asserting that a full table reports `heap` while a
  table whose only free slot is an anchor reports `table`, on a 2×2 fixture
  small enough to sweep by hand; and breaking the chain head-insert fails the
  one-dispatch re-allocation, the only assertion that traverses `offset` —
  `compact_active_blocks` is a flat per-slot scan, so a broken link still yields
  the right *count*, which is why the size check alone proved nothing. The
  coord search is bounded (`coords_in_bucket`, mirroring
  `volume_diagnostics_test.cpp`) rather than a loop whose only exit is success.
  What stays untested is named rather than implied: the
  `kFailLock`-on-contention path and the retry-loop early exit are both
  concurrency properties, and every fixture here dispatches one coord at a time
  precisely to keep contention out of what it measures. The hang itself remains
  unreproducible in a unit test — it needs ~131 k entries and thousands of
  concurrent invocations — so what is claimed here is the cost model and the
  reason codes, not a re-measured device.

- **2026-08-08 — `fuse_viewer` draws recon's buffers: interop seam B, end to
  end, and the release mark must be published *before* the mesh is taken.**
  *Amends* the 2026-07-07 viewer decision ("the mesh **handoff** is still a host
  mesh (interop seam A)") and closes the interop seam's "what is left is a
  consumer that draws it". recon's marching-cubes kernel writes the vertex
  arena, index run and `VkDrawIndexedIndirectCommand`; gfx binds those very
  handles as a `pipelines::LiveMesh` and issues `vkCmdDrawIndexedIndirect`, so
  the index count is read GPU-side out of the command recon wrote and never
  crosses the CPU either. What goes away is `download` → `to_gfx_mesh` →
  `upload_mesh` — a full readback plus a full re-upload of geometry that never
  conceptually left the device, every remesh.
  **`fuse_viewer` first, not `fuse_render`, and that order is the decision.**
  `fuse_render` builds *two* devices on purpose, and a `VkBuffer` is valid only
  on its creating device, so zero-copy is structurally impossible there without
  first adopting the shared-device bootstrap — and migrating it would delete the
  only coverage the two-device path has. `fuse_viewer` already had the shared
  device, the device extract and the device texturing; the host round trip was
  the one thing left.
  **The verification half is the point, as everywhere else on this seam.** The
  consumer checks `DeviceMesh::vertex_usage` / `index_usage` / `indirect_usage`
  **and `sharing_mode`** before binding, rather than trusting that the flags it
  published in `MarchingCubesConfig` reached the producer: Vulkan cannot be
  asked what a `VkBuffer` was created with, and the sharing mode is the term
  that actually varies — this machine runs the two-families plan
  (`family 0 (gfx) + family 1 (recon)`), where reading an `EXCLUSIVE` buffer
  from a non-owning family is undefined with nothing to report it, and on Apple
  undefined in the way that appears to work. A failure **latches**: the usage
  bits and families come from two constants in this file, so a mesh that is
  unusable once is unusable every time, and collecting the ones that follow
  would only walk the ring to exhaustion one undrawable generation at a time.
  No barrier is recorded here — recon's shared `dispatch()` already ends with
  one reaching `VERTEX_INPUT` / `DRAW_INDIRECT` (the 2026-08-03 decision).
  **The ordering fault this turned up is the finding worth keeping.** The
  renderer must publish its release mark **in the same critical section as the
  take, and compute it first**. Taking is what frees the fuse thread to extract
  again, and the fuse thread reads the mark in the same breath as it tests for
  an uncollected mesh — so a take published *ahead* of the mark for the same
  frame lets it run on a mark one iteration stale, ask for a slot beyond the
  ring's depth, and be refused. Measured, not reasoned: **25 refused extracts
  and 40 of 200 meshes published** on a preloaded 200-frame room0 run. It is
  invisible whenever fusion is *slower* than the render loop, which is every
  streaming run — it took `--preload` at `-O3` to surface it at all. The mark
  itself is `oldest_in_flight − 1` over a `frame_generations[frames_in_flight]`
  array whose entry for the current slot is cleared first (`begin_frame`
  fence-waited it), floored by what this frame is about to draw so the fallback
  cannot retire a live generation.
  **Three consumer obligations that have no analogue in the host path.** (1)
  *The producer must not publish over an uncollected mesh* — it still holds a
  ring slot, and the producer cannot free it (`release_through` is the
  consumer's monotonic high-water mark, so releasing that generation retires
  every older one with it, including ones in-flight frames are drawing). It
  skips the extract instead, whose result would have been discarded anyway; at
  400 frames that is **80 remeshes instead of 400**, so it is also a large
  saving. (2) *An empty extract must still be published.* It claims and stamps a
  slot like any other, so a mesh that never reaches a consumer is a slot nothing
  can ever release — `slot_count` of those and every later extract is refused,
  permanently. It draws nothing either way (`indexCount` 0). (3) *The release is
  applied by the **fuse** thread*, at the top of its next remesh:
  `MarchingCubes::release_through` is not atomic and its header makes
  serializing it against the extracting thread the caller's job, so the render
  thread records the mark and the fuse thread applies it — which also puts the
  release *before* the extract it makes room for.
  **The mesh and its atlas stay one value**, which is what this example has that
  the iOS scanner does not (its `FusionConfig::texture` is off precisely for
  want of the atlas ring). `uv0` is a coordinate into the image of the camera
  that textured it and every `texture()` call rewrites every vertex's `uv0`
  against the *current* frame, so they are published, taken and committed
  together; a failed atlas upload commits neither and the previous coherent pair
  keeps drawing. The `AtlasVersion` `shared_ptr` ring is unchanged — it owns
  gfx-side storage and is correctly ref-counted per frame slot; only the *mesh*
  ring moved to generation tracking.
  **Cost, booked rather than waved at.** `slot_count` is `frames_in_flight + 1`
  (3), and each slot is a full vertex arena that never shrinks: **~210–220 MiB
  resident across the ring** on 400-frame room0 (it varies run to run, since
  each slot is fitted independently to whatever surface it was handed), against
  ~70 MiB for the single arena a recon-only consumer keeps. Against that it
  deletes the `GpuMesh`
  (~67 MB device), two host mesh copies (~67 MB each — the `rmesh::Mesh` and the
  `vg::assets::Mesh`), and the per-remesh readback *and* upload. Verified on
  room0: 400/400 frames fused, **991 167 vertices / 330 389 triangles** —
  identical to the figure this file records for the host path — rendering a
  coherent, projectively textured room at 96.5 fps, with zero validation errors,
  zero refused extracts and one dispatch per extract.

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
  clean `ICameraCapture` in the later `sensor` tier — the **interface** is
  rebuilt here; each *driver* is placed by the buildable-and-testable rule (see
  the 2026-08-02 sensor-tier decision), so a platform-only driver such as ARKit
  lives with the app that can build and run it.

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
  ingests. Needs zero gfx changes — and, since the 2026-08-02 layout decision,
  no vertex conversion either: `mesh::Vertex` **is** `gfx::assets::Vertex`,
  indices are already `uint32_t`, and the kernel fills the `tangent` slot, so
  the host handoff is a bulk copy (and seam B the same buffer). The impedance
  the *port* reconciled is historical: the prior engine's two-stream vertex
  merged into one interleaved layout, and its per-triangle atlas baked into
  per-vertex `uv0` + a `Material` texture (gfx has no per-triangle UV; the
  `texture` tier now fills `uv0` for the live single-camera case — see the
  2026-07-07 decision). The triangle-mesh path is
  the first milestone — a
  `PointCloud` handoff waits on gfx growing a point-splat pipeline (it renders
  only meshes today).
- **B — Shared Vulkan resources (zero-copy; the live target).** recon writes the
  mesh/atlas into a `VkBuffer`/`VkImage` on a **single `VkDevice` shared with gfx**
  (one process) and gfx draws it directly — no external-memory import. Realized by
  the create/adopt device seam: a neutral bootstrap builds one device from both
  libraries' merged `DeviceRequirements` (real since 2026-08-02, in
  `examples/viewer/shared_device.hpp`); two queues from one graphics+compute
  family would avoid any queue-family ownership transfer, but MoltenVK offers no
  such family, so on Apple the buffer is cross-family and pays
  `VK_SHARING_MODE_CONCURRENT` or an explicit release/acquire — which
  `MarchingCubesConfig::queue_families` now expresses (2026-08-03); see that
  decision and the MoltenVK queue gotcha. The handoff is a ring of mesh/atlas
  slots with variable topology drawn indirectly, both real as of 2026-08-03 —
  but released by a **host-side report** rather than the intra-device timeline
  semaphore originally specified, since a cross-library GPU wait deadlocks
  against a swapchain rebuild on the shared queue. All four blockers are
  settled, and **`fuse_viewer` is the consumer that draws it** as of 2026-08-08
  — `pipelines::LiveMesh` over recon's arena, index run and indirect command,
  with the ring released by generation as its frames retire (see that decision
  for the ordering the release report has to honour). `fuse_render` stays on
  seam A, since it builds two devices by design. See DESIGN.md → "The interop
  seam".

## Key gotchas (verified)

- **MoltenVK is the Vulkan driver on Apple** — there is no other. Validate
  MoltenVK *compute* on the target Apple GPU early (prove the path before
  building on it, the gfx playbook). Metal supports compute; MoltenVK translates
  Vulkan compute → Metal compute.
- **MoltenVK caps a physical device's advertised `apiVersion` to whatever its
  instance requested.** `Instance::create` negotiates
  `VkApplicationInfo::apiVersion = VK_API_VERSION_1_2` (all recon needs), so
  `vkGetPhysicalDeviceProperties` through a recon instance reports **1.2 on
  hardware that reports 1.4 through an instance created at the ceiling**. Never
  answer "what can this GPU do" through a library's own instance: create a probe
  instance with the version from `vkEnumerateInstanceVersion` and query through
  that. This cost a false "Vulkan 1.3 unsupported" in the iOS smoke, which would
  have argued against the shared `VkDevice` on a device that fully supports it —
  so the neutral bootstrap of the 2026-07-04 interop decision must itself request
  ≥ 1.3 (gfx's floor), not inherit recon's 1.2. Related: when MoltenVK is linked
  *directly* (no loader, as on iOS) `VK_KHR_portability_enumeration` does not
  exist, so instance creation must fall back without it — which
  `Instance::create` already does.
- **MoltenVK gives one queue per family, several families — not several queues
  in one family.** An Apple M5 Max (MoltenVK 1.4.2) reports **four**
  graphics+compute+present families with `queueCount == 1` each. So the "two
  queues from one family" shape the interop-seam-B plan assumes (it avoids a
  queue-family ownership transfer) **cannot be had on Apple**: an embedder
  sharing a device between recon and gfx either takes two *families* — and pays
  `VK_SHARING_MODE_CONCURRENT` or an explicit release/acquire on any buffer both
  touch — or serializes both libraries onto one queue behind a mutex. Do not
  assume a single-family two-queue carve-out is available; check `queueCount`
  and plan the fallback (see the 2026-08-02 bootstrap decision, which does).
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
- **A bare `cmake -S . -B build` leaves `CMAKE_BUILD_TYPE` empty, so everything
  compiles at `-O0`** — the flags are `-std=c++17 -Wall -Wextra -Wpedantic
  -Werror` and no optimisation at all. Every CI leg passes one explicitly
  (`viewer.yml` uses Release), so this bites only local runs, and it bites the
  *examples* hardest because their whole job is measurement: `fuse_viewer`'s
  `atlas pack` row read **11.18 ms** against **0.32 ms at `-O2`**, a 35x
  phantom, and it was read as a real cost before the flags were checked. Always
  configure with `-DCMAKE_BUILD_TYPE=Release` before quoting an overlay number,
  and quote the build type with the figure.

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
host-side **diagnostics** scan (occupancy + collision-chain health) and the
constant-time `load_factor()` a per-frame caller can actually afford (the
2026-08-08 overflow-scan decision).
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
**`resize` preserves block indices** so per-voxel data survives a grow:
`VoxelHashMap::resize` snapshots the active set, re-inits the larger table, then a
`hash_rehash.comp` kernel re-inserts each block with its *original* pointer (the
shared `insert_block`'s `preset_ptr` — reused across normal allocation and rehash,
so the tested insert path stays single-sourced — instead of a fresh heap draw),
and the host rebuilds the free-block heap to exclude those live indices.
`VoxelBlockGrid::resize` grows every attribute buffer first (copying the old
contents forward), so a block keeps its `ptr` and its `tsdf`/`weight`/`color`
data at the same offset. The resize + block-grid tests prove the indices *and*
per-voxel data survive the grow on MoltenVK.

The first **`tsdf` tier** slice then lands on top: `TsdfIntegrator`
(`tsdf/tsdf_integrator.hpp`) fuses a posed depth frame (float metres, reusing
`vr::DepthCameraParams`) into a `VoxelBlockGrid`'s `tsdf`/`weight` attributes
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
discontinuities `> trunc_dist`), the prior engine's `sampleDepthBilinear`. An
optional `ColorFrame` fuses a posed color image through its **own separate
camera** (a dedicated `ColorCameraParams` — the color analogue of
`DepthCameraParams`, sans depth range) into a `color` attribute — RGB packed in a
`uint`'s low bytes (the mesh tier's layout), running-averaged with the SDF
weights. A voxel's **first colour
observation assigns** (gated on `color_attr == 0`, i.e. whether colour — not
depth — was seen, so a separate/unregistered colour camera never blends the
first colour toward black); dynamic mode clears a receded voxel's colour
whenever the grid carries the attribute, including on a depth-only frame. The
depth and colour projections share a `project_to_image` helper in
`tsdf_common.glsl`.

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
vertex color on MoltenVK.

A second `MarchingCubes::extract` overload meshes straight off a sparse
`volume::VoxelBlockGrid` (`mesh/shaders/marching_cubes_sparse.comp`) — the real
`tsdf`/`weight`/`color` blocks the integrator fills. It runs one invocation per
**workgroup** of each active block, striding over its voxels; a cell on a block's
`+face` reaches its far corners into neighbouring blocks, and the kernel resolves
that 2×2×2 neighbourhood **itself**, probing the hash table on-device (eight
probes per workgroup, amortised over the block's cells — see the 2026-08-08
neighbour-probe decision, which moved it off the host and states what the `mesh`→`volume`
coupling costs). Only the corner *sampling* differs from the dense kernel; the
shared per-cell body (cube index, gradient normal, reversed-winding emission,
hybrid colour) lives in `mesh/shaders/marching_cubes_common.glsl`, which both
kernels `#include` (the volume/tsdf tiers' shared-`.glsl` discipline). A corner
whose fused `color` attribute is 0 — the tsdf integrator's "colour unobserved"
sentinel (a written colour carries alpha 0xFF, so it is non-zero) — falls back to
opaque white rather than dragging the interpolated vertex toward black, mirroring
the integrator's first-observation-assigns anti-darkening rule; and the host
rejects a vertex arena larger than `maxStorageBufferRange` with a clean `Status`
instead of an opaque allocation failure — measured against what the surface
needs, since the arena is fitted to it rather than to a worst case ~48x larger
(so a grid too big for a single dispatch is caught before it, by the
`worst_case` bound the 32-bit triangle counter needs anyway).
`tests/marching_cubes_sparse_test.cpp` writes an analytic sphere into a real
6³-block grid so the surface crosses interior block boundaries, and proves the
sparse mesh matches the dense path **triangle-for-triangle** (plus cross-block
colour, that an unobserved colour meshes white, and that a sub-threshold weight
gates every cell out) on MoltenVK — the exact-count equivalence being the
cross-block-addressing proof. `mesh` depends only on the `volume` tier (to its
left, so the strict dependency rule holds).

The first **`texture` tier** slice — **live projective texturing** — fills the
mesh appearance the 2026-07-06 hybrid-colour path reserved. `ProjectiveTexturer`
(`texture/projective_texturer.hpp`) takes a `mesh::Mesh` + one posed frame's
depth + `vr::DepthCameraParams` and rewrites every `Vertex::uv0` via
`texture/shaders/texture_score.comp`: one thread per triangle projects its three
vertices into the camera (`project_to_image` mirrors `tsdf_common.glsl`'s
`project_pinhole`; the bilinear occlusion sampler shares the tsdf sampler's
intent — hole renormalisation + a depth-discontinuity fallback to the nearest
tap — but is integer-centred, see the 2026-07-07 decision) and keeps the
triangle only when all three are in front, in-frame, and unoccluded (projected
depth within an occlusion threshold of the sensor depth — the line-of-sight
test), writing `uv0 = (pixel + 0.5)/size` (the prior engine's half-texel atlas
UV) or the `(-1,-1)` sentinel otherwise. A `num_vertices` push constant bounds
the shader's `indices → vertices` addressing (a malformed mesh's out-of-range
index is skipped, not an OOB write), and the host rejects a vertex/index/depth
buffer past `maxStorageBufferRange`. The caller binds the frame's own image
(registered to the depth camera) as the renderer atlas, so gfx's
`HybridMeshPipeline` textures where a camera saw the surface and falls back to
fused voxel colour elsewhere (the 2026-07-07 decision). No score / winner-take-
all atomic (single camera; independent triangles).
`tests/texture_projective_test.cpp` textures a three-triangle mesh
(visible / depth-occluded / behind-camera) and checks the exact projected UVs,
the sentinel fallback, that a closer depth reverts a textured triangle, and that
a translated pose shifts the projection; plus that the occlusion threshold
discriminates a small within-tolerance offset from an out-of-tolerance one (and
honours an explicit tighter threshold), that a **rotated** pose projects through
`Rᵀ` to hand-computed pixels, and that a depth **discontinuity** across the
bilinear taps textures a foreground vertex via the nearest-tap fallback — on
MoltenVK.

The two GPU tiers hand off **without a host round trip**. Both write and read the
same interleaved `Vertex` array, so routing meshing → texturing through a host
`Mesh` cost a full readback *and* a full re-upload of bytes that had never left
the device (~45 MB each way on a ~940 k-vertex room scan).
`MarchingCubes::extract_device` therefore returns a `mesh::DeviceMesh` — the
vertex + index `VkBuffer`s and the live counts — which
`ProjectiveTexturer::texture` binds directly, rewriting `uv0` in place with only
the depth frame uploaded and nothing read back. `MarchingCubes::download` takes
the single host copy when one is finally needed, so the mesh crosses to the host
**once** instead of three times; `extract` is exactly `extract_device` +
`download`, so the host API and its tests are unchanged. A `DeviceMesh`
**borrows** the extractor's persistent buffers (the grow-only arena), so it is
valid only until the next extract on that extractor. `download` enforces that
with a **generation stamp**, not a handle comparison: because the arena is
grow-only and reused *in place*, a superseded view names the very same
`VkBuffer` as the live one, so comparing handles accepts it and hands back the
newer geometry under the older counts — the extractor therefore numbers its
extracts and `download` takes only its current one. The index run is the
identity `0,1,2,...` (independent triangles), held beside the arena and refilled
only on a grow, because the texturing kernel addresses vertices through an index
buffer and the renderer wants a real one at the interop seam; `download`
regenerates it on the host instead of reading it back, since it is a known
sequence and the buffer is write-combined (`SequentialWrite`), where host reads
are pathologically slow. `DeviceMesh` lives in its own
`mesh/device_mesh.hpp` so `mesh/mesh.hpp` stays Vulkan-free for pure-host
consumers such as the coming glTF exporter. `tests/texture_device_mesh_test.cpp` proves the device path is not merely
faster but *identical*: one extraction feeds both paths — the host copy textured
through the upload/readback route, and the same device buffers textured in place
— and every vertex must match on `uv0`, position, normal, and colour, with a
guard that some vertices were textured and some were not, so an all-sentinel mesh
cannot pass vacuously.

The **`examples/`** harness runs the vertical slice end-to-end on real data:
`fuse_replica` (`examples/fuse_replica/`) reads a posed Replica-SLAM RGB-D
sequence (nvblox's `fuse_replica` layout — `results/frameNNNNNN.jpg` +
`depthNNNNNN.png`, `traj.txt` camera-to-world poses, `cam_params.json`
intrinsics) via a small `examples/common` reader (stb_image decode + a tinyply
PLY writer, pinned examples-only FetchContent deps) and drives the spine per
frame: `allocate_from_depth` (growing the map through the block-index-preserving
`resize` on overflow) → `integrate` depth+colour → periodic `extract` → a binary
PLY via tinyply (interop-seam-A export, in example form). Verified on Replica
room0 (400 frames) — a coherent ~4 m metric room with plausible surface colour at
~60 fps on MoltenVK.

The gfx-linked **viewer examples** (`examples/viewer/`, behind the off-by-default
`VR_BUILD_VIEWER` — the 2026-07-07 opt-in-gfx decision) render the coloured,
**projectively textured** reconstruction through `volumetric_kit_gfx`'s
`HybridMeshPipeline`: `fuse_render` fuses a sequence and writes a colour **PNG**
headlessly (a gfx `OffscreenTarget`, CI-runnable), and `fuse_viewer` opens a
**live window** — the nvblox `FuserVisualizer` analogue — fusing on a background
thread while the render thread draws the growing mesh each frame following the
capture trajectory — on **one shared `VkDevice`** since 2026-08-02, and drawing
recon's own buffers through `pipelines::LiveMesh` + `vkCmdDrawIndexedIndirect`
since 2026-08-08 (**interop seam B**; `fuse_render` stays seam A, since it
builds two devices by design). Both run
the `texture` tier: `fuse_render` projects one keyframe (the `--follow` frame,
else the middle fused frame) onto the final mesh and binds that frame's image as
the atlas; `fuse_viewer` re-textures the growing mesh with the **current**
keyframe on every remesh (on the fuse thread, in place on the device buffers)
and swaps in that frame's image as the atlas in lockstep with the mesh version —
a per-slot atlas **ring** realising the gfx device-adopt decision's "per-slot
atlas ringing", each atlas version carrying its own descriptor pool so one bound
by an in-flight frame outlives its replacement. `--no-texture` A/Bs both against
the pure vertex-colour path.

`fuse_viewer` also carries the **perf overlay** (the 2026-08-01 decision): two
Dear ImGui panels drawn through gfx's `ui` tier, off with `--no-overlay`. A
*Performance* panel shows gfx's `Profiler` snapshot — fps, whole-frame CPU, real
GPU spans for `mesh draw` / `overlay draw` (timestamp queries; this GPU reports
64 `timestampValidBits` through MoltenVK), and the **renderer's** device memory
(`set_memory_source` points the profiler at gfx's allocator, so the panel covers
the mesh/atlas/swapchain footprint) — with recon's per-fused-frame stages
appended as wall-clock rows: `frame` (decode, or ~0 on a preload hit),
`allocate` (including any map resize), `integrate`, `extract`, `texture` and
`atlas pack`, plus the render thread's `atlas upload` — which is all that is
left of the old `mesh upload` now that the geometry never leaves the device
(seam B deleted the `download` and `to_gfx_mesh` rows outright). `texture`
is the projective-texturing dispatch alone; repacking the keyframe to RGBA8 is
comparable host work, so it gets its own `atlas pack` row rather than being
charged to the GPU pass. A *Reconstruction* panel shows fused-frame progress,
fuse ms/frame (the stage total **excluding** the `frame` read — that is
dataloading, not fusion, and it stays visible as its own row), the mesh's
vertex/triangle counts and version, the vertex arena's fill (emitted vs
capacity, in MiB) **and how many dispatches the extract took** — 2 means the
planner undershot and had to refit, which the `..dispatch` row cannot show
because it sums both attempts — the map's bucket count and block-heap
*capacity* (`num_blocks` is `bucket_size · num_buckets`, what a resize doubles
and what every attribute array is sized by — not occupancy, which would need the
diagnostics readback), recon's own device memory (`Allocator::memory_stats` —
its VMA allocator's share of the device, reported apart from the renderer's
because the two keep **independent VMA allocators over the one shared
`VkDevice`**, so the two figures partition its memory rather than
double-counting it), and the host-side preload cache. The example owns the ImGui *platform* backend
(`imgui_impl_glfw`), as gfx's own examples do, since gfx's `ui` tier
deliberately wraps only the Vulkan renderer backend.

The overlay's first finding **corrected a wrong assumption and redirected the
roadmap**. At `--remesh-every 1` on a room0 mesh of ~790 k vertices / ~264 k
triangles, `extract` cost ~55 ms/frame, which looked like whole-volume marching
cubes and pointed at the incremental block-mesh pool. The `ExtractTimings`
breakdown said otherwise:

| phase | ms |
|---|---|
| `arena alloc` | **49.8** |
| `readback` | 2.8 |
| `dispatch` | **2.0** |
| `neighbour lut` | 0.6 |
| `compact` / `descriptors` / `inputs` | < 0.2 |

(`neighbour lut` is the host table of the day; it is gone as of 2026-08-08 and
so is the field — see that decision, where the same instrument caught it costing
102 ms of a 133 ms extract at 107 k blocks.)

The GPU marching cubes was 2 ms — the pool would have optimised the one thing
that was already fast. The cost was `make_output_buffers` allocating a fresh
worst-case vertex arena (5 triangles per cell → hundreds of MB) **every call**,
so the driver faulted in and zeroed that many pages per frame. Making the arena
persistent and grow-only (below) took a 100-frame `--preload --mesh-every 1`
room0 run from **6.3 s to 0.8 s** — **15.9 → 132.5 fps**, an **8.3×** end-to-end
win — with a mesh identical triangle-for-triangle (same 793,473 vertices /
264,491 triangles, same canonical hash over the ordered triangle set). The
lesson is recorded because it generalises: *measure the phases before choosing
the optimisation* — three of us (the TODO, the roadmap, and the first analysis)
had independently guessed the wrong bottleneck.

The follow-up **fits the arena to the surface instead of to the worst case**, so
what the extractor retains is the mesh's real size rather than the 5-triangles-
per-cell ceiling it can never reach (a room scan fills ~2% of it). Correctness
does not rest on the guess: the kernel counts *every* triangle the field
produces and drops only those past `capacity`, so an undersized arena is
**detected**, never silently truncated — the host refits to the reported count
and re-runs, and since the count is a property of the field rather than of the
atomic ordering, that retry is guaranteed to fit. Steady state is therefore one
dispatch, against the two an unconditional count-then-fill pass would always
cost, and the plan is **predictive rather than reactive**: each extract records
the triangles-per-active-block density it measured, and the next call scales
that by *its own* active set (a seed of 64/block covers the first call, against
the 2560 worst case), so a growing scan plans ahead of its surface instead of
discovering every size increase by throwing a dispatch away. It scales that
density by nothing else — in particular **not** by what an arena already holds,
which is what made it compound across the output ring until an iPad Pro lost the
device (the 2026-08-03 slot-independence decision); and with `slot_count > 1`
"what the extractor retains" is one such fitted arena *per slot*, which is what
`ExtractTimings::arena_bytes` sums. Measured on the
full 400-frame room0, peak RSS: **1250 → 985 MB (−21%)** at `--mesh-every 0`,
**1568 → 1302 MB (−17%)** at the default `--mesh-every 50`, with the mesh
identical triangle-for-triangle in both (991,167 vertices / 330,389 triangles,
one canonical hash over the sorted triangle set across main, this branch, and
both flag settings). It is also much *faster* end to end at `--mesh-every 0` —
17.4 s → 5.6 s (23.0 → 71.6 fps) — because on main the single final extract
spends ~12 s faulting in and zeroing a ~2 GB worst-case arena.

Landing it exposed a latent bug in the kernel: `mcEmitCell` **returned** on
overflow instead of continuing, abandoning the rest of that cell's triangles
*uncounted*, so the reported total was a lower bound. Harmless while capacity
was the unreachable worst case, fatal the moment the host started trusting the
count — the first refit undershot and the retry overflowed again. The emit loop
now `continue`s, which is what makes "tri_count is the field's true total" an
honest contract, and `marching_cubes_sparse_test` pins it with a fixture whose
density (one block of a sign-alternating field, ~1400 triangles where the seed
plans ~64) *forces* the refit path: reverting the `continue` fails it
deterministically with an `out_of_memory`, where the sphere fixtures fit inside
the growth headroom and could not tell the difference. Two further consequences
of the host trusting the count: the arena's `maxStorageBufferRange` guard now
tests the *request* rather than the request plus growth headroom (or it would
reject the top third of legal mesh sizes), and the extract stamps its
generation when it first touches the arena rather than on success — a call that
overwrites the arena and then fails must still invalidate every outstanding
`DeviceMesh`, and so must the dense overload, which shares that arena and can
now reallocate it.

`recon_gfx_bridge.hpp` hands a `mesh::Mesh` to gfx as an
`assets::Mesh` — a bulk copy since the 2026-08-02 decision made the two vertex
structs byte-identical (layout `static_assert`s on both sides of the seam),
carrying `uv0` through: a real atlas coordinate where a keyframe textured, else
the `(-1,-1)` sentinel that takes the per-vertex-colour path. Verified: the
untextured follow-camera render
is a correct first-person room view and the live window runs the fly-through;
`fuse_render` texturing sharpens the frame-200 first-person view (crisp rug /
pillow / window mullions) where `--no-texture` is soft at 2 cm voxels.

All three fuse examples take **`--preload`**, which decodes the sequence into RAM
(`ReplicaDataset::preload`) before fusing, so the loop measures compute rather
than the reader. Streaming is **decode-bound**: per frame the reader costs
~10 ms of disk read + JPEG/PNG decode against ~1.8 ms of GPU fusion, so ~75-80%
of a streaming loop's wall clock is dataloading. With the periodic remesh kept
out of the timed region (`--mesh-every 0`) preloading lifts room0 from ~70 fps to
**~550 fps** (400 frames fused in 0.7 s) after a one-off ~4 s decode; at the
default `--mesh-every 50` the eight intermediate marching-cubes extracts sit
*inside* the timed loop and roughly halve that (~65 → ~310 fps), so quote the
flags with the number. It costs ~6 MB/frame of RAM (2.5 GB for the 400-frame
sequence) — a benchmarking and short-sequence tool, not a capture-scale one — so
each example prints `preload_bytes_projected` (which probes the sequence on disk,
so a trajectory listing more poses than it has images does not overstate the
figure) *before* spending it,
and `fuse_viewer` hands its window `quit` flag to `preload` so closing the window
mid-decode does not stall the shutdown join (the same reason its final extract is
skipped once the user has quit). Frames are served through a `FrameView` that
borrows the cached frame (no per-iteration copy of a ~6 MB frame) or owns one it
decoded on demand, so the streaming and preloaded paths run the same loop.
Verified equivalent on room0 at stride 1 *and* stride 4: streamed and preloaded
runs produce the same mesh triangle-for-triangle (identical hash over the
canonically-ordered triangles; the PLY *bytes* differ between any two runs either
way, since marching cubes appends through an atomic and triangle order is
nondeterministic). The follow-up, if an unbounded capture or startup latency
makes all-in-RAM the wrong trade, is a decode thread pool — which would also
speed the *streaming* path, where a single decoder thread caps the loop near
~96 fps.

The first **`sensor` tier** slice is the capture *contract*, not a driver: an
`ICameraCapture` (`sensor/camera_capture.hpp`) polled for a `CapturedFrame` — a
**non-owning view** of one posed RGB-D frame in exactly the shape the fusion
entry points already borrow (`const float*` depth in metres +
`DepthCameraParams`, packed-RGB colour + `ColorCameraParams`), so a captured
frame feeds `allocate_from_depth` / `integrate` with no repacking. `poll()`
returns `Result<std::optional<CapturedFrame>>`, separating "no new frame this
tick" from "the device failed" exactly as gfx's `WindowedApp::begin_frame` does;
frames are **dropped rather than queued**, which is what a live reconstruction
wants. Polling (not callbacks) is the common denominator: it wraps a push-based
source such as ARKit's session queue, while the reverse would force every
consumer onto the device's thread. Both of `poll()`'s non-error returns go
through the `no_frame()` / `some_frame(f)` helpers, because neither has a
spelling that compiles on its own (see the 2026-08-02 decision), and the
contract takes its camera types from `core/camera_params.hpp` and links
`recon_core` alone, so an out-of-tree driver compiles against the math
vocabulary rather than preprocessing Vulkan to describe a camera.

The substance is `sensor/camera_conventions.hpp` — the two conversions a capture
integration gets *silently* wrong, kept here (rather than in whichever
platform-bound driver produced the numbers) precisely because they are pure
arithmetic and can be pinned by host tests on any platform. (1)
`cv_from_gl_camera` reinterprets a pose from the OpenGL/ARKit camera convention
(+Y up, −Z forward) into the one this repo projects with (+Y down, +Z forward):
a **right**-multiplication by `diag(1, −1, −1, 1)`, implemented as negating the
second and third basis columns. Left-multiplying instead would mirror the
camera's *position* through the world origin rather than turning it in place —
which is why the test asserts the translation column survives untouched. (2)
`depth_from_registered_color` derives `DepthCameraParams` from a colour camera
the depth is registered to (ARKit: 1920×1440 colour, 256×192 depth, one physical
camera), scaling focal lengths by the size ratio and the principal point by
`c' = (c + 0.5)·s − 0.5`. That half-pixel term is not cosmetic: pixel centres sit
at integer coordinates here, so a `W`-wide image spans `[−0.5, W−0.5]`, and
dropping it biases every unprojected ray by `0.5·(1 − s)` — ~0.43 px at ARKit's
scale, a fixed bias rather than noise. It is exactly what keeps a centred
principal point centred (`(W−1)/2 → (W′−1)/2`), which is how
`tests/sensor_conventions_test.cpp` discriminates it from the naive `c·s`: both
mutations (naive rescale, and left- instead of right-multiplication) were
confirmed to fail the suite. The derived depth camera **shares the colour pose**
by construction, since two independently-assigned poses for one physical camera
are free to drift apart. It also rejects a colour focal length that is not
finite and positive — the unprojection divides by it, so a zero or NaN focal
yields inf/NaN rays that fusion reads as garbage block coordinates rather than
reporting. Host-only, so these run everywhere — the point of the
2026-08-02 decision that keeps the math here while ARKit's driver lives in
`volumetric_kit_ios`. Its colour counterpart is
`sensor/color_conventions.hpp` — `to_canonical`, the one boundary conversion,
which decodes the declared `ColorEncoding::Transfer` to linear and *then* rotates
the declared `Primaries` into the working basis (skipping the second step is the
quiet failure: "linear" alone does not name a space), refuses `Bt2020Pq` rather
than approximating an HDR curve into 8-bit SDR, and on the canonical path — what
ARKit's `{Bt709, Bt709}` takes — carries the colour bytes across verbatim while
still forcing alpha `0xFF`, so the "alpha is always written" guarantee holds on
every path rather than only where the curve runs. The *type* and `is_canonical`
live in `core` instead (the 2026-08-02 colour decision), because `tsdf` must test
them to refuse a frame and cannot include from `sensor`. Alongside all of it, a
`FakeCapture` implements
`ICameraCapture` end to end (start/poll/stop, frame handed over once, a device
failure distinguished from an empty poll) so the tier's actual deliverable —
the interface — is compiled and exercised in the repo that publishes it.

Next: first-class **glTF/GLB export via tinygltf** (the same reader gfx's
`load_gltf` uses, so the seam is one shared glTF implementation across both
siblings) + the gfx-vertex converter (interop seam A) — the example's coloured
PLY dump is deliberately a throwaway (tinyply), not that first-class exporter
(Assimp was considered and rejected as disproportionate: a large import-first lib
gfx does not use; gfx vendors tinygltf + tinyobjloader). On the `mesh` side
(greppable `TODO(mesh)`s in `marching_cubes.hpp` / `.cpp`): shared-vertex dedup,
the incremental block-mesh pool, and fitting the *dense* extract to its surface
as the sparse one does (the two share one retained arena, so a large dense call
grows it to the dense worst case). On
the `texture` side, a later slice restores the multi-keyframe post-scan atlas
(best-of-N view, packed atlas, the winner-take-all vertex atomic once dedup
shares vertices) — the counterpart to the live single-camera path now wired end
to end through both viewers.
