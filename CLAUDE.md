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

`core` → `volume` → `tsdf` → `mesh` → `texture` → `interop` (later: `compress`,
`sensor`, `track`, `codec`, `stream`).

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
- **`texture`** — projective texturing: fills the mesh's per-vertex `uv0` with a
  posed camera's image coordinates where it has line of sight (per-vertex-color
  fallback elsewhere), a compute pass.
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
  copy; `texture` consumes them now, and the `volume` / `tsdf` / `mesh` copies
  migrate next — a mechanical follow-up, greppable `TODO(core)`.)
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
  viewer opts in and pays the gfx fetch. The alternative (a standalone neutral repo, where the
  `shared_device_bootstrap` device-adoption proof still lives) was weighed and
  set aside for discoverability: the example lives with the pipeline it demos. The
  handoff is a **host mesh** (interop seam A, two devices: recon fuses on its own,
  gfx renders on its own), not the zero-copy shared `VkDevice` (that stays the
  `shared_device_bootstrap`'s concern).

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
  a host-built neighbour table, and a worst-case arena allocation all hide
  inside one call, and picking between interop seam B and the incremental
  block-mesh pool needs their split. Other tiers follow the same shape only
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
  the actual point — a vertex buffer the kernel wrote can be **bound and drawn
  as-is**, which is what interop seam B requires. gfx's vertex-input description
  reads position/normal/uv0/color at exactly these offsets with this stride; it
  does not bind `tangent` at all.
  **The cost is paid knowingly, and it is not free.** Every vertex grows 48 → 64
  bytes (+33%) for a `tangent` slot marching cubes cannot produce — it has no
  surface parameterisation to derive one from, so the kernel writes the same
  `(1, 0, 0, 1)` placeholder the host converter used to synthesize. A
  recon-only consumer (`fuse_replica`, the codec tiers, a headless exporter)
  pays that for a field it never reads: measured on the 400-frame room0,
  **peak memory +11% and throughput −10%**, against a converter saving that
  only a gfx-linked consumer sees. The alternative — teaching gfx a
  vertex-input variant for recon's tighter layout — keeps recon's bytes but
  needs a change in the sibling repo plus a pin bump here, and leaves the
  seam-B buffer still un-bindable without one. We chose the renderer's
  convention because the seam is the point: the whole reason `uv0` and `color`
  live on this vertex at all is the 2026-07-06 hybrid-colour path. Revisit if a
  non-renderer consumer ever dominates the mesh tier's traffic — the tangent is
  then 16 bytes of dead weight per vertex with no offsetting win.

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
  no per-triangle UV; the `texture` tier now fills `uv0` for the live
  single-camera case — see the 2026-07-07 decision). The triangle-mesh path is
  the first milestone — a
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
voxel of each active block (the tsdf integrator's iteration); a cell on a block's
`+face` reaches its far corners into neighbouring blocks, resolved through a
**host-built 2×2×2 neighbour table** (each active block plus its seven
`+x/+y/+z` neighbours, built from the compacted active set — the meshing dispatch
is quiescent, so no device-side hash probe, and no coupling to the hash table's
internal buffers). Only the corner *sampling* differs from the dense kernel; the
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
depth + `volume::DepthCameraParams` and rewrites every `Vertex::uv0` via
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
capture trajectory (a host-mesh handoff, interop seam A; two devices). Both run
the `texture` tier: `fuse_render` projects one keyframe (the `--follow` frame,
else the middle fused frame) onto the final mesh and binds that frame's image as
the atlas; `fuse_viewer` re-textures the growing mesh with the **current**
keyframe on every remesh (on the fuse thread, before the host-mesh conversion)
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
`allocate` (including any map resize), `integrate`, `extract`, `texture`,
`atlas pack`, `to_gfx_mesh`, plus the render thread's `mesh upload`. `texture`
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
its VMA allocator's share of its device, separate from the renderer's, since the
two run on two devices), and the host-side preload cache. The example owns the ImGui *platform* backend
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
discovering every size increase by throwing a dispatch away. Measured on the
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

`recon_gfx_bridge.hpp` converts `mesh::Vertex →
gfx::assets::Vertex` (synthesizing `tangent`, passing `uv0` through — a real
atlas coordinate where a keyframe textured, else the `(-1,-1)` sentinel that
takes the per-vertex-colour path). Verified: the untextured follow-camera render
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
