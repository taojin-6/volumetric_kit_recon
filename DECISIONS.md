# Locked decisions

The dated design record for `volumetric_kit_recon` — the *why* behind each
locked decision, with its measurements, the review findings that shaped it,
and what each fix was verified against.

Each dated; newest context wins. Change the decision *and* this list together.

[CLAUDE.md](CLAUDE.md) carries the one-line index of these plus the rules
that follow from them. Change a decision and both files in the same commit.

---

### 2026-06-21 — Single Vulkan path (MoltenVK on Apple), like gfx.

Compute is
Vulkan compute (GLSL → SPIR-V), one path across Linux / Android / macOS / iOS /
Windows — chosen over a Metal + CUDA split for cross-platform reach and a
single shader source. *Supersedes* the earlier "Metal-first / CUDA-later"
decision. Native Metal is not pursued (MoltenVK runs our GLSL); native CUDA is
now a planned NVIDIA accelerator, *not* the baseline — see 2026-07-04.

### 2026-06-21 — Trivial interop (same Vulkan API).

Because the renderer is
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

### 2026-06-21 — Independent siblings; gfx untouched.

recon and gfx stay
standalone. Now that both are Vulkan, a shared `volumetric_kit_core` is
attractive and may be revisited — but for now each mirrors the Vulkan core
independently so neither repo's release is coupled to the other. We keep the
`Status`/`Result` shape close to gfx's so a later extraction stays cheap.

### 2026-06-21 — Vertical slice first.

v1 = `core` → `volume` → `tsdf` →
`mesh` → `interop`. Compression, sensor capture, tracking, codecs are later
tiers, added once the spine renders end-to-end.

### 2026-06-21 — Codec ships DCT-only.

KLT (trained basis) and the iQuantizer
refinement are excluded as research.

### 2026-07-04 — Native CUDA accelerator, under the Vulkan baseline.

Vulkan
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

### 2026-07-04 — GLM for host/device math (dropped the hand-rolled POD types).

The `vr::Vec3f/Vec3i/Vec4f/Mat4f` vocabulary aliases GLM instead of hand-rolled
structs. GLM gives tested math, byte-for-byte packed layouts for the Vulkan
buffer ABI (via scalar block layout, since `std430` 16-byte-aligns a `vec3` --
see 2026-07-05), `__host__ __device__` operators for the CUDA accelerator, and
parity with gfx (which also uses GLM) so the interop seam needs no vector
conversions. `core` therefore takes one header-only external
dependency (GLM); Eigen was rejected (its alignment + expression templates fight
a GPU-upload POD contract). The `vr::` names stay so the backing type is
swappable, and `vr::normalize` keeps a zero-length guard GLM lacks.

### 2026-07-04 — Zero-copy interop = one shared `VkDevice` + a create/adopt seam (refines "Trivial interop" above).

Being both-Vulkan removes the *cross-API*
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

### 2026-07-05 — Shader buffer ABI is scalar block layout, not `std430`.

The
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

### 2026-07-05 — Compute core is explicit, not reflected; dispatch via `submit_single_time`.

The Vulkan compute foundation mirrors gfx's core (VMA
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

### 2026-07-05 — Per-voxel storage is a structure-of-arrays attribute store (`VoxelBlockGrid`), not the prior engine's AoS `Voxel`-in-the-hashmap.

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

### 2026-07-06 — Per-kernel resources are bundled (`core/compute_kernel.hpp`), still not reflected.

The volume tier had grown parallel `*_layout_` /
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

### 2026-07-06 — Hybrid color renders through a gfx pipeline (amends "interop seam A needs zero gfx changes").

The renderer's shipped PBR path is
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

### 2026-07-06 — Depth sampling is texture-centred (pixel centres at i+0.5), a deliberate ~½-pixel convention.

The `tsdf` bilinear depth sampler
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

### 2026-07-07 — The viewer example opts into gfx behind `VR_BUILD_VIEWER` (amends "Independent siblings; gfx untouched").

The gfx-linked reconstruction
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
`VkDevice`**. The mesh **handoff** is still a host mesh (interop seam A); the
shared device is the precondition seam B needs, not seam B itself.
*Amended 2026-08-08 (below):* `fuse_viewer` now draws recon's buffers
directly — seam B — so the precondition has been collected on.
`fuse_render` deliberately stays two-device seam A.

### 2026-07-07 — Projective texturing is a new `texture` tier; live single-camera first.

Filling the mesh `Vertex::uv0` (the atlas coordinate the
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

### 2026-08-01 — Perf instrumentation starts in the viewer example, not a shared contract package.

The question left open when a private
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

### 2026-08-01 — iOS is a downstream concern; recon cross-compiles to it unchanged.

The iOS app shell lives in a **separate sibling repo**,
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

### 2026-08-02 — `mesh::Vertex` *is* the renderer's vertex layout.

recon's
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

### 2026-08-02 — The neutral shared-`VkDevice` bootstrap lands in the viewer example, and prefers two families over a shared queue.

The 2026-07-04
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

### 2026-08-02 — The `sensor` tier is a *contract*, not a driver collection: a capture driver lives here only if this repo can build

and test** it.** The
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

### 2026-08-02 — The mesh arena's extra buffer usage is declared by the *consumer*, not named by this tier — and usage alone does not reach seam B.

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

### 2026-08-02 — One color-space rule, and a *named* working space: 8-bit color is encoded, float color is linear, converted once at the sensor boundary and encoded once at presentation.

Color arrives from a sensor encoded for a
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

### 2026-08-03 — A buffer names the *families* that will read it, and the dispatch barrier widens only as far as its queue family may.

Seam-B
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

### 2026-08-03 — The mesh arena is a ring of slots released by the *host*, not a timeline semaphore.

Seam-B blocker (1) lifetime. `MarchingCubesConfig`
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

### 2026-08-03 — The draw command is written by the kernel that counts it, and the counter's unit changes from triangles to indices.

Seam-B blocker (4),
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

### 2026-08-03 — Nothing in the mesh extractor reads "the current slot" except the code that writes it: the capacity plan is slot-independent, and a slot is marked outstanding only where a `DeviceMesh` is handed out.

Found on an iPad
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

### 2026-08-04 — A limit, a lifetime, or a staleness the caller cannot see is the library's to check, not to document.

A whole-repo review found fifteen
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

### 2026-08-08 — The sparse mesh kernel resolves its own 2×2×2 neighbourhood by probing the hash table on-device, and `mesh` therefore reads a `volume` buffer — a coupling paid for, not stumbled into.

*Reverses* the "host-built
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
matters because `dispatch` is now 77% of extract — of `extract_device`, on
desktop, the readback being most of the rest — and the levers are the probe
(spread the 8 across subgroups, or one lane per bucket slot with a reduction),
**not** the cell count (coarser voxels) or the block count (incremental
extraction) the first analysis pointed at. Left unmeasured rather than guessed
at, per this file's own lesson about profiling the phase before optimising it.
(*The block-count lever is reinstated by the 2026-08-09 incremental-extraction
decision below, on a device measurement this desktop one could not see.*)
`ExtractTimings::neighbour_lut_ms` is **removed**, not retained as a zeroed
phase: a permanently-0.00 row in the overlay that diagnosed this is worse than
an absent one.

### 2026-08-08 — The overflow scan stays exhaustive; what gets bounded is its cost per slot, not its length.

An M5 iPad Pro lost the GPU inside
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

### 2026-08-08 — `fuse_viewer` draws recon's buffers: interop seam B, end to end, and the release mark must be published *before* the mesh is taken.

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
No barrier is recorded here, and **the reason is the fence, not the barrier
scope** — a distinction the first cut got wrong. recon's extract blocks on its
own fence inside `submit_single_time` before anything is published (an
availability operation over every device write it made) and the renderer's
later `vkQueueSubmit` makes those writes visible to the draw; that chain is
what orders the two. recon's shared `dispatch()` barrier *does* also name
`DRAW_INDIRECT`, but it names `VERTEX_INPUT` only where its queue family
advertises graphics — Vulkan forbids that stage on a compute-only family, and
the bootstrap matches recon's family on `VK_QUEUE_COMPUTE_BIT` alone, so off
Apple recon can sit on exactly such a family (the 2026-08-03 decision says so
and this seam must not restate it as unconditional).
**The ordering fault this turned up is the finding worth keeping.** The
renderer must publish its release mark **in the same critical section as the
take, and compute it first**. Taking is what frees the fuse thread to extract
again, and the fuse thread reads the mark in the same breath as it tests for
an uncollected mesh — so a take published *ahead* of the mark for the same
frame lets it run on a mark one iteration stale, ask for a slot beyond the
ring's depth, and be refused. Measured, not reasoned: **25 refused extracts**
on a preloaded 200-frame room0 run, against **zero** after the fix. The
refusal count is the discriminator and the publish rate is not — under the
fault ~40 of those 200 frames published a mesh, which is indistinguishable
from the *healthy* rate the uncollected-mesh gate produces (80 of 400,
below), so anyone re-measuring a regression has to read the refusals. It is
invisible whenever fusion is *slower* than the render loop, which is every
streaming run — it took `--preload` at `-O3` to surface it at all. The mark
itself is `oldest_in_flight − 1` over a `frame_generations[frames_in_flight]`
array whose entry for the current slot is cleared first (`begin_frame`
fence-waited it), floored by what this frame is about to draw so the fallback
cannot retire a live generation. The panel snapshot is read in that same
section, for a smaller version of the same reason: `begin_frame`'s per-slot
fence wait can span a whole frame and the fuse thread routinely publishes
inside it, so a snapshot taken before it printed one generation's vertex
counts above another's arena and dispatch count.
**Consumer obligations that have no analogue in the host path**, and they are
all consequences of the mark being a single monotone high-water value. (1)
*The producer must not publish over an uncollected mesh* — it still holds a
ring slot, and the producer cannot free it (`release_through` is the
consumer's mark, so releasing that generation retires every older one with it,
including ones in-flight frames are drawing). It skips the extract instead,
whose result would have been discarded anyway; at 400 frames that is **~80–90
remeshes instead of 400** (87 on the run below), so it is also a large saving.
(2) *An empty extract
must still be published, and must still be **committed**.* It claims and
stamps a slot like any other, so a mesh that never reaches a consumer is a
slot nothing can ever release — `slot_count` of those and every later extract
is refused, permanently. On the consumer side that means committing it as the
live view (which is what parks its generation for release) even though it
draws nothing, and *not* running it through the bindable check: recon names an
empty extract's buffers as it found them, so a slot that was never sized
carries **null handles** beside `empty()` being true, and a check that folds
`valid()` in with the usage bits reads recon's own legal path as a
configuration fault. Since that fault latches (below), doing so stopped the
viewer drawing *and* extracting for good — reachable from `--max-frames 0`, a
frame-0 load failure, or a first depth frame with nothing in range. (3) *A
taken generation that is not committed must be bounded.* The mark cannot skip
one, so a generation the consumer takes and then drops keeps its slot until
some *newer committed* generation sweeps past it — and the commit that would
produce one is exactly what failed. Two dropped takes fill a three-slot ring
and every later extract is refused, permanently, with no way back. So a failed
atlas upload **parks** its pair and retries it on the next frame instead of
dropping it, and the take is gated on nothing being parked, which bounds the
uncommitted set at one. (4) *The release is applied by the **fuse** thread*,
at the top of its next remesh: `MarchingCubes::release_through` is not atomic
and its header makes serializing it against the extracting thread the caller's
job, so the render thread records the mark and the fuse thread applies it —
which also puts the release *before* the extract it makes room for.
**The mesh and its atlas stay one value**, which is what this example has that
the iOS scanner does not (its `FusionConfig::texture` is off precisely for
want of the atlas ring). `uv0` is a coordinate into the image of the camera
that textured it and every `texture()` call rewrites every vertex's `uv0`
against the *current* frame, so they are published, taken and committed
together; a failed atlas upload commits neither and the previous coherent pair
keeps drawing while the new one is retried (obligation 3). The `AtlasVersion`
`shared_ptr` ring is unchanged — it owns gfx-side storage and is correctly
ref-counted per frame slot; only the *mesh* ring moved to generation tracking.
**The final full-volume extract is not gated on collection.** It supersedes
whatever the last in-loop remesh published, and recon refuses cleanly if the
ring really is full, so skipping it on an uncollected mesh only threw away the
complete surface — silently, and on the very path that produces one: a
minimized window makes `begin_frame` return no frame, so the render loop never
reaches its take, the bounded wait times out, and the run ends replaying a
partial mesh beside "fused 400/400". `quit` is re-checked *after* that wait
too; the guard and the extract used to be adjacent statements, and a window
closed inside the (up to one second) wait fell straight through into a full
marching-cubes pass the guard exists to skip.
**Cost, booked rather than waved at.** `slot_count` is `frames_in_flight + 1`
(3), and each slot is a full vertex arena that never shrinks: **~205–220 MiB
resident across the ring** on 400-frame room0 (it varies run to run, since
each slot is fitted independently to whatever surface it was handed), against
~70 MiB for the single arena a recon-only consumer keeps. Against that it
deletes the `GpuMesh`
(~67 MB device), two host mesh copies (~67 MB each — the `rmesh::Mesh` and the
`vg::assets::Mesh`), and the per-remesh readback *and* upload.
**The residency class changes with it, and that is the part that does not
travel.** The `GpuMesh` this replaces was `DeviceLocal`; recon's arena and
index run are host-visible mapped memory, because `core::storage_buffer`
allocates every recon buffer that way. So the vertex-input stage now fetches
the whole mesh out of system RAM on every *presented* frame — ~64 MiB at these
counts — where the old path paid ~67 MB per *remesh* into device-local
memory. (`share_vertices` would cut that ~4x, and this example deliberately
does not take it: it runs the `texture` tier, which the flag is incompatible
with — see the 2026-08-08 decision.) On Apple's unified
memory, which is the only hardware this was measured on, that is free and the
seam is a clear win; on a discrete GPU it would invert. Booked as a greppable
`TODO(mesh)` beside the host-visible indirect command's, waiting on the same
discrete-GPU consumer to measure a device-local arena plus staging.
Verified on room0: 400/400 frames fused, **991 167 vertices / 330 389
triangles** — identical to the figure this file records for the host path —
rendering a coherent, projectively textured room at **~100–120 fps** (the
panel's figure moves with what is on screen; 96.5, ~106 and 102–119 across
three runs), with zero validation errors, zero refused extracts and one
dispatch per extract.

### 2026-08-08 — In-block vertex sharing is a *second compiled kernel*, not a branch, and the two emitters must interpolate an edge in the same direction.

`MarchingCubesConfig::share_vertices` makes the cells that meet on an edge
index one vertex instead of emitting three private ones each: every cell emits
a vertex only on the three edges it **owns** (+x/+y/+z from its base corner,
derived from the tables by `mcEdgeOwner` rather than tabulated a fourth time),
and its neighbours look that up in `shared int s_edge_vtx[]`. Edges on a
block's `+face` are still duplicated — sharing them would need the
neighbouring workgroup's shared memory — which is why the saving is **~4x**
rather than the ~6x full sharing would give. Measured on Replica room0:
3.4x fewer vertices, 3.5% faster end to end. Two consumer-visible
consequences, both published on `DeviceMesh` rather than left to be inferred:
the index run stops being the identity `0,1,2,...` (`shares_vertices`), and
`vertex_count` stops being `3 * triangle_count`.
**Two kernels, chosen at `create`, and that is the decision.** Sharing needs
~8 KiB of `shared` arrays, and a `shared` array is reserved when the pipeline
is created whatever a push constant later says — so the first cut, one kernel
with a `share_vertices` push constant, made the **default** path pay
sharing's threadgroup budget (32 B → 8 224 B, verified by SPIR-V
disassembly; **44 B → 8 428 B** on today's kernels, re-verified the same way
after incremental extraction's stage 2 gave the default path a block
reservation and sharing an edge-owner table — the gap is the point, not the
endpoints), which bounds residency to 3 workgroups on Apple's ~32 KiB and
to **1** at Vulkan's guaranteed 16 KiB floor — on the kernel this file records
as 77% of an extract. It also cost the default path its memory *shape*:
routing both modes through the sharing emitter gave every corner its own
interleaved `atomicAdd(vertex_count, 1u)`, so a triangle's three 64-byte
writes landed at three arbitrary offsets in a multi-hundred-MB arena and the
index run came out non-monotonic — lost write coalescing in the kernel, lost
vertex-fetch locality in the seam-B draw that reads it. `fuse_viewer`,
`fuse_render`, the whole `texture` tier and the iOS scanner all take that
path for a feature none of them enables. So `marching_cubes_sparse.comp` is
the default and is byte-for-byte what this tier always did, and
`marching_cubes_sparse_shared.comp` is its sibling; what they do identically
(the neighbourhood probe, the corner gather, the block addressing) is
`marching_cubes_sparse_common.glsl`, the same shared-`.glsl` discipline
`marching_cubes_common.glsl` applies to the dense/sparse pair.
**The bug that made "sharing moves only the vertex count" false was the edge
DIRECTION.** `kEdgeToVert` lists edges 2/3/6/7 max-corner-first, so a cell's
owned +y edge (edge 3 = corners {3,0}) is its neighbour's edge 1 (corners
{1,2}) — the same segment, traversed opposite ways — and both `mix` and the
near-tangent guard are direction-dependent. Normally that is ulps; under the
guard it is a **whole voxel**, because at `sa = -1e-9, sb = +1e-9, iso = 0`
the guard forces `denom` to `+1e-6` one way and `-1e-6` the other and *both*
come out at ratio ~1e-3, hugging whichever endpoint was passed first.
`mcEdgeVertex` therefore orders the endpoints canonically (by corner shift —
the same rule `mcEdgeOwner` names the edge by, so the two cannot disagree),
and `mcEmitCell` now calls it instead of carrying a verbatim copy of its
body, which is what made the drift possible. That also makes the shared and
unshared meshes comparable **triangle for triangle on exact position floats**,
which is the invariant the suite asserts.
**`vertex_count` counts vertices, not claims.** With a per-vertex global atomic,
a phase-1 claim that overflowed the arena had no usable slot, so the cells
referencing that edge re-claimed a replacement and double-counted: an
overflowing dispatch reported `V_owned + ~3T` instead of the true ~0.75T —
*more* than not sharing at all — and the host refits a **grow-only** arena to
that number: ~95 MB pinned on room0 against the 15.9 MB the shared surface
needs, which is the entire thing the feature exists to save. That path is
taken by design on the first sharing extract of any real surface, since
`kSeedTrisPerBlock` deliberately plans low. It was first fixed with a
`kVertexDropped` sentinel distinct from `kNoVertex`; the stage-2 restructure in
the 2026-08-09 entry below **removes the sentinel**, because a block that counts
first and reserves once gives every vertex a deterministic slot whether or not
it lands inside the arena, so the totals are exact by construction rather than
by bookkeeping.
**Sizing two buffers means two budgets, and they must not read each other.**
The vertex arena and the index run are now grown **independently**: they used
to be released together because `arena_capacity()` came off the *arena*, which
stopped being true the moment sharing moved it to the index run — and keeping
the coupling grew whichever buffer *fitted* to 1.5x what it already held,
compounding on every such event. That is numerically the ring runaway the
2026-08-03 slot-independence decision exists to prevent, reintroduced one
buffer over, and it refaulted the arena — the phase measured at 49.8 ms of a
55 ms extract. Relatedly, `plan_capacity` bounds its triangle request by
**both** buffers' device limits (`max_triangles_for` covers only the index
run's 12 B/triangle; the arena's three private vertices are 16x tighter), and
`plan_vertex_capacity` clamps in *both* modes: a predicted quantity is
clamped, never rejected (the 2026-08-04 rule), and the unshared branch had no
device clamp at all, so on a driver at Vulkan's guaranteed 2^27 a plan could
be rejected by `ensure_output_buffers` — **permanently**, since the density is
recorded only on success and a scan's active set only grows.
**A failure disarms the draw command rather than clamping it.** Clamping
bounds how many indices a draw *reads* and says nothing about their *values*,
and on every failure path the values are the problem: a triangle whose vertex
claim overflowed is counted but never written, so its three index slots hold
whatever the VMA block last did, and the dense kernel never writes the index
run at all. Arbitrary `uint32`s into a 64-byte-stride arena with
`robustBufferAccess` enabled nowhere is a GPU fault, where "this slot has no
drawable geometry" is the honest statement — so `indexCount` goes to 0, on the
two sparse failure returns *and* at the end of the dense overload, which
publishes no `DeviceMesh` and used to leave a live-looking command behind.
**What the library checks rather than documents**, per the 2026-08-04 rule.
`ProjectiveTexturer::texture` **refuses** a mesh carrying `shares_vertices`:
it decides visibility per *triangle* and writes `uv0` per *vertex*, so where
an interior vertex is referenced by up to six triangles that disagree the last
writer wins nondeterministically, and a triangle left holding one sentinel
interpolates from `(-1,-1)` across its whole face — `Status::ok`, no
validation diagnostic, visible only as flicker. The real fix is a
per-primitive camera id, which the planned packed multi-camera atlas needs
anyway; this flag keeps the two decisions independent. And the sharing
kernel's compile-time `kMaxSharedCells` is mirrored by a host constant that
gates the per-*extract* refusal of a larger `block_size` (not `create`'s — the
block size arrives with the grid), with nothing but a comment tying the two
across languages; so the kernel **reports** whether it actually shared, in a
scratch word past the draw command, and the host fails on it. Raising the host
constant alone would otherwise admit a block the kernel silently declines.
**Verified by mutation**, since a test that cannot fail is worse than none.
`share_vertices` had **zero** coverage in the first cut — `bool sharing =
false;` left every suite green — and two of the assertions added beside it
were tautological restatements of what `download()` had just computed. Four
independent mutations each fail a named line now: dropping the canonical
endpoint order fails the shared-vs-unshared triangle-for-triangle equality;
removing the `kVertexDropped` sentinel failed a *magnitude* bound on the
refitted arena (the shape passes either way, because the second dispatch's
counter is the true total whatever the first reported — what changes is what
the grow-only arena is refit to), and that sentinel and that mutation are both
retired by the stage-2 restructure below, which carries its own; restoring the
coupled buffer growth fails a
case built from the measured capacities, where the dense overload's 3:1 vertex
demand grows the arena of a sharing extractor whose index run already fits;
and deleting the `sharing_applied` report while raising the host constant
turns a refusal into a silent empty mesh. What is **not** covered is named
rather than implied: the arena-overflow path is exercised only through a short
plan, not through concurrent contention, and no fixture measures the
threadgroup-occupancy claim above — it rests on the SPIR-V allocation and the
documented limits, not on a re-measured device. (The figure there moved to
8452 B when stage 2 added the sharing kernel's six reservation words; the
default kernel's 44 B is unchanged, and the gap is the decision.)

### 2026-08-09 — A dirty block is one the fuse *changed*, the flags are anchored to a grid the library checks, and tracking them is opt-in.

`tsdf` can now
report which blocks a fuse invalidated — the input an incremental re-mesh
needs, and the first thing on the roadmap that the whole-volume extract's
`dispatch` phase (77% of an extract since the 2026-08-08 neighbour-probe
decision) actually points at. Four things about its shape are the decision,
and each was a defect in the first cut.
**(1) The flag means the field moved, not that a store happened.** The kernel
marked unconditionally at the top of the fuse. In **classic** mode — the
default, and what `fuse_replica` runs — free space ahead of the surface is
fused too (that is what classic *is*), so the only narrowing above the mark
was the occlusion return and the mark collapsed to *in-frustum ∧ valid depth
∧ not occluded*: a frustum survey wearing a different name, which is exactly
what `dirty_block_count`'s header said it was not. It now compares the new
`tsdf`/`weight`/`color` against the old and marks after the stores it
describes, so a voxel converged at `max_weight` — re-storing bit-identical
numbers, the steady state of any revisiting scan — marks nothing. Note the
asymmetry that gave it away: the dynamic-clear store was already guarded by
`weight > 0.0`; the fuse store was guarded by nothing.
**Booked honestly: this narrows the set but does not make it small.** On
room0 at 120 frames it reports **62% of active blocks changed** per 20-frame
window and 59% per *single* frame — because in classic mode a moving camera
keeps shifting the running average of everything it sees, which is a real
change, not a miscount. The dilated re-mesh set is 83%. So the honest
headline is that an incremental extract's ceiling here is ~1.2x, not the
order-of-magnitude the first cut's `speedup %.1fx` implied — and that figure
is **removed** rather than corrected, because it divided active blocks by
re-mesh blocks and so assumed extract cost is proportional to block count
with zero fixed cost, which this example's own phase table refutes (`compact`
walks every table slot regardless, `arena` is sized by the whole surface,
`readback` copies all of it; only `dispatch` scales).
**(2) A flag is keyed by block SLOT, so it means nothing without a grid — and
the library checks which one.** Slots come from a LIFO heap, so the same index
means a different block on a different grid, and a different *coordinate*
after a removal. Neither is knowable from the flags, and neither was knowable
to the caller, whose only instruction was a `@param` sentence — the shape the
2026-08-04 rule exists to close. `VoxelBlockGrid` therefore publishes
`topology_epoch()` (bumped by `remove`/`clear`, deliberately **not** by
`resize`, which preserves every index), and the integrator records the grid
pointer plus that epoch. A different grid **resets** the flags and re-anchors
— refusing instead would bind an integrator to one grid for life, and one
integrator over many grids is already how the tsdf suite is written, which is
how the silent OR-ing went unnoticed. A removal on the *same* grid **refuses**
until `reset_dirty()`, because the geometry that went away is stale and no
surviving flag can say so: "re-mesh everything" is the only true answer, and
only the caller can act on it. The pointer is held for identity and never
dereferenced.
**(3) A grow carries the flags forward.** The array was reallocated and zeroed
on every map grow under a comment asserting the opposite ("the surviving flags
stay correct"), so a window straddling a `resize` forgot every block fused
since the last reset — silently, and precisely on the frames a growing scan
brings in the most new surface, which are the frames that trigger the grow. It
now memcpy's and zeroes only the tail, mirroring `VoxelBlockGrid::resize`,
which grows the attribute arrays the same way and cites the same
index-preserving rehash.
**(4) It returns the block set, and tracking is off by default.** A count
cannot drive the incremental extract the API exists for, and the walk already
computes the coordinates, so `dirty_remesh_blocks` hands back `Vec3i`s and
`.size()` is the count. It takes the caller's already-compacted active set
rather than compacting its own — the flags carry slots, not coordinates, so
every caller has just compacted, and a second compaction inside was a
dispatch, a fence wait and a full readback (0.15–0.26 ms at the examples'
defaults) on a call already O(active). The walk also inverted: it gathers over
active blocks testing their `+{0,1}³` octant, which makes the
existence filter *structural* (only blocks that exist are considered) and
needs no dedup, against the two hash sets it replaced. And
`TsdfIntegratorConfig::track_dirty_blocks` defaults **off** — the 2026-08-01
bar is "nothing measured when the caller did not ask", and always-on cost
every integrator a `num_blocks * 4` host-visible array (6 MB at
`VoxelGridParams::defaults()`, doubling per grow) plus a store per voxel, paid
by `fuse_viewer`, `fuse_render`, the iOS scanner and the whole test suite for
a flag none of them reads. Off binds a 1-element dummy to the slot and pushes
`track_dirty = 0`, the `color_dummy_` pattern this file already uses.
The per-voxel store is now `atomicOr` and guarded by the change test, so the
atomic is paid only by voxels that moved (none once a block converges) — a
non-atomic race on a shared address is undefined rather than benign without
`coherent`/`NonPrivatePointer`, and the two doc sites claiming the kernel
"needs no atomics" are amended rather than left to read as whole-kernel.
**Verified by mutation, and the fixture is the reason.** The original
assertions could not fail: their dirty set was a contiguous 1-D run whose `-z`
dilation is a no-op, so dropping the dilation, dropping the existence filter,
inverting the octant to `+x/+y/+z` (which the PR body itself called "the
natural wrong guess") and `return active.size()` all passed, as did a
vacuous pre-integrate `== 0` that held with the guard deleted. The fixture is
now a 2×2×2 cube straddling the band plus one off-camera block, so changed=4,
re-mesh=8 and active=9 are three different numbers, and eight independent
mutations each fail a named line: the copy-forward, the change predicate, the
re-anchor reset, the dynamic-clear mark, the octant, the feature itself, the
opt-out, and the topology check. Fusion numerics are untouched — room0 at 120
frames gives 832 518 vertices / 277 506 triangles with tracking on *and* off,
identical to what this file records for main. What is **not** covered is named
rather than implied: the threading obligation on `dirty_block_count` /
`reset_dirty` / `dirty_remesh_blocks` is documented (mirroring
`MarchingCubes::release_through`) and **not** checked, because the reader they
race is a fuse thread that can free the mapping mid-read and a flag a second
thread could race is not a check; and the `atomicOr` contention path is
exercised by no fixture, since every fixture here dispatches one frame at a
time.

### 2026-08-09 — Incremental mesh extraction is worth building at a ~4x ceiling, not the ~18x one window suggested, and the worst frame rather than the median sizes its design (amends the dirty-block decision above).

The dirty-block decision above measured coverage on **room0** — 62% of active
blocks changed per 20-frame window, 59% per single frame, **83%** once dilated
into the `-x/-y/-z` octant — and put an incremental extract's ceiling at
**~1.2x**, a clear "do not build it". It also *removed* the first cut's
`speedup %.1fx` read-out rather than correcting it, because dividing active
blocks by re-mesh blocks assumes extract cost is proportional to block count
with zero fixed cost, which this repo's own phase table refutes. Both of those
stand as **method**. The **verdict** does not: room0 is the wrong scene to have
measured coverage on, and this entry amends it. (Same date as the entry it
amends, so read this one as the later — file order is not the tie-breaker the
top of this file promises.)

**Measured on an iPad Pro M5**, one real handheld walk, map saturated at
111 942 blocks, sampling `dirty_remesh_blocks` / active per window:

| to re-mesh | ceiling (1 ÷ share) |
|---|---|
| 5.6% | 17.9x |
| 18.4% | 5.4x |
| 31.5% | 3.2x |
| 34.4% | 2.9x |

Only the left column is measured. The right one is that same invalid division,
kept because an upper bound is still worth having and labelled so it is not
read as a speedup: the realised factor is strictly below it by whatever share
of an extract does not scale with the block count, which is **not** measured on
this device.

**Median share 25% → a ~4x ceiling; worst sampled 2.9x, best 17.9x.** At the
ceiling the walk's `meshing` row goes 51.85 ms → **≥13 ms**, taking its ~64 ms
frame to **≥25 ms**; the real numbers land above both. So it is worth building
— but at roughly a *quarter* of what the best sampled window promised, and
quoting that window would have been the same error as quoting room0, one
optimism in place of one pessimism.

**room0 is a correctness fixture, not a coverage one**, and that is the whole
gap. It is one enclosed room shot through a 90° cone (`fx` 600 over 1200 px),
so the camera sees very nearly the entire surface every frame and *almost
everything it sees genuinely changes* — in classic mode a moving camera keeps
shifting the running average of every voxel in view, which is a real change,
not a miscount. Like for like, both sides dilated: **83% on room0 against the
25% device median**, so room0 is **~3.3x** pessimistic — and pessimistic
precisely because it is a good correctness fixture: total visibility is what
makes it reproduce triangle-for-triangle. Every conclusion drawn from it alone
about *coverage* was wrong. This is the same family as the
correct-on-the-machine-that-ran-it hazard the seam-B decisions kept turning up,
one level out — not wrong hardware but a wrong *scene*, and nothing in a green
test run says so.

**A proxy for the instrument was worse than either.** Before
`dirty_remesh_blocks` existed, a frustum survey stood in for it: it read 87% on
room0, and on device it was wrong in **both directions** — 4.5% against the
**18.7%** the instrument read on the same walk, off by ~4x in the optimistic
direction. (That 18.7% is a separate reading from the table above, whose
nearest row is 18.4%; which windows the two cover is not on record.) A survey
of what the camera *could* touch is not a measurement of what a fuse *did*
change, and the two do not even err consistently. That is why the dirty-block
decision above insists the flag mean the field moved rather than that a store
happened; a cheaper approximation of this quantity has already been tried and
it does not work.

**The variance, not the mean, is the design constraint.** A system that reaches
17.9x on its best sampled window and 2.9x on its worst still has to fit the
worst, and a frame-rate budget sized on the median tears exactly when the
camera sweeps into new geometry, which is when a scanning user is most likely
to be looking. So incremental extraction is planned **beside** a remesh
*cadence* cap rather than as a replacement for one: the cadence bounds the
worst frame, the incremental path buys back the median. Sizing anything on the
~4x alone would reintroduce the per-frame cliff that the whole-volume extract
at least had the virtue of being honest about.

**Dilation is cheap and needs no hedging.** The `-x/-y/-z` octant a
marching-cubes stencil reaches back through costs **1.16–1.31x on device** and
1.30–1.40x on room0 across three resolutions. It is a property of the stencil,
not of the scene, so the ratio is stable and no allocator has to carry a margin
against it. It is also already *inside* every figure above:
`dirty_remesh_blocks` returns the dilated set, so the ~4x ceiling is computed
on the blocks an incremental extract would actually redo and must not be
divided by the dilation a second time.

**The four stages, so "stage 2" has an antecedent.** Stage 1 is the input and
has landed: `TsdfIntegrator::dirty_remesh_blocks`, the decision above. Stage 2
is per-block contiguous emission. Stage 3 is dirty-only dispatch over that set.
Stage 4 is compaction of the retired spans. Each needs the one before it.

**Stage 2 has landed on both sparse kernels; stage 3 is next.** Both used to
append through *global* atomics, so one block's output interleaved with every
other block's and **per-block ranges could not exist**: the default
`marching_cubes_sparse.comp` bumped `index_count`
once per triangle (`marching_cubes_common.glsl`) and wrote that triangle's
three vertices at `tri * 3`, while `marching_cubes_sparse_shared.comp` bumped
two independent counters, `vertex_count` per vertex and `index_count` per
triangle. **Both** had to restructure to **count → reserve one span per block →
emit** — the default kernel to one `atomicAdd`, the sharing kernel to two —
because `share_vertices` selects one of the two at `create` and is fixed for
the object's lifetime, so a kernel stage 2 skipped would be permanently
ineligible for stages 3 and 4. Both now do.

**What the existing golden test covers, and what it does not.** It compares the
two meshes' triangle sets resolved through the index buffer, *sorted*, on
position alone (`canonical_triangles` in `marching_cubes_sparse_test.cpp`) —
because the kernel appends through an atomic, output has never been
byte-identical run to run, and this entry's first draft claiming stage 2's
output *is* byte-identical overstated both the property and the coverage. That
test does validate stage 2's **geometry**: the same triangles have to come out.
It cannot see stage 2's actual deliverable — that each block's output occupies
one contiguous, non-overlapping span — so stage 2 ships an assertion of its own:
attribute each triangle to a block by its centroid, walk the index buffer in
order, and require the owning block to change **exactly `distinct - 1`** times.
Exactly, not approximately, because the attribution is provably exact — the only
centroid that could fall into the neighbouring block is one lying on the far
face of its cell, which takes three sign changes around a 4-cycle, and parity
forbids it. It runs on a clean extract and on one that refit against an
undersized arena, where truncating a span shortens it without splitting it.
Without that assertion a span off by a triangle first surfaces in stage 3,
confounded with exactly the dirty-set machinery stage 2 is sequenced first to
avoid.

**It is a cost, not a win, and that corrects this entry's first draft**, which
claimed stage 2 "removes the interleaved per-vertex atomics the vertex-sharing
review measured as destroying write coalescing — a win even if stages 3 and 4
are never built". Measured on room0 at `--voxel 0.012`, Release, the extract
dispatch goes **1.17 → 1.28 ms, ~10%**, in four interleaved samples each way.
The draft conflated the two kernels: the per-*vertex* interleaving that review
measured is the **sharing** kernel's, while the default kernel — the one
measured here — already wrote each triangle's three vertices contiguously at
`tri * 3` and interleaved only whole triangles, so there was little coalescing
left to win and the second visit costs more than it recovers.

**What moved that number, and what did not.** The first cut of the counting
phase gathered every cell in full and measured **+13%** (1.167 → 1.315 ms).
Counting on the eight corner *signs* alone — no `sdf`/colour array copied out of
the gather and no sRGB decode, for a phase whose only consumer is the cube index
— brought it to the ~10% above. Nothing about *where* the per-cell triangle
count is kept has ever moved it: caching the count rather than the cube index
changed 1.318 → 1.315 ms, and moving that cache out of `shared` into a private
register changed 1.28 → 1.28 ms. The cost is the second visit itself, not the
lookup. The private cache was taken anyway, and on a different argument: it
returns the default path's threadgroup footprint to **44 B**. The 2026-08-08
decision above splits the two kernels precisely so this path does not carry
sharing's, and a two-phase emitter that spent 512 B of `shared` — plus an
`atomicOr` and a zeroing pass — on a cache no invocation but its own writer ever
reads would have handed part of that back for nothing.

**The sharing kernel's restructure costs nothing measurable, and that is not
the same argument.** Three interleaved samples each of the old and new kernels
span 1.77–1.82 ms on the extract dispatch and their means differ by less than
that spread (room0 at 120 frames, `--voxel 0.012 --share-vertices
--device-extract --preload`, Release). Two reasons it is free where the default
kernel's cost ~10%: this kernel already made two visits over a cached
classification, so the counting phases reuse a `s_case` that had to exist
anyway; and the pass that runs over 100% of the cells reads the eight corner
*signs* alone (`mcCellSigns`), the same lesson that took the default kernel from
+13% to +10%. Caching the per-cell triangle count beside the cube index — free,
in bits the cube index does not use — then lets phase four claim a cell's whole
run with one threadgroup atomic instead of one per triangle (~320k against
~766k at this scale), and lets both later passes walk a `tri_table` row by count
rather than each re-finding its `-1` terminator, so the terminator is walked in
exactly one place, `mcCellTriangleCount`, which is also where that walk is
bounded against a truncated host upload. Geometry is unchanged as a triangle set
and as a vertex count (766 117 / 668 792, byte-length-identical PLY with
different bytes — the arena order is precisely what moved). Stage 2 overall is
therefore still justified as the precondition, and the cost is the default
kernel's ~10% on the one phase that scales with the block count — `dispatch`,
which the 2026-08-08 neighbour-probe decision puts at 77% of a *desktop*
`extract_device` and which is unmeasured as a share on device.

**Each cursor is bounded by its own block's reservation, on both kernels.** The
default kernel's span bound was argued as making a count/emit disagreement fail
*safe*; the sharing kernel needs it more and did not get it in the first cut.
Its count and emit phases run *different* predicates over *two* ranges, and one
escape exists — the "cannot happen" re-gather in the owned-edge pass, which
fails in the **under**-reservation direction, because an owner that emits
nothing sends every cell referencing its edges down the duplicate path instead.
Forced on one cell of a `block_size` 3 block, that drove consumption past
reservation in 1070 of 3000 trials, writing into the next block's range with
`Status::ok` and exact counters. The bound turns it into dropped geometry.
Both bounds are guards on a state no fixture can reach, so **neither is
mutation-covered and that is stated rather than implied**: what is covered is
the arithmetic they guard.

**What stage 2's mutations cover on the sharing kernel.** Dropping the
duplicate-count branch — the one the count phase exists for — fails a named
index-range bound on the refit fixture, whose mesh the first cut discarded
while asserting magnitudes that passed either way. Putting the vertex claims
back on the global counter, leaving the triangle span intact, fails the
**vertex**-span assertion and nothing else: `block_layout` walks the index run,
so it is structurally blind to the second range this kernel reserves, and every
transition count in the file passes against a fully interleaved arena. Both
assertions run on a clean extract *and* on one that refit against an undersized
arena, and on a 27-block fixture rather than the single-block one — with one
block, a cursor that overran has no next block to land in, so the failure is
unobservable there in principle. The unshared extract of the same field is the
oracle for both.

**The tension with seam B is real and is *not* yet resolved.** Incremental
extraction wants **in-place mutation** of a persistent arena; the seam-B ring
wants **immutable snapshots** a renderer can still be drawing. Note what is
already true: `MarchingCubes::Slot` holds an `arena`, an `index_run` *and* an
`indirect` per slot, so both buffers are ringed today, and the ring is what
makes an extract's immediate `vmaDestroyBuffer` safe — a slot is only written,
grown or freed once `release_through` has covered it.

The direction under consideration splits them: keep the **index buffer** ringed
and rebuilt each extract, and make the **vertex arena** one persistent
allocation in which a re-meshed block *appends* a new span and its old span
retires once released. The attraction is memory — on the default kernel a
slot's arena is **16x** its index run (a 64-byte `Vertex` against a 4-byte
index, three of each per triangle), so ringing the run instead of the arena is
where nearly all of a ring's extra residency goes. Two things are unsettled,
and neither is a detail:

- Growing one persistent arena **frees the storage an in-flight draw is
  reading** — precisely the hazard `MarchingCubesConfig::slot_count > 1` exists
  to remove. A chunked arena that never frees while a generation is outstanding
  is the obvious answer and is not designed.
- `release_through` is a **whole-slot high-water mark**
  (`released_through_ = std::max(released_through_, generation)`). Retiring
  *ranges within* an arena needs a granularity it does not have.

So `LiveMesh` and gfx are untouched only in the sense that no API changes; until
both are answered the *guarantee* behind that API changes, and that is a cost to
the sibling rather than zero. The 16x is a ratio and not a measurement — no
arena/index pair for a named scene and slot count is on record — and it does not
hold under `share_vertices`, which the only seam-B consumer cannot set anyway,
since `fuse_viewer` runs the `texture` tier and that tier refuses sharing.

**What this does not establish, named rather than implied.** The ~4x is one
device, one walk, one scene — an M5 iPad Pro with unified memory, where the
host-visible arena the 2026-08-08 seam-B decision books as a discrete-GPU risk
is free. Nothing here re-measures that. Nor are the device figures recorded with
voxel size, integration mode or window length, against this repo's own rule that
a figure carries its configuration — so read them as a coverage sample and not
as a phase budget. Four windows are enough to establish that room0 is
unrepresentative and that the worst case is near 3x, and not enough to size a
cadence cap. And the instrument that produced the phase split is a *desktop*
one: `extract_device` is 2.1–2.7 ms against 20 ms for `extract`, i.e. **87–90%
of "the extract" was a readback no seam-B consumer pays**, which is why the
device figures above are quoted against the walk's `meshing` row rather than
against that 20 ms — and why that share and the 77% above are shares of
different denominators, 77% being `dispatch` within the 2.1–2.7 ms rather than
within the 20 ms. Related and worth not re-deriving: dispatch scales
**sublinearly** in cells (2.84x for 3.95x the blocks) with the emit rate flat at
~7.7% across a 4x resolution sweep — the kernel is healthy, it is simply handed
63 M cells — and steady-state `arena alloc` is **0.03 ms**, the earlier 4–17 ms
readings having been one-shot first-allocation noise that reversed two
conclusions before 200-extract means replaced them.

Provenance. The device numbers are instrumented by `volumetric_kit_ios` PR #14
(console read-out logging), where `meshing` is that scanner's own overlay row
and not an `ExtractTimings` phase this repo defines. On desktop the two are
separate instruments: `fuse_replica --dirty-every N` prints the changed and
dilated coverage, and `fuse_replica --device-extract` prints the phase split.

### 2026-08-09 — Timings are `core` vocabulary and the device half is measured, not inferred; counters stay in the tier that means them.

*Promotes* the 2026-08-01 decision's parked contract, on the trigger that
decision named. It held that the viewer example would own the instrumentation
and that a shared contract waits for a **second consumer**. There are now three
— `fuse_viewer`, the iOS scanner, and `fuse_replica --device-extract` — and the
cost of waiting had become visible rather than theoretical: `fuse_viewer` grew
`StageTimes`/`StageScope` in `examples/viewer/stage_metrics.hpp`, and the
scanner solved the same problem again and worse, as a 2048-byte `snprintf` into
a `UILabel`. Two implementations of one idea, neither reusable, and the platform
with the hardest problems got the weaker one.

**The split is timings uniform, counters local**, which is the same
mechanism-in-`core` / policy-in-the-tier line the 2026-07-06 kernel-bundle
decision drew. `vr::StageMetrics` carries `{name, cpu_ms, gpu_ms, has_gpu}`
rows, accumulating by name, with `kBreakdownPrefix` marking a row as a
decomposition of the one above it so a total does not count that stage twice
(not hypothetical: it reported every extract twice in the viewer before the skip
existed). `active_blocks`, `dispatches`, `triangle_capacity`, `AllocFailures`,
`load_factor` stay exactly where they are — a millisecond does not care which
tier produced it and those counters mean nothing outside theirs.

**It is recon's own type rather than the renderer's, and that is the fifth
application of one idiom.** `gfx::FrameMetrics::Section` is structurally
identical and nothing about it is renderer-specific — but recon cannot include
from a sibling it does not depend on, and a shared package for four fields was
already weighed and judged too thin. So each side declares in its own vocabulary
and the neutral app that knows both converts in a loop, exactly as
`DeviceRequirements` does at the device seam and
`MarchingCubesConfig::extra_vertex_usage` at the buffer seam.
`examples/viewer/stage_metrics.hpp` survives as *only* that mapping, which is
what stops the promotion leaving two copies behind.

**`vr::GpuTimer` is the half that was actually missing.** Every recon timing
before it was wall clock around `Device::submit_single_time`, which blocks on a
fence — so host record, submit, the stall and device execution collapsed into
one number, and "the kernel is slow" could not be told from "we are waiting".
That is not an abstract gap: the whole current optimisation roadmap —
incremental extraction, the vertex-layout narrowing, the `max_buckets`
decision — rests on numbers that cannot make the distinction, including a
`meshing` stage measured at 373 ms on an M5 iPad Pro.

**recon resolves more cheaply than a renderer can, and the blocking design is
why.** A render loop runs ahead of the GPU, so gfx's profiler buffers per
in-flight slot and publishes a snapshot that lags. Every recon dispatch is
already fence-blocked, so the timestamps are readable the instant the submit
returns: no ring, no deferred publish, no lag. The `submit_single_time(record,
GpuTimer*, label)` overload exists for exactly that reason rather than leaving a
caller to bracket the work — reading a timestamp before its submit completes
returns nothing useful and nothing at the call site makes that visible, so it is
a staleness the library sequences (the 2026-08-04 rule). It resolves **only** on
the path where the fence signalled; on the wait-failure path the submit may
still be pending, and the span stays unresolved rather than publishing a zero
that reads as a fast dispatch.

**Availability is not an error.** A queue family may report
`timestampValidBits == 0`, and the two-family bootstrap can hand recon a
compute-only family on a discrete GPU. `create` then succeeds with `available()`
false: `begin` returns `kNoSpan`, `end` does nothing, `report_into` contributes
no rows. A caller writes one code path and gets host timings. Failing instead
would let an optional diagnostic break a scan.

**Measured, and the number is the point.** A 128 MB device-local
`vkCmdFillBuffer` through the timed overload: **0.817 ms of device time inside a
14.256 ms blocking submit — 5.7%.** That is one cold measurement and the fixed
cost includes first-use residency, so it is not a claim about the steady state;
what it establishes is that the two quantities are nothing like each other,
which is the entire premise. The test asserts the inequality rather than a
magnitude — `gpu_ms < wall_ms` is what catches a span that is the wall clock
relabelled or resolved against the wrong period, and a fill sized so the work
cannot round to zero is what keeps `gpu_ms > 0` from passing vacuously. The
host half is pinned separately and runs on the GPU-less CI legs: accumulation,
content-matching (two pointers, equal strings, one row — literals are not
guaranteed to be pooled and across TUs routinely are not), the breakdown skip
(dropping it reports 30 where 20 is right), and the wrap case where masking the
endpoints before subtracting reports several thousand years.

**Review: the window needed an owner, and `report_into` became it.** The first
cut had `GpuTimer::spans_` as a host-side index into device query state with
nothing keeping the two in step, and five separate findings were that one gap
seen from different sides. `submit_single_time` sequenced begin/end/resolve but
never ended a *window*, so a timer created once — the shape the class's own
example shows — silently stopped timing after `max_spans` (measured: 40 submits
left `count() == 32`, every call returning OK) and every publish re-emitted the
whole window, which `StageMetrics` accumulates by name, so frame N reported
frames 1..N summed under this frame's label (measured: one row at the sum of 32
spans). The fix is that **publishing ends the window**: a caller creates one
timer, submits as often as it likes, and reports per frame, with `reset` left
for discarding instead of publishing. `max_spans` is then a per-frame bound
rather than a lifetime one, and exhausting it warns through the log handler
instead of going quiet. Verified as the realistic loop: 40 frames × 2 submits
through one timer now publish two measured rows every frame, with frame 39's
device total at 0.95× frame 0's rather than 40× it.

The same gap had two device-side edges. A span is recorded by commands *inside*
a command buffer, so a buffer abandoned before submission left its query pair
never reset and never written — undefined to read
(VUID-vkGetQueryPoolResults-None-09401), and worse once the pool had been used,
since the pair still holds an earlier window's value with its availability bit
set and resolves as a plausible duration under the abandoned label. Every early
return in `submit_single_time` now drops the span. And `resolve` no longer fails
the dispatch: `VR_TRY` on a query readback turned an optional profiler into
something that could fail compute work that had already completed, on the one
primitive every tier goes through — the exact outcome this entry's availability
rule exists to prevent. It logs and leaves the span unmeasured, which
`report_into` already skips.

Three more, each small and each a rule this repo had already written down. The
labels are now **borrowed on `StageRow::name`'s terms** rather than copied into
the timer: copies made the published rows point into a `std::string` the timer
would free, which the one consumer that exists reads from another thread, and
one lifetime rule across the whole vocabulary is simpler than two. `create`
bounds `max_spans` above as well as below, because `max_spans * 2` is the pool's
`uint32_t` query count and `0x80000000` built a **zero**-query pool that `begin`
believed had room for two billion spans. And the moves are hand-written per the
RAII rules — the defaulted pair left `device_` live in the source, so a
moved-from timer reported `valid() == true`, a failure that hides especially
well here because a hollow timer is byte-indistinguishable from the supported
`timestampValidBits == 0` degradation and reports itself as a missing GPU
capability. `tests/compute_raii_test.cpp` gained the three mandated move tests;
the first two fail on the pre-review code.

On the host side `StageMetrics` grew `merge`, because the viewer's remesh→fuse
fold was written as `add_cpu(row.name, row.cpu_ms)` and dropped `gpu_ms` and
`has_gpu` at the only place rows are combined — silently, since an absent
`has_gpu` is indistinguishable from a genuinely host-only stage. Carrying both
halves belongs on the vocabulary type rather than in each consumer.
`OptionalStageScope` folded back into `StageScope` as a second constructor,
which deletes ~35 duplicated lines and makes that class's null check
load-bearing instead of dead code beside a copy of itself.

Scope stopped at `core` initially, because the mesh tier was held by in-flight
incremental-extraction work.

**Extended the same day to `volume`, `tsdf` and `texture`**, and the plumbing
point is that it went through `dispatch()` rather than through each tier. Every
compute tier already routes there for the workgroup guard and the barrier, so a
device span costs a tier one argument and no submit path of its own; each holds
one `GpuTimer`, created in its `create()` because a query pool of a few
timestamps costs nothing and a diagnostic that can fail on first use is worse.
The host scope opens **before** the validity check, so a refused call still
costs its row — a stage silent on failure reads on an overlay as a stage that
did not run, which is the reading a frozen pipeline most needs not to give. In
`volume` the timer threads through the retry loop, so a contended frame's rounds
accumulate under one label: that is the honest total, and reporting only the
last would hide exactly the cost that makes contention worth seeing.

**That one argument is `GpuStageScope`, and it is the review's main fix.** The
first cut hand-copied a three-part idiom at four sites — open a `StageScope`,
pass `metrics != nullptr ? &gpu_timer_ : nullptr` to `dispatch`, then call
`report_into` as a plain statement afterwards — and the third part is skipped by
*every early return between them*. `volume`'s retry loop makes that concrete: a
round that fails returns through `VR_ASSIGN` with the successful rounds' spans
still in a timer that now outlives the call, so the next frame publishes a
failed frame's device time under its own label, and after `max_spans` such
failures `begin` refuses every span and that tier's GPU column freezes for the
map's lifetime. A destructor cannot be skipped by a return, so the scope owns
all three parts, and taking *it* rather than a `(GpuTimer*, const char*)` pair
also closes both halves of the argument mistake: a timer with no label published
an anonymous `"gpu"` row that `StageMetrics` merges across unrelated stages, and
a label with no timer read as instrumented while measuring nothing. Both
compiled. The same scope inoculates the deferred `mesh` sites.

**A stage that runs two kernels and times one misattributes the difference.**
`integrate`'s host row wraps the active-set compaction as well as the fusion
dispatch, so that kernel's device time fell into the gap between the halves and
read as submit overhead — the fourth instance of the mistake the *Measured
lessons* section below exists to record, and this one shipped in the entry's own
prose. `compact_active_blocks` now reports itself as a `kBreakdownPrefix`
sub-row, so the gap is overhead and nothing else. It is named `"  ..active set"`
rather than `"  ..compact"` because rows merge by content and `mesh` already
publishes a `"  ..compact"` phase for the same call inside its extract; two
stages' compactions summed into one row is the collision the prefix exists to
avoid. On the test fixture it is *half* of `integrate`'s host row (0.22 of
0.45 ms), so this was not a rounding-error correction.

**Availability is not an error — including when the pool will not allocate.**
Creating each tier's timer with `VR_ASSIGN` made a failed `vkCreateQueryPool`
refuse to construct `VoxelHashMap`, `TsdfIntegrator` and `ProjectiveTexturer`,
i.e. fail the whole reconstruction spine for a caller who never asked for
timing. `GpuTimer::create` now degrades that to `available() == false` plus a
logged warning, exactly as it already did for `timestampValidBits == 0`; the
class had been carrying two opposite policies. `GpuTimer::abandon` is the same
rule mid-run: when `submit_single_time`'s fence wait fails it deliberately leaks
the command buffer because the GPU may still execute it, and that buffer still
carries the span's `vkCmdResetQueryPool` and both timestamp writes — so the two
queries retire with it rather than being recycled into the next `begin`
(VUID-vkCmdResetQueryPool-None-02841). Long-lived per-tier timers are what made
that reachable; a throwaway timer died with the failed dispatch.

**The consumer was wired in the same change, because leaving it was a trap.**
`fuse_viewer` already wrapped `allocate`, `integrate` and `texture` in
`StageScope`s under exactly the literals the tiers now publish, and rows
accumulate by name — so passing the out-param without removing them would have
inflated `integrate` and `texture` ~2x and `allocate` up to ~6x (the tier's
scope fires once per retry round inside the outer one), silently, with
`shared_fuse_ms` inflating along with it. The tiers' rows replace the wrappers.
What a wrapper legitimately covered and the tier's row does not — the `resize`
between retries, by far an overflowing frame's largest cost — gets its own row
in both examples rather than being folded into `allocate`, where it would sink
that stage's device share on precisely the frames whose host cost is not the
kernel. Untimed, as it was in `fuse_replica`, the most expensive frames
contributed nothing at all to the table.

**The bug this shipped with for an hour is the one worth recording.** Each tier
gained a `GpuTimer` member and none of them *created* it, so every tier silently
reported host-only — a default-constructed timer is unavailable by design, which
is indistinguishable from the supported no-timestamps device. The first cut of
the test printed `timestamps unavailable` for all three rows and **exited 0**.
The fix is not just creating the timers: the test now asks the device
independently, by creating a `GpuTimer` of its own, and *fails* when a tier
reports no span on hardware that a probe says can time. A capability a test can
establish for itself must never be inferred from the thing under test.
Confirmed by mutation — deleting one tier's `GpuTimer::create` fails the suite
by name.

**And the test that pinned it was itself the review's flakiest finding.** Its
drain assertion compared two ~16 µs spans within a 1.8x band and failed 5 runs
in 60 on unmodified correct code — a ~1-in-12 false red on every CI leg — and
degenerated to `0.0 < 0.0` whenever a span resolved inside one timestamp tick,
which is a healthy device. Both window properties are now asserted on *counts*
instead: a window holds `max_spans` and then silently stops recording, so
running more calls than one window holds and requiring the last to still report
a device span catches "an untimed call recorded a span anyway" and "publishing
did not end the window" alike, with no timing ratio anywhere. Its companion,
`CHECK(untouched.empty())`, was asserting that a `std::vector` nothing was
passed to is empty — it held against exactly the global-sink tier it claimed to
be the only guard against. Both mutations now fail the suite by name, the second
via `gpu_ms >= cpu_ms` (32 accumulated spans against one call's wall clock is a
factor, not a coin flip); 80 consecutive runs of the replacement pass.

First numbers, **Release** (`-DCMAKE_BUILD_TYPE=Release`; at `-O0` these are
worthless — see the build-type gotcha in CLAUDE.md), on the deliberately tiny
fixture the test runs (64x64 depth, three triangles), so they are
fixed-overhead-dominated and not a workload claim:

| stage | host | device | device share |
|---|---|---|---|
| `allocate` | 0.368 ms | 0.144 ms | 39.0% |
| `integrate` | 0.455 ms | 0.018 ms | 3.9% |
| `  ..active set` | 0.224 ms | 0.009 ms | 4.1% |
| `texture` | 0.246 ms | 0.018 ms | 7.3% |

**Read the device shares knowing what the instrument costs.** A timed submit is
~0.13 ms more expensive than an untimed one on MoltenVK — measured on an M5 Max
at 0.020 ms for a barrier-only submit with no timestamps against 0.145 ms with
one and 0.153 ms with two, so the whole penalty is the *first*
`vkCmdWriteTimestamp` in a command buffer and it is per submit, not per query.
On a real workload that vanishes (a 300-frame room0 A/B shows no regression),
but on this fixture it is a third of `integrate`'s host row, so these
percentages are ratios the instrument moved and not only fixed overhead. Quote
them as what they are: a demonstration that the halves separate, not a
measurement of the pipeline.

What is *not* yet wired: `mesh`'s GPU column — `ExtractTimings` keeps its six
host phases and its counters, and giving each phase a device half means
threading the timer through several dispatch sites inside one extract. The
number above says it must not be done one timed submit per phase: this run's
real phases are compact 0.15, inputs 0.00, arena 0.00, descriptors 0.00,
dispatch 0.63, readback 3.73 ms, so four of six would gain more than they
measure. Bracket several dispatches inside **one** timed submit with
`GpuTimer::begin`/`end` plus a single `resolve()` — which `submit_single_time`'s
own doc already points at for exactly this — and it is `TODO(mesh)` in
`marching_cubes.cpp`. Debug-utils labels are also deferred (`TODO(core)` in
`gpu_timer.hpp`): recon enables `VK_EXT_debug_utils` only when validation is on,
so a label needs `Device` to record whether it was enabled — the same
declare/verify shape as the enabled extension list, and its own change.

### 2026-08-10 — A breakdown row is one the *caller's* row already contains, so the host total skips it and the device total must not; and a ceiling the library knows is the library's to name.

Refines the entry above, on three things the first cut of `kBreakdownPrefix` got
wrong once a sub-row carried a *device* half.

**The two halves do not nest the same way, so they cannot obey one skip.** A
host scope spans a whole call, including everything that call invokes — so a
sub-row's host time is already inside its parent's row and summing both
double-counts, which is the observation `kBreakdownPrefix` was built on. A
device span covers **one dispatch**. `integrate`'s span is its fusion kernel and
nothing else; the compaction it runs first is a separate submit that no row
above it contains. Skipping breakdown rows in `total_gpu_ms` therefore did not
deduplicate that time, it *deleted* it: the host half stayed counted through
`integrate` while the device half was counted nowhere at all, so the two totals
silently stopped describing the same work — in the one release where a
`..active set` row had just been added to make that kernel visible. `total_cpu_ms`
still skips; `total_gpu_ms` no longer does. The asymmetry is not an exception to
the rule, it *is* the rule: skip what a parent already contains, and a device
span never has a parent.

**Whether a row is a breakdown is a property of where it was called, not of what
it does.** `compact_active_blocks` hard-coded `"  ..active set"`, which is right
under `tsdf`'s open stage and wrong everywhere else: a caller compacting at top
level got a row `total_cpu_ms` skips — their only stage, missing from the total
they read it from, while still sitting in `rows()` looking accounted for. Naming
it plainly instead would just move the double-count back. So `StageMetrics`
answers `in_stage()` — maintained by `StageScope` itself, not threaded down by
hand — and the callee picks between two literals. This is the 2026-08-04 rule
applied to the one thing left that the caller could not see: it is the library
that knows whether a scope is open, so it is the library that checks rather than
documents. The same reasoning gave the frustum compaction the reporting
parameter its sibling had; it is the same round trip over a smaller set, and a
caller who switches to it to make that trip cheaper has to be able to read what
that bought rather than watch the row disappear.

**A tuning constant belongs beside the reading it qualifies.** `load_factor`'s
own header tells callers linear probing degrades sharply past ~0.7 and to grow
well under 1.0. The viewer's new gauge warned at 0.85, twice — once for the bar
colour and once for the text, two literals three lines apart that could drift —
and `volumetric_kit_ios` carried a third copy. So the number a reader saw
disagreed with the number the library documented, and the band between them is
precisely where every insert is slowest: an M5 iPad Pro lost the GPU inside
`allocate_from_depth` at 96% (see the 2026-08-08 overflow-scan entry), after
sitting healthy-looking well past 0.7 for half a scan. `VoxelHashMap::kGrowThreshold`
now names it once, at the library's own number, and the gauge reads it for both
the colour and the words. A UI drawing its own ceiling is a second source of
truth for a property of the table.

**And a gauge cannot answer an unknown with the last good value.** The viewer's
`load_factor()` read was `if (result) { store }` — on failure the bar kept
rendering the previous frame's fraction as current, which is the calm-green-over-
a-stopped-scan reading the gauge exists to prevent, arrived at by a different
route. It now reports on stderr like every other call in that loop and publishes
a negative the panel draws as `unavailable` rather than as a low number. The
read also moved out of the `share_mtx` critical section, beside the
`memory_stats()` call that was hoisted out for the same reason.

### 2026-08-11 — The per-block span table is opt-in, is retired by generation rather than described in prose, and is deliberately *not* per slot.

Stage 2 left both sparse kernels computing a block-to-range mapping and throwing
it away. Publishing it as `MarchingCubes::block_spans()` is what stage 3
(dirty-only dispatch) re-meshes against, and it is not derivable on the host: the
reservation atomics hand ranges out in workgroup **arrival** order, not block
order, so nothing outside the dispatch knows which range belongs to which block.
Four decisions came out of reviewing the first cut, and the last three are all
the same mistake — a fact only the library can know, written down for the caller
instead of checked.

**Opt-in, because it is sized by the grid rather than by the surface.**
`num_blocks * 16` is 24 MB at `VoxelGridParams::defaults()` and doubles with
every `VoxelHashMap::resize`, held for the extractor's lifetime — paid by
`fuse_viewer`, `fuse_replica`, the iOS scanner and the whole test suite, for a
table whose only reader today is a test. That is exactly the bargain
`TsdfIntegratorConfig::track_dirty_blocks` already struck for the same slot-keyed
table shape at a quarter the width, and the bar it set is verbatim: *nothing
measured for a caller who did not ask*. `MarchingCubesConfig::track_block_spans`
gates it; a 1-element dummy keeps the descriptor valid and a `write_spans` push
flag keeps the kernel off it, which is the mechanism `color_dummy_` already uses
one binding over. The table is also counted in `ExtractTimings::arena_bytes`,
which it was not: that instrument is how the ring's runaway growth was diagnosed,
and omitting a component of what stays resident is the same defect that folding
the index runs into it fixed.

**Retired by generation, because a borrowed pointer's staleness is not
documentable.** The accessor hands out mapped device memory, and a grow
move-assigns the buffer — `Buffer::operator=` destroys the current state first,
so the next extract over a grown grid frees the pages a cached pointer names.
That is the growing-scan path the feature exists to serve. `block_spans_generation()`
carries the same counter `DeviceMesh::generation` does, so the two are directly
comparable, and the accessor returns `nullptr` until an extract has actually left
a table describing its own output. This matters most on the failure paths: both
kernels publish a block's span **before** their capacity guard, deliberately —
the counters must carry the block's full total for the host's refit — so after a
failed extract the table names triangles at bases the arena never held.
`disarm_indirect_command()` stopped those being *drawn*; nothing stopped them
being *read*. One comparison does.

**Not per slot, and that is the ring collision the 2026-08-09 entry recorded as
unresolved — resolved in the conservative direction.** The arena, index run and
draw command are per `slot_count`; this table is one instance describing the
current dispatch. Making it per-slot would multiply the 24 MB by the ring depth
to serve a consumer that does not exist yet. So the table stays single and the
generation stamp makes the mismatch *visible* instead of silent: a consumer
holding generation N whose `block_spans_generation()` has moved to N+1 knows the
spans describe another slot's arena. When stage 3 needs to read a held mesh's
spans, that is when the per-slot cost is worth paying, and the check is already
the thing that would have to change.

**The grow carries the old spans forward and zeroes only the new tail.**
`VoxelHashMap::resize` preserves every block's index — which is precisely why
`VoxelBlockGrid::topology_epoch()` deliberately does not move across one — so a
slot means the same block on both sides and replacing the table wholesale would
discard every span on the one event the volume tier guarantees they survive.
`TsdfIntegrator::prepare_dirty_flags` grows its flags exactly this way, for
exactly this reason. Zeroing the tail is the other half: only blocks in an
extract's active set are written, which on a first extract is a few thousand of
1.5M entries, and the rest were being published as readable while holding
whatever VMA handed over.

**And the slot itself is anchored, not just the table.** The generation above
answers "which extract does this table describe"; it cannot answer "does this
slot still name the block its span was written for". A slot is meaningful only
against one grid and one topology epoch — the block heap is LIFO, so after a
`remove()` a reused slot names a *different* block and its span reads as that
block's geometry under `Status::ok`. `block_span_valid(grid, slot)` records both
and checks both, the same shape `TsdfIntegrator::prepare_dirty_flags` uses for
the sibling table. It takes the **grid** rather than trusting the caller to
re-extract after a topology change: between a `remove()` and the next extract the
per-slot stamps are still set, so a query that re-checked nothing would call a
stale span live. A `resize()` deliberately does not break it, for the same reason
the buffer carries its spans forward. The stamps are per slot so that "meshed and
emitted nothing" stays distinct from "no extract has touched this since the
anchor" — the latter being what a newly allocated block reads — and they are
written from the active set the *host* dispatched rather than read back from the
device, which only knows where geometry went, not which blocks it was asked
about. Three things the test caught that the assertion was not written for:
`VoxelHashMap::remove` reached through `map()` frees the index *without* bumping
`topology_epoch` (its own header says so), so the test goes through the grid's
`remove`; the fixture had to build its own grid and extractor and run last,
because removing whichever block compaction happened to put last changes nothing
when that block carries no surface, and a test that fails on some runs is worse
than one that fails on all of them; and which slot is unmeshed is not guessable,
so the assertion counts instead — exactly the active blocks are valid, no more
and no fewer.

**And the host/device mirror is pinned per field.** `BlockSpan` is four
same-typed `uint32`s, so every permutation is 16 bytes and `sizeof` alone cannot
see a transposition — while the GLSL side was an anonymous `uvec4` both kernels
filled positionally, with `s_ibase` (named for the *index* base) landing in
`triangle_base` and correct only because the atomic result had been divided by
`kIndicesPerTriangle`. The struct now has per-field `offsetof` asserts like every
other mirror in the repo, is declared once in
`shaders/marching_cubes_block_span.glsl` rather than copied into two kernels that
could disagree on field order and both compile, and both kernels assign it **by
name** — the same fix `SparsePushConstants` already applies to its own adjacent
same-typed scalars.

## Measured lessons

Not decisions, but the measurements that overturned an assumption about
where this pipeline spends its time. Kept because the mistake generalises.

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
that was already fast. (*On device that ordering inverts: see the 2026-08-09
incremental-extraction decision, where meshing dominates the frame and the pool
is back on. The lesson below is about the method, not about the pool.*) The cost
was `make_output_buffers` allocating a fresh
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
`ExtractTimings::arena_bytes` sums — arenas *and* index runs since the
2026-08-08 sharing decision, because sharing removes the proportionality that
let the run be dismissed as a sixteenth of the arena it covers. Measured on the
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
