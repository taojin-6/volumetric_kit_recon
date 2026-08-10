# volumetric_kit_recon

The production-grade **reconstruction + compression** compute backend of the
`volumetric_kit` family. It turns posed depth/RGB-D frames into a sparse TSDF
volume, extracts geometry, compresses it, and hands the result to the renderer
— all as a **Vulkan compute** workload, cross-platform, mirroring the renderer.

This file is the **living source of truth** for *how* this repo is built — its
conventions, its architecture, and every locked decision as a one-line rule. The
*why* behind each rule — the rationale, the measurements, the review findings —
lives beside it in [DECISIONS.md](DECISIONS.md). Decisions live in these two
files, not in chat history or commit messages; update both in the same change
that alters one.

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

Each dated; newest context wins. The rule lives here, one line each; the full
rationale, the measurements, the review findings that shaped it and what each
fix was verified against live in [DECISIONS.md](DECISIONS.md), in this same
order. Change the decision, its entry there, and this list together.

- [**2026-06-21**](DECISIONS.md#2026-06-21--single-vulkan-path-moltenvk-on-apple-like-gfx) —
  Single Vulkan path (MoltenVK on Apple), like gfx.
- [**2026-06-21**](DECISIONS.md#2026-06-21--trivial-interop-same-vulkan-api) —
  Trivial interop (same Vulkan API).
- [**2026-06-21**](DECISIONS.md#2026-06-21--independent-siblings-gfx-untouched) —
  Independent siblings; gfx untouched.
- [**2026-06-21**](DECISIONS.md#2026-06-21--vertical-slice-first) —
  Vertical slice first.
- [**2026-06-21**](DECISIONS.md#2026-06-21--codec-ships-dct-only) —
  Codec ships DCT-only.
- [**2026-07-04**](DECISIONS.md#2026-07-04--native-cuda-accelerator-under-the-vulkan-baseline) —
  Native CUDA accelerator, under the Vulkan baseline.
- [**2026-07-04**](DECISIONS.md#2026-07-04--glm-for-hostdevice-math-dropped-the-hand-rolled-pod-types) —
  GLM for host/device math (dropped the hand-rolled POD types).
- [**2026-07-04**](DECISIONS.md#2026-07-04--zero-copy-interop--one-shared-vkdevice--a-createadopt-seam-refines-trivial-interop-above) —
  Zero-copy interop = one shared `VkDevice` + a create/adopt seam (refines
  "Trivial interop" above).
- [**2026-07-05**](DECISIONS.md#2026-07-05--shader-buffer-abi-is-scalar-block-layout-not-std430) —
  Shader buffer ABI is scalar block layout, not `std430`.
- [**2026-07-05**](DECISIONS.md#2026-07-05--compute-core-is-explicit-not-reflected-dispatch-via-submit_single_time) —
  Compute core is explicit, not reflected; dispatch via `submit_single_time`.
- [**2026-07-05**](DECISIONS.md#2026-07-05--per-voxel-storage-is-a-structure-of-arrays-attribute-store-voxelblockgrid-not-the-prior-engines-aos-voxel-in-the-hashmap) —
  Per-voxel storage is a structure-of-arrays attribute store
  (`VoxelBlockGrid`), not the prior engine's AoS `Voxel`-in-the-hashmap.
- [**2026-07-06**](DECISIONS.md#2026-07-06--per-kernel-resources-are-bundled-corecompute_kernelhpp-still-not-reflected) —
  Per-kernel resources are bundled (`core/compute_kernel.hpp`), still not
  reflected.
- [**2026-07-06**](DECISIONS.md#2026-07-06--hybrid-color-renders-through-a-gfx-pipeline-amends-interop-seam-a-needs-zero-gfx-changes) —
  Hybrid color renders through a gfx pipeline (amends "interop seam A needs
  zero gfx changes").
- [**2026-07-06**](DECISIONS.md#2026-07-06--depth-sampling-is-texture-centred-pixel-centres-at-i05-a-deliberate--pixel-convention) —
  Depth sampling is texture-centred (pixel centres at i+0.5), a deliberate
  ~½-pixel convention.
- [**2026-07-07**](DECISIONS.md#2026-07-07--the-viewer-example-opts-into-gfx-behind-vr_build_viewer-amends-independent-siblings-gfx-untouched) —
  The viewer example opts into gfx behind `VR_BUILD_VIEWER` (amends
  "Independent siblings; gfx untouched").
- [**2026-07-07**](DECISIONS.md#2026-07-07--projective-texturing-is-a-new-texture-tier-live-single-camera-first) —
  Projective texturing is a new `texture` tier; live single-camera first.
- [**2026-08-01**](DECISIONS.md#2026-08-01--perf-instrumentation-starts-in-the-viewer-example-not-a-shared-contract-package) —
  Perf instrumentation starts in the viewer example, not a shared contract
  package.
- [**2026-08-01**](DECISIONS.md#2026-08-01--ios-is-a-downstream-concern-recon-cross-compiles-to-it-unchanged) —
  iOS is a downstream concern; recon cross-compiles to it unchanged.
- [**2026-08-02**](DECISIONS.md#2026-08-02--meshvertex-is-the-renderers-vertex-layout) —
  `mesh::Vertex` *is* the renderer's vertex layout.
- [**2026-08-02**](DECISIONS.md#2026-08-02--the-neutral-shared-vkdevice-bootstrap-lands-in-the-viewer-example-and-prefers-two-families-over-a-shared-queue) —
  The neutral shared-`VkDevice` bootstrap lands in the viewer example, and
  prefers two families over a shared queue.
- [**2026-08-02**](DECISIONS.md#2026-08-02--the-sensor-tier-is-a-contract-not-a-driver-collection-a-capture-driver-lives-here-only-if-this-repo-can-build) —
  The `sensor` tier is a *contract*, not a driver collection: a capture driver
  lives here only if this repo can build
- [**2026-08-02**](DECISIONS.md#2026-08-02--the-mesh-arenas-extra-buffer-usage-is-declared-by-the-consumer-not-named-by-this-tier--and-usage-alone-does-not-reach-seam-b) —
  The mesh arena's extra buffer usage is declared by the *consumer*, not named
  by this tier — and usage alone does not reach seam B.
- [**2026-08-02**](DECISIONS.md#2026-08-02--one-color-space-rule-and-a-named-working-space-8-bit-color-is-encoded-float-color-is-linear-converted-once-at-the-sensor-boundary-and-encoded-once-at-presentation) —
  One color-space rule, and a *named* working space: 8-bit color is encoded,
  float color is linear, converted once at the sensor boundary and encoded once
  at presentation.
- [**2026-08-03**](DECISIONS.md#2026-08-03--a-buffer-names-the-families-that-will-read-it-and-the-dispatch-barrier-widens-only-as-far-as-its-queue-family-may) —
  A buffer names the *families* that will read it, and the dispatch barrier
  widens only as far as its queue family may.
- [**2026-08-03**](DECISIONS.md#2026-08-03--the-mesh-arena-is-a-ring-of-slots-released-by-the-host-not-a-timeline-semaphore) —
  The mesh arena is a ring of slots released by the *host*, not a timeline
  semaphore.
- [**2026-08-03**](DECISIONS.md#2026-08-03--the-draw-command-is-written-by-the-kernel-that-counts-it-and-the-counters-unit-changes-from-triangles-to-indices) —
  The draw command is written by the kernel that counts it, and the counter's
  unit changes from triangles to indices.
- [**2026-08-03**](DECISIONS.md#2026-08-03--nothing-in-the-mesh-extractor-reads-the-current-slot-except-the-code-that-writes-it-the-capacity-plan-is-slot-independent-and-a-slot-is-marked-outstanding-only-where-a-devicemesh-is-handed-out) —
  Nothing in the mesh extractor reads "the current slot" except the code that
  writes it: the capacity plan is slot-independent, and a slot is marked
  outstanding only where a `DeviceMesh` is handed out.
- [**2026-08-04**](DECISIONS.md#2026-08-04--a-limit-a-lifetime-or-a-staleness-the-caller-cannot-see-is-the-librarys-to-check-not-to-document) —
  A limit, a lifetime, or a staleness the caller cannot see is the library's to
  check, not to document.
- [**2026-08-08**](DECISIONS.md#2026-08-08--the-sparse-mesh-kernel-resolves-its-own-222-neighbourhood-by-probing-the-hash-table-on-device-and-mesh-therefore-reads-a-volume-buffer--a-coupling-paid-for-not-stumbled-into) —
  The sparse mesh kernel resolves its own 2×2×2 neighbourhood by probing the
  hash table on-device, and `mesh` therefore reads a `volume` buffer — a
  coupling paid for, not stumbled into.
- [**2026-08-08**](DECISIONS.md#2026-08-08--the-overflow-scan-stays-exhaustive-what-gets-bounded-is-its-cost-per-slot-not-its-length) —
  The overflow scan stays exhaustive; what gets bounded is its cost per slot,
  not its length.
- [**2026-08-08**](DECISIONS.md#2026-08-08--fuse_viewer-draws-recons-buffers-interop-seam-b-end-to-end-and-the-release-mark-must-be-published-before-the-mesh-is-taken) —
  `fuse_viewer` draws recon's buffers: interop seam B, end to end, and the
  release mark must be published *before* the mesh is taken.
- [**2026-08-08**](DECISIONS.md#2026-08-08--in-block-vertex-sharing-is-a-second-compiled-kernel-not-a-branch-and-the-two-emitters-must-interpolate-an-edge-in-the-same-direction) —
  In-block vertex sharing is a *second compiled kernel*, not a branch, and the
  two emitters must interpolate an edge in the same direction.
- [**2026-08-09**](DECISIONS.md#2026-08-09--a-dirty-block-is-one-the-fuse-changed-the-flags-are-anchored-to-a-grid-the-library-checks-and-tracking-them-is-opt-in) —
  A dirty block is one the fuse *changed*, the flags are anchored to a grid the
  library checks, and tracking them is opt-in.
- [**2026-08-09**](DECISIONS.md#2026-08-09--incremental-mesh-extraction-is-worth-building-at-a-4x-ceiling-not-the-18x-one-window-suggested-and-the-worst-frame-rather-than-the-median-sizes-its-design-amends-the-dirty-block-decision-above) —
  Incremental mesh extraction is worth building at a ~4x ceiling, not the ~18x
  one window suggested, and the worst frame rather than the median sizes its
  design (amends the dirty-block decision above).

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

Landed and proven on MoltenVK, left to right: `core` → `volume` → `tsdf` →
`mesh` → `texture`, plus `sensor` (the contract only, no drivers) and **interop
seam B** — gfx drawing recon's own buffers with no host round trip.
[DECISIONS.md](DECISIONS.md) carries the *why* behind anything here that looks
arbitrary; it usually isn't.

- **`core`** — the Vulkan compute foundation: VMA `Allocator`, RAII `Buffer`,
  `ShaderModule`, descriptor + `ComputePipeline` wrappers, the `ComputeKernel`
  bundle + `KernelSetBuilder`, the shared-queue-safe
  `Device::submit_single_time` dispatch, and the shared `dispatch()` /
  `group_count` / `storage_buffer` / range-guard helpers of `compute_util.hpp`.
  Vocabulary: `Status`/`Result`, the GLM aliases, `camera_params.hpp`,
  `color_space.hpp`. `Device::create` enables `scalarBlockLayout`; `adopt`
  requires the creator did, and both record the queue family's `queueFlags`.

- **`volume`** — `VoxelHashMap` drives init / allocate-from-coords, -depth,
  -points / remove / compact / compact-in-frustum / resize as GLSL kernels
  (`volume/shaders/hash_*.comp`) over the scalar-block-layout ABI. Depth
  allocation unprojects a posed frame and dilates each surface block into the
  `(2·tb+1)³` truncation band — a solid cube, not a ray march; frustum-culled
  compaction gives the per-frame working set. `resize` preserves block indices,
  so per-voxel data survives a grow. `VoxelBlockGrid` composes the map with
  independently-allocated SoA attribute arrays (`tsdf`, `weight`, `color`, …),
  each `num_blocks·voxels_per_block`, so a consumer materialises only what it
  needs. Host `diagnostics()` scans occupancy; `load_factor()` is the
  constant-time read a per-frame caller can actually afford.

- **`tsdf`** — `TsdfIntegrator` fuses a posed depth frame into a grid's
  `tsdf`/`weight`: projective `sdf = depth − Zc`, `±trunc_dist`, an
  inverse-square-with-behind-dropoff weight, running average capped at
  `max_weight`. Depth is sampled bilinearly, nearest at image edges and across
  discontinuities. `IntegrationMode` selects **classic** (keep free space ahead
  of the surface) or **dynamic** (clear it, so a receded surface leaves no
  ghost). An optional `ColorFrame` fuses colour through its own separate
  `ColorCameraParams`; a voxel's first colour observation assigns rather than
  blends. Opt-in `track_dirty_blocks` reports which blocks a fuse *changed*.

- **`mesh`** — `MarchingCubes` over a dense grid or, the real path, a sparse
  `VoxelBlockGrid`: one workgroup per active block, with the cross-block 2×2×2
  neighbourhood resolved by probing the hash table on-device. A block counts
  its triangles, reserves one span for all of them with a single atomic, and
  only then writes, so **a block's triangles are contiguous in the arena** —
  the precondition for meshing only the blocks a fuse changed, taken at ~10% on
  the dispatch, and true of the default kernel alone (`share_vertices` still
  appends per triangle). The vertex arena
  is fitted to the surface, grow-only, and held as a **ring of slots** the
  consumer releases by generation; the kernel writes a real
  `VkDrawIndexedIndirectCommand`. `extract_device` returns a borrowed
  `DeviceMesh` (valid until the next extract, enforced by a generation stamp),
  `download` takes the single host copy. `share_vertices` selects a second
  compiled kernel that indexes in-block vertices — 3.4x fewer on room0, and
  refused by the `texture` tier, which decides visibility per triangle.

- **`texture`** — `ProjectiveTexturer` rewrites every `Vertex::uv0` against one
  posed frame: a triangle is kept only where all three vertices are in front,
  in frame, and **unoccluded** (projected depth agrees with the depth map),
  else the `(-1,-1)` sentinel takes gfx's per-vertex-colour path. Live single
  camera, so the frame the caller binds *is* the atlas.

- **`sensor`** — the capture *contract*: `ICameraCapture` polled for a
  `CapturedFrame` (frames dropped, not queued), plus the boundary math that is
  silently wrong when guessed — `cv_from_gl_camera`,
  `depth_from_registered_color`, `to_canonical`. Links `recon_core` alone;
  drivers live with the platform that can build *and* test them.

**Examples** (`examples/`). `fuse_replica` runs the spine on a posed
Replica-SLAM RGB-D sequence and writes a PLY. Behind the off-by-default
`VR_BUILD_VIEWER`: `fuse_render` writes a headless colour PNG (seam A — it
builds two devices by design), and `fuse_viewer` opens a live window on one
shared `VkDevice`, fusing on a background thread, drawing recon's buffers
directly, and carrying the two-panel perf overlay. All three take `--preload`,
which makes the loop measure compute rather than the JPEG/PNG decoder.

**Next.** **Incremental mesh extraction**, decided and staged — read the
2026-08-09 entry before starting it: stage 2 costs a measured ~10% and its
collision with the seam-B ring is recorded there as unresolved. Beside it:
first-class glTF/GLB export via tinygltf + the gfx-vertex converter (the
example's tinyply dump is deliberately a throwaway). On `mesh`, the greppable
`TODO(mesh)`s: cross-block vertex sharing, per-vertex normals, the incremental
block-mesh pool (the staged work above), and fitting the *dense* extract to its
surface as the sparse one does. On `texture`: the multi-keyframe post-scan
atlas.

**Measure the phases before choosing the optimisation.** Three independent
guesses at this pipeline's bottleneck have been wrong, each corrected by an
`ExtractTimings` breakdown that pointed somewhere else entirely — see
[DECISIONS.md](DECISIONS.md#measured-lessons).
