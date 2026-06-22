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
  compute-pipeline + descriptor-set wrappers, sync (fences, timeline
  semaphores), the `Status`/`Result` idiom, a pluggable log handler, and
  portable POD math. Vulkan is reached through one umbrella header
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
  decision. Native Metal/CUDA acceleration is a possible future optimization, not
  the baseline.
- **2026-06-21 — Trivial interop (same Vulkan API).** Because the renderer is
  also Vulkan, recon and gfx share a `VkDevice` (or use UUID-matched compatible
  devices) and pass `VkBuffer`/`VkImage` directly. The cross-API external-memory
  machinery (CUDA↔Vulkan UUID import, Metal-objects, the MoltenVK shared-event
  export) is **not needed** for the recon→gfx path. This is the main reason for
  the Vulkan choice.
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

## Provenance & salvage policy

The algorithms here are a clean re-implementation of the proven core of the
prior reconstruction engine (on-disk directory `implicit_world_reconstruction`).
Its *implementation* is good and is what we port; what we leave behind is its
name and its prototype-grade packaging, not its numerics. The older research
codec (directory `implicit_surface_compression`) is **reference-only**. Both
source repos are left untouched on disk — never build or write in them.

- **Port (with refactor) from the prior engine:** `core/{math,types}`,
  `voxel_hashing`, `tsdf`, `mesh_extraction`, `mesh_io`. Its Metal/CUDA kernels
  are **re-implemented as Vulkan compute shaders (GLSL → SPIR-V)** — the
  algorithms transfer, the kernel source does not.
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
  carry a per-triangle atlas as a `Material`+texture. A `PointCloud` handoff is
  the lighter first milestone.
- **B — Shared Vulkan resources (zero-copy).** Because both sides are Vulkan,
  recon writes into a `VkBuffer`/`VkImage` on a device shared with (or
  UUID-matched to) gfx, and gfx renders it directly — the ordinary
  same-device/same-API case: a queue-family ownership transfer plus a
  timeline-semaphore handoff, **no external-memory import**. Sharing one
  `VkInstance`/`VkDevice` between the two libraries is the cleanest form and the
  target design.

## Key gotchas (verified)

- **MoltenVK is the Vulkan driver on Apple** — there is no other. Validate
  MoltenVK *compute* on the target Apple GPU early (prove the path before
  building on it, the gfx playbook). Metal supports compute; MoltenVK translates
  Vulkan compute → Metal compute.
- **Vulkan via the link-time loader through one umbrella header**
  (`core/vulkan.hpp`), exactly as gfx — never `#include <vulkan/...>` directly,
  so adopting volk later for the iOS/Android loader stays a one-header change.
- **Host buffer layout must match the shader.** Host POD structs (`HashEntry`,
  `Voxel`, …) and their GLSL `std430` mirrors must agree byte-for-byte. The
  `static_assert`s on the C++ side guard half of that; keep the GLSL definitions
  in lockstep.
- **GLSL compute, not CUDA/Metal kernels.** Warp/wave tricks become Vulkan
  subgroup ops; device atomics use GLSL atomics. The prior engine's kernels are a
  reference for the *algorithm*, rewritten in GLSL.

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
- Mark deferred work inline with a greppable `TODO:` comment.
- Prefer plain, behavior-level tests over friend-class backdoors.
- Full Doxygen on public classes/functions, matching
  `include/volumetric_kit/recon/core/result.hpp`. Don't write "move-only" in
  prose — the deleted-copy/defaulted-move declarations convey it.

## Where to start

Current state: scaffolding + the `core` foundation (`Status`/`Result`, logging,
contract checks, version, POD math) + the `volume` POD data model
(`volume/hash_types.hpp`). Next: stand up the **Vulkan core** (deps: Vulkan +
VMA + glslc/shaderc; instance → device with a compute queue → VMA allocator →
compute pipeline) and prove it with a **Vulkan compute smoke** dispatched through
MoltenVK on Apple, then port the `volume` hash ops as compute shaders.
