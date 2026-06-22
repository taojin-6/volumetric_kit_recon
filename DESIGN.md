# Design

This document explains the architecture of `volumetric_kit_recon` and the
reasoning behind it. For the terse, authoritative list of decisions and
conventions, see [CLAUDE.md](CLAUDE.md).

## Purpose and scope

`volumetric_kit_recon` is the compute backend that builds and compresses
volumetric geometry. Its pipeline is: **posed depth/RGB-D frames → sparse TSDF
volume (voxel-hashed) → extracted geometry (mesh/points) → optional compressed
bitstream → handoff to a renderer.**

It is deliberately *not* a renderer. Rendering is owned by the sibling library
`volumetric_kit_gfx` (Vulkan/MoltenVK). Keeping them separate means each is
independently useful: a headless capture/compression service links only the
reconstruction compute, never the renderer, and a viewer links only the
renderer. They meet only at the interop seam.

## Tiered architecture

The library is a stack of tiers with a strict rule: **a tier may depend only on
tiers to its left.** This keeps dependencies acyclic, makes each tier separately
testable and consumable, and lets downstream projects link only what they need.

```
core → volume → tsdf → mesh → interop
                       (later: compress, sensor, track, codec, stream)
```

- **core** — the Vulkan foundation, mirroring the renderer's core: instance,
  device (with a compute queue), a VMA allocator, RAII buffer/image, compute
  pipelines, descriptor sets, and sync primitives, alongside `Status`/`Result`
  error handling, a pluggable log handler, and portable POD math. Vulkan is
  reached through one umbrella header (`core/vulkan.hpp`).
- **volume** — the central data model: a sparse **voxel hash map**. Block-hashed
  storage keeps memory proportional to the observed surface, not the bounding
  volume. Carries SDF + weight per voxel, with optional color, and supports
  on-device rehash/grow.
- **tsdf** — truncated-signed-distance-field integration over the hash map, as an
  `ITSDFIntegrator` strategy with classic and dynamic (moving-object) variants.
- **mesh** — marching-cubes extraction with an incremental block-mesh pool, host
  mesh containers, and OBJ/PLY + glTF/GLB export.
- **interop** — the handoff to `volumetric_kit_gfx` (below).

## Backend strategy: one Vulkan path

Compute runs as **Vulkan compute shaders** (GLSL → SPIR-V), with **MoltenVK** on
Apple — exactly the cross-platform strategy `volumetric_kit_gfx` uses for
rendering. One code path serves Linux, Android, macOS, iOS, and Windows, rather
than a Metal + CUDA split. Vulkan is reached through a single umbrella header so
the loader choice (link-time loader now, volk later for iOS/Android) stays a
detail. Because the renderer is also Vulkan, the two libraries can share a
`VkDevice` — which is what makes the geometry handoff cheap (see the interop
seam below).

## Error handling

No exceptions cross the API boundary (mobile builds use `-fno-exceptions`).
Fallible calls return `Status` (success or an error domain + message) or
`Result<T>` (a value or a `Status`). `VR_TRY` and `VR_ASSIGN` remove the
check-and-propagate boilerplate. Programmer errors (precondition violations) fail
fast via `VR_CHECK` (log + abort), distinct from recoverable runtime failures.
`Status` is intentionally backend-neutral — a generic `int64_t` detail code, not
a Vulkan or CUDA type — so the same idiom serves every tier.

## The interop seam

Two complementary contracts let reconstruction feed the renderer:

- **A — Data handoff (available first).** The `mesh` tier produces geometry in the
  exact shape the renderer ingests (interleaved vertices + 32-bit indices), or
  serializes glTF/GLB. This needs no changes on the renderer side. A converter
  reconciles the in-memory layouts (single interleaved stream, synthesized
  tangents, vec4 color, atlas carried as a material texture).
- **B — Shared Vulkan resources (later).** Because both sides are Vulkan,
  reconstruction writes into a `VkBuffer`/`VkImage` on a device shared with (or
  UUID-matched to) the renderer, which renders it directly — the ordinary
  same-device, same-API case (a queue-family ownership transfer plus a
  timeline-semaphore handoff). No external-memory import and no per-platform
  translation; sharing one `VkInstance`/`VkDevice` is the cleanest form.

## Packaging

The repo installs and exports like its sibling: per-tier targets under a shared
export set, a generated `volumetric_kit_reconConfig.cmake`, and consumption via
both `find_package(volumetric_kit_recon)` and `FetchContent`. Quality gates
(warnings-as-errors, formatting hooks, sanitizer and test CI) are wired from the
start, not retrofitted.

## What this repo excludes

Production-only is a hard rule. Experimental and learned/neural components are
excluded entirely and remain in the upstream research repos. The codec, when it
lands, is DCT-based and deterministic.

## Roadmap

- **v1** — `core` → `volume` → `tsdf` → `mesh` → `interop` (Vulkan compute,
  MoltenVK on Apple); data-path handoff; an end-to-end example rendering through
  `volumetric_kit_gfx`.
- **v1.1** — shared-Vulkan-resource (zero-copy) handoff to the renderer.
- **v2** — broaden platform coverage (Linux / Android / Windows) on the same
  Vulkan path; performance passes.
- **v2.x** — `compress` (DCT) → `sensor` → `track` → `codec`/`stream`.
