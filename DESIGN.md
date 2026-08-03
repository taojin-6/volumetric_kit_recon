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
- **tsdf** — truncated-signed-distance-field integration over the hash map, with
  classic and dynamic (moving-object) modes selected per call by an
  `IntegrationMode` enum on one concrete `TsdfIntegrator` (a lean runtime flag,
  not a strategy-class hierarchy).
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

The reconstruction backend and the renderer meet at one seam, and the design
goal is that each runs **standalone** *and* the two run **as one** with the live
mesh handed over zero-copy. Two contracts serve that, in increasing order of
coupling.

- **A — Data handoff (available first).** The `mesh` tier produces geometry in the
  exact shape the renderer ingests — an interleaved 64-byte vertex (`position`,
  `normal`, `tangent` with a handedness `w`, `uv0`, `color`) plus 32-bit indices
  — or serializes glTF/GLB, which the renderer's loader already reads. This needs
  no renderer changes. The vector types already match — both sides use GLM — so
  the converter reconciles only packing and channels: one interleaved
  stream, synthesized tangents (the renderer does not generate them),
  `int32`→`uint32` indices, `color` widened to vec4, and a per-triangle atlas
  *baked* into per-vertex `uv0` + a per-mesh material texture (the renderer has no
  per-triangle UV concept). This is the file/host path — enough for a static
  capture-and-view workflow.

- **B — Shared Vulkan resources (the live target).** For realtime reconstruction
  the mesh is regenerated on the GPU every integration step; copying it to the
  host each frame is the thing to avoid. Because both libraries are Vulkan, the
  reconstruction compute writes the mesh (and any atlas) into a `VkBuffer` /
  `VkImage` on a **device shared with the renderer**, and the renderer draws it
  directly — no host round-trip, no external-memory import. This is the primary
  design target, and it turns on one fact of Vulkan: a `VkBuffer` is valid only on
  the `VkDevice` that created it, so zero-copy requires **one shared `VkDevice`**,
  not two UUID-matched ones (separate devices would need external-memory FD
  import — exactly the machinery being avoided). Single process, single device.
  (This is the Vulkan-compute→renderer path. Geometry from the optional
  native-CUDA accelerator instead lives on a CUDA context, not the shared
  `VkDevice`, so *that* handoff does need CUDA↔Vulkan external-memory import — a
  cost scoped to the CUDA path.)

### Device ownership: create or adopt

The standalone-and-together requirement is met by never letting a library assume
it *created* the device it runs on. Each library's `core` exposes device
construction two ways:

- `Device::create(reqs)` — builds a `VkDevice` and **owns** it (real
  `vkDestroyDevice` on teardown). The standalone path.
- `Device::adopt(handles, reqs)` — wraps a `VkDevice` the caller already built and
  **does not own** it (teardown leaves it alone). The shared path.

Everything downstream — allocator, pipelines, the tier stack — takes a `Device&`
and is oblivious to which path produced it; ownership rides on the deleter,
exactly like every other RAII wrapper here.

Two pieces make adoption safe and keep the repos independent:

- **A requirements descriptor** each library publishes (`DeviceRequirements`: the
  device extensions, features, queue capabilities, and API version it needs). A
  host sharing one device merges the requirements of every library that will use
  it, creates a device satisfying the union, and hands it to each.
- **A verify step.** Vulkan offers no way to query which extensions/features a
  *logical* device had enabled at creation, so the creator *declares* what it
  enabled (alongside the raw handles) and `adopt` checks the library's
  requirements against that declared set — returning a non-OK `Status` if the host
  under-provisioned it, rather than crashing three layers later.

The shared bundle is expressed entirely in **raw Vulkan handles plus plain PODs**
(`AdoptedDevice`), never a type either repo imports from the other; each repo
defines its own structurally-identical copy and the integrating application fills
both. That application owns the shared instance/device and a small bootstrap that
computes the union — the only place that knows about both libraries, so the two
stay true independent siblings. Queues are assigned by that bootstrap: the target
is two queues from one graphics+compute family (renderer on one, compute on the
other — same family, so the buffer handoff needs no queue-family ownership
transfer); where a driver exposes a single queue (common on MoltenVK), the two
share it under a caller-provided submit mutex.

Handoff synchronization is an ordinary intra-device **timeline semaphore** on the
shared device (the compute pass signals a value; the renderer waits it before
drawing) — the cross-API `MTLSharedEvent` export the renderer warns against does
not apply once both sides are on one `VkDevice`. The produced mesh/atlas are
ring-buffered (matching the renderer's frames-in-flight) so the producer never
stalls on the consumer, recycled through the renderer's fence-gated retire queue.
Variable marching-cubes topology is drawn indirectly: the compute pass writes a
`VkDrawIndexedIndirectCommand`, so the vertex/index counts never travel back to
the host.

## Color space

Color arrives from a sensor encoded for a display and is then **averaged** — by
the TSDF's running mean, by marching cubes interpolating along a cell edge, by
texture filtering, by the shading multiply. Averaging is a linear operation, and
a display encoding is deliberately not linear, so performing one on the other is
wrong by construction. It is wrong quietly: two observations of linear `0.0` and
`1.0` fuse to `0.214` rather than `0.5`, which reads as a plausibly darker
surface rather than as an error. The mistake is small between similar samples
and largest across high-contrast pairs — silhouettes, shadow boundaries, and
depth discontinuities, which is where a reconstruction is actually inspected.

This is the same class of defect as a flipped camera axis, and it gets the same
treatment the geometry conventions already get in
`sensor/camera_conventions.hpp`: the conversions live in platform-neutral C++ in
the `sensor` tier rather than in whichever driver produced the numbers, because
they are pure arithmetic that host tests can pin on any platform, and because a
per-driver copy is a per-driver opportunity to be silently wrong.

**One rule decides every case:**

> **8-bit is encoded. Float is linear. Convert once at the sensor boundary,
> encode once at presentation.**

Nothing between those two points needs to ask what space it is holding. The rule
falls out of what each representation is *for*: eight bits are only sufficient
for color because a display curve spends them perceptually, so 8-bit storage
must stay encoded — naively linearizing it trades a blending error for banding
in the darks, which is not a trade. A float has the range to be linear, so it
is.

Concretely:

| Representation | Space | Why |
| --- | --- | --- |
| Sensor `ColorFrame::pixels` (8-bit) | encoded | as delivered; the driver declares which curve |
| Voxel `color` attribute (`uint32`) | encoded | 8 bits are only enough when spent perceptually |
| `mesh::Vertex::color` (`Vec4f`) | **linear** | float working value; also what glTF `COLOR_0` specifies |
| Atlas image (8-bit texture) | encoded, `_SRGB` format | the sampler decodes and filters in linear, for free |
| Swapchain (`_SRGB` format) | encoded by hardware | the single encode, at the end |

So the integrator decodes both operands, blends in linear, and re-encodes to its
8-bit attribute; marching cubes decodes the corner colors and writes the
interpolated result as linear float, which is the boundary where the
representation changes and therefore where the conversion belongs. The renderer
then converts nothing at all: its vertex colors are already linear, its atlas is
decoded by the sampler, and the swapchain applies the one encode.

### Declaring an encoding, not converting it

A driver states what it produces; it never converts. The conversion has one
implementation, in `sensor/color_conventions.hpp` beside the camera ones:

```cpp
struct ColorEncoding {
  enum class Transfer  { Linear, Srgb, Bt709, Bt2020Pq };
  enum class Primaries { Bt709, DciP3, Bt2020 };
  enum class Range     { Full, Video };
};
```

carried on `ColorCameraParams` alongside the intrinsics that are already there.
ARKit declares `{Bt709, Bt709, Full}` — its `capturedImage` is bi-planar YCbCr
that a BT.601 *matrix* converts to **gamma-encoded** R'G'B'; the matrix and the
transfer function are independent, and conflating them is its own silent error.
A depth camera emitting linear 16-bit color declares `Transfer::Linear` and is
converted by nothing.

Adding a sensor is then a declaration rather than a conversion, which is the
property that makes this scale: the failure mode of a new integration becomes a
wrong *label* — inspectable, and testable against a known patch — instead of a
bespoke curve buried in a platform driver.

### The storage question, left open deliberately

Blending in linear while storing 8-bit encoded means a decode/encode pair per
colored voxel per frame, and a re-quantization each time the running mean is
written back. The requantization lands in the perceptually-uniform space, which
is the right place for it, but it is still repeated. Two positions:

- **Keep `uint32`, convert in the shader.** The attribute stays four bytes and
  the change is a drop-in. Cost is arithmetic against a ~1.3 ms integrate — small
  in expectation, and to be *measured* rather than assumed.
- **Widen to `RGBA16`.** Exact, no repeated requantization, twice the color
  memory in the sparse grid.

Take the first, measure, and escalate only if banding appears in the darks. The
rule above is unaffected either way: it constrains *what space* a value is in,
not how many bits hold it.

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
- **v1.1** — shared-`VkDevice` zero-copy handoff: the live textured mesh drawn
  directly by the renderer (the create/adopt device seam, indirect draw, and a
  timeline-semaphore ring). This is the headline interop target.
- **v2** — broaden platform coverage (Linux / Android / Windows) on the same
  Vulkan path; performance passes.
- **v2.x** — `compress` (DCT) → `sensor` → `track` → `codec`/`stream`.
