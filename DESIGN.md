# Design

This document explains the architecture of `volumetric_kit_recon` and the
reasoning behind it. For the terse, authoritative list of decisions and
conventions, see [CLAUDE.md](CLAUDE.md); for the full dated record behind each
decision — rationale, measurements, review findings — see
[DECISIONS.md](DECISIONS.md).

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
core → volume → tsdf → mesh → texture → interop
  └→ sensor                   (later: compress, track, codec, stream)
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
- **mesh** — marching-cubes extraction (whole-volume today; the incremental
  block-mesh pool is decided and staged as of 2026-08-09, not shipped), host
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

This is the same class of defect as a flipped camera axis, and it earns the same
treatment the geometry conventions get: pure arithmetic, kept in
platform-neutral C++ where host tests can pin it on any platform, rather than
copied into each driver where every copy is another chance to be silently wrong.

**One rule decides every case:**

> **8-bit color is encoded. Float color is linear. Convert once at the sensor
> boundary, encode once at presentation.**

Nothing between those two points needs to ask what space it is holding. The rule
falls out of what each representation is *for*: eight bits are only sufficient
for color because a display curve spends them perceptually, so 8-bit storage
must stay encoded — naively linearizing it trades a blending error for banding
in the darks, which is not a trade. A float has the range to be linear, so it
is. It says *color* because it is a claim about color and not about eight bits:
a future 8-bit confidence or occupancy channel is a number, and carries no
curve.

### The working space

"Linear" alone does not name a space. Two sensors with different primaries
produce linear values in different RGB bases, and averaging *those* is wrong the
same way averaging encoded values is — quietly, and worst where color is
saturated. So the rule needs one more constant, and the design turns on naming
it rather than on which one is named:

> The working space is **linear BT.709 primaries, D65 white** — the linear half
> of sRGB. Its **canonical encoded form** is that space through the exact
> piecewise sRGB transfer function, full range. Every 8-bit color downstream of
> the sensor boundary is in that one form.

The working space is what `mesh::Vertex::color` holds, what the shading multiply
operates in, and what glTF `COLOR_0` means. It is also what
`VK_COLOR_SPACE_SRGB_NONLINEAR_KHR` presents, so the working→display matrix is
identity on every surface this repo has run on — which is exactly why it would
otherwise go unwritten, and why it is written down before a P3 sensor or an HDR
swapchain makes it something other than identity.

The sensor-boundary conversion is therefore two steps, not one: decode the
declared transfer function, then apply the declared primaries' 3×3 into the
working basis. Both are identity for a BT.709 source, so the common capture path
costs nothing and only a genuinely different sensor pays.

**The cost is booked.** BT.709 is the narrowest of the gamuts a driver can
declare, so a Display P3 or BT.2020 source clips on conversion — and an 8-bit
encoded attribute could not hold the out-of-gamut values regardless. Widening
the working space therefore means widening the storage with it; the two move
together, and both are named constants rather than assumptions spread through
the kernels. Revisit when a wide-gamut sensor is actually in hand. Choosing
BT.2020 now would spend the same eight bits across a wider gamut and band worse
for every sensor we have.

Concretely:

| Representation | Space | Why |
| --- | --- | --- |
| Sensor `CapturedFrame::color` (8-bit) | encoded, as declared | as delivered; the driver declares the curve and converts nothing |
| Voxel `color` attribute (`uint32`) | encoded, canonical | 8 bits are only enough when spent perceptually |
| `mesh::Vertex::color` (`Vec4f`) | **linear** working | float working value; also what glTF `COLOR_0` specifies |
| Atlas image (8-bit texture) | encoded, `_SRGB` format | the sampler decodes and filters in linear, for free |
| Render target (`_SRGB` format) | encoded by hardware | the single encode, at the end |
| 8-bit host export (PLY) | encoded | what every external viewer assumes of `uchar` colors |

So the integrator decodes both operands, blends in linear, and re-encodes to its
8-bit attribute; marching cubes decodes the corner colors and writes the
interpolated result as linear float, which is the boundary where the
representation changes and therefore where the conversion belongs.

### Declaring an encoding, not converting it

A driver states what it produces; it never converts. The declaration is a plain
value in `core`, because it travels with a frame through every tier:

```cpp
// core/color_space.hpp
struct ColorEncoding {
  enum class Transfer { Srgb, Bt709, Linear, Bt2020Pq };
  enum class Primaries { Bt709, DisplayP3, Bt2020 };

  Transfer transfer = Transfer::Srgb;
  Primaries primaries = Primaries::Bt709;
};
```

It rides **beside** the camera — a field on `sensor::CapturedFrame` and
`tsdf::ColorFrame` — and deliberately *not* inside `ColorCameraParams`. That
struct is uploaded verbatim to the fusion kernels under scalar block layout,
pinned at 88 bytes by `static_assert`s with GLSL mirrors in `tsdf/shaders/` and
`texture/shaders/`; putting a field in it would spend a cross-tier shader ABI
change to carry something the kernel needs at most as a push constant. Both
frame structs already hold a `ColorCameraParams` by value, so a sibling field
costs nothing.

There is no `Range` field, and the omission is the point. By the time a frame
reaches this contract it is packed R'G'B', so the YCbCr matrix and any
limited-range expansion have already been applied — the contract requires
full-range R'G'B'. That conversion (de-planarize, the 601-vs-709 matrix, range
expansion) is the driver's, done before the contract, and it is as easy to get
silently wrong as the transfer function; what makes it the driver's is that it
is the one part that varies with the pixel format rather than with the color.
A field nothing consumes is a label free to drift. `Primaries` earns its place
because the working space gives it a consumer; `Range` would not have one.

ARKit declares `{Transfer::Bt709, Primaries::Bt709}` — spelled out, because
scoped enums do not name themselves from an aggregate initializer and this is
the line an out-of-tree driver copies. Its `capturedImage` is bi-planar YCbCr
that a BT.601 *matrix* converts to gamma-encoded R'G'B'; the matrix and the
transfer function are independent, and conflating them is its own silent
error. Its
primaries are already the working basis, so only the transfer is in question —
and BT.709's camera OETF differs from the sRGB EOTF by a couple of codes in the
toe. **`Transfer::Bt709` 8-bit content is accepted as sRGB rather than
converted**, which is what display pipelines do in practice (BT.1886) and what
keeps the common capture path a genuine no-op instead of a per-frame pass over
1920×1440 pixels. A bounded, stated error beats an unstated one. A depth camera
emitting linear 16-bit color declares `Transfer::Linear` and is decoded by
nothing.

Adding a sensor is then a declaration rather than a conversion, which is the
property that makes this scale: the failure mode of a new integration becomes a
wrong *label* — inspectable, and testable against a known patch — instead of a
bespoke curve buried in a platform driver.

### Encoding at presentation

"Presentation" is every point where linear working values become 8-bit for
something outside this pipeline, and there are three in tree, not one:

- The **swapchain** (`fuse_viewer`), where gfx already prefers
  `VK_FORMAT_B8G8R8A8_SRGB` and the hardware encodes.
- The **offscreen target** (`fuse_render`), `R8G8B8A8_UNORM` today and to become
  `_SRGB`. It renders straight to PNG, so nothing else would encode and the
  image would simply come out dark. It is also the CI-visible leg, which is why
  it is worth naming rather than leaving to the word "swapchain".
- **Host export**, where a PLY's `uchar` vertex colors are read as sRGB by every
  external viewer and must be encoded on write, while glTF `COLOR_0` is linear
  and is written through unchanged. The two disagree, so the exporters cannot
  share one path.

Recon never encodes by hand where hardware can: writing linear into an `_SRGB`
target *is* the encode. The surface declares its space the same way a driver
does — `VkFormat` plus `VkColorSpaceKHR` — so a P3 or HDR display is reached by
that declaration changing, at which point the presentation step gains the
primaries half the sensor boundary already has. Nothing else in the pipeline
moves, which is the return on naming the working space.

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

Take the first and measure. The trigger to escalate is **not** banding — that is
a display symptom of the wrong variable. It is the running mean *latching*:
`(prev·w_old + obs·w_obs)/w_sum` re-quantized to 8 bits stops moving once the
per-frame delta falls below half a code, and the voxel's color then freezes
short of its true mean.

Measured, at the default `max_weight = 5` and a 2 m observation
(`tests/core_color_space_test.cpp` pins it): the mean stops **~10 codes short of
its target, uniformly across the range** — 0→64 settles at 55, 0→255 at 245 —
and a gap narrower than ~10 codes never moves the voxel *at all*. The residual
is range-independent because the sRGB curve makes a fixed fraction of the linear
gap a roughly fixed number of codes. So the ceiling on fused color accuracy here
is ~4%, not the ~0.2% an 8-bit attribute suggests, and it is a *convergence*
limit rather than a precision one — which is exactly why banding was the wrong
thing to watch. Whether ~4% matters is the measurement that decides `RGBA16`;
the rule above is unaffected either way, since it constrains *what space* a
value is in, not how many bits hold it.

### What it takes to land

Smaller than it reads, and almost entirely recon's:

- `tsdf/shaders/tsdf_integrate.comp` decodes both operands, blends, re-encodes.
- `mesh/shaders/marching_cubes_common.glsl` decodes the corner colors and writes
  the interpolated result as linear float.
- The examples move their atlas and offscreen formats to `_SRGB`, and encode on
  PLY write.
- **`volumetric_kit_gfx` needs no change at all.** Its vertex colors arrive
  linear, its atlas is decoded by the sampler, and `hybrid_mesh.frag`'s
  `albedo *= ambient + diffuse` is correct the moment its operands are — that
  multiply was never the bug, only what was fed to it. Every format is
  caller-set (`OffscreenTargetDesc::color_format`, `TextureDesc::format`), the
  swapchain already prefers `_SRGB`, and gfx's readback sizes an `_SRGB` format
  like any other. The seam holds because `mesh::Vertex` is already gfx's layout
  and glTF already calls `COLOR_0` linear — the two conventions agreed all along.

The curve gets one host implementation in `core/color_space.hpp` and one GLSL
mirror in `core/shaders/color_common.glsl`, in the shared-`.glsl` discipline the
`volume` and `tsdf` tiers already use — reached through a cross-tier include
spelled like the C++ header path, the first one in the repo, so its provenance
is visible at the include site. It lives in `core` rather than `sensor` for the
reason the camera parameter blocks did: `sensor` branches off `core` beside the
fusion tiers, so `tsdf` and `mesh` cannot include from it, and a curve those
kernels need is vocabulary.

The **type and the `is_canonical` predicate go to `core` with the curve**, not
to `sensor` — implementing this moved them. `TsdfIntegrator::integrate` refuses
a non-canonical frame rather than fusing it through the wrong curve, and `tsdf`
cannot include from `sensor`; a property of `ColorEncoding` belongs with
`ColorEncoding`. `sensor/color_conventions.hpp` keeps what is genuinely the
boundary's: `to_canonical`, which walks a frame, and the cost and limits of
doing so. That refusal is what makes "convert once at the sensor boundary" a
contract rather than a hope — a mislabelled frame is an error, not a quietly
wrong reconstruction.

Both implementations must be the **exact piecewise sRGB function**, not a
`pow(x, 2.2)` approximation. Hardware `_SRGB` sampling uses the exact curve, and
`hybrid_mesh.frag` selects between the atlas and the vertex color *per triangle
across one surface*, so an approximated voxel decode would show up as a seam
exactly where texturing stops.

Four tests pin it, each in the tier that owns the thing it pins: a host round
trip over all 256 codes; a GPU test comparing the GLSL curve to the host one
over the same 256, in the style of the existing on-device ABI round-trips; a
known color patch carried through fuse → mesh → download and checked in linear;
and the convergence check above. The bar is the one the sensor conventions set —
reverting the decode, or substituting `pow(x, 2.2)`, has to *fail a test* rather
than merely look different.

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
