# Design

This is the detailed architecture and implementation reference for
`volumetric_kit_recon`. [AGENTS.md](AGENTS.md) contains the shared working
instructions; [DECISIONS.md](DECISIONS.md#decision-index) contains the dated
rules, rationale, measurements, and review findings. Read the sections relevant
to the task, and update them with the code when a contract changes.

- [Tiered architecture](#tiered-architecture) and [package names](#naming-conventions-use-these-consistently)
- [Interop and device ownership](#the-interop-seam), [Vulkan gotchas](#key-gotchas-verified), and [color](#color-space)
- Implementation: [core](#core), [volume](#volume), [tsdf](#tsdf), [mesh](#mesh), [texture](#texture), [sensor](#sensor), [codec](#codec), [eval](#eval)
- [Examples](#examples) and [next work](#next-work)

The tier descriptions and next-work notes below retain the implementation
state recorded through 2026-10-01. They distinguish current behavior from
provisional defaults and unmeasured work; confirm the affected code when using
an older measurement to make a new decision.

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
  `…_sensor`, `…_codec`, `…_eval`, `…_io`, `…_interop` (+ later `…_track`, `…_stream`),
  plus `…_sensor_utils` (GPU pre-processing), the opt-in `…_sensor_orbbec`
  driver (`VR_WITH_ORBBEC`), `…_sensor_video` decoder (`VR_WITH_FFMPEG`), and
  `…_io_assimp` mesh importer (`VR_WITH_ASSIMP`);
  umbrella alias
  `volumetric_kit::recon`.

## Tiered architecture

Strict left-to-right dependency rule: a tier may depend only on tiers to its
left. No upward includes.

`core` → `volume` → `tsdf` → `mesh` → `texture` → `interop`, with `sensor`
branching off **`core`**, `codec` off **`volume`** and `eval`/`io` off **`mesh`**
(later: `track`, `stream`).

- **`core`** — the Vulkan foundation *and* the vocabulary every tier trades in
  (`Status`/`Result`, the GLM math aliases, and the posed pinhole
  `DepthCameraParams`/`ColorCameraParams` of `core/camera_params.hpp`),
  mirroring `volumetric_kit_gfx`'s core:
  instance, device (one compute queue, which carries transfers too), VMA
  allocator, RAII buffer/image, the `CommandBatch` that records one call's
  uploads, dispatches and readbacks into one submit,
  compute-pipeline + descriptor-set wrappers (and the `ComputeKernel` bundle +
  `KernelSetBuilder` that groups a kernel's layout/pipeline/set behind one
  shared pool), sync (fences, timeline semaphores), the `Status`/`Result` idiom,
  a pluggable log handler, and
  the GLM-backed vector/matrix math. Vulkan is reached through one umbrella header
  (`core/vulkan.hpp`), as in gfx — no other code includes `<vulkan/...>`
  directly.
- **`volume`** — the sparse voxel hash map in Vulkan buffers; allocate / compact
  / rehash as compute shaders. (POD layouts already landed in `volume/hash_types.hpp`.)
- **`tsdf`** — TSDF integration compute shaders (classic + dynamic), and a
  triangle mesh's distance field written in (signed, or as a shell).
- **`mesh`** — marching-cubes compute shaders and host mesh containers.
- **`io`** — host asset loading and export, branching off `mesh`; encoded
  images, depth PNGs and PLY/PNG export are in `recon_io`, static mesh import
  in the optional Assimp-backed `recon_io_assimp`. GPU tiers never link back
  to file I/O. Dataset playback and camera conventions remain with their
  capture adapters.
- **`texture`** — projective texturing: fills the mesh's per-vertex `uv0` with a
  posed camera's image coordinates where it has line of sight (per-vertex-color
  fallback elsewhere), or with several cameras' coordinates into an atlas of
  their images, a compute pass. A colour image is registered to its depth
  camera or taken by a colour camera of its own, whose line of sight the
  depth map decides too; depth may be on the host or the device.
- **`sensor`** — the capture *contract*: `ICameraCapture`, the `CapturedFrame`
  view the fusion tiers consume, and the boundary conversions a capture
  integration gets silently wrong — camera conventions (pose handedness,
  registered-depth intrinsics) and colour (`to_canonical`). Reads
  `DepthCameraParams` + `ColorCameraParams` from `core/camera_params.hpp` and
  `ColorEncoding` from `core/color_space.hpp`, so it
  depends on **`core` alone** — it sits beside the fusion tiers, not on top of
  them — and bundles **no drivers**: one ships here only if this repo can build
  *and* test it (the 2026-08-02 decision). The one that does, Orbbec, is a
  target of its own (`sensor/orbbec/`), so `recon_sensor` never links a vendor
  SDK. It also reads and writes the rig calibration file calib produces
  (`rig_calibration.hpp`). The HEVC and JPEG decoders are another target of
  their own (`sensor/video/`, over FFmpeg), link `core` alone, and know no
  camera.
  The GPU pre-processing is a third (`sensor/utils/`, Vulkan and shaders), so
  `recon_sensor` itself stays free of both.
- **`codec`** — the per-frame TSDF geometry codec: separate `Encoder` and
  `Decoder` classes over a private 8³ DCT transform, a geometry-only intra
  frame (block coordinates, an observed-voxel mask, the first K coefficients),
  static-table rANS in independent segments of blocks (2026-09-27). Links
  **`volume` alone**. Color is not coded: the
  player textures the decoded mesh from RGB that travels beside it (the
  2026-09-26 decision).
- **`eval`** — quality measurement for the tests and examples that judge a
  reconstruction or a codec: mesh-to-mesh distance, accuracy / coverage and
  the F-score (`eval/mesh_distance.hpp`). Host-side, deterministic, and linked
  by nothing in the pipeline. It is infrastructure, not one of the excluded
  eval harnesses (2026-09-27).
- **`interop`** — the handoff to `volumetric_kit_gfx` (below).

## Backend strategy: one Vulkan path

Compute runs as **Vulkan compute shaders** (GLSL → SPIR-V), with **MoltenVK** on
Apple — exactly the cross-platform strategy `volumetric_kit_gfx` uses for
rendering. One code path serves Linux, Android, macOS, iOS, and Windows, rather
than a Metal + CUDA split. Vulkan is reached through a single umbrella header so
the loader choice (link-time loader now, volk later for iOS/Android) stays a
detail. Because the renderer is also Vulkan, the two libraries can share a
`VkDevice` — which is what makes the geometry handoff cheap (see the interop
seam below).

Native CUDA is an optional NVIDIA accelerator under this baseline (the
2026-07-04 decision); it must stay numerically in lockstep with GLSL.

## Error handling

No exceptions cross the API boundary (mobile builds use `-fno-exceptions`).
Fallible calls return `Status` (success or an error domain + message) or
`Result<T>` (a value or a `Status`). `VR_TRY` and `VR_ASSIGN` remove the
check-and-propagate boilerplate. Programmer errors (precondition violations) fail
fast via `VR_CHECK` (log + abort), distinct from recoverable runtime failures.
`Status` is intentionally backend-neutral — a generic `int64_t` detail code, not
a Vulkan or CUDA type — so the same idiom serves every tier.

## The interop seam

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
  seam A, since it builds two devices by design.

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
stay true independent siblings. The neutral bootstrap prefers separate queue
families, using concurrent sharing or explicit ownership transfer for shared
resources; when that is unavailable, the libraries share a queue behind a
caller-provided submit mutex. MoltenVK's one-queue-per-family constraint is
covered under [Key gotchas](#key-gotchas-verified).

The mesh/atlas are ring-buffered, and slots are recycled by the host's report
as gfx frames retire. A cross-library GPU wait can deadlock against a swapchain
rebuild, so the release contract is host-side (the 2026-08-03 decision).
Marching cubes writes a `VkDrawIndexedIndirectCommand` for variable topology;
the extractor reads back the small control result, while gfx draws the
geometry buffers directly.

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
- **Device locality and host visibility are independent flags.** Non-local
  host memory costs PCIe traffic on a discrete GPU, a cost hidden by Apple's
  unified memory. A type with both `DEVICE_LOCAL` and `HOST_VISIBLE` is still
  local (UMA or BAR). Every buffer `storage_buffer` makes is host-visible;
  its actual locality is recorded in `Buffer::memory_info()`. The hash table's
  bucket locks used non-local host memory until an RTX 5090 took 1.97 s to
  allocate a 5 000-triangle sheet
  (3.4 ms device-local) and lost a 320 000-triangle one to the driver's
  7-second watchdog (Xid 8 / 109; the 2026-09-28 measured lesson). The bulk
  data costs as surely: the voxel arrays, frames and arena in non-local memory put
  `integrate` at 14.6 ms of device time against 0.067 ms resident (the
  2026-09-28 residency decision). Memory the kernels touch is
  `device_storage_buffer`, which requires `DEVICE_LOCAL` rather than preferring
  it, on Apple too, reached from the host through a
  `CommandBatch`; and the CPU never reads VRAM directly, since BAR memory
  reads uncached (6.6 s for one mesh download).
  Small parameters may stay host-visible: under 64 KB, it measured nothing.
  So may a table the host reads in place, as `mesh`'s span table is: the
  kernel writes each block's entry once, and `block_spans()` is a pointer. A
  GPU test failing on the Linux boxes with a bare `vkWaitForFences` is a lost
  device: read the host's kernel log for the Xid before calling it load.
- **A bare `cmake -S . -B build` leaves `CMAKE_BUILD_TYPE` empty, so everything
  compiles at `-O0`** — the flags are `-std=c++17 -Wall -Wextra -Wpedantic
  -Werror` and no optimisation at all. Every CI leg passes one explicitly
  (`viewer.yml` uses Release), so this bites only local runs, and it bites the
  *examples* hardest because their whole job is measurement: `fuse_viewer`'s
  `atlas pack` row read **11.18 ms** against **0.32 ms at `-O2`**, a 35x
  phantom, and it was read as a real cost before the flags were checked. Always
  configure with `-DCMAKE_BUILD_TYPE=Release` before quoting an overlay number,
  and quote the build type with the figure.

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
- The **offscreen target** (`fuse_render`), `R8G8B8A8_SRGB`. It renders
  straight to PNG, so the target performs the encode. It is also the
  CI-visible presentation path.
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

### Implementation and validation

The color-space contract is implemented across these boundaries:

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

The curve has one host implementation in `core/color_space.hpp` and one GLSL
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
  (The C++ `eval` tier is not one of them: it is the repo's own quality
  measurement, held to the same bar as any tier — the 2026-09-27 decision.)

The codec is deterministic in the sense the 2026-09-26 decision defines:
a bitstream is reproducible on the device that wrote it and decodable on any.

## Packaging

The repo installs and exports like its sibling: per-tier targets under a shared
export set, a generated `volumetric_kit_reconConfig.cmake`, and consumption via
both `find_package(volumetric_kit_recon)` and `FetchContent`. Quality gates
(warnings-as-errors, formatting hooks, sanitizer and test CI) are wired from the
start, not retrofitted.

## Implementation reference

Landed and proven on MoltenVK, left to right: `core` → `volume` → `tsdf` →
`mesh` → `texture`, plus `sensor` (the contract and the Orbbec driver) and
**interop seam B** — gfx drawing recon's own buffers with no host round trip.
[DECISIONS.md](DECISIONS.md) carries the *why* behind anything here that looks
arbitrary; it usually isn't.

### core

the Vulkan compute foundation: VMA `Allocator`, RAII `Buffer`,
`ShaderModule`, descriptor + `ComputePipeline` wrappers, the `ComputeKernel`
bundle + `KernelSetBuilder`, the shared-queue-safe
`Device::submit_single_time` dispatch (safe from several threads at once:
each submit records on a command pool of its own, kept with its fence for
the next, and only the queue submit is locked; a kernel's set, a buffer and a `GpuTimer` stay the caller's to
keep to one thread), and the shared `dispatch()` /
`group_count` / `storage_buffer` / range-guard helpers of `compute_util.hpp`
— `StorageInput` among them, the host array (staged onto the device in the
call's batch) or device buffer a call binds at its image's exact range.
`MemoryUsage::DeviceLocal` requires `VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT`:
allocation fails instead of spilling into a non-local heap. `Buffer::memory_info()`
records the selected memory type's flags, type index and heap index; its
optional value is absent for an empty buffer or an adoption with unknown
backing memory. `is_device_local()` accepts `DEVICE_LOCAL` with or without
`HOST_VISIBLE`. The VMA allocator and exported-buffer factory both record the
actual allocation type. `StorageInput` refuses a borrowed buffer with unknown
or non-local memory before submitting work, even when its size and storage
usage fit. Adopters must supply the bound allocation's actual memory
metadata: the adopting constructor has no default for it, and `std::nullopt`
says it is unknown. Callers holding non-local data can pass a host array for staging, or
upload/copy into `device_storage_buffer` with `CommandBatch` before using the
device overload. Transfer-only sources and the documented small-parameter or
host-read-table exceptions do not gain a blanket residency restriction.
**`CommandBatch`** (`core/command_batch.hpp`)
is how the host reaches device memory: one call's uploads, fills, copies,
dispatches (indirect too) and readbacks in one command buffer, one fence
wait, spans and labels kept. `acquire` takes over a buffer another queue
family or an API outside Vulkan wrote, the receiving half of the ownership
transfer. A barrier goes wherever a command could see an
earlier one's writes: around every dispatch, and between two transfers
only when they share a buffer one writes, unless they are fills, uploads
or copies rising through it without overlap, and no command in the run
writes a copy's source. `zero` clears a range at any
alignment. `dispatch()` is a batch of one.
Descriptor-set copies share their binding-mutation counter. A dispatch records
the set handle and counter; `submit` refuses a set changed through any copy or
a recorded wrapper reassigned to another set, before any GPU work is submitted.
An upload of up to 64 KiB, 4-byte aligned, goes inline
(`vkCmdUpdateBuffer`) and a larger one through a staging buffer the batch
allocates, which `reserve_upload` hands to a caller packing its own bytes;
an upload or copy given a `GpuStageScope` is timed like a dispatch.
Readbacks, which are small results, land in one host buffer
allocated at `submit`, and the staging is freed once the wait is done, or
leaked if the wait fails. Nothing goes through a mapping, so memory type
never changes what a batch does, and usage is checked on every buffer, as
is a push against the kernel's range. A refused call poisons the batch, and
`submit` refuses a kernel whose set was rewritten after its dispatch was
recorded, since the set is bound only then (the 2026-09-28 residency
decision, which also records why there is no staging arena and no mappable
device memory). So a batch that dispatches one kernel over several cameras
binds each dispatch a set of its own, from a grow-only `KernelSets`, through
`dispatch`'s set overload, which refuses a temporary since the batch binds
that very object at `submit` (the 2026-09-30 decision).
Vocabulary: `Status`/`Result`, the GLM aliases, `camera_params.hpp`,
`color_space.hpp`, and `stage_metrics.hpp` — the `{name, cpu_ms, gpu_ms,
has_gpu}` rows every tier reports timings in, with `GpuTimer` measuring the
device half through the timed `submit_single_time` overload (a window is
ended by publishing it). `kBreakdownPrefix` marks a row its parent's row
already contains, so `total_cpu_ms` skips it and `total_gpu_ms` **does
not** — a host scope spans a whole call, a device span one dispatch, so a
sub-row's device time has no parent to be counted through; `in_stage()` is
how a callee reached from both positions knows which of the two it is
writing. A tier opens one **`GpuStageScope`** per call
(`core/gpu_timer.hpp`): it times the host span, is what `dispatch()` takes to
record the device one, and publishes both in its destructor, so no early
return can strand a span. Timing is unavailable, never an error — a query
pool that will not allocate degrades like a family with no timestamps, and
`abandon()` retires the pool when a failed fence wait leaks the command
buffer carrying its queries. `Device::create` enables `scalarBlockLayout`;
`adopt` requires the creator did, and both record the queue family's
`queueFlags`. `create_exported_buffer` (`core/external_memory.hpp`) makes
a buffer CUDA imports, on a device that `exports_memory()`:
`VK_KHR_external_memory_fd`, which `create` enables where offered and
`requirements()` names as optional (`external_memory`); beside it,
`find_memory_type` is the type a resource bound by hand takes, the first
that fits, which Vulkan's ordering makes the plainest; it never returns a
protected, lazily allocated or AMD device-coherent type. `Image`
(`core/image.hpp`) holds a `VkImage` another API made, freed by its
maker's deleter and kept in one layout a copy reads (GENERAL or
TRANSFER_SRC_OPTIMAL), and `CommandBatch::copy` copies an R8 or R8G8 one
into a buffer; `VK_EXT_metal_objects`, enabled and named
the same way (`metal_objects`, `imports_metal_textures()`), is how a
VideoToolbox picture's planes arrive as images. Separately from
all of that, `core` carries the seam an
**external** GPU profiler reads: `VK_EXT_debug_utils` is requested by
default and *independently of validation* (2026-08-30), so a Release build —
the only one worth profiling — carries labels; `dispatch()` wraps each
submission in a region named by `ComputeKernel::name`, and
`Device::set_object_name` names the buffers. Labels are **not** tied to a
`GpuStageScope`: a span costs a timestamp (~0.13 ms per submit on MoltenVK)
and exists only where the caller asked for `StageMetrics`, so pairing them
would perturb the captured workload and leave uninstrumented calls anonymous.
Naming is re-applied wherever a handle is replaced — a grid `resize`, a hash
rehash, a mesh-arena grow, a span-table grow — since a name
lives on the handle. One change
serves both profilers: Nsight renders the regions as trace ranges (and
groups a capture by `VkPipeline`, which `KernelSetBuilder::add` therefore
names too), MoltenVK maps them onto Metal debug groups and
`MTLComputePipelineState` labels for Xcode. Debug utils is an *instance*
extension, so it cannot ride `enabled_device_extensions` and is **declared,
never probed** — the loader's answer for a disabled extension is not portable
(null from a conformant loader, a live pointer from a directly-linked
MoltenVK). Both seams carry the declaration:
`AdoptedDevice::enabled_debug_utils` on adopt and
`DeviceConfig::instance_debug_utils_enabled` on create, the latter filled in
by the `Device::create(const Instance&, …)` overload every recon call site
uses; `DeviceRequirements::debug_utils` is how an embedder hears recon wants
it at all. It is never required — its absence costs the capture's names and
nothing else — and resolution is **all-or-nothing**, so a driver returning
two of the three entry points reports no labels rather than a region that can
be opened and not closed. The label is recorded *outside* the `GpuTimer`
bracket, so no `gpu_ms` measures the markers around the work.

### volume

`VoxelHashMap` drives init / allocate-from-coords, -depth,
-points, -triangles / remove / compact / compact-in-frustum / resize as GLSL
kernels
(`volume/shaders/hash_*.comp`) over the scalar-block-layout ABI. Depth
allocation unprojects a posed frame and dilates each surface block into the
`(2·tb+1)³` truncation band — a solid cube, not a ray march — a 16 × 16
pixel tile a workgroup, each distinct block of the tile dilated once and its
band shared out over the lanes (2026-09-30);
`allocate_from_triangles` deliberately does **not** dilate that cube
(2026-08-31): a triangle wider than the band — which is one block, 40 mm, at
the defaults — would hole through the middle, so a block is allocated when its
centre is within `trunc_dist` + the block's half-diagonal of some triangle,
and the work is split per *candidate block* rather than per triangle so one
lane cannot own a 250 000-block quad. The host pass that counts those blocks
is also where an index is bounds-checked and a zero-area or non-finite
triangle is dropped — the former being the one input that divides by zero in
the closest-point solve, which lives in `core/shaders/triangle_common.glsl`
so mesh-to-SDF evaluates the field with the same function. That pass is the
public `triangle_candidate_offsets`, and the decode and band test are
`volume/shaders/triangle_candidates.glsl`: `tsdf`'s mesh integrator bins over
both, so the blocks allocated and the blocks binned cannot drift apart.
Frustum-culled compaction gives the per-frame working set, from a depth
camera's pinhole
intrinsics or — since 2026-08-12 — from a *render* camera's `view_proj`, whose
planes are read off the matrix itself and so hold for any handedness, provided
depth maps to `[0, 1]` (gfx's convention; a GL matrix puts the near plane at
the harmonic mean `2nf/(n+f)`, over-culling a shell about one near-distance
thick — pinned by a block that lies inside it). That
overload's `margin_m` widens every plane **except the near one** by a distance
in metres, applied after normalizing, for a consumer whose cull runs a few
frames behind its draw; widening near too would move it behind the eye at
`margin_m > z_near` and admit a cone of geometry behind the camera. A
degenerate or non-finite matrix leaves its planes unnormalized and unwidened,
so the cull degrades to keeping blocks rather than dropping them.
The result travels as a `BlockList` — pointer, count, and the
`topology_epoch` it was compacted at, paired by `VoxelBlockGrid::block_list`
so the three cannot be mispaired — which is what `mesh` meshes a subset
from. `resize` preserves block indices,
so per-voxel data survives a grow. `VoxelBlockGrid` composes the map with
independently-allocated SoA attribute arrays (`tsdf`, `weight`, `color`, …),
each `num_blocks·voxels_per_block`, so a consumer materialises only what it
needs. Every buffer of both is device-local, reached through a
`CommandBatch`: `create`, `clear` and `remove` zero on the device (`clear`
and `remove` before any index is freed, `remove` by a kernel that finds
each coord's block itself, so its cost is the count's, not the grid's),
`resize` copies there, a call's
inputs are uploaded in its first round, and a round reads back its counts.
The map's device-buffer `remove` overload binds a producer's coordinate list
directly. Garbage collection uses it after zeroing the listed attributes, so
the stale-block list stays on the GPU through deletion; only the stale and
retry counts reach the host.
A compaction still reads its list back, in the count's own submit while the
set stays within a quarter past its last count (the 2026-09-28 residency
decision), except `compact_active_blocks_on_device`'s: its
`DeviceBlockList` stays on the device, stamped with the epoch, a
compaction serial and the heap's free count, and `check_device_block_list`
refuses one that a compaction, allocation, resize, remove, clear or move
has made stale. While the last one still holds, the call returns it again,
dispatching and reporting nothing, so an extract after a fuse reuses the
fuse's compaction (2026-09-30).
`topology_epoch()` lives on the *map* — the object that frees a block
index — and is a globally unique token re-drawn at `create`, at every
`clear` and at every `remove` that accepts its input, never at `resize`: a slot-keyed cache (mesh's spans)
anchors on it, so no path may free an index without moving it
and no two grids may ever share a value. Host `diagnostics()` scans occupancy;
`load_factor()` is the constant-time read a per-frame caller can afford (a
host copy of the heap counter, read back by every round that moves it, which
`diagnostics()` checks against the device's own), and
`kGrowThreshold` is the occupancy it says to grow at — named here so a UI or
an embedder cannot draw a ceiling that disagrees with it.
`allocate_from_depth` also takes a list of frames (`DepthInput`), every
frame dispatched in each round's one submit, on a set of its own; a round
that retries dispatches them all again, and the rounds are the call's, not
each frame's. Opt-in
`StageMetrics*` on `allocate_from_depth` (an `"allocate"` row summing every
retry round) and on both compaction entry points (an `"active set"` row,
breakdown-prefixed when the caller already has a stage open).
Every block slot carries a `BlockStamp` (`hash_types.hpp`): ticks of the
map's clock (`tick()`), in a device buffer beside the heap
(`stamps_buffer()`), each written by the pass that knows its fact
(2026-10-01). Every allocation kernel stamps `requested` on each block it
asks for, inserted or found, and the grid's block pass (`stamp_blocks`)
stamps `weighted` on each holding an observed voxel. Every pass that
writes voxels -- `TsdfIntegrator`, `MeshIntegrator`, the codec's
inverse -- advances the clock first and stamps `changed` on what it
changed, so the clock counts writes (one a set, for the rig) and a
reader's recorded tick is older than every later change, however the calls
interleave. The init kernel zeroes the records, the delete kernel zeroes a
freed slot's, and `resize` copies them forward, the rehash stamping
nothing. A consumer compares ticks and resets nothing:
`free_stale_blocks(max_age)` frees the blocks whose newer stamp is older
than that, zeroing them with a kernel over the pass's list, so the band the
allocator still asks for stays, and `rig_viewer` runs it every
`--free-after` sets; `mesh`'s incremental extract re-meshes what changed
since the tick it last meshed at.

### tsdf

`TsdfIntegrator` fuses a posed depth frame into a grid's
`tsdf`/`weight`: projective `sdf = depth − Zc`, `±trunc_dist`, an
inverse-square-with-behind-dropoff weight, running average capped at
`max_weight`. Depth is sampled bilinearly, nearest at image edges, across
discontinuities, or when any tap is non-positive or non-finite. A NaN hole
therefore leaves a valid nearest sample usable. Zero is a missing sample even
when `min_depth` is zero, in allocation and integration alike. `IntegrationMode` selects
**classic** (keep free space ahead
of the surface) or **dynamic** (clear it, so a receded surface leaves no
ghost). An optional `ColorFrame` fuses colour through its own separate
`ColorCameraParams`; a voxel's first colour observation assigns rather than
blends, and `coverage_in_alpha` has a pixel with a zero high byte fuse no
colour, as one outside the image fuses none. Each call is one tick of the
map's clock, and stamps `changed` on every block whose `tsdf`, `weight` or
`color` it changed -- a store that leaves a different value, Dynamic's
clear included, so converged surface stamps nothing -- always, one lane a
subgroup reading the stamp before its atomic, which measured as nothing
(2026-10-01). Opt-in `StageMetrics*` reports an
`"integrate"` row with both halves, over a `"  ..active set"` sub-row for the
compaction dispatch it also makes. That compaction leaves its list on the
device (`compact_active_blocks_on_device`), so a fuse is two submits, the
compaction's count the only thing read back; the frames are staged.
`integrate` also takes a list of frames
(`FrameInput`, a `DepthInput` and its colour, so one list feeds both
calls): one compaction and one submit for them all, each frame a dispatch
of its own in order, so every voxel takes them in turn as integrating them
one at a time does, bit for bit over the same blocks. A frame with no
pixels fuses nothing, as it allocates nothing (2026-09-30).
`MeshIntegrator` writes a triangle mesh's distance field instead
(2026-09-27), **overwriting** every voxel of every block the band reaches:
weight 1 within `trunc_dist` of the mesh, the codec inverse's fresh zeros
elsewhere, so a mesh-derived grid reads like a decoded frame. `Signed` signs
by the closest triangle's face normal, the research codec's rule, and builds
no topology and checks nothing: the field is the codec's input, not a
reconstruction of the mesh, so it gives up the sign past edges sharper than
90° and grows a skirt past an open mesh's rim. `Shell` is `d − σ` and takes
any mesh. The blocks must be
allocated by `allocate_from_triangles` — a missing one is refused before
anything is written — and triangles are binned per block over that
allocation's own candidates, so no voxel measures the whole mesh. The fill
replays the slots the count pass recorded, so no bin comes up short; a bin
past `kMaxBinTriangles` is refused, and the write splits into dispatches of
at most `kMaxDispatchBinEntries` bin entries, each submitted on its own.
Counts, fill cursors and occupied bins stay on the device: a hierarchical
256-way prefix scan carries entry totals, occupied counts, largest bin and
overflow, then compacts bins in slot order. The host reads 20 bytes of
validation/size control plus 8 bytes per bounded dispatch range, at most
34,960 bytes of ranges even for the uint32 candidate limit. Prefix windows
of `kMaxDispatchBinEntries - kMaxBinTriangles + 1` bound every range; the host
only splits them further for the device thread limit. Bulk scratch grows
with demand and stays allocated. If the compact list outgrows its retained
capacity, only compaction/range planning retries after growth; normal calls
need one planning submission plus one per write. No grid write precedes
missing-block, overflow, bin-size and binding-range validation
(2026-10-02). Ties break on the triangle index, so the same mesh writes the same
bytes. Each call is a tick, and every block it writes is stamped `changed`.

### mesh

`MarchingCubes` over a sparse `VoxelBlockGrid`, and only that
(the dense analytic entry point was removed 2026-08-31; the prior engine
never had one). Two workflows, named by their entry points: `extract_host`
returns an owned host `Mesh` for export — the complete transaction, giving
its ring slot back so a PLY writer never learns the ring exists — and
`extract_device` returns a borrowed `DeviceMesh` a renderer draws directly,
released by generation. One workgroup per active block, with the cross-block 2×2×2
neighbourhood resolved by probing the hash table on-device. A block counts
its output, reserves one range for all of it with a single atomic, and
only then writes, so **a block's output is contiguous in the arena** —
the precondition for meshing only the blocks a fuse changed, taken at ~10% on
the dispatch. True of **both** sparse kernels: `share_vertices` reserves two
ranges rather than one, since a shared vertex breaks `v = 3t`, and measured no
cost. Each cursor is bounded by its block's own reservation, so a count that
disagreed with the emit would drop geometry rather than write over the next
block's range. Opt-in `track_block_spans` publishes that range as
`block_spans()` — vertex and **triangle** base/count per block slot, the
mapping stage 3 re-meshes against and the host cannot derive, since the
atomics hand ranges out in workgroup arrival order. Off by default (it is
sized by the grid, not the surface), borrowed, and readable only while
`block_spans_generation()` still names the mesh you hold —
`block_span_valid(grid, slot)` answers the same question per slot, against
the `topology_epoch` the spans were written for and the *serial* of the
extract that wrote them, since a LIFO-reused slot names a different block and
a block dropped from the active set keeps its last stamp. Nothing in the
table itself says "not mine": a grow carries every old span forward, so
`block_spans()` is a fetch for slots `block_span_valid` approved, not an
array to iterate. Before an incremental GPU dispatch, active slots without a
stamp from the previous extract have their spans zeroed. This includes reused
slots absent from a full fallback and newly allocated slots with no voxel
changes yet; neither may inherit another block's arena range. The vertex arena
is fitted to the surface, grow-only, and held as a **ring of slots** the
consumer releases by generation; the kernel writes a real
`VkDrawIndexedIndirectCommand`. `extract_device` returns a borrowed
`DeviceMesh` (valid until the next extract, enforced by a generation stamp),
`download` takes the single host copy and bridges the two workflows. The
arena, index run and draw command are device-local, and each extract
attempt is one batch that reads back only the 32-byte command; the span
table stays host-visible, since the host reads it. With the spans off the
active list never reaches the host: the extract compacts onto the device,
or takes the fuse's list back from the map, and binds it in place
(2026-09-30). An
`extract_device` overload meshes a
caller-supplied `volume::BlockList` instead of compacting the whole map —
what a camera's frustum-culled set arrives as, though nothing in the extractor
knows a frustum produced it (2026-08-12). The arena is rebuilt from that
dispatch alone, so a block outside the list costs no triangles and no *live*
bytes — the arena is grow-only, so one full extract sizes it for the whole set
and it never shrinks back;
the surface does not hole at the cull edge, since the on-device probe still
resolves neighbours that were never dispatched; the list is refused if its
`topology_epoch` has moved (a LIFO-reused `ptr` being a lie that meshes
cleanly), if it holds more blocks than the heap has slots, or if it is null
with a count — all three `VoxelBlockGrid::check_block_list`, which `codec`
shares, and all three above the slot claim, so a refusal is a **rollback**
and an outstanding `DeviceMesh` survives it; and `compact_ms` reads 0 while
every row that scales with the active set shrinks with it (`readback_ms` and
`descriptor_ms` are per-call constants and do **not**). The list must be
duplicate-free — unchecked, and a repeat emits the block twice and races its
span — and is built by `VoxelBlockGrid::block_list`, whose rvalue overload
is deleted so a temporary compaction cannot leave it dangling.
Offered on `extract_device` **only** — an incremental pass keeps the triangles
of blocks it does not re-mesh, so culling would leave them drawn and give the
arena win back. *Alternating* the two is safe, though: a culled pass publishes
no `arena_state_` (and records no density, a culled set being denser per block
than the map it came from), so the next incremental request falls back to a
full extract and reports it. `share_vertices` selects a second
compiled kernel that indexes in-block vertices — 3.4x fewer on room0, and
textured like any other mesh since the `texture` tier moved to a per-vertex
verdict (2026-08-11). `DeviceMesh::shares_vertices` still publishes it,
because `v = 3t` no longer holds and a consumer sizing an arena cannot derive
that from the buffers, and because the `texture` tier's several-view atlas
chooses per triangle and so refuses a shared mesh (2026-09-28).
`extract_device_incremental` re-meshes only the blocks changed since its
last extract: it keeps the map's tick that extract ran at, reads each
block's `changed` stamp against it (2026-10-01), dilates the *changed* set
into the *re-mesh* set on-device
over the 2×2×2 neighbourhood the gather already resolved, reuses each block's
existing range where the new count fits and appends past the watermark where
it does not, retiring what it leaves behind to zero-area triangles. It runs
under `share_vertices` (2026-08-11), which reuses **two** ranges rather than
one — in place only when *both* counts fit, since a triangle indexes into the
vertex range beside it — and which retires an order of magnitude more cheaply,
not less: that kernel owns its index run, so a dead triangle costs 12 bytes
against the default kernel's 192, and its dead vertices need no writing at
all. What it
may trust is one `{watermark, epoch, serial, tick, iso}` struct, cleared at the top of
**both** extract paths and re-established only on the publishing return, so
no failure leaves it describing geometry that is
gone; the anchor is compared *above* the call that re-anchors it, or it
compares a value with itself. Everything else is a **silent fallback to a
full extract**, which is why `ExtractTimings::incremental` reports which pass
the caller got and `remeshed_blocks` (counted on-device, since the dilation
never reaches the host) reports what it saved — `dispatches` counts refit
rounds and reads 1 on both. Occupancy past `kMaxArenaOccupancy`x the live
count (summed off all current active spans, including newly emitted blocks,
never from the arena's own total, which ratchets)
withholds the state so the next pass compacts — asked on **both** axes under
sharing, since retirement leaves dead triangles occupying index slots while
dead vertices are merely unreachable, so the two buffers drift apart and
either can be the binding one. `slot_count == 1` only, so it
is off in `fuse_viewer` today — a `TODO(mesh)` on the class.

### texture

`ProjectiveTexturer` rewrites every `Vertex::uv0` against one
posed frame, one thread per **vertex**: it is kept where the vertex is in
front, in frame, and **unoccluded** (projected depth agrees with the depth
map). Three outcomes, not two, and a consumer must test the **sign** and
never `== (-1,-1)`: a visible vertex gets `uv`, one in frame but occluded
gets `-uv - 1` — negative, so gfx takes the per-vertex-colour path, but the
coordinate is *carried* so a mixed triangle interpolates between real
projections instead of smearing toward the atlas origin — and one behind the
camera or holding a non-finite position gets the bare `(-1,-1)`, there being
nothing to carry. A vertex in front but outside the image carries the clamped
border coordinate; conflating it with the behind-camera case drew the whole
image inside one triangle along the frustum edge. Live single camera, so the
frame the caller binds *is* the atlas. A **`TextureView`** is one frame: a
depth map on the host or the device (`depth_buffer`), its camera, the size
of its colour image, which is the tile at its own resolution so
low-resolution depth textures at the capture's, and, when that image is not
registered to the depth camera, the `color_camera` it was taken with and
optionally its `coverage` (a `GpuFramePrep` frame's colour, 0 where the lens
saw nothing), the buffers held by `shared_ptr`. The single-camera pass takes
one view (the `DeviceMesh` and host `Mesh` overloads), and the several-view
overloads texture from **several** into an atlas of their images side by
side, in floor(sqrt(n)) rows so four views make two rows of two
(`texture_atlas.hpp`: `side_by_side_atlas`, `pack_atlas`), one thread per
**triangle**: each takes the view facing it most squarely among those
that see its **front** and all three of its vertices, a `fallback` view
only where no other does, and all three point into that view's tile. Per
triangle because vertices in different tiles
would interpolate across the atlas, so that path needs an unshared mesh and
refuses a shared one, as it refuses a view with no depth range and tiles
that overlap (2026-09-28). With a colour camera the depth camera still
decides visibility, and the coordinate is the colour camera's, kept only
where its image recorded the vertex, both cameras see the same side of the
surface, and its line of sight is clear: the pass walks that line's
projection across the depth map, at most 64 samples, for a surface in front
of it, which is what catches the parallax fringe beside an occluding edge.
The sight line is clipped in camera space to the near bound and image side
planes before projection. A zero or very small near bound cannot bypass the
walk or spend its sample budget outside the image. Only positive, in-range
depth samples can occlude that line; a zero-valued hole is ignored even when
the accepted near bound is zero.
A registered image is the case where the two cameras are one (the
2026-09-28 colour-camera decision). Opt-in `StageMetrics*` on every overload
reports a `"texture"` row with both halves, the several-view inputs'
transfers included. Every pass is one batch, a host depth frame staged and
a device view's depth and coverage copied into the several-view pass's
buffers, device to device; a host `Mesh`, the export path, is staged up and
read back in a batch of its own.

### sensor

the capture *contract*: `ICameraCapture` polled for a
`CapturedFrame` (frames dropped, not queued) and asked `exhausted()` after
an empty poll, since "nothing this tick" from a live device and "nothing,
ever" from a replay are the same empty optional (2026-09-14; non-pure,
`false` by default, so a live driver overrides nothing). A source opened
for raw frames hands them out through `poll_raw()` instead, and
`raw_frames()` says which of the two it serves (non-pure too:
`Unsupported` and `false`) — plus the boundary
math that is silently wrong when guessed — `cv_from_gl_camera`,
`depth_from_registered_color`, `to_canonical`. Links `recon_core` alone;
drivers live with the platform that can build *and* test them. Two
implementers in this tree: `examples/common/replica_capture.hpp`, which
plays a Replica sequence back through the contract, so every example run
produces real frames through it; and **`sensor/orbbec`'s `OrbbecCapture`**
(`VR_WITH_ORBBEC`), a live Femto Mega. Its `poll()` undistorts colour and
*then* registers depth to it — the SDK's registration ignores the colour
lens, so the order is what puts both on one pinhole camera, posed by
`Options::cam_to_world`. It reads the camera's rig sync role
(`waits_for_primary`) and never writes it, and its hardware test opens only
the camera `VR_ORBBEC_TEST_SERIAL` names (the 2026-09-26 decision).
**`OrbbecRig`** reads several synced cameras as one: `poll_set()` hands out
one set per primary frame, with a missing secondary's slot left empty, and
`poll()` hands out the same frames one at a time — one of the two per
`start()`. Opened with `raw`, it reads the same two ways raw, through
`poll_raw_set()` and `poll_raw()`, as `raw_frames()` says, and
`sensor/utils`' `prepare_set` prepares a raw set with a thread and a
`GpuFramePrep` per camera. A secondary's frame near no primary frame
is let go, so a camera
whose clock is off costs its own frames, not the rig's sets. It opens from
the rig's **sync configuration** (`orbbec_sync_config.hpp`, the SDK's
`femto_mega_sync.json` layout) and refuses cameras that differ from it
unless `apply_sync_config` writes it; the lab rig's is
`config/femto_mega_sync.json`, which only the example and tests name. Each
camera's pose comes from the
**calibration file** (`sensor/rig_calibration.hpp`, the family's config
layout: OpenCV world-to-camera `rvec`/`tvec`). Both classes share the
internal `CameraStream`. The rig's hardware test opens only the rig
`VR_ORBBEC_TEST_RIG` names and never writes to it (the 2026-09-27
decision).
`color_codec = Hevc` (`VR_WITH_FFMPEG`) puts H.265 on the wire: each
camera's `HevcColorDecoder` decodes every colour frame, in order, ahead of
the mailbox, as BT.601 full range unless the stream names its matrix, and
posts RGB frames stamped with the RGB mode's profile -- whose calibration
`open` holds to be the H.265 mode's, byte for byte -- so everything after
is MJPEG's path. The SDK hands over every colour frame, depth or not; a
pair without depth is dropped after decoding. A gap in the frame index, or
an empty frame, waits for the next key frame, where the decoder is reset;
pictures come out in display order, each settling its own pair. All of it
is counted in `stats().lost` (the 2026-09-28 decision).
**`sensor/video`'s `HevcDecoder`** (`VR_WITH_FFMPEG`) decodes H.265 access
units to host pictures, `Rgb24` or the `Yuv420` planes with their matrix
and range, plus the stream's transfer and primaries as an optional
`ColorEncoding` (empty when that type cannot name them).
`Options::unlabelled_color` stands in for a stream that names no matrix,
and `reset()` restarts a stream after lost access units. `Auto` takes the
first hardware back end that decodes HEVC (VideoToolbox, asked through
`VTIsHardwareDecodeSupported`; CUDA, then VAAPI on Linux, each by decoding
a built-in clip), probing only as far as it needs, else software.
It moves to software when the hardware refuses a stream, and the pictures
the hardware still held come out too. A named back end is never swapped for
another; it returns `Unsupported` instead. A display window off the coded
corner is cropped on the host, except on VideoToolbox, which cannot be
(FFmpeg hands its pictures over already cut from the wrong corner), so the
decoder reads each SPS there and treats such a stream as refused.
`VR_TEST_HEVC_BACKEND` makes its test require one back end, which is how
CI holds the Linux legs to NVDEC (the 2026-09-27 decoder decision). With
`Options::device` and `VR_WITH_CUDA` (Linux, the CUDA 13 toolkit's
headers; libcuda is loaded at run time), an NVDEC picture stays on the
GPU: copied device to device into a Vulkan buffer CUDA imported (`core`'s
`create_exported_buffer`, on a device that `exports_memory`), and handed
out as NV12 in `DecodedPicture::device`, which a reader acquires from
`VK_QUEUE_FAMILY_EXTERNAL`; any other picture comes to the host, as does
every one after the device path fails (the 2026-09-28 decoded-frame
decision). The decoder says once, as a warning ahead of which
`Options::label` names whose decoder it is, every way a device's
pictures end up on the host: a device path that never opens, one that
fails, Auto's move to software. `OrbbecCaptureStats::host_pictures`
counts them, so a run that should stay on the device can be held to 0.
**`JpegDecoder`** decodes MJPEG's JPEGs as I420, BT.601 full
range: given a device, with `VR_WITH_CUDA`, nvJPEG decodes an 8-bit 4:2:0
one into the same kind of buffer, on the GPU's hardware JPEG engine where
it has one (`backend()`) and its cores for the rest, and libnvjpeg too is
loaded at run time. A JPEG nvJPEG refuses goes to software alone; any
other failure lets the device path go. Anything else decodes in software
to host planes, 4:2:2 converted. On VideoToolbox, given a device that
`imports_metal_textures`, an H.265 picture's two IOSurface planes become
Metal textures imported as `Image`s (`vt_pictures.mm`, Objective-C++),
made once per surface and kept, handed out in `DecodedPicture::image`,
and `JpegDecoder` takes an 8-bit 4:2:0 JPEG to the same images through
VideoToolbox's hardware JPEG decoder (`vt_jpeg.cpp`), leaving one past the
device's image extent to software.
**`sensor/utils`'s `GpuFramePrep`** undistorts a `RawFrame` on the device:
depth sampled at the nearest pixel, colour bilinearly and converted from
Y'CbCr in the same pass, each camera keeping its intrinsics and pose.
`ChromaLocation` follows the picture through `DecodedPicture`, the Orbbec
frame handoff and `YuvImage`: JPEG is centred, and HEVC keeps the decoded tag
with left alignment when unspecified. Host resampling preserves that location;
with libswscale before 9, vertically subsampled inputs go through planar RGB
before packing RGB24 to avoid its packed converter's rounded chroma weights.
The GPU samples the location consistently for host planes, device buffers and
NV12 images. Existing callers that leave the field unset keep left alignment. The
resulting `DeviceFrame` feeds the `Buffer` overloads of `allocate_from_depth` and
`integrate` (and `ColorFrame::buffer`, with `coverage_in_alpha`, since a
pixel the lens maps outside the picture is a 0 word), so nothing is
uploaded and nothing registered. Colour comes as I420 or NV12, as host
planes or already on the device (`YuvImage::device`, with per-plane
offsets and strides), and device planes are bound where they are, from
the first plane, once the batch has taken them over from the queue family
that wrote them (`YuvImage::queue_family`; the 2026-09-28 decoded-frame
decision). NV12's planes may come as images instead (`YuvImage::image`),
which the batch copies into the pass's input. The raw frame's host data goes up
through one batch into device-local inputs, its planes packed into a staging buffer the
pass keeps whatever their strides, and both passes run in the same submit,
the copy timed with them. Kept because several passes allocating a 4K
frame's staging at once made VMA allocate a block for every set (46 ms a
rig set on an RTX 5090, 4.8 ms kept). The frame *holds* its device-local
buffers, and `prepare` reuses one only once no frame does, so a frame kept
past the next is still itself; the whole frame is checked before anything
is uploaded, a depth range from 0 included. `depth_within_color` (off by
default; `rig_viewer` turns it on) zeroes depth outside the colour camera's
view, by the colour pass's own coverage test, so nothing is fused that no
colour camera can colour. `OrbbecCapture` opened with
`raw` hands out `RawFrame`s through the contract's `poll_raw`, lenses and
the depth-to-colour extrinsic from the factory calibration, the planes
converted by the matrix and range the stream codes them in, and
`fuse_orbbec --gpu` fuses them. Given `OrbbecStreamOptions::device`, its
H.265 colour is decoded onto that device and stays there, the picture
carried through the mailbox by an SDK frame whose bytes only name it, so
a copy of the frame owns nothing (`picture_frames.hpp`); for raw MJPEG it
streams the camera's JPEGs, and each camera's `JpegColorDecoder` decodes
them on a thread of its own onto the device. Its hardware test holds both
codecs
to the SDK's own undistortion and registration on a still scene (the
2026-09-28 GPU pre-processing decision).

### codec

four of five PRs in (2026-09-26 lists them). The defaults
are room0's, and provisional until the per-band quantization study: K = 64
with one step of 0.2 for DC and AC alike (2026-09-27). At 1 cm that is
11.0 B/block (374x under raw) since frame v2 (2026-10-01), 0.64 mm
accuracy RMS, and host coding inside a 30 fps frame interval. The public API
is `CodecParams`, **`Encoder`** (`encoder.hpp`) and **`Decoder`** with
`read_frame_info` (`decoder.hpp`). `Encoder::encode(grid)` takes the map's
own active list (`compact_active_blocks_on_device`, so a fuse's list is
reused and its own kept for the extract after), keeps the blocks with an
observed voxel (`DctTransform::observed`, on the device), sorts them by
(z, y, x), transforms them, and writes the frame. The same content gives
the same bytes whatever the hash table's order.
`Decoder::decode(frame, grid)` leaves the caller's grid holding exactly the
frame. It merges the grid's sorted active set with the frame's
coordinates, removing, allocating, and keeping shared blocks in their
slots, then rewrites every voxel's `tsdf` and `weight`, and stamps
`changed` only on a block it leaves different. Attributes, geometry (exact
`voxel_size` / `trunc_dist`, off the header), parsing and the heap are all
checked before the grid is touched. The attributes must be `tsdf` and `weight` and
nothing else (`VoxelBlockGrid::attribute_count`), since a kept block would
carry any other one stale. A grid too small for the frame is `OutOfMemory`,
whether its heap has too few slots or its hash table cannot place the
blocks. Both are recovered by `resize` and decoding again; the library never
grows a grid. Lock contention that outlasts four rounds is `IoError`, never
`OutOfMemory`, and a free heap that refuses a removed block is
`InvalidArgument`. A failure after the grid has changed leaves it holding neither
frame until a decode succeeds. Both classes report `StageMetrics`
(`"codec encode"` / `"codec decode"`). Their breakdown rows share no name
except the map's own `"  ..active set"`. The private pieces under
`src/volumetric_kit/recon/codec/` are the `DctTransform`, the rANS
reference coder and the intra frame. The transform takes a
`volume::BlockList` to a `DctBlocks` — K quantized 16-bit coefficients per
block in 3-D zigzag order, a 16-word observed mask, and the params and
`trunc_dist` they were made with — and back, each call one `CommandBatch`
over device-local buffers. The SDF is normalized by `trunc_dist`
before the transform and the steps are fractions of it, so the inverse
refuses a `DctBlocks`
whose `trunc_dist` is not its grid's. `CodecParams::validate` refuses a step
small enough for the ±32767 clamp to engage (√512 / 32767), and one past
`kMaxStep` (64), where every coefficient is 0 and a decoded one could
overflow to infinity. The forward
never reads an unobserved voxel's `tsdf`: it fills each one from the nearest
observed voxel along x, then y, then z, since a fused block's zeros there
are a step K = 32 cannot hold, and they decoded a partially observed block
with 2.8x the error. The transform refuses any block size but 8; a list
`VoxelBlockGrid::check_block_list` refuses, the O(1) checks it shares with
`mesh`; and, on the device with one hash probe per block, any entry whose
coord the grid does not hold, which the inverse then writes nothing into.
Each block is found by that probe and its ptr never read, so the decoder
lists blocks it has just allocated by coordinate alone. "Observed" is
`volume::kObservedWeight`, the threshold the mesher reads too. The inverse
writes weight 1.0 on observed voxels and a fresh block's zeros elsewhere.
The frame (`bitstream.hpp`, the 2026-09-27 entry) is segments of R sorted
blocks (default 64). Each segment is one independent rANS stream
(`rans.hpp`: a 32-bit state, 16-bit words and 12-bit probabilities, integer
only, the reference the GPU kernels must match byte for byte). Every integer
is coded as a class from a fixed per-frame table plus one raw field, its
sign the low bit. A partial mask is coded a z plane at a time and a line
at a time, each against the one before it (v2, 2026-10-01: the mask had
been half of room0's bits). The format
requires strictly increasing coordinates, so a decoded list is duplicate-free:
the delta code cannot step backwards within a segment, and the reader checks
each segment's raw first coordinate against the one before. A flag bit other
than `kSectionRequired` is reserved, so a known section that sets one is
refused. `read_intra_frame` never reads outside its buffer, and it takes the
caller's `max_blocks`, because a frame's size cannot bound its block count
(probability-one blocks cost no bits). It checks that limit only once the
count agrees with the segment table, so a sound frame past it is
`OutOfMemory` and a corrupt count stays `InvalidArgument`. The limit bounds
its allocation too, at
under 1.1 KB per block, and every size is checked in 64 bits so a 32-bit
build refuses rather than wraps. `DctBlocks` lives in `dct_blocks.hpp`, so
the host-only frame never includes Vulkan. `RansReader::finish` is a
**consistency** check, not an integrity one. It cannot see a flipped raw
bit, nor a symbol swapped for one of equal frequency, so integrity is the
transport's.

### eval

`MeshDistance` (point-to-surface distance up to a reach,
through a hash of cells half the reach on a side, searched nearest first
and pruned by distance, over a **copy** of the triangles),
`compare_meshes` giving accuracy, coverage and an optional F-score, and
`ReferenceMesh`, which indexes a reference once so a sweep can judge many
meshes against it. They refuse, with `Status`, what would read out of
bounds, overflow or mean nothing:
- a bad reach, or indices out of range;
- a corner that is not finite or past the cell keys' range;
- a reach so small that the triangles would average more than
  `kMaxCellsPerTriangle` cells each;
- a threshold past the reach.

The surface is every triangle but one collapsed to a point, which is what
an incremental extract retires a triangle to. The points measured are the
vertices those triangles use. The closest point is the face projection
when it lands inside, else the nearest edge, so a degenerate triangle
counts as the segment it collapses to and a thin one is measured to float
rounding, where Ericson's region test lost it now and then. A `stride` picks
vertices by a hash of their position, so the figures reproduce whatever
order marching cubes' atomics emitted the mesh in.

### io

`io::load_color_packed` decodes JPEG/PNG to top-left row-major encoded RGB
bytes in the existing `R | G<<8 | B<<16` packing. It performs no implicit
linearization or profile conversion; the capture adapter declares the input
encoding. `io::load_depth_metres` requires a genuine 16-bit single-channel
PNG and a finite positive units-per-metre divisor. Zero samples stay zero.
Expected dimensions are explicit so a camera cannot silently use different
intrinsics. The PNG writer takes already-encoded RGBA8 and opens its output
only after encoding succeeds; PLY export converts linear mesh colors to
canonical sRGB bytes (clamped, NaN as 0) and preserves triangle indices.
Errors name the file and backend reason. Public functions validate their input
and translate backend exceptions that reach them into `Status`/`Result`
(tinyply's writer is `noexcept`); private third-party types never appear in
installed headers.

`recon_io_assimp` adds `io::load_mesh` and its owned `TriangleMesh` position /
index container. Assimp handles format decoding and triangulation. Recon
walks every node instance with composed transforms and corrects triangle
winding when the determinant is negative. Exactly coincident positions join
across UV/material seams, with no tolerance-based merging. This is geometry
import, not topology repair: degenerate and duplicate triangles are retained,
and manifold/sign checks belong to the consumer. Non-triangle primitives,
animation, skinning, morph targets and invalid transforms on mesh instances
are refused; nodes without meshes are not checked.

Mesh coordinates retain the imported scene's scale; neither a metre unit nor
a target height is guessed. No Assimp normalization or handedness flags are
enabled, and importer unit/up-axis conversions are switched off where the
installed Assimp allows: before 5.3 it still applies Collada's `<unit>`, and
5.4.x converts FBX regardless. This is a host boundary for file input and output, separate from
GPU-resident reconstruction. The default build has no Assimp dependency;
`VR_WITH_ASSIMP=ON` finds an installed package and exports a separate target.
The image and PLY backends use the existing pinned stb and tinyply sources,
now private implementation dependencies of the installed I/O library.

## Examples

(`examples/`.) Five of the six poll their frames through
`sensor::ICameraCapture&` — the fuse loop never learns what is behind it.
`rig_viewer` reads the rig's raw *sets* (`OrbbecRig::poll_raw_set`) instead,
as `fuse_orbbec --gpu --rig` does, since the contract has no set, and both
fuse each set through `fuse_device_frame.hpp`'s `fuse_set`: every camera's
band in one allocation, then every camera in one integrate. The
four dataset examples take `ReplicaCapture` as the source: frame cap, stride
and the depth gate are its options, stamped on each frame it hands out, and its
disk probe at `open` visits only the frames those options select. An empty
poll is retried after a millisecond until the source reports itself
`exhausted()`, so a live driver in the same construction site waits and the
replay ends. Each frame fuses
through `examples/common/fuse_frame.hpp` (the one allocate-and-grow-then-
integrate loop, carrying the frame's encoding declaration across, into the one
grid layout `grid_layout.hpp` defines, which its `create_fusion_grid` builds and
`codec_replica`'s player shares), and a frame kept past the next
poll — `fuse_render`'s keyframe, `fuse_viewer`'s newest fused frame for its
final texture pass — is copied into an
`RgbdFrame` of its own (`examples/common/rgbd_frame.hpp`, the type the
reader decodes into; `CapturedFrame` is its view), never borrowed: the empty
poll that ends a replay is a poll. `fuse_replica`
runs the spine on a posed
Replica-SLAM RGB-D sequence and writes a PLY; `--incremental` drives the
changed-only extract and implies `--device-extract`, and `--dirty-every`
reports the changed and re-mesh fractions over windows of its own, each
keeping its own tick, so the two run together. Behind the off-by-default
`VR_BUILD_VIEWER`: `fuse_render` writes a headless colour PNG (seam A — it
builds two devices by design), and `fuse_viewer` opens a live window on one
shared `VkDevice`, fusing on a background thread, drawing recon's buffers
directly, and carrying the two-panel perf overlay. The four dataset examples
take `--preload`, which makes the loop measure compute rather than the
JPEG/PNG decoder.
The live counterpart is its own example, not a `fuse_replica` flag:
**`fuse_orbbec`** (`VR_WITH_ORBBEC`) fuses an `OrbbecCapture` through the same
`fuse_frame.hpp` and writes a PLY after `--frames` frames; `--rig sync.json`
fuses the rig as an `OrbbecRig`, posed by `--calibration`. With `--gpu` the
source serves raw frames, which the loop learns from `raw_frames()`, and each
is prepared by `GpuFramePrep` and fused through `fuse_device_frame.hpp`, the
one header that pulls in `sensor/utils`.
**`rig_viewer`** (`VR_BUILD_VIEWER` with `VR_WITH_ORBBEC` and
`VR_WITH_FFMPEG`) is `fuse_viewer`'s live-rig sibling: raw sets prepared,
fused and textured from every camera on the GPU, and the atlas filled by
device copies recorded in gfx's frame (the 2026-09-29 decision), fusing
depth only inside each colour camera's view unless given `--all-depth`, and
texturing a camera a set lacks from its last frame, a fallback view
(`--hold-ms`). The two viewers share `viewer_common.hpp`: the teardown guards, and the render side of
the mesh ring.
**`codec_replica`** fuses a Replica sequence as `fuse_replica` does, and
streams the growing grid through the codec: every `--encode-every` frames it
encodes, then decodes into a player grid built from `read_frame_info` and
sized for that frame's blocks. It reports bytes, the bitrate at the coded
frame rate, and both calls' stage rows. It then judges the last decoded
surface against the source's, mesh to mesh, with the `eval` tier: accuracy,
coverage and the F-score at half a voxel, since a fused TSDF is projective
and sampling it would measure the fusion's bias too. `--sweep` prints the
rate–distortion table the defaults are chosen from. Its `main.cpp` stays the
example's story; the stream, its report and the sweep are
`examples/common/codec_stream.hpp` and `codec_sweep.hpp`.

## Next work

**Incremental mesh extraction has landed, all three stages** —
`MarchingCubes::extract_device_incremental`, over the span table of the
2026-08-11 table decision and the `changed` stamps of the second 2026-10-01
one (which replaced the 2026-08-09 dirty flags); read both,
plus the two 2026-08-11 dispatch entries (the second reverses the first's
`share_vertices` clause), before touching it. It runs under `share_vertices`,
which is the configuration the memory-bound consumer wants — unsharing to avoid
it cost 4 177 MB against 33 MB on the iPad. Two things it does
**not** yet do, and both are `TODO(mesh)`s: it runs at `slot_count == 1` only,
so it is silently off in `fuse_viewer` (extending the ring is the open design
question — release-gated range reuse, or copying the retained run into the newly
claimed slot); and the ~4x win the 2026-08-09 entry sized it for is **still
unmeasured**, because room0 re-meshes 81.67% of its blocks per window and so
caps at ~1.22x. The iPad's 25% dirty rate is where the number lives, and
`fuse_replica --incremental` now reports `incremental` / `remeshed_blocks` so a
run cannot quietly measure the fallback instead. Beside it: **view-culled meshing has landed as a
library API and has no in-tree consumer yet** — `extract_device` takes a
`volume::BlockList`, `make_frustum_planes` takes a `view_proj`, and what is
missing is a caller that culls: `fuse_viewer` would have to publish its render
camera across the fusion-thread boundary, and the scanner that motivates it
lives in `volumetric_kit_ios` — a `TODO(mesh)` on the overload. So the win is
correct and unquantified — expect it roughly linear in the visible fraction on
the `ExtractTimings` rows that scale with the active set (the upload, the
dispatch, the arena; `readback_ms` and `descriptor_ms` are per-call constants
and will read flat), and quote nothing until a run says so. Beside that:
first-class glTF/GLB export in `io` and the gfx-vertex converter. PLY export
has moved from the example into the validated I/O module. On `mesh`, the greppable
`TODO(mesh)`s: cross-block vertex sharing, per-vertex normals, extending
incremental extraction past one slot, revisiting degenerate retirement if
relocation proves common rather than rare — and, the sharing kernel's form of
that same question, recording a block's *reservation* beside its live span so a
surface oscillating around a threshold stops relocating on every up-tick —
and `ExtractTimings`' device half — which must
bracket several dispatches in **one** timed submit, since a timed submit costs
~0.13 ms on MoltenVK and four of the six phases run under that. On `texture`,
the `TODO(texture)`s: packing the multi-view atlas on the GPU into an image gfx
samples directly (it needs `core` images), keeping a static keyframe set's
depth and coverage in the pass between calls, blending views at their seams,
and a per-triangle tile
index in gfx so a shared mesh can be textured from several views; and the
multi-keyframe post-scan atlas, which the multi-view path can carry. On
`core`: the `TODO(core)` for `VK_EXT_memory_budget` on `Device::create`,
which would turn the viewer's heap gauges from VMA heuristics into driver
truth. The debug-utils labels that TODO
sat beside **have landed** (2026-08-30) — on the *kernel* rather than the span,
which is the correction that entry records.

**On `tsdf`**, mesh → TSDF has its two modes (2026-09-27), a second way into
the grid the codec encodes: a mesh sequence converts frame by frame. Both modes
cost the same, measured on an 81 920-triangle sphere at scan density (M5 Max,
Release): 12.8 ms to write, 10.8 ms of it on the GPU, after 14.2 ms to
allocate; 12.6 ms and 10.6 ms resident (the 2026-09-28 residency decision).

**On `sensor`**, each a `TODO(sensor)`: the GPU pre-processing has landed for
one camera (`GpuFramePrep`, the 2026-09-28 GPU pre-processing decision: at 4K
it takes the host from 15.5 ms of undistortion and registration a frame to
none, and the run's CPU eightfold down) and for the rig's raw sets. NVDEC
and nvJPEG hand their pictures over on the device, and VideoToolbox both
kinds, and a raw Orbbec frame's colour stays on the device over either
codec; what is left there is the colour kernel reading Apple's plane images
directly, measured first (`undistort_color.comp`); and processing a host rig
set's frames in parallel, one thread per camera, rather than the ~11 ms one
after another costs for four (`orbbec_rig.cpp`). For H.265: the camera's
encoder settings, its key-frame interval above all, which sets what a lost
frame costs (`camera_stream.cpp`),
and software decoding at 4K, one thread with little headroom
(`hevc_color.cpp`). The rig's next consumer is calib's viewer, showing its
synchronised sets.

**Device residency, the steps after `core`** (the 2026-09-28 residency
decision ranks them): `volume`, `tsdf`, `mesh` and `texture` are resident,
and `GpuFramePrep` stages its raw frame in one submit, the rig's raw sets a
thread per camera, and the hardware decoders' pictures stay on the device;
next the examples (the codec's transform is resident since 2026-10-01). The
benchmark kit that sized
them sits on the home box in `~/recon-bench` (a throwaway allocator patch
behind environment variables); re-measure there after each.

**Measure the phases before choosing the optimisation.** Three independent
guesses at this pipeline's bottleneck have been wrong, each corrected by an
`ExtractTimings` breakdown that pointed somewhere else entirely — see
[DECISIONS.md](DECISIONS.md#measured-lessons).

**Performance work on the live rig path is planned in [PERF.md](PERF.md)**:
the path camera to screen, its submits and bytes per set, the measured
baseline, the rules for that work, and the claimable work items (batching
a set's cameras, deduplicating allocation, per-remesh copies, the
discrete-GPU gaps). Read it before optimising anything on that path, and
update your item's row in the PR that lands it.
