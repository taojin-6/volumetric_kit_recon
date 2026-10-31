# Design

This is the detailed architecture and implementation reference for
`volumetric_kit_recon`. [AGENTS.md](AGENTS.md) contains the shared working
instructions; [DECISIONS.md](DECISIONS.md#decision-index) contains the dated
rules, rationale, measurements, and review findings. Read the sections relevant
to the task, and update them with the code when a contract changes.

- [Tiered architecture](#tiered-architecture) and [package names](#naming-conventions-use-these-consistently)
- [Interop and device ownership](#the-interop-seam), [Vulkan gotchas](#key-gotchas-verified), and [color](#color-space)
- Implementation: [core](#core), [tsdf](#tsdf), [texture](#texture), [sender](#sender); the voxel grid, meshing, the codec and streaming are grid's ([its DESIGN.md](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DESIGN.md)), and the camera side sensor's ([its DESIGN.md](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DESIGN.md))
- [Examples](#examples) and [next work](#next-work)

The tier descriptions and next-work notes below retain the implementation
state recorded through 2026-10-01. They distinguish current behavior from
provisional defaults and unmeasured work; confirm the affected code when using
an older measurement to make a new decision.

## Purpose and scope

`volumetric_kit_recon` is the compute backend that builds and compresses
volumetric geometry. Its pipeline is: **posed depth/RGB-D frames → sparse TSDF
volume (voxel-hashed) → extracted geometry (mesh) → optional compressed
bitstream → handoff to a renderer.**

The voxel grid and what reads it -- voxel hashing, a mesh's distance field,
marching cubes and the mesh containers, mesh files, quality measurement, the
codec, the `.vrst` container, live serving and the codec stream -- are
`volumetric_kit_grid`'s, a repository of its own that recon builds on;
recon fuses depth into its grid, textures its meshes and sends what it codes.

It is deliberately *not* a renderer. Rendering is owned by the sibling library
`volumetric_kit_gfx` (Vulkan/MoltenVK). Keeping them separate means each is
independently useful: a headless capture/compression service links only the
reconstruction compute, never the renderer, and a viewer links only the
renderer. They meet only at the interop seam.

## The family

- **`volumetric_kit_recon`** (this repo) — capture → fusion → meshing →
  compression as **Vulkan compute** (GLSL → SPIR-V; MoltenVK on Apple), one code
  path across Linux / Android / macOS / iOS. Produces meshes / volumes
  / compressed TSDF bitstreams.
- **[`volumetric_kit_grid`](https://github.com/taojin-6/volumetric_kit_grid)** — the voxel grid,
  meshing, mesh files, quality measurement, the codec and the `.vrst`
  stream and its live serving, on volumetric_kit_core alone.
- **[`volumetric_kit_sensor`](https://github.com/taojin-6/volumetric_kit_sensor)** —
  cameras, capture and the device-frame contract, on volumetric_kit_core
  alone.
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
  `volumetric_kit::recon::`. The `core` tier's headers stay in
  `volumetric_kit::recon`: recon declares no `core` namespace, which would
  capture every `core::Status` its code writes.
- Headers: `include/volumetric_kit/recon/<tier>/…`.
- Macros: `VR_` prefix (`VR_CORE_API`, `VR_BUILD_VIEWER`); the error and
  contract macros are volumetric_kit_core's, used under its names (`VKC_TRY`,
  `VKC_ASSIGN`, `VKC_CHECK`, `VKC_VK_TRY`).
  Deliberately *not* the `VK_` prefix — that belongs to Vulkan. (The prior
  engine's `VK_DEVICE_HOST`-style macros are renamed `VR_*` on port.)
- CMake: fetched by commit (`FetchContent`); component targets
  `volumetric_kit::recon_core`, `…_tsdf`, `…_texture`, `…_sender`,
  `…_interop` (+ later `…_track`); umbrella alias `volumetric_kit::recon`.
  The camera and sensor targets are the sensor package's
  (`volumetric_kit::sensor_*`, [its names](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DESIGN.md#names)).
- grid's names are its own: `volumetric_kit::grid`, `vgrid::` in docs,
  `include/volumetric_kit/grid/<tier>/…`, `VGRID_` macros and
  `volumetric_kit::grid_<tier>` targets ([grid's
  conventions](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DESIGN.md#naming-conventions)).

## Tiered architecture

Strict left-to-right dependency rule: a tier may depend only on tiers to its
left. No upward includes.

`core` → `tsdf` → `texture` → `interop` (later: `track`), every tier free
to link grid's tiers below it and the sensor package's frame tier: `tsdf`
fuses into grid's `volume`, `texture` textures grid's `mesh`. `sender`
comes last: it links grid's `codec_stream`, `tsdf` and the sensor
package's frame contract (`sensor_frame`). Neither grid nor the sensor
package includes anything of recon's ([grid's tiers](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DESIGN.md#tiered-architecture)).

- **The sensor package** (`volumetric_kit_sensor`, its own [DESIGN.md](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DESIGN.md))
  — the camera vocabulary, the sensor interface and its drivers, the video
  decoders, GPU frame preparation, the sensor array and image files.
  recon's libraries link its `frame` tier alone: `DeviceFrame`, the pinhole
  cameras, `ColorEncoding` and the GLSL pinhole.
- **`core`** — recon's own vocabulary every tier trades in: the GLM math
  aliases, `device_requirements()` and `log_message`. The Vulkan foundation
  under it (instance, device, VMA allocator, buffers and images, `CommandBatch`, `ComputeKernel` and
  `KernelSetBuilder`, timers) and the `Status`/`Result` idiom are
  volumetric_kit_core's, used under its names. Vulkan is reached through the
  core's one umbrella header (`core/vulkan/vulkan.hpp`) — no other code
  includes `<vulkan/...>` directly.
- **`tsdf`** — TSDF integration compute shaders (classic + dynamic), the
  per-camera `ChangeDetector` that flags pixels whose depth changed for
  real, the `Fuser` that fuses a set of frames (grow, allocate, integrate),
  and the grid it starts from (`create_fusion_grid`).
- **`texture`** — projective texturing: fills the mesh's per-vertex `uv0` with a
  posed camera's image coordinates where it has line of sight (per-vertex-color
  fallback elsewhere), or with several cameras' coordinates into an atlas of
  their images, a compute pass. A colour image is registered to its depth
  camera or taken by a colour camera of its own, whose line of sight the
  depth map decides too; depth may be on the host or the device.
- **`sender`** — a sender's set step: a prepared set fused, freed on
  schedule and its frame coded in two batches through the codec stream's
  halves (`SetSender`), and the fuser's inputs for prepared frames
  (`fuse_inputs`) (2026-10-26). It prints nothing.
- **`interop`** — the handoff to `volumetric_kit_gfx` (below).

## Backend strategy: one Vulkan path

Compute runs as **Vulkan compute shaders** (GLSL → SPIR-V), with **MoltenVK** on
Apple — exactly the cross-platform strategy `volumetric_kit_gfx` uses for
rendering. One code path serves Linux, Android, macOS and iOS, built with GCC
or Clang, rather than a Metal + CUDA split (Windows and MSVC are not
supported: the 2026-10-06 platforms decision). Vulkan is reached through a single umbrella header so
the loader choice (link-time loader now, volk later for iOS/Android) stays a
detail. Because the renderer is also Vulkan, the two libraries can share a
`VkDevice` — which is what makes the geometry handoff cheap (see the interop
seam below).

Native CUDA is an optional NVIDIA accelerator under this baseline (the
2026-07-04 decision); it must stay numerically in lockstep with GLSL.

## Error handling

Build recon with C++ exceptions enabled. Backend handlers and checked host
allocation paths translate failures into `Status`; this does not imply that
every host allocation in the library is guarded. Frame preparation retains
its staging-allocation handler, and fusion retains its growth and rehash
handlers (2026-10-10).
Fallible calls return `Status` (success or an error domain + message) or
`Result<T>` (a value or a `Status`), both `[[nodiscard]]`. `VKC_TRY` and
`VKC_ASSIGN` remove the check-and-propagate boilerplate. Programmer errors
(precondition violations) fail fast via `VKC_CHECK` (log + abort), distinct from
recoverable runtime failures. `Status` is intentionally backend-neutral: a
`Code::Backend` failure names its backend (`Status::Backend`) and carries that
backend's code as a generic `int64_t`, never a Vulkan or CUDA type, so the
same idiom serves every tier; the core's `vk_result.hpp` turns a failed
`VkResult` into one (`vk_error`, `VKC_VK_TRY`), and `Status::with_context`
prefixes a message without losing its domain, backend or detail.

All of it is volumetric_kit_core's (the 2026-10-03 decision), and recon writes
it under the core's names (2026-10-04): `core::Status` in recon's namespaces,
`volumetric_kit::core::Status` to a consumer, so a recon error is the same type
as calib's and gfx's. Diagnostics go through the core's one process-wide log sink;
recon's `log_message(level, message)` tags them with source `"vr"`, which the
default sink prints as `[vr <level>]`, and an application's handler receives
`(level, source, message)`.

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
  the first milestone. gfx is to draw points from `DeviceFrame` depth in place
  (GF5); recon builds no point cloud (the 2026-10-16 depth-sharing decision).
- **B — Shared Vulkan resources (zero-copy; the live target).** recon writes the
  mesh/atlas into a `VkBuffer`/`VkImage` on a **single `VkDevice` shared with gfx**
  (one process) and gfx draws it directly — no external-memory import. Realized by
  the create/adopt device seam: a neutral bootstrap builds one device from both
  libraries' merged `DeviceRequirements` (the core's `SharedDevice`, real since
  2026-08-02); two queues from one graphics+compute family would avoid any
  queue-family ownership transfer, but MoltenVK offers no such family, so on
  Apple the buffer is cross-family and pays `VK_SHARING_MODE_CONCURRENT` or an
  explicit release/acquire — which
  `MarchingCubesConfig::queue_families` now expresses (2026-08-03); see that
  decision and the MoltenVK queue gotcha. The handoff is a ring of mesh/atlas
  slots with variable topology drawn indirectly, both real as of 2026-08-03 —
  but released by a **host-side report** rather than the intra-device timeline
  semaphore originally specified, since a cross-library GPU wait deadlocks
  against a swapchain rebuild on the shared queue. All four blockers are
  settled, and **`fuse_viewer` is the consumer that draws it** as of 2026-08-08
  — `pipelines::LiveMesh` over recon's arena, index run and indirect command,
  with the ring released by generation as its frames retire, through
  `mesh::MeshExchange` (2026-10-08). `fuse_render` stays on
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
(`AdoptedDevice`). recon and gfx both take it, and `DeviceRequirements`, from
volumetric_kit_core, never from each other, so the integrating application hands
each the core's payload unconverted. That application owns the shared
instance/device and a small bootstrap that computes the union (the core's
`SharedDevice`) — the only place that knows about both libraries, so the two
stay true independent siblings. The neutral bootstrap prefers separate queue
families, using concurrent sharing or explicit ownership transfer for shared
resources; when that is unavailable, the libraries share a queue behind a
caller-provided submit mutex. MoltenVK's one-queue-per-family constraint is
covered under [Key gotchas](#key-gotchas-verified).

The mesh/atlas are ring-buffered, and slots are recycled by the host's report
as gfx frames retire. A cross-library GPU wait can deadlock against a swapchain
rebuild, so the release contract is host-side (the 2026-08-03 decision).
Marching cubes writes a `VkDrawIndexedIndirectCommand` for variable topology;
the extractor reads its few result words from a ring slot, while gfx draws
the geometry buffers directly.

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
- **MoltenVK drops `precise` on a vector times a scalar.** Its Metal
  translation (1.4.2) keeps NoContraction on `OpFMul`, `OpFAdd` and `OpFSub`
  (as `spvFMul`, `spvFAdd`, `spvFSub`) but not on `OpVectorTimesScalar`, so
  `precise vec3 p = v * s;` reaches Metal as a plain product that fast math
  may fuse into what follows. Splat the scalar, `v * vec3(s)`, as the
  marching-cubes corners do.
- **Vulkan via the link-time loader through one umbrella header**
  (the core's `core/vulkan/vulkan.hpp`) — never `#include <vulkan/...>` directly,
  so adopting volk later for the iOS/Android loader stays a one-header change.
- **Host buffer layout must match the shader.** Host POD structs (`HashEntry`,
  `BlockIndex`, …) and their GLSL mirrors must agree byte-for-byte, so the shaders
  read them through **scalar block layout** (`GL_EXT_scalar_block_layout`; Vulkan
  1.2 core, MoltenVK-supported), *not* `std430` — `std430` 16-byte-aligns a
  three-component vector, so an `std430` `HashEntry` puts `pos` at offset 16 in
  32 B where the host packs it at offset 4 in 20 B (see the 2026-07-05 ABI
  decision). Scalar layout is the C/CUDA layout, so one struct maps 1:1 across
  CPU / GLSL / CUDA. The `static_assert`s on the C++ side guard only the host
  packing; keep the `layout(scalar)` GLSL definitions in lockstep.
  `device_requirements()` asks for the `scalarBlockLayout` feature, and every
  `create` that builds kernels refuses a device that did not enable it
  (`check_device_requirements`); `vr_embed_shaders` validates the emitted
  SPIR-V with `--scalar-block-layout`.
- **GLSL compute is the baseline; CUDA is the optional accelerator.** In the
  Vulkan path, warp/wave tricks become Vulkan subgroup ops and device atomics use
  GLSL atomics; the prior engine's kernels are a reference for the *algorithm*,
  rewritten in GLSL. The native-CUDA accelerator (2026-07-04) keeps its warp
  intrinsics/atomics but must stay numerically in lockstep with the GLSL path.
- **Device locality and host visibility are independent flags.** Non-local
  host memory costs PCIe traffic on a discrete GPU, a cost hidden by Apple's
  unified memory. A type with both `DEVICE_LOCAL` and `HOST_VISIBLE` is still
  local (UMA or BAR). A buffer's actual memory type is recorded in
  `Buffer::memory_info()`. The hash table's
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
  A table the host reads is device-only like the rest, and comes back by
  readback in the batch that wrote it (2026-10-04). A
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
| Sensor `RgbdFrame` colour (8-bit) | encoded, as declared | as delivered; the driver declares the curve and converts nothing |
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

A driver declares the encoding it produces and never converts: the sensor
package's `ColorEncoding`, which rides beside the camera
([its design](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/DESIGN.md#declaring-an-encoding-not-converting-it)).

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

A historical host arithmetic experiment, described in the 2026-08-02 color-space
decision, used `max_weight = 5` and a 2 m observation: the mean stops
**~10 codes short of its target, uniformly across the range** — 0→64 settles at 55, 0→255 at 245 —
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

- `tsdf/shaders/tsdf_integrate.glsl` decodes both operands, blends, re-encodes.
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

The curve's host implementation for the sensor boundary is the sensor
package's `frame/color_space.hpp`. The voxel colour attribute's format, the curve
included, is grid's `volume`'s (2026-10-28):
`volume/shaders/voxel_color.glsl`, which fusion writes and the mesher reads
through, exported to consumers' kernels, and its host half
`volume/voxel_color.hpp`. grid's PLY writer encodes with a copy of its own.
`grid_volume_voxel_color` holds the GLSL to `voxel_color.hpp` and that to the
published curve; `grid_io_ply` holds the PLY copy to `voxel_color.hpp`; and
`recon_tsdf_voxel_color_space`, in tsdf, which links both, holds
`voxel_color.hpp` to `color_space.hpp`, so grid takes no colour vocabulary
from the sensor package.

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
- **Hardening applied on port:** replace glog/fmt with the pluggable log
  handler; adopt `Status`/`Result` repo-wide;
  rename `VK_*` device macros → `VR_*`; break the `ReconstructionPipeline`
  god-object so each tier has a standalone API; drop committed build artifacts,
  hardcoded paths, and machine-specific config.
- **Drop:** the prior engine's own renderer (gfx replaces it) and its SwiftUI
  demo app (reference only). Its sensor driver framework is rebuilt behind a
  clean `IRgbdSensor`, now the sensor package's — the **interface** is
  rebuilt there; each *driver* is placed by the buildable-and-testable rule (see
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

A consumer fetches recon by commit (`FetchContent`) and links its per-tier
targets; recon has no install rules or package config (the 2026-10-22
decision). recon fetches sensor and grid by commit after the core, and its
libraries link sensor's `sensor_frame` alone ([README](README.md#building)). Quality gates
(warnings-as-errors, formatting hooks, sanitizer and test CI) are wired from the
start, not retrofitted.

## Implementation reference

Landed and proven on MoltenVK, left to right: `core` → `tsdf` → `texture`,
on grid's `volume` and `mesh` ([grid's reference](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DESIGN.md#implementation-reference)),
and **interop seam B** — gfx drawing grid's mesh buffers with no host round
trip.
[DECISIONS.md](DECISIONS.md) carries the *why* behind anything here that looks
arbitrary; it usually isn't.

### core

The Vulkan foundation below is volumetric_kit_core's vulkan tier
(DECISIONS.md, 2026-10-04): recon uses its types under the core's names, and
what is recon's own is `device_requirements()` (what recon's kernels need of a device)
and the vector types. The description
stays here because recon's tiers are written against it; the contract is the
core's headers.

the Vulkan compute foundation: VMA `Allocator`, RAII `Buffer`,
`ShaderModule`, descriptor + `ComputePipeline` wrappers, the `ComputeKernel`
bundle + `KernelSetBuilder`, the shared-queue-safe
`Device::submit_single_time` dispatch (safe from several threads at once:
each submit records on a command pool of its own, kept with its fence for
the next, and only the queue submit is locked; a kernel's set, a buffer and a `GpuTimer` stay the caller's to
keep to one thread), and the shared `dispatch()` /
`group_count` / `mapped_storage_buffer` / range-guard helpers of `core/vulkan/compute_util.hpp`
— `StorageInput` among them, the host array (staged onto the device in the
call's batch) or device buffer a call binds at its image's exact range.
`MemoryUsage::DeviceOnly` requires device-local memory the host cannot map
where the GPU has any (else the device-local pool, never the BAR window):
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
device overload. Transfer-only sources and the documented small-parameter
exception do not gain a blanket residency restriction.
**`CommandBatch`** (`core/vulkan/command_batch.hpp`)
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
Vocabulary: `Status`/`Result`, the GLM aliases, and the core's
`core/base/stage_metrics.hpp` — the rows
every tier reports timings in: host time (`cpu_ms`), the part of it blocked on
the device (`wait_ms`, which every core wait reports to the scopes open on its
thread), and a device half (`gpu_ms`, with `device` saying whether it was
measured, ran untimed, or cannot be). A timed batch tiles its device time: a
timestamp at each change of tag, so a stage's time holds every command
recorded inside it, and a tile counts to every scope open around its tag
(the 2026-10-23 entry). `kBreakdownPrefix` marks a row its parent's row
already contains, so both totals skip it; `in_stage()` is how a callee
reached from both positions knows which of the two it is writing. A tier
opens one **`GpuStageScope`** per call (`core/vulkan/gpu_timer.hpp`): it times
the host span, is what `dispatch()` takes to tag the device one, and
publishes all three halves in its destructor, so no early return can strand
a span. Timing is unavailable, never an error — a query pool that will not
allocate degrades like a family with no timestamps, and a batch that finds
no free range runs untimed. `Device::create` enables `scalarBlockLayout`
because `device_requirements()` asks for it, recon's
`check_device_requirements` refuses a created or adopted device without it,
and both record the queue family's `queueFlags`. `create_exported_buffer` (`core/vulkan/external_memory.hpp`) makes
a buffer CUDA imports, and `create_exported_image` a single-plane image,
whose `CommandBatch::acquire` and `release` carry its layout across, on a
device that `exports_memory()`:
`VK_KHR_external_memory_fd`, which `create` enables where offered and
`requirements()` names as optional (`external_memory`); beside it,
`find_memory_type` is the type a resource bound by hand takes, the first
that fits, which Vulkan's ordering makes the plainest; it never returns a
protected, lazily allocated or AMD device-coherent type. `Image`
(`core/vulkan/image.hpp`) holds a `VkImage` another API made, freed by its
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
rehash, a mesh-arena grow — since a name
lives on the handle. One change
serves both profilers: Nsight renders the regions as trace ranges (and
groups a capture by `VkPipeline`, which `KernelSetBuilder::add` therefore
names too), MoltenVK maps them onto Metal debug groups and
`MTLComputePipelineState` labels for Xcode. Debug utils is an *instance*
extension, so it cannot ride `enabled_device_extensions` and is **declared,
never probed** — the loader's answer for a disabled extension is not portable
(null from a conformant loader, a live pointer from a directly-linked
MoltenVK). Both seams carry the declaration:
`AdoptedDevice::enabled_debug_utils` on adopt and the
`instance_debug_utils_enabled` argument on create, the latter filled in by the
`Device::create(const Instance&, …)` overload every recon call site uses. It is never required — its absence costs the capture's names and
nothing else — and resolution is **all-or-nothing**, so a driver returning
two of the three entry points reports no labels rather than a region that can
be opened and not closed. The label is recorded *outside* the `GpuTimer`
bracket, so no `gpu_ms` measures the markers around the work.

The GLSL pinhole tsdf and texture project through is the sensor package's
(`sensor_frame` exports `pinhole.glsl`). grid's depth allocation
unprojects through its own copy of the one line it needs (grid's
`volume/shaders/depth_pinhole.glsl`), which `recon_volume_depth_unproject`
holds to it bit for bit.

### tsdf

`TsdfIntegrator` fuses a posed depth frame into a grid's
`tsdf`/`weight`: projective `sdf = depth − Zc`, `±trunc_dist`, an
inverse-square-with-behind-dropoff weight, running average capped at
`max_weight`. Depth is sampled bilinearly at integer pixel centres, nearest
at image edges, across discontinuities, or when any tap is non-positive or
non-finite, and colour at the nearest pixel (2026-10-08). A NaN hole
therefore leaves a valid nearest sample usable, and a camera with a
non-finite intrinsic, depth bound or pose entry is refused. Zero is a
missing sample even when `min_depth` is zero, in allocation and integration
alike. Near an outline the true distance replaces the projective one, in
both modes (2026-10-29): a voxel whose sample is not a clean interpolation
inside the band -- a jump or a gap across its taps, past the band, or no
return -- and whose footprint window (+-ceil(half a voxel's footprint)
pixels, 3x3 to 7x7) holds a return and a gap or a jump past the band takes
the Euclidean distance to the nearest back-projected return within
+-ceil(the footprint + 1) pixels (2 to 6), if under `trunc_dist`. Its sign
is its own pixel's: negative behind that pixel's surface (faded as a band
sample is), positive in front of it, beside it, or over a gap (a quarter of
a sample's weight); a voxel more than a voxel behind its nearest return,
beside it, is in the shadow behind the outline and keeps the projective
sample. A positive one brings no colour, and under Dynamic may make a voxel
observed, as a band sample does. Outlines then lie at the true edge instead
of a cell's step, and thin parts mesh solid. A pass a frame over its pixels
(`tsdf_rim_map.comp`) first finds the smallest window around each pixel
that holds an outline, and the depth range of the returns within the
widest search, so a voxel tests its footprint's window with one read and
skips the search when it lies a band or more in depth from all of them.
`IntegrationMode` selects
**classic** (accumulate: free space ahead of the surface fuses as
+`trunc_dist` at a sample's weight) or **dynamic** (change evicts, conflict
averages; below). An optional `ColorFrame` fuses colour through its own
separate `ColorCameraParams`; a voxel's first colour observation assigns rather than
blends, and `coverage_in_alpha` has a pixel with a zero high byte fuse no
colour, as one outside the image fuses none. Each call is one tick of the
map's clock, and stamps `changed` on every block whose `tsdf`, `weight` or
`color` it changed -- a store that leaves a different value, Dynamic's
reset included, so converged surface stamps nothing -- always, one lane a
subgroup reading the stamp before its atomic, which measured as nothing
(2026-10-01). Opt-in `StageMetrics*` reports an
`"integrate"` row with both halves, over a `"  ..active set"` sub-row for the
compaction dispatch it also makes. That compaction keeps only the blocks
the frames can change: one frustum a frame from its depth camera, no near
cut (Dynamic resets free space up to the camera), far at
`max_depth + trunc_dist`, the union compacted in one scan
(`compact_active_blocks_in_frusta_on_device`). The kernel changes a voxel
only inside that reach, so the result is bit-identical to fusing every active
block, while the dispatch scales with the reached blocks, not the map. A
frame with a non-positive focal length, whose principal point lies outside
its image (the side widening then stops containing it), or whose planes are
not finite fuses the whole active set; the pose must be rigid. A grid with blocks ticks even when none is
reached (2026-10-06), which the map's host copy of the heap counter
answers. The list and its count stay on the device: a one-thread kernel
(`tsdf_dispatch_args.comp`) turns the count into the fuse's indirect
dispatches, so a fuse is one submit that reads nothing back; the frames
are staged. `record_integrate` records it into a caller's batch after an
allocation of the same frames (2026-10-25): the passes are bounded by the
grid's block slots, none runs behind a closed gate word, the tick is taken
at record, and every frame's cameras go up in one write each ahead of the
compaction. `integrate` is that, one submit and its wait.
`integrate` also takes a list of frames
(`FrameInput`: a depth image, its `DepthCameraParams` and its colour; the
`Fuser` allocates from the same list through `to_depth_pinhole`): one
compaction and one submit for them all, each frame a dispatch
of its own over the union in order. Under Classic every voxel takes them in
turn as integrating them one at a time does, bit for bit. A frame with no
pixels fuses nothing, as it allocates nothing (2026-09-30).

**Dynamic: change evicts, conflict averages** (2026-10-29). Each frame
names its fixed camera's slot (`FrameInput::slot`, distinct in a set; the
sender's is the camera's index) and noise (`FrameInput::noise`: sensor's
`DepthNoise`, the frame's `DeviceFrame::depth_noise` as its producer states
it; 2026-10-30), and the integrator owns a `ChangeDetector` (its config from
`TsdfIntegrator::create` and `FuserConfig::change`). A Dynamic set is one
instant, three passes a frame: every frame's change test; every frame's
reset (`tsdf_reset.comp`, the integrate kernel's body built as
`VR_TSDF_RESET`), which, on a ray its camera flagged -- the voxel's nearest
pixel -- sets every voxel in front of the new sample and in its band back
to unobserved (tsdf, weight and colour 0) and keeps the history behind the
band; a `Vanished` ray, with no new sample, resets up to the band of the
surface that left (the mask word's depth); then every frame's fuse, in slot
order. Free space past the band averages in as +`trunc_dist` at a tenth of
a sample's weight, geometry only (it is a sample of what lies behind, so it
brings no colour), and only into a voxel that holds weight: it wears away
a surface no change removed but never evicts one, cameras that disagree
average into one surface, and free space never makes an unobserved voxel
observed, so a reset region and the blocks a mover left stay unobserved
and the free pass frees them. The grid is the same bit for bit whatever
the frames' order. Whatever a change test would refuse (the detector's
`check`) is refused before any work. A closed gate stops the change tests
too, so a set fused again is tested once. A camera's first `warm_up` frames
build its history and reset nothing, and its first Dynamic set zeroes that
history in a submit of its own. Integrate a grid with one mode and one integrator: the change
histories are the integrator's, by slot, whatever grid it fuses.
Each camera's change test also marks its reach (`tsdf_change_reach.comp`,
in the `"  ..detect"` row): every pixel within 6 -- the rim distance's
widest search -- of a flagged one takes the set's stamp, with a bit near a
departure (a surface farther or vanished); a uint32 a pixel a camera, made
and zeroed with the camera's first set in one submit for the set's new
cameras. Within the reach of a change, the fuse gives no rim distance, so a
moving surface is fused from its fresh projective samples alone; and beside
a departure the reset also clears a positive voxel its camera sees past or
into a gap whose nearest return is gone from the band or is a pixel that
changed, so what a rim distance left beside an object leaves with it, while
an outline that stayed keeps its own (2026-10-29). It clears such a voxel
whoever fused it: the front of another camera's band there is fused again
by that camera the same set, at one sample's weight.
`Fuser` owns the shared grow/allocate/integrate sequence (2026-10-10). A set
records into a caller's batch (`record_fuse`, 2026-10-25): the fuse's checks
(`check_frames`), a grow ahead past `kGrowThreshold`, blocking, then one
allocation round of every frame's band
(`record_allocate_from_depth`) and `record_integrate`. Where the set needs
its whole band -- the default, or a grid holding no block -- the fuse is
gated on a word the round closes on any failure, and `finish`, once the
batch has resolved, fuses a refused set again, blocking
(`FuseReport::redone`): allocation retrying pure lock contention up to four
times beyond the tier's own rounds and growing for capacity, then the
integrate, at the tick the set was recorded at (`rewind_tick`). `fuse` is
the record, one submit and `finish`. `max_grows_per_set` bounds successful
grows including grow-ahead (default 4; zero fixes capacity). An incomplete
band returns an error before integration; a failed call may already have
resized or allocated zeroed blocks. An unsuccessful grow-ahead can still be
followed by successful allocation into the existing grid.
`create_fusion_grid` (`tsdf/fusion_grid.hpp`) makes the grid a fusing caller
starts from: `block_grid_params`' layout carrying float `tsdf` and `weight`
and the packed `color` the fuser writes, at `fusion_buckets`' start --
16,384 buckets at 1 cm and coarser, scaled by (1 cm / voxel)² up to 4× finer,
or the caller's count, capped at the ceiling the grid can never grow past on
the device (a capped caller's count logged as a warning) (2026-10-25).

`allow_partial` explicitly opts live callers into integrating what fits.
`fuse_viewer` and `rig_viewer` enable it with a two-grow bound. That mode
leaves pure lock contention for the next set and privately backs off a failed
grow for 60 map ticks at the same size. Once its grid holds a block, a set's
fuse is ungated, taking what the round placed, and a grow for a capacity
limit is made after it, so what the set could not allocate lands with the
next (2026-10-10, amended 2026-10-22). Its fuser belongs to one grid.
`FuseReport` carries the before/after bucket counts, grows, allocation
failures, any failed-grow status and final occupancy; failed allocation
requests can refer to the same block repeatedly. Host allocation exceptions
become `OutOfMemory`, and a host failure during rehash rolls the map back.
Scanner-specific memory callbacks and allocation-stop thresholds are deferred
to the iOS port, alongside the actual caller and peak-memory measurements.

`ChangeDetector` (`tsdf/change_detector.hpp`) tells a fixed camera's real
change from its noise, a pixel against its own past (2026-10-29). Each
camera slot (up to 64) keeps a history per pixel on the device -- a running
depth, the pixel's own temporal noise `sigma_r`, floored by the camera's
depth noise (sensor's `DepthNoise`, `sigma(z) = a + b z + c z^2`, its only
knowledge of the sensor, as the camera's producer states it with each frame;
a camera whose noise is unknown, all zero, is refused, never given a model),
two cumulative sums, four counters and two flags, 20 bytes -- and one
pass a camera a frame (`record_detect`) tests every valid sample and writes
the camera's mask, a `uint32` a pixel: `PixelChange::None`, `Nearer`,
`Farther` or `Vanished` in its low two bits (`change_of`). Two-sided CUSUM
on `r = (d - d_hist) / sigma_r` (k 0.5, h 8) or one frame past 6 sigma
alarms; an alarm restarts the pixel's history and freezes its noise
estimate for 30 samples; a camera's pixels test after its first 30
recorded frames. An invalid sample changes nothing, but over no return: a
pixel first valid once its camera tests had returned nothing until then, so
its history lasts only while it returns. A lone invalid sample there is a
dropout and changes nothing (it only marks the history, and the next valid
sample clears the mark); the second invalid sample in a row ends the
history, and flags `Vanished` after three valid samples, a lone dropout
skipped, the word's high 30 bits the depth that left in 0.1 mm
(`vanished_depth`): what a mover leaves where nothing behind it returns,
flagged a frame after it left. A camera missing from a set is
simply not detected, and one whose image size changes starts over. A gate
word, as `record_integrate` takes one, has a pass test nothing, though a
camera's first frame at a new size starts its history all the same: that
frame zeroes the camera's state in a submit of its own. 24 bytes a pixel in
all, 8.8 MB a 640 x 576 camera.

### texture

`ProjectiveTexturer` rewrites every `Vertex::uv0` against one
posed frame, one thread per **vertex**: it is kept where the vertex is in
front, in frame, its normal points toward the camera (a zero normal passes),
and it is **unoccluded** (projected depth agrees with the depth map), so the
back of a sheet thinner than the threshold does not take its front's image
(2026-10-08). Three outcomes, not two, and a consumer must test the **sign**
and never `== (-1,-1)`: a visible vertex gets `uv`, one in frame but occluded
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
(`texture_atlas.hpp`: `side_by_side_atlas`), one thread per
**triangle**: each takes the view facing it most squarely among those
that see its **front** and all three of its vertices, a `fallback` view
only where no other does, and all three point into that view's tile. Per
triangle because vertices in different tiles
would interpolate across the atlas, so that path needs an unshared mesh and
refuses a shared one, as it refuses tiles that overlap (2026-09-28). Both
passes refuse a view with no depth range, through one view check. With a
colour camera the depth camera still decides visibility, and the coordinate
is the colour camera's, kept only where its image recorded the vertex, it
faces the surface's front as the depth camera does, and its line of sight is
clear: the pass walks that line's
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

### sender

`recon_sender` is what a sender does with a prepared set
(`sender/set_sender.hpp`, the 2026-10-26 entry). `fuse_inputs`
(`sender/fuse_inputs.hpp`) hands prepared frames to the fuser, nothing
uploaded: each frame's slot is its place in the set and its noise the
frame's `depth_noise`, so a frame whose producer states none is refused
under Dynamic.

**The set step** (`SetSender`). `send` records the set's fusion
(`Fuser::record_fuse`), its free pass every `free_after` sets
(`record_free_stale`) and, for a set `code_due`, its frame's front half
into the set's front batch after its prep, submits it and waits once, and
finishes the fusion; then the frame's back half in a batch of its own, its
wait, the write, the serve and the decode, as `code` does. A coded set is 2
submits, a set not coded 1. A fusion done again (`FuseReport::redone`)
records the front half again after it, in a batch of its own
(`"wait: redo"`). `sent` runs once the set is published: a coded set's
frame written and served, before the decode; a set not coded, after its
fusion. `Sent` holds the fusion's report, a free pass's refusal, which does
not stop the set, and the frame. The frame goes through the stream's halves;
a set due a frame of a moved-from stream is refused before anything is
recorded.

## Examples

(`examples/`.) Every example that fuses prepares its frames on the GPU
(`sensor::GpuFramePrep`) and fuses them through `tsdf::Fuser`, by
`examples/common/fuse_frame.hpp`'s `fuse_set`, which hands each
`DeviceFrame` over with its encoding declaration and prints what a set did
to the grid (a grow, a declined one, blocks left out), into the grid
`tsdf::create_fusion_grid` makes; `codec_mesh` converts into
`volume::block_grid_params`' layout too.
`fuse_set`'s caller names the integration mode, the one `fusion.dynamic`
selects (`integration_mode`): `Classic` unless set, except in
`fuse_orbbec`, `fuse_realsense` and `rig_viewer`, whose default is
`Dynamic`; `rig_viewer`'s View panel switches it as the rig runs. The four
dataset examples poll their frames through `sensor::IRgbdSensor&` — the
fuse loop never learns what is behind it — and prepare each on its own.
The three live ones read their cameras raw as a `SensorArray`,
`fuse_orbbec` one Orbbec camera or a rig, `fuse_realsense` one
`RealSenseSensor` or `StereoDepthSensor` and `rig_viewer` an Orbbec rig,
and prepare each set in one batch (`SensorArray::process`). The prep crops
each frame with colour to what both its cameras see, by their fields of
view (sensor's 2026-10-29 same-view entry): depth outside the colour
camera's view is not fused, and colour outside the depth camera's view has
coverage 0, so it colours and textures nothing. Every example's prep, the
dataset ones' too, drops depth outliers, sensor's default (its 2026-10-31
depth-outlier entries): a return with fewer than 20 returns, itself
included, within 8 pixel footprints at its depth fuses as no return. No
example has an option for it.
The four dataset examples take `ReplicaSensor` as the source: frame cap,
stride and the depth gate are its options, stamped on each frame it hands
out, and its disk probe at `open` visits only the frames those options
select. Its frames carry the depth PNG's samples as stored and the colour
as packed words (`RgbdFrame::color_packed`), one pinhole camera for both. An
empty poll is retried after a millisecond until the source reports itself
`exhausted()`, so a live driver in the same construction site waits and the
replay ends. A frame kept past the next poll — `fuse_render`'s keyframe,
`fuse_viewer`'s newest successfully fused frame for its final texture pass —
is kept as prepared, a `DeviceFrame` holding its buffers, and textures through
its own colour camera: `fuse_render` reads its colour back to the host for
the atlas, and `fuse_viewer` copies it into the atlas on the device;
`fuse_viewer` keeps that frame through the next preparation and fusion,
replacing it only on success (`fuse_keyframe`). A failure therefore leaves
its depth, colour and cameras available for the final and later remeshes.
Preparing while the previous frame is held allocates a separate output pair.

Every example reads a scenario with `--config path.json`; `--help` prints
that entry point. The optional core `config` tier owns the JSON format, typed
settings, application defaults and static input validation. Each example
uses `core::config::Config` directly. `examples/common/config.hpp` handles
startup and diagnostics, and `replica_config.hpp`, `orbbec_config.hpp` and
`codec_config.hpp` only copy validated settings into runtime API types and
attach resources, except that the codec's rules stay the codec's:
`encoder_config` maps `codec.quantization_table` to the weights
(`codec_quantization.hpp`, the tables' one home), and before GPU setup
refuses what `EncoderConfig::validate` refuses: a step out of the codec's
bounds, or a segment or update size past the longest. No local parser, flag
registration or config validator remains.
[Configuration examples](config/README.md) document paths, defaults and
migration from the previous CLI.

`fuse_orbbec` selects stream-only work with `operation: "stream"`; core
refuses explicit fusion, codec or output settings for that operation before
setup.
Calibration and SDK sync files stay in their domain formats and are referenced
by scenario paths. Device and camera capability checks remain in their runtime
APIs. Stage rows print through one table,
`stage_table.hpp`. Reports live beside the helpers rather than in a `main`:
`fuse_replica`'s extract reports and `fusion.dirty_every` survey in
`remesh_report.hpp`, the viewers' panels in `viewer_panels.hpp`,
`rig_viewer`'s panel, progress line, `render.texture_stats` and end-of-run
rates and atlas-copy times in `rig_report.hpp`,
the live array's report (its counters, each camera's line through a
counters function, a rig's skew) and clock in `live_sets.hpp`, the Orbbec
camera lines and counters (`lost` among them) in `fuse_orbbec_sets.hpp`, and
the RealSense camera's line, its depth and its scenario's options in
`realsense_camera.hpp`.
The per-set table of `fuse_orbbec`, `fuse_realsense`, `codec_replica` and
`rig_viewer` is `set_log.hpp`'s.
A `main.cpp` is the example's story: options, setup, the loop, the report.

`fuse_replica` runs the spine on a posed Replica-SLAM RGB-D sequence and
writes a PLY; `fusion.dirty_every` reports the
changed and re-mesh fractions over windows of its own, keeping its own tick.
Behind the off-by-default `VR_BUILD_VIEWER`: `fuse_render` writes a headless colour PNG (seam A — it
builds two devices by design), and `fuse_viewer` opens a live window on one
shared `VkDevice`, fusing on a background thread, drawing recon's buffers
directly, and carrying the two-panel perf overlay. Both viewers mesh only
what their render camera sees: the render thread publishes its `view_proj`,
and the fuse thread re-meshes the blocks inside it after fusing and whenever
the view moves (2026-10-06). The four dataset examples
take `source.preload`, which makes the loop measure compute rather than the
JPEG/PNG decoder.
The live counterpart is its own example, not a `fuse_replica` flag:
**`fuse_orbbec`** (`VS_WITH_ORBBEC`) reads one `OrbbecSensor`, or with
`source.sync_config` one per camera of the file on the host's clock
(`source.apply_sync` writing the settings a camera differs in), as a
`SensorArray` posed by `source.calibration`; it waits on the cameras'
notifications for each set (`sensor::RigSets`), records its prep, fuse,
`fusion.free_after` pass and, when it codes, the frame's front half into one
batch through `sender::SetSender`, as codec_replica does (the frame's rANS
half is a second batch), and writes a PLY after `run.frames` frames.
`operation: "stream"` only streams: each set is polled, counted and released, with no
GPU preparation, fusion or mesh, so a run measures the cameras' stream and
decode alone. `diagnostics.readback: true` reads each set's luma back through
`PictureReadback`, a stage of its own, as the calibration app does.
`diagnostics.dump_sets` writes the first sets' camera JPEGs, and with the
readback their luma as PNGs (R = G = B), into `diagnostics.dump_directory`
(`SetDump`), so the device's luma can be compared with the JPEGs' off line.
The directory must not exist: it is checked before the cameras open and made
with the first set. The run reports the cameras' open time and, at the end
or when it fails, the array's counters, each camera's `lost` frames and, for
a rig, each secondary's mean and worst skew to the primary. Colour is H.265
at 3840x2160 and 25 fps unless the scenario names another mode, and every
camera opens at `source.calibration`'s colour controls, or at auto
exposure.
With `output.stream`, `output.csv` or `output.serve` it also codes each
fused set as one frame of a `codec_stream::CodecStream` (the 2026-10-20
decision), decoding it too only with `output.csv` or `codec.sweep`
(`set_codec_outputs`), for their decode columns and the sweep's grid. Each
set's row is stamped published once its frame is written and served, before
any decode, or after its fusion when it is not coded. `main.cpp` opens the
stream before the cameras and refuses there a port in use and a PLY that is
the stream's or the CSV's file under another name (`check_apart`). A
record's set is the set's index in the run
and its pts the set's time; a set stamped before the last frame, as a clock
stepping back stamps it, takes that frame's time. The session's Geometry
track names that time's clock (`live_session`): the host's system clock
(`stream::kHostClock`) when every camera stamps on it, as a rig's always
do, else clock domain 1, the cameras' own. Each program says which before
its cameras open (`fuse_orbbec`'s `orbbec_on_host_clock`; `fuse_realsense`'s
always the host's), and the run checks the opened cameras against it
(`on_host_clock`).
With `source.audio`, which the core allows only with a stream and every
camera on the host's clock, the run records a mic beside the cameras
(`live_audio.hpp`). `open_audio` opens the input before the GPU: sensor's
`Microphone` where sensor builds `sensor_microphone` (`VS_WITH_PORTAUDIO`;
`VR_EXAMPLE_MICROPHONE` in the examples), refused as `Unsupported` in any
other build. `live_session` adds its Audio track, track 1: signed 16-bit
samples at its rate and channels, on the host's clock, a record a chunk.
`AudioFeed` starts the mic once the cameras stream, drains it on the run's
thread after each set is sent, appending each chunk as a key record at its
capture time with its sequence, and drains it once more before the stream
finishes; grid's `CodecStream::append` muxes the records with the frames by
pts. A drain that fails ends the audio with a warning, and fusion goes on.
The mic's counters follow the cameras', with what the stream, finished
first, wrote and dropped of its chunks. With `output.serve`, grid's server
serves both tracks, as the file records them, except the audio past the last
set, which a viewer may miss as the stream finishes. With no mic, the session
and its bytes are as before audio.
A run that fails part-way still finishes its stream and
reports the frames it coded; one that coded no set leaves none.
The report's bitrate is at `source.fps`, and `codec.sweep` sweeps the last
grid. A Ctrl+C or SIGTERM (`interrupt.hpp`, `rig_viewer`'s handler too,
installed first) ends the run. During the setup, the run returns before the
cameras stream, removing the empty stream and releasing the cameras. Once
they stream, it ends after the set in hand, as `run.frames` would: the
stream is finished, its viewers closed, the cameras released and the PLY
written. A second signal of the same kind ends the process.
That run is `examples/common/live_fusion.hpp`'s `run_live_fusion`, which
takes the program's camera opener, its cameras' clock and its counters, so
it is the one loop of both live programs that fuse to a PLY.
**`fuse_realsense`** (`VS_WITH_REALSENSE`) is that run on one RealSense
D400 camera (core's `FuseRealsense` profile), its depth the scenario's
`source.depth`, chosen in the opener (`open_realsense_array`) alone, so the
loop is the same for both:
- `camera`: a `RealSenseSensor`, the camera's own depth with its projector
  on, at the scenario's modes and depth gate (`realsense_options`).
- `ffs` (with sensor's `sensor_ffs`, `VS_WITH_FFS`; `VR_EXAMPLE_FFS` in the
  example): a `StereoDepthSensor` over a `RealSenseStereo`, the camera's
  infrared cameras with the projector off (`realsense_stereo_options`),
  rectified to the depth size as luma at alpha 0 and matched by FFS
  (`ffs_depth_options`) on the sensor's own thread; its depth stays on the
  device, where the prep reads it in place. A build without `sensor_ffs`
  refuses the scenario before any GPU or camera opens
  (`check_realsense_depth`).
Either is read as a `SensorArray` of one, at the origin; its YUYV colour is
staged and converted on the GPU by the array's prep, and its counters are
the interface's. The profile takes no Orbbec rig keys, codec, outputs beyond
the PLY, stream operation or diagnostics.
**`rig_viewer`** (`VR_BUILD_VIEWER` with `VS_WITH_ORBBEC`, and
`VS_WITH_FFMPEG` off Apple; CI's viewer leg compiles it) is `fuse_viewer`'s live-rig
sibling: an `OrbbecSensor` per camera of the sync file, opened as
fuse_orbbec opens a rig (`open_orbbec_cameras`, at the calibration's colour
controls), read as a
`SensorArray`, its raw sets prepared, fused and textured from every camera
on the GPU, and the atlas filled by
device copies recorded in gfx's frame (the 2026-09-29 decision), and
texturing a camera a set lacks from its last frame, a fallback view
(`fusion.hold_ms`).
`rig_viewer`'s panels show the newest set; its fuse thread also records every
set into a `SetLog`, the newest 15,000 kept, which it prints at the end with
the sets fused and the meshes drawn a second (each from its first to its
last), the array's and each camera's counters, and the atlas copies' device
time from gfx's profiler. A set's rows are its poll, its stages and, when it
meshed, its remesh's, merged before the mesh's hand-over publishes it; a
set it did not mesh is published at its last stage and tabled apart, and a
remesh for a view change between sets is no set's.
The two viewers hand meshes over through `mesh::MeshExchange`,
take their window glue (the GLFW surface, the shared device and ImGui's GLFW
backend) from gfx's `gfx_glfw` (2026-10-12), and share `viewer_common.hpp`: the
shared device's settings and gfx's adoption of it, the teardown guards and the
render camera the fuse thread meshes (`SharedView`).
Both draw with gfx's `StreamedAtlas` through `viewer_atlas.hpp`'s `LiveAtlas`:
a mesh's payload names the colour buffers of the cameras that textured it and
their tiles, the frame that commits the mesh copies them into the atlas, and
gfx's `RetireQueue` holds them until the frame loop's timeline reaches that
frame; an untextured mesh draws in vertex colour against the pipeline's own
fallback (2026-10-08).
At teardown, `LiveAtlas` destroys the gfx atlas first, waiting for submitted
work or draining the queues when the newest frame was never submitted, then
reclaims the retained colour buffers without waiting for that frame's number.
`fuse_render` uploads its keyframe into a `StreamedAtlas` in the frame that
draws it.
**`codec_replica`** fuses a Replica sequence as `fuse_replica` does, and
streams the growing grid through the codec: every `codec.encode_every` frames it
encodes, in the set's batches (`sender::SetSender`), and the stream
decodes into its player grid. It reports the
bitrate at the coded frame rate and, at an intra period above 1, a
GOP's (an I-frame and the period's P-frames, each at its type's
mean), and for each frame type its bytes (mean, p50, p95, max) and
both calls' stage rows. It then judges the last decoded
surface against the source's, mesh to mesh, with the `eval` tier: accuracy,
coverage and the F-score at half a voxel, since a fused TSDF is projective
and sampling it would measure the fusion's bias too. `codec.sweep` prints the
rate–distortion table the defaults are chosen from. Its codec stream, report,
sweep and quantization tables are grid's ([grid's
examples](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DESIGN.md#examples)), as are `codec_mesh` and `serve_stream`.

## Next work

**Pipelined stages**: the core's `CommandBatch` submits without waiting,
ordered by timeline semaphores; the per-set host waits go, in
`tsdf::Fuser` (an allocation's failure count and occupancy read a set
late); then a pipeline over acquire, prep, the grid chain (fuse, mesh,
texture, still serial) and consumers.

**View-culled meshing has landed** (2026-10-06): `fuse_viewer` and
`rig_viewer` publish their render camera's `view_proj` to the fuse thread,
which meshes the blocks inside it (a 0.25 m margin) and re-meshes when the
view moves. The win is correct and unmeasured — expect it roughly linear in
the visible fraction on the `ExtractTimings` rows that scale with the active
set (the dispatch, the arena; `readback_ms` and `descriptor_ms` are per-call
constants and will read flat, and the frustum compaction still scans every
hash slot), and quote nothing until a run on a large scan says so. Beside that:
the gfx-vertex converter; grid's next work holds meshing's and mesh files'
([its next work](https://github.com/taojin-6/volumetric_kit_grid/blob/main/DESIGN.md#next-work)). On `texture`,
the `TODO(texture)`s: keeping a static keyframe set's
depth and coverage in the pass between calls, blending views at their seams,
and a per-triangle tile
index in gfx so a shared mesh can be textured from several views; and the
multi-keyframe post-scan atlas, which the multi-view path can carry.

**On `fuse_orbbec`'s stream** (the 2026-10-20 entry): a capture-volume
crop (`fusion.max_depth_m` is the limit there is), raw RGBD recording and
replay, colour, an upright world, and the session's calibration
(`codec_session` leaves it out). With
`output.csv` or `codec.sweep` the stream still decodes each frame inside the
set loop: on
the lab rig (4 Femtos, RTX 4090, Release; 1 cm, Dynamic, period 30; 303
sets) a P-frame's decode took 2.70 ms host and 2.14 ms device, 37% and 38%
of the set's fusion and coding, and 6.49 / 4.38 ms at 5 mm. A loop slower
than the rig gets the newest set, so sets drop; that cost is unmeasured.

**On `tsdf`**, Dynamic resets on change (2026-10-29). Next, each judged on
the lab rig: a surface that leaves to no return flags nothing where its
pixel returned anything in its camera's warm-up, or where it returned for
under three samples (a decay gated by visibility, for what stays of a
walking column that vanishes and of a slow sphere);
partial evidence lowering the weight cap along a ray, so slow motion slides
instead of stepping; a rule for pixels that drop in and out, the source of
most remaining false resets; and the sigma edge and incidence terms of the
noise model.

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
