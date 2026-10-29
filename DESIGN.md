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
volume (voxel-hashed) → extracted geometry (mesh) → optional compressed
bitstream → handoff to a renderer.**

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
- Macros: `VR_` prefix (`VR_CORE_API`, `VR_WITH_CUDA`); the error and
  contract macros are volumetric_kit_core's, used under its names (`VKC_TRY`,
  `VKC_ASSIGN`, `VKC_CHECK`, `VKC_VK_TRY`).
  Deliberately *not* the `VK_` prefix — that belongs to Vulkan. (The prior
  engine's `VK_DEVICE_HOST`-style macros are renamed `VR_*` on port.)
- CMake: fetched by commit (`FetchContent`); component targets
  `volumetric_kit::recon_camera`, `…_core`, `…_volume`, `…_voxelize`,
  `…_tsdf`, `…_mesh`,
  `…_texture`,
  `…_sensor`, `…_codec`, `…_eval`, `…_image_io`, `…_mesh_io`, `…_stream`,
  `…_networking`,
  `…_sender`, `…_interop` (+ later `…_track`),
  plus `…_sensor_utils` (GPU pre-processing), `…_sensor_array` (several
  sensors read as one), `…_sensor_apple` (Apple's pixel buffers as images),
  the opt-in `…_sensor_orbbec`
  driver (`VR_WITH_ORBBEC`), `…_sensor_video` decoders (every Apple build,
  elsewhere `VR_WITH_FFMPEG`), `…_sensor_video_syntax` (H.265 syntax read
  on the host, every build), and
  `…_mesh_io_assimp` mesh importer (`VR_WITH_ASSIMP`);
  umbrella alias
  `volumetric_kit::recon`.

## Tiered architecture

Strict left-to-right dependency rule: a tier may depend only on tiers to its
left. No upward includes.

`core` → `volume` → `tsdf` → `mesh` → `texture` → `interop`, with `sensor`
branching off **`core`**, `voxelize` and `codec` off **`volume`** and
`eval`/`io` off **`mesh`** (later: `track`); `io`'s image target links the
family core's base tier alone. `camera` sits under `sensor` and beside
`core`: it links the family core's base tier and GLM, and no other recon
tier. `stream` links that base tier alone, and `networking` links `stream`.
`sender` comes after `networking`: it links `networking`, `codec`, `tsdf`
and `sensor`'s GPU pass (`sensor_utils`).

- **`camera`** — the camera vocabulary the drivers and the sensor tier
  share: the double-precision `CameraModel` (image size, intrinsics, OpenCV's
  rational lens), the lens as a scalar template, rigid transforms, and the
  sensor array's calibration file (`camera/array_calibration.hpp`). No
  Vulkan (the 2026-10-06 decisions).

- **`core`** — recon's own vocabulary every tier trades in: the GLM math
  aliases, the posed pinhole `DepthCameraParams`/`ColorCameraParams` of
  `core/camera_params.hpp`, the colour space, `device_requirements()` and
  `log_message`. The Vulkan foundation under it (instance, device, VMA
  allocator, buffers and images, `CommandBatch`, `ComputeKernel` and
  `KernelSetBuilder`, timers) and the `Status`/`Result` idiom are
  volumetric_kit_core's, used under its names. Vulkan is reached through the
  core's one umbrella header (`core/vulkan/vulkan.hpp`) — no other code
  includes `<vulkan/...>` directly.
- **`volume`** — the sparse voxel hash map in Vulkan buffers; allocate / compact
  / rehash as compute shaders, a stateless `grow_grid` helper, a default
  hash layout (`kDefaultBucketSize`, `kDefaultMaxChain`) and a default block
  layout (`block_grid_params`). Camera-free: depth allocation takes its own
  `DepthPinhole`, and nothing in it includes the camera vocabulary
  (2026-10-28).
- **`voxelize`** — a triangle mesh's distance field written into a grid
  (signed, or as a shell), `MeshIntegrator`. Links **`volume` alone**: it
  fuses no depth (2026-10-28).
- **`tsdf`** — TSDF integration compute shaders (classic + dynamic), the
  `Fuser` that fuses a set of frames (grow, allocate, integrate), and the grid
  it starts from (`create_fusion_grid`).
- **`mesh`** — marching-cubes compute shaders and host mesh containers.
- **`io`** — host asset loading and export, in two targets: encoded images,
  depth PNGs and PNG export in `recon_image_io`, on the family core's base
  tier alone; PLY export, the host triangle containers and opt-in mesh
  normalization helpers in `recon_mesh_io`, off `mesh`; static mesh import
  in the optional Assimp-backed `recon_mesh_io_assimp`. GPU tiers never link
  back to file I/O. Dataset playback and camera conventions remain with
  their capture adapters.
- **`texture`** — projective texturing: fills the mesh's per-vertex `uv0` with a
  posed camera's image coordinates where it has line of sight (per-vertex-color
  fallback elsewhere), or with several cameras' coordinates into an atlas of
  their images, a compute pass. A colour image is registered to its depth
  camera or taken by a colour camera of its own, whose line of sight the
  depth map decides too; depth may be on the host or the device.
- **`sensor`** — the sensor *interface*: `IRgbdSensor`, the `RgbdFrame` it
  hands out as captured (each camera's `camera::CameraModel`, the pixels it
  holds), and the boundary conversions a capture integration gets silently
  wrong — camera conventions (pose handedness, registered-depth intrinsics)
  and colour (`to_canonical`). Reads `camera`'s models, the
  `core/camera_params.hpp` pinhole cameras and `ColorEncoding` from
  `core/color_space.hpp`, so it
  depends on **`camera` and `core` alone** — it sits beside the fusion tiers,
  not on top of them — and bundles **no drivers**: one ships here only if this repo can build
  *and* test it (the 2026-08-02 decision). The one that does, Orbbec, is a
  target of its own (`sensor/orbbec/`), so `recon_sensor` never links a vendor
  SDK. The HEVC and JPEG decoders are another target of
  their own (`sensor/video/`, on the GPU's hardware) and link no camera tier:
  `core` and, on Apple, the Apple importer (`sensor/apple/`), which links
  `core` alone. The H.265 syntax they read on the host is a target of its
  own as well, on the family core's base tier alone, in every build.
  The GPU pre-processing and the pictures' readback are a fourth
  (`sensor/utils/`, Vulkan and shaders), so `recon_sensor` itself stays free
  of all of them.
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
- **`stream`** — the `.vrst` session container: a session header (tracks,
  each naming its clock, and the calibration document as bytes) and the
  records appended after it, each payload stored byte for byte, readable
  from a killed writer (`stream/session.hpp`, 2026-10-12). Host-side, with
  no Vulkan and no other recon tier.
- **`networking`** — a session served and read live over TCP, its wire the
  `.vrst` file's own bytes: `StreamServer` and `StreamClient`, over BSD
  sockets and threads of their own (2026-10-21). Host-side, on `stream`,
  so the format and the sockets are tested apart; it logs through recon's
  log seam (`core/log.hpp`), header-only over the family core's base tier.
- **`sender`** — a sender's set step and its codec stream: a prepared set
  fused, freed on schedule and coded in two batches (`SetSender`), each
  frame written to a `.vrst` file, served and decoded back
  (`CodecStream`), and the fuser's inputs for prepared frames
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
  (`check_device_requirements`); `vr_compile_shaders` validates the emitted
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

It rides **beside** the camera — a field on `sensor::RgbdFrame` and
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
- **Hardening applied on port:** replace glog/fmt with the pluggable log
  handler; adopt `Status`/`Result` repo-wide;
  rename `VK_*` device macros → `VR_*`; break the `ReconstructionPipeline`
  god-object so each tier has a standalone API; drop committed build artifacts,
  hardcoded paths, and machine-specific config.
- **Drop:** the prior engine's own renderer (gfx replaces it) and its SwiftUI
  demo app (reference only). Its sensor driver framework is rebuilt behind a
  clean `IRgbdSensor` in the later `sensor` tier — the **interface** is
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

A consumer fetches recon by commit (`FetchContent`) and links its per-tier
targets; recon has no install rules or package config (the 2026-10-22
decision). `VR_CAMERA_ONLY` builds `recon_camera` alone, with no Vulkan
([README](README.md#building)). Quality gates
(warnings-as-errors, formatting hooks, sanitizer and test CI) are wired from the
start, not retrofitted.

## Implementation reference

Landed and proven on MoltenVK, left to right: `core` → `volume` → `tsdf` →
`mesh` → `texture`, plus `sensor` (the contract and the Orbbec driver) and
**interop seam B** — gfx drawing recon's own buffers with no host round trip.
[DECISIONS.md](DECISIONS.md) carries the *why* behind anything here that looks
arbitrary; it usually isn't.

### core

The Vulkan foundation below is volumetric_kit_core's vulkan tier
(DECISIONS.md, 2026-10-04): recon uses its types under the core's names, and
what is recon's own is `device_requirements()` (what recon's kernels need of a device),
the camera and colour-space vocabulary and the vector types. The description
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
Vocabulary: `Status`/`Result`, the GLM aliases, `camera_params.hpp`,
`color_space.hpp`, and the core's `core/base/stage_metrics.hpp` — the rows
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

`core/shaders/pinhole.glsl` is recon's GLSL pinhole: the rigid inverse,
unprojection, divide, in-image projection and nearest pixel that tsdf,
texture and the sensor prep's colour test go through, and that recon
exports to its consumers' kernels (README, "Building"). Its functions take
values, so a scalar-layout camera and a std430 one both pass theirs. The
acceptance stays two callers' own: texture extrapolates past the image, and
the colour test bounds the rounded pixel. volume's depth allocation
unprojects through its own copy of the one line it needs
(`volume/shaders/depth_pinhole.glsl`), which `recon_volume_depth_unproject`
holds to this one bit for bit.

### volume

`VoxelGridParams::validate` rejects non-finite metric fields and block extents,
and bounds `ceil(trunc_dist / (block_size * voxel_size))` to 127. This is the
representation limit of the depth allocator's `uint32` work-item count:
256 distinct tile centres times `(2 * 127 + 1)^3` is 4,244,832,000, whereas
radius 128 overflows. It also keeps the shared host/GLSL radius conversion
within its integer range. Validation happens before creating a map; the
coordinate helpers retain that precondition. `block_grid_params` is the
default block layout: 8³-voxel blocks in the default hash layout, as many
blocks as bucket entries, at the caller's resolution, band and table; the
fusion grid, the decode grid and codec_mesh build with it (2026-10-25).

`VoxelHashMap` drives init / allocate-from-coords, -depth,
-triangles / remove / compact / compact-in-frusta / resize as GLSL
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
lane cannot own a 250 000-block quad; as a depth tile does, a workgroup asks
for each distinct block its lanes name once. The host pass that counts those
blocks is also where an index is bounds-checked and a zero-area or non-finite
triangle is dropped — the former being the one input that divides by zero in
the closest-point solve, which lives in `volume/shaders/triangle_common.glsl`
so mesh-to-SDF evaluates the field with the same function. That pass is the
public `triangle_candidate_offsets`, and the decode and band test are
`volume/shaders/triangle_candidates.glsl`: `voxelize`'s mesh integrator bins
over both, so the blocks allocated and the blocks binned cannot drift apart.
Frustum-culled compaction gives the per-frame working set, from frusta built
by `make_frustum_planes` off a depth camera's pinhole
intrinsics or — since 2026-08-12 — off a *render* camera's `view_proj`, whose
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
`compact_active_blocks_in_frusta_on_device` keeps the blocks inside any of
several frusta in one scan, on the device in a list of its own, which
`check_device_block_list` refuses as not the active set and
`check_device_block_subset` accepts until the next frustum compaction
rewrites it or the map allocates, resizes, removes, clears or moves; fusion
culls to the scanning cameras with it and marching cubes to the viewing one
(2026-10-06). A block list is a `DeviceBlockList`, on the device
(2026-10-13). A sorted block list is in `coord_less` order (z, then y,
then x); `volume/shaders/coord_order.glsl` mirrors it for kernels
(`vrCoordLess`), with the binary searches they run over one. `resize`
preserves block indices, so per-voxel data survives a grow.
`VoxelBlockGrid` composes the map with independently-allocated SoA
attribute arrays (`tsdf`, `weight`, `color`, …), each
`num_blocks·voxels_per_block`, so a consumer materialises only what it
needs. Every buffer of both is device-local, reached through a
`CommandBatch`: `create`, `clear` and `remove` zero on the device (`clear`
and `remove` before any index is freed, `remove` by a kernel that finds
each coord's block itself, so its cost is the count's, not the grid's),
`resize` copies there, and a call's inputs are uploaded in its batch.
`VoxelHashMap::allocate` and `remove`, and `VoxelBlockGrid::remove`, take
a device buffer of `BlockIndex`, bound where its producer wrote it; a test
uploads its own coordinates first (`tests/device_blocks.hpp`). Their work
is recorded into a caller's batch by `record_allocate` and `record_remove`
(the 2026-10-24 one-batch entry): K = 3 rounds, each sized on the device
from the last one's tally, the count a `DeviceCount` a kernel wrote and an
optional gate word, the outcome gathered into a result slot that `settle`
reads once the batch has run. The blocking calls are record, one submit and
settle, repeated while lost races alone remain, 10 rounds at most. Each also takes
a caller's `GpuStageScope`, whose tile every round counts to, as the
decoder's `"  ..apply"` does. A grid's `create` builds the kernels its removal
and block pass run, so the first remove compiles nothing. The grid's
zeroes the listed blocks and frees them through the map's, the zeroing
recorded into the map's first round (`remove`'s `first_round`) ahead of
the delete, so it takes no submit of its own.
One body serves it and garbage collection, so the stale-block list stays on
the GPU through deletion, its count sizing the removal there; only the
removal's outcome reaches the host.
A compaction still reads its list back, in the count's own submit while the
set stays within a quarter past its last count (the 2026-09-28 residency
decision), except `compact_active_blocks_on_device`'s: its
`DeviceBlockList` stays on the device, stamped with the epoch, a
compaction serial and the heap's free count, and `check_device_block_list`
refuses one that a full compaction, allocation, resize, remove, clear or
move has made stale. While the last one still holds, the call returns it
again, dispatching and reporting nothing, so an extract after the encoder
reuses the encoder's compaction (2026-09-30); an extract records its own
into its batch when none holds (2026-10-24). A fuse culls instead and leaves
none: `record_compaction` records its compaction into the fuse's own batch,
the list and its count left on the device for the dispatches after it.
It is `const` (2026-10-26), so a caller holding the grid read-only, such as
the encoder or a caller handed a const grid, records one into its own batch:
it moves no block, stamp, heap count, epoch or tick, and writes only the
map's compaction output (the list, its count, its kind's serial and the
frusta, `mutable`), which any compaction rewrites. Its batch binds the
map's kernel sets, so the map is neither resized nor its frusta grown until
that batch resolves: a resize waits for the queue and the core then refuses
the batch, while a frusta grow under a running batch is undefined.
`topology_epoch()` lives on the *map* — the object that frees a block
index — and is a globally unique token re-drawn at `create`, at every
`clear` and at every `remove` that accepts its input, never at `resize`: a
`DeviceBlockList` anchors on it, so no path may free an index without
moving it and no two grids may ever share a value. `load_factor()` is the
occupancy read a per-frame caller can afford (a host copy of the heap
counter, which every call that moves it refreshes, `settle` for a recorded
one), and `kGrowThreshold` is
the occupancy it says to grow at — named here so a UI or an embedder cannot
draw a ceiling that disagrees with it. No host call reads the table back;
a kernel binds `entries_buffer()` and reads it through `hash_common.glsl`
or `hash_lookup.glsl` (2026-10-13), whose layout is internal: the host
`HashEntry` and fail slots are in `src/volumetric_kit/recon/volume/hash.hpp`,
and the slot sentinels and hash are the GLSL's alone in the library
(2026-10-17).
`grow_grid` centralizes the growth arithmetic: double, or reach a larger
requested minimum, within `VoxelBlockGrid::max_num_buckets` (block pointers
within int32, each attribute array within one binding). It retains no policy
or retry state and reports resize failures through `Status`. `bytes_at`
reports the attributes and grid-sized table's storage footprint. It excludes
transient rehash/staging buffers, host scratch and allocator/driver overhead,
so it is not a peak-memory budget for resize (2026-10-10).
`allocate_from_depth` takes a frame's camera as volume's `DepthPinhole`, the
fields and 96-byte layout of `DepthCameraParams`, which tsdf fills by
`to_depth_pinhole`. It also takes a list of frames (`DepthInput`), every
frame dispatched in each round's one submit, on a set of its own; a round
that retries dispatches them all again, and the rounds are the call's, not
each frame's. `record_allocate_from_depth` records one such round into a
caller's batch, its outcome in a result slot and an optional gate closed on
any failure; the blocking call is that round, one submit and `settle`,
repeated while lost races alone remain. `settle` moves the heap's host copy
only to a recording newer than every one settled and every blocking call
since, so recordings settle in any order. Opt-in
`StageMetrics*` on `allocate_from_depth` (an `"allocate"` row summing every
retry round) and on both compaction entry points (an `"active set"` row,
breakdown-prefixed when the caller already has a stage open).
Every block slot carries a `BlockStamp` (`hash_types.hpp`): ticks of the
map's clock (`tick()`), in a device buffer beside the heap
(`stamps_buffer()`), each written by the pass that knows its fact
(2026-10-01). Every allocation kernel stamps `requested` on each block it
asks for, inserted or found, and the grid's block pass (in `free_stale_blocks`)
stamps `weighted` on each holding an observed voxel. Every pass that
writes voxels -- `TsdfIntegrator`, `MeshIntegrator`, the codec's
inverse -- advances the clock first and stamps `changed` on what it
changed, so the clock counts writes (one a set, for the rig) and a
reader's recorded tick is older than every later change, however the calls
interleave. A fuse its gate closed on wrote nothing, so `Fuser::finish`
takes its tick back (`rewind_tick`) before fusing the set again. The init kernel zeroes the records, the delete kernel zeroes a
freed slot's, and `resize` copies them forward, the rehash stamping
nothing. A consumer compares ticks and resets nothing:
`free_stale_blocks(max_age)` frees the blocks whose newer stamp is older
than that, through the grid's remove body over the pass's list, so the band the
allocator still asks for stays. `record_free_stale` records it into a
caller's batch, at the tick then: the compaction, the stamp pass sized from
its count, and `record_remove` of the list it writes, one recording of
K rounds, so a block a lost race keeps waits for the next pass;
`free_stale_blocks` is that, one submit and `settle`, its result the heap's
gain (2026-10-25). The live examples run it every
`fusion.free_after` sets (`free_stale_blocks_every`, `fuse_frame.hpp`); the
iOS scanner reads `changed` through `read_block_stamps`.

### voxelize

`MeshIntegrator` writes a triangle mesh's distance field into a grid
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
bytes. One workgroup writes each block, and a voxel measures only the
triangles no lower bound rules out; a bound, less a rounding margin of the
triangle's own, rules a triangle out only when it is strictly farther than
the voxel's best so far, or than `trunc_dist` where that is nearer, so every
voxel writes what measuring its whole bin writes (2026-10-21;
`mesh_integrate.comp` has the bounds and the rounding count). Blocks past
`kMaxVoxelsPerBlock` (16^3 voxels) are refused. Each call is a tick, and
every block it writes is stamped `changed`.

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
alike. `IntegrationMode` selects **classic** (keep free space ahead
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
compaction dispatch it also makes. That compaction keeps only the blocks
the frames can change: one frustum a frame from its depth camera, no near
cut (Dynamic clears free space up to the camera), far at
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
turn as integrating them one at a time does, bit for bit. A Dynamic set is
one instant: the clear kernel (`tsdf_clear.comp`, the integrate kernel's
body `tsdf_integrate.glsl` built without its fuse) runs for every frame
before the integrate kernel fuses any, so one camera's free space never
erases what another fuses in the same set, and the grid does not depend on
the frames' order but for the running average's arithmetic (2026-10-08). A
frame with no pixels fuses nothing, as it allocates nothing (2026-09-30).
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
only then writes, so **a block's output is contiguous in the arena** (both
sparse kernels; `share_vertices` reserves two ranges, since a shared vertex
breaks `v = 3t`). Each cursor is bounded by its block's own reservation, so a
count that disagreed with the emit would drop geometry rather than write over
the next block's range. Every extract meshes the whole active set or a
caller's list and rewrites its slot from zero; incremental extraction was
removed on 2026-10-06. The vertex arena
is fitted to the surface, grow-only, and held as a **ring of slots** the
consumer releases by generation; the kernel writes a real
`VkDrawIndexedIndirectCommand`. `extract_device` returns a borrowed
`DeviceMesh` (valid until the next extract, enforced by a generation stamp),
`download` takes the single host copy and bridges the two workflows. The
arena, index run and draw command are device-only. An extract is recorded
into a caller's batch (`record_extract`, the 2026-10-24 one-batch entry): a
one-thread kernel sizes the marching cubes from the compaction's count,
gated, and a finalize gathers the results into a result slot that `finish`
reads, refitting in a blocking batch of its own when the surface outgrew
the slot; `DeviceMesh::ready` names the timeline value the batch set.
`extract_device` records the same over the map's device list (its last
compaction while that holds) and submits it once. Given `StageMetrics`, it
reports a
`"marching cubes"` row over the compaction's `"  ..compact"`, the submit's
`"  ..dispatch"` and a refit's `"  ..refit"`; its timer is made by the
first such call. A whole-map extract never
brings the active list to the host: it compacts onto the device, or takes
the map's last list back from it, and binds it in place (2026-09-30). An
`extract_device` overload meshes a `volume::DeviceBlockList` the caller
compacted instead — the viewers' frustum-culled set of what their camera
sees (2026-10-06), bound in place like the map's own. The arena is rebuilt
from that dispatch alone, so a block outside the list costs no triangles and
no *live* bytes — the arena is grow-only, so one full extract sizes it for
the whole set and it never shrinks back; the surface does not hole at the
cull edge, since the on-device probe still resolves neighbours that were
never dispatched; a list from another map, or one a later compaction,
allocation, resize, remove or clear has made stale, is refused by
`VoxelHashMap::check_device_block_subset` above the slot claim, so a refusal
is a **rollback** and an outstanding `DeviceMesh` survives it; and
`compact_ms` reads 0 while every row that scales with the active set shrinks
with it (`readback_ms` and `descriptor_ms` are per-call constants and do
**not**). A culled pass records its density like a full one: a viewer only
ever culls, and a plan left on the seed refits on every call.
`share_vertices` selects a second
compiled kernel that indexes in-block vertices — 3.4x fewer on room0, and
textured like any other mesh since the `texture` tier moved to a per-vertex
verdict (2026-08-11). `DeviceMesh::shares_vertices` still publishes it,
because `v = 3t` no longer holds and a consumer sizing an arena cannot derive
that from the buffers, and because the `texture` tier's several-view atlas
chooses per triangle and so refuses a shared mesh (2026-09-28).
A vertex on a corner whose distance is the iso-value is that corner,
`fl(k · voxel_size)` on each axis: it is selected, not interpolated, so it
is the same bits on every device and from both kernels. Every corner is that
rounded product before anything interpolates from it (`precise`, with the
voxel size splatted for MoltenVK; NVIDIA's compiler otherwise fuses it into
the interpolation). The edge ratio and `mix()` are still the device's, so an
interpolated vertex may differ across devices: on NVIDIA and Apple by an ulp
of the ratio times the voxel along its edge (an ulp of a coordinate, unless
the coordinate is small against the voxel), and in every coordinate on a
device whose `mix()` rounds all three, as lavapipe's does.
`MeshExchange<Payload>` (`mesh/mesh_exchange.hpp`) hands `extract_device`'s
meshes from the extracting thread to a renderer drawing them in place, each
with a payload that stays with it (its atlas), and owns the ring's release
(2026-10-08): the renderer's `begin_frame` retires a frame slot, computes the
mark below every generation a frame in flight, the live mesh or a parked
take holds, and takes the newest mesh, under one lock; the producer applies
the mark on its own thread before each extract. The consumer's commit
commits, parks or drops each mesh taken (2026-10-26). Header-only and
gfx-free; it takes any ring with `release_through`, so a host model of the
ring tests it. The `DeviceMesh` →
`LiveMesh` adapter is the viewers' `recon_gfx_bridge.hpp`.

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

### camera

`recon_camera` links `core_base` and GLM; the drivers, the sensor tier and
calib build on it. It holds only what a caller uses (the second 2026-10-06
decision): unprojection and rescaling come with their first caller.

- **`camera/camera_model.hpp`** — `CameraModel`: `ImageSize`,
  `PinholeIntrinsics` and `RationalDistortion` (OpenCV's eight coefficients),
  all double, and `check_camera_model`. Conventions: +Z forward, +Y down,
  pixel centres at integers, intrinsics in pixels of the model's size.
- **`camera/projection.hpp`** — `distort_rational<T>`, the lens's forward
  model, which `sensor/utils`' `lens.glsl` mirrors in float; the tests'
  reference.
- **`camera/geometry.hpp`** — GLM's double types, `check_rigid`,
  `rigid_inverse`, `nearest_rotation` (the polar factor, for an SDK's
  rotation that is not one), and Rodrigues both ways
  (`rotation_from_rodrigues`, `rodrigues_from_rotation`, accurate near 0 and
  pi). `RodriguesTransform` is OpenCV's `x' = R(rvec) x + tvec`.
- **`camera/color_controls.hpp`** — `ColorControls`, a colour camera's
  manual exposure (an exposure time and a gain in the camera's units, above
  0; auto exposure off, auto white balance on), which a rig's calibration
  records and a driver writes and reads back.
- **`camera/array_calibration.hpp`** — the sensor array's calibration file,
  version 2 under version 1's `device_calibration` key: per sensor by serial,
  an optional colour-camera pose, colour and depth `CameraCalibration`s
  (model and `factory` / `calibrated` source) and `depth_to_color`, and an
  optional world: a sensor at the origin, or a printed tag
  (`AprilTagWorld`, in OpenCV's marker axes, z up out of a floor tag).
  Version 3 adds a correction: a sensor's `depth_offset_m` (metres, + where
  its depth reads too far, at most 0.05 m), written only when it is not 0,
  or the array's `color_controls`, the colour cameras' manual exposure,
  written only when present; the file is version 3 only then, so an
  uncorrected calibration is written as version 2, byte for byte, and an
  older reader refuses a corrected one; a correction in a file that says
  version 2 is refused.
  `format_` / `write_array_calibration` write it
  through the core's `write_file`; version 1 is read for its poses.

### sensor

the sensor *interface*: `IRgbdSensor` (below) polled for an `RgbdFrame`
and asked `exhausted()` after an empty poll, since "nothing this tick" from
a live device and "nothing, ever" from a replay are the same empty optional
(2026-09-14; non-pure, `false` by default, so a live driver overrides
nothing) — plus the boundary math that is silently wrong when guessed —
`cv_from_gl_camera`, `depth_from_registered_color`, `to_canonical`, which
only the iOS scanner and the tests call until the scanner moves onto the
interface (2026-10-07). Links `recon_camera` and `recon_core` alone;
drivers live with the platform that can build *and* test them. The
implementer every dataset example runs is
`examples/common/replica_sensor.hpp`, which plays a Replica sequence back
through the interface, its frames prepared on the GPU as a camera's.
**`sensor/orbbec`** (`VR_WITH_ORBBEC`, which
needs the colour decoders, `VR_WITH_FFMPEG` off Apple, and the Orbbec SDK
2.10.6 or later) is the live Femto Mega, through the sensor interface below:
**`OrbbecSensor`**, one camera. A synced rig is an `OrbbecSensor` per camera
in a `SensorArray` (the 2026-10-07 rig decision), opened from its sync file by
`open_orbbec_sensors`. It hands out every frame
as captured, for the GPU pass; nothing on the host undistorts, registers or
converts (the 2026-10-06 raw-frames decision). It reads the camera's rig
sync role (`waits_for_primary`), and given the camera's entry of the rig's
**sync configuration** (`Options::sync`; `orbbec_sync_config.hpp`, the
SDK's `femto_mega_sync.json` layout) refuses a camera whose stored settings
differ, unless `apply_sync` writes them, only where they differ, since they
persist in its flash. The SDK's effective settings are compared again after
the write: normalization that still differs is refused, `Unsupported`;
its cached response does not verify persistence on the camera. At every
open it then sets the colour camera's exposure,
`OrbbecStreamOptions::color_controls` (a manual one, or auto exposure when
empty; auto white balance on either way, never off): checked against the
camera's controls and ranges before any write, written only where the
camera differs, and read back into `SensorInfo::color_controls`, a
difference refused naming the field (`write_color_controls`, internal; the
2026-10-27 colour-controls decision).
The lab rig's is `config/femto_mega_sync.json`, which
only the examples and tests name. No test opens a camera (the 2026-10-06
no-hardware decision); `fuse_orbbec` is how one is checked. Behind it is the
internal `CameraStream`, and the types of `orbbec_stream.hpp`: the stream
options, the camera's report, and its counters (`OrbbecStreamStats`).
H.265 at 3840x2160 and 25 fps, depth 640x576, is the default
(`color_codec = Hevc`; `Mjpeg` stays selectable, for calibration captures
and diagnostics): each camera's
`HevcColorDecoder` decodes every colour frame, in order, ahead of the
mailbox, as BT.601 full range unless the stream names its matrix, and posts
the picture with its depth, so everything after is MJPEG's path. A frame's
colour camera is read off the profile of the mode streamed. The SDK hands
over every colour frame, depth or not; a pair without depth is dropped
after decoding. A gap in the frame index, or an empty frame, waits for the
next key frame, where the decoder is reset; pictures come out in display
order, each settling its own pair. All of it is counted in `stats().lost`
(the 2026-09-28 decision).
**`sensor/video`'s decoders** (every Apple build, and elsewhere
`VR_WITH_FFMPEG`) hand out pictures on the device only (the 2026-10-06
device-only decoder decision), on the hardware of the device they are given,
fixed per build: VideoToolbox on Apple, directly, with no FFmpeg (the
2026-10-25 decision), and through CUDA elsewhere, so `VR_WITH_FFMPEG` needs
`VR_WITH_CUDA` (the CUDA 13 toolkit's headers; libcuda and libnvjpeg are
loaded at run time). A **`DecodedPicture`** is a frame's colour as it is: its
`YuvImage` (`DecodedPicture::yuv`: the planes, the queue family that wrote
them, the size, the matrix's weights, the range and the chroma siting), with
the stream's pts and its transfer and primaries as an optional
`ColorEncoding` (empty when that type cannot name them) beside it.
**`HevcDecoder`** decodes H.265 access units on the hardware. Off Apple,
through FFmpeg's hardware decoding: NVDEC's picture copied device to device,
by one CUDA copy, into two plane images CUDA imported, an R8 luma and an
R8G8 chroma of the picture's size (`core`'s `create_exported_image`, on a
device that `exports_memory`), handed out as NV12's `YuvImage::image`, as
VideoToolbox's are, with `kQueueFamilyExternal` (the 2026-10-26 plane-image
decision). A set of images is made, mapped and handed over to CUDA in
`GENERAL` once, and reused only once no picture holds either image; a
reader takes each picture back from outside Vulkan, and none hands it back.
The images are shared with `Options::queue_families` as
`PixelBufferImporter`'s are. Each CUDA mapping is held from the moment it
exists, so a failed mapping or host allocation releases it, in its context,
before the Vulkan image it maps. On Apple,
through a `VTDecompressionSession` of its own
(`vt_hevc.cpp`): it reads the parameter sets and each picture's slice
header itself (`hevc_syntax.hpp`, below), hands VideoToolbox the
picture's slices as one length-prefixed sample of a format made from the
sequence's VPS, SPS and PPSs (a new session only where a changed set needs
one), and puts the pictures, which come back in decode order, into display
order by their picture order count: shown once more wait than the SPS
reorders, and all at a sequence started afresh and at the end. The two
IOSurface planes are images in `YuvImage::image`, through `sensor/apple`'s
`PixelBufferImporter`, written at the range the VUI declares (video range
without one), so the samples are the stream's own. A picture whose
reference is missing, after a skipped access unit, VideoToolbox reports as
bad data, `IoError`.
`Options::unlabelled_color` stands in for a stream that names no matrix,
and `reset()` restarts a stream after lost access units (on Apple, on a
fresh session at the next key frame). A stream the hardware cannot decode
or hand out -- not 8-bit 4:2:0, or on VideoToolbox a display window off the
coded corner, which it fills from the corner instead -- is refused,
`Unsupported`, from then on, after the pictures decoded before it. On
Apple each SPS is read whole, to its trailing bits, before it can refuse
the stream: a malformed one is `IoError` and cannot latch a refusal.
VideoToolbox's decoder sessions for both decoders open in one place
(`videotoolbox.cpp`): NV12 on Metal-compatible IOSurfaces, on the hardware
decoder or not at all.
**`JpegDecoder`** decodes a baseline 8-bit 4:2:0 JPEG, BT.601 full
range as JFIF defines it, with no FFmpeg: on nvJPEG's hardware JPEG engine
into the same kind of buffer, I420, or through VideoToolbox's hardware
decoder (`vt_jpeg.cpp`) into the same images. Another subsampling, or a
JPEG larger than the hardware takes (16384 a side on nvJPEG's engine, the
device's image extent on Apple), is `Unsupported`, as is every JPEG on a GPU
with no hardware JPEG engine (an RTX 4090, say). For both decoders the
codes are the contract: `IoError` is data that does not decode and costs
only itself (and, for H.265, the frames to the next key frame);
`Unsupported` is a stream the hardware refuses, or a `create` with no
device path (no device or allocator, or a GPU the build's path cannot
reach); `Backend` or `OutOfMemory` is the device path failing. VideoToolbox
errors retain that distinction from session creation, the decode call and
its callback, for both decoders. Its decode-time `kVTParameterErr` also
denotes corrupt data (including a truncated JPEG fixture), so it is
`IoError` there. The
Orbbec driver's colour decoders stop for good on any code but `IoError`, and
the sensor's `poll` returns it, so a refused stream or a failing device path is
not quiet. A `Backend` failure names the library that failed
(`Status::backend()`): VideoToolbox, CUDA or nvJPEG, Vulkan for the hand-over
onto the device, or FFmpeg for its `AVERROR_EXTERNAL` (a hardware call
failed); a hardware failure FFmpeg reports otherwise reads as lost frames.
`VR_TEST_HEVC_BACKEND` makes the decoder tests fail rather than skip where
no device path opens, which is how CI holds its legs to NVDEC on 24.04 (and
nvJPEG where the GPU has the engine) and VideoToolbox on the Mac.
**`sensor/video`'s CUDA interop** (`cuda_interop.hpp`, `VR_WITH_CUDA`) is
public, and is how both decoders reach CUDA: `cuda_ordinal_of` finds the
CUDA device whose UUID is the Vulkan device's; `CudaContext` retains that
GPU's primary context, which every holder shares and none leaves current;
`CudaStream` is a non-blocking stream of its holder's own;
`CudaBufferMapping::map` imports a `create_exported_buffer` buffer's memory,
taking its descriptor (`InvalidArgument` once taken), and maps the whole
buffer, which stays the caller's; and `CudaImageMapping::map` does the same
for a `create_exported_image` image of 8-bit `UNORM` with one, two or four
channels, mapped as a one-level CUDA array of its format and extent,
flagged as a colour attachment where its usage makes it one, as CUDA
requires. The one crossing it documents is CUDA writes, Vulkan reads: the writing stream
or an event is synchronized, then the reader acquires the buffer or image
from `VK_QUEUE_FAMILY_EXTERNAL`, an image once handed over to CUDA in
`GENERAL` before its first write.
**`sensor/video`'s H.265 syntax** (`hevc_syntax.hpp`, target
`recon_sensor_video_syntax`) is public and builds on every platform: host
code on the family core's base tier, with no codec, GPU or FFmpeg (the
2026-10-25 public-syntax decision). It is what VideoToolbox's path reads.
`split_annex_b` finds an access unit's NAL units (`hevc::Nal`, borrowed,
emulation prevention left in); `vps_id`, `parse_sps` and `parse_pps` read
the parameter sets' ids and an SPS's size, conformance window, format,
reorder depth and VUI colour (`ColorCodes`, `color_codes.hpp`: the H.273
codes FFmpeg's frames are described from too), each SPS through its
trailing bits; and `parse_slice_start` reads a slice header as far as its
picture order count, given the parameter sets by id. Bytes that do not
parse are `IoError`, naming why; a NAL unit of another type than the call
reads is `InvalidArgument`; a host allocation failure is `OutOfMemory`,
never an exception. Every parser reads through `hevc::BitReader`:
the RBSP's bits, MSB first past the emulation prevention bytes, as u(n),
ue(v) and se(v), zeros past the end with `overrun()` set, and whether what
is left is the trailing bits.
**`sensor/apple`'s `PixelBufferImporter`** (`recon_sensor_apple`, on every
Apple build, iOS included, with no FFmpeg) hands an 8-bit NV12
`CVPixelBuffer` on an IOSurface to Vulkan in place: each plane becomes a
Metal texture imported as an image (`VK_EXT_metal_objects`), with a view and
`SAMPLED` and `TRANSFER_SRC` usage, so `GpuFramePrep` samples it and
`PictureReadback` copies it, made once per
surface and kept while pictures keep arriving on it, the images holding the
pixel buffer. The images are EXCLUSIVE unless `create` is given the queue
families that read them, by `BufferDesc::queue_families`' rule with the
device's own family among them: two or more distinct make them CONCURRENT, so
a renderer on another family (gfx's, on MoltenVK) reads them in place. The
decoders' `Options::queue_families` and
`OrbbecStreamOptions::color_queue_families` pass the families through, and
NVDEC's plane images take them by the same rule (one check,
`sharing_families.hpp`); nvJPEG's buffers ignore them and are taken over
from `kQueueFamilyExternal`. It sets a `YuvImage`'s planes and leaves the
matrix, range and chroma siting to its caller, who alone knows how the
samples are coded (a stream's unlabelled colour, say). An iOS camera's
pictures, such as ARKit's `capturedImage`, can come through it with no
conversion on the host.
**`IRgbdSensor`** (`sensor/rgbd_sensor.hpp`) is one sensor: its
`SensorInfo` from when it opens (id, the cameras' factory models at the
opened modes, `depth_to_color`, rig role, clock, pose source, rate, the
colour exposure as read back), and
frames held up to `set_queue_depth`, the newest taken by `poll` or all,
oldest first, by `drain`. **`OrbbecSensor`** implements it: `open` refuses
a `min_depth` of 0, which the GPU pass would, a camera in a sync mode the
driver does not know, and one whose sync settings differ from
`Options::sync` without `apply_sync`; `sync_clock_to_host` stamps each frame
with the SDK's global timestamp, the camera's clock mapped onto the host's
`std::chrono::system_clock` and re-fitted as the two drift, refusing at
`open` a camera without one, and sets the camera's clock to the host's once
at each `start`, before it streams (`timerSyncWithHost`, never the
process-wide `enableDeviceClockSync`); and its `stats()` counts the driver's
lost frames failed, a JPEG the decoder had no time for being dropped.
**`SensorArray`** (`sensor/array/`, target `recon_sensor_array`) reads
several `IRgbdSensor`s as one: it starts the secondaries before the primary,
drains every sensor each `poll_set`, groups the frames into a `FrameSet` by
its internal `TriggerGrouper`, around the primary's frames on the host
clock within `tolerance_us` (by default 0.4 of the fastest member's frame
period, 13.3 ms at 30 fps), and stamps each frame with its sensor's pose
from the `ArrayCalibration`, which must pose every sensor and, where it
names `color_controls`, find every sensor opened at them
(`SensorInfo::color_controls`), or the open is refused. `wait_set`
blocks for a set on one condition variable that every sensor's
notification wakes (`IRgbdSensor::set_frame_notify`, which `open` asks of
each; the Orbbec driver notifies from its mailbox as a pair lands, the camera
goes or the decoder fails), and on the grouping's next deadline, when a
trigger stops waiting for a late sensor; it never sleeps a fixed time. A
set's `arrival_ns` is its latest frame's (`RgbdFrame::arrival_ns`, the
system clock at the driver's frame callback). Opened with a
device, it makes a pass per sensor from `Options::prep`, each with its
sensor's `depth_offset_m` from the calibration, the offset's one source (an
offset in `Options::prep` is refused); its `record_process(batch, set)`
records a set's preparation into a
caller's batch through `GpuFramePrep::record_set`: every frame checked, then
every pass's copies and every pass's kernels; `process(set)` is that, one
submit and its wait. A `FrameSet`'s
`skew_ns(i)` is sensor i's frame's time less the trigger's, signed, and its
`spread_ns()` the latest of its frames' times less the earliest.
`RigSets(array, frames)` (`sensor/array/rig_sets.hpp`) reads a run of N
frames from a started array: `next` waits for each set (`wait_set`), keeps
each secondary's skew to the primary, and cuts the set that reaches N to its
first frames in the array's order; after `kSilenceLimit` (10 s) with no set
it returns `IoError`, naming a lone sync secondary's cause. The prep stays
the caller's, recorded into the caller's batch (2026-10-25).
**`sensor/utils`'s `GpuFramePrep`** undistorts an `RgbdFrame`
(`sensor/rgbd_frame.hpp`) on the device: depth sampled at the nearest pixel,
colour bilinearly and converted from Y'CbCr in the same pass, each camera
keeping its intrinsics. The frame carries each camera's `camera::CameraModel`
in double, narrowed to float once for the passes; the colour camera's pose,
`color_to_world`, and the sensor's `depth_to_color`, both refused unless
rigid, pose the outputs; a sequence number; and `pixels`, the owner of its
depth, any host colour and any `color_bitstream` (the colour as the camera
coded it, when that decodes on its own), so a consumer may keep frames past
the next poll and past the capture.
`ChromaLocation` travels in the picture's `YuvImage`: JPEG is centred, and
HEVC keeps the decoded tag with left alignment when unspecified.
The GPU samples the location consistently for planes in a buffer and NV12
images. Existing callers that leave the field unset keep left alignment. The
resulting `DeviceFrame` feeds the `Buffer` overloads of `allocate_from_depth`
(its camera through `to_depth_pinhole`) and `integrate` (and
`ColorFrame::buffer`, with `coverage_in_alpha`, since a
pixel the lens maps outside the picture is a 0 word), so nothing is
uploaded and nothing registered. Y'CbCr colour comes as I420 or NV12 on the
device only (the 2026-10-06 device-only `GpuFramePrep` decision): planes in a
buffer (`YuvImage::device`, with per-plane offsets and strides) are bound
where they are, from
the first plane, once the batch has taken them over from the queue family
that owns them (`YuvImage::queue_family`; the 2026-09-28 decoded-frame
decision). NV12's planes may come as images instead (`YuvImage::image`,
with `SAMPLED` usage and a view), taken over the same way in the layout they
are in (an EXCLUSIVE image only from outside Vulkan), which the colour pass
samples where they are, with no copy: a second entry over the same body
(`undistort_color_images.comp`, `undistort_color.glsl`) binds them as two
combined image samplers, reads them at integer coordinates and makes each
sample its 8-bit code again (`round(v * 255)`), so its words are the buffer
form's, bit for bit (the 2026-10-25 plane-image decision). A dataset decoded
on the host hands its colour over as packed R'G'B' words instead
(`RgbdFrame::color_packed`, `io::load_color_packed`'s layout; the
2026-10-07 packed-colour decision), copied into the pass's slot after the
depth; the colour pass samples R, G and B as byte planes four bytes apart,
through the same bilinear sampler and coverage test. The depth, and any
packed colour after it at the binding alignment, is copied into the pass's
one persistent slot (`core::HostRing`), which the batch binds in place where
mapped device memory is cached and copies once into its device-only mirror
elsewhere; nothing is allocated per frame (a batch staging per call made VMA
allocate a block for every set: 46 ms a rig set on an RTX 5090, 4.8 ms
kept). `record(batch, frame)` records the copy and both passes into a
caller's batch, and `prepare` is that, one submit and its wait; a record
before the pass's last batch resolves is refused, as it binds the slot. The
frame *holds* its device-local
buffers through a lease, and the pass reuses one only once no frame holds
it, so a frame kept past the next is still itself; the whole frame is
checked before anything is recorded or timed, a depth range from 0 and a
plane's queue family included, and `prepare` is `prepare_batch` over a set
of one. Both return `OutOfMemory` for a host allocation failure,
preserving previously returned frames.
`depth_within_color` (off by
default; `rig_viewer` turns it on) zeroes depth outside the colour camera's
view, by the colour pass's own coverage test, so nothing is fused that no
colour camera can colour. `depth_offset_m` (0 by default) is subtracted
from every return in the depth pass, before that test: for `z > 0`,
`z = max(z - offset, 0)`, so 0 stays no return whatever the sign; a
`SensorArray` sets it per sensor from the calibration.
**`sensor/utils`' `PictureReadback`** copies a set's colour pictures to the
host, luma alone or every plane, for calib's detector: NV12 images from
their corner, and planes in a buffer where they sit, after the batch takes
either over from `YuvImage::queue_family`, images in the layout they are in;
every copy in one batch, one submit and one wait. Both read overloads take
mutable pictures/frames and, after a successful submit and wait, update each
picture's `queue_family` to the readback's family (EXCLUSIVE) or
`kQueueFamilyIgnored` (CONCURRENT, and images that needed no transfer).
Subsequent reads and preparation use the updated picture;
copies made before readback still carry the old ownership metadata.
Creation, including the GPU timer, and reading return `OutOfMemory` for
host allocation failure. The copies land in host memory it keeps (`Staging`,
cached for reading), grown to the largest read, and each `PictureView` points
into it until the next read, so the frames may go at once. A picture is
checked by the rules `GpuFramePrep` applies to one (`yuv_planes.hpp`), with
`TRANSFER_SRC` where the pass needs a buffer's storage usage, or images'
`SAMPLED` usage and views.
An Orbbec frame holds its SDK pair and the SDK
context, with the models and the depth-to-colour extrinsic from the factory
calibration (the extrinsic's rotation made one: the Femto Mega's is not, the
2026-10-06 sensor-frame decision; `open` refuses one that does not come out a
rotation), the depth frame's index as the sequence, and the planes
converted by the matrix and range the stream codes them in. Given
`OrbbecStreamOptions::device`, its H.265 colour is decoded onto that device
and stays there, the picture carried through the mailbox by an SDK frame
whose bytes only name it, so a copy of the frame owns nothing
(`picture_frames.hpp`); for MJPEG it streams the camera's JPEGs, and each
camera's `JpegColorDecoder` decodes them on a thread of its own onto the
device, its picture's frame keeping the camera's frame as well, so the
frame's `color_bitstream` is the camera's JPEG, byte for byte. An H.265 frame
carries none, since it decodes only after the frames before it.
`OrbbecDeviceInfo` names each camera's lens model as the SDK reports it at
the opened mode (`brown-conrady-k6`, say). Each colour decoder, MJPEG's and
H.265's, decodes on a `DecodeWorker` of its own (`decode_worker.hpp`), which
owns its queue, what a full one drops, its thread, and the failure that ends
decoding.

### codec

The codec is implemented, intra frames and P-frames (frame type 1), and it
codes and decodes on the device alone (2026-10-13): the host does the I/O
around it (a frame's bytes, and their hash) and the control (a frame's
header, sections and tables, and small counts and status words, a decode's
and an encode's read from results slots once their batches have run).
The CPU rANS coder is the tests' oracle, under `tests/`
(`codec_oracle*.hpp`), which pins the format's bytes.
Quantization is one scalar quantizer with a step per DCT basis (2026-10-02):
`step[u,v,w] = quantization_scale * quantization_weights[u + 8*v + 64*w]`.
DC is the table's first entry. All blocks share the same 512-entry table, but
only the K kept bases use their weights: the global scale and those K weights
travel once in the self-contained v3 frame.
The defaults retain room0's K = 64 and uniform effective step 0.2, expressed
as scale 0.2 and weights of one. Frequency-dependent tables are measured
candidates, not a claim that high frequencies always deserve fewer bits.
The public API
is `CodecParams`, **`Encoder`** (`encoder.hpp`), **`Decoder`** with
`read_frame_info` (`decoder.hpp`), and a player's `make_decode_grid` and
`decode_growing` (`decode_grid.hpp`). A frame is two batches, the caller's
(the 2026-10-25 entry): `Encoder::record_front` records, after what the
batch holds (a sender's fusion), every step whose count only the device
knows, and `record_back` the rANS coding, sized on the host from the
front's results; `finish` lays the frame out, its payload left in an
encoder slot (`EncodedFrame`), and `encode(grid)` is those, a submit and a
wait each, the head and payload joined: 2 submits. The front takes a
const grid and the map's own active list -- the one it holds
(`held_device_list`), or one compacted into the batch
(`record_compaction`), which leaves the map none, so an extract after
compacts its own -- keeps the blocks with an observed voxel, or with
`EncoderConfig::true_sdf` those holding surface (`DctTransform::record_observed`,
their count left on the device; the true-SDF passes below share it), sorts
them by (z, y, x) on the device with a stable rank-merge sort on
`coord_less` itself, over all of int32 (`detail::BlockLists`, whose header
has the mechanism), its last level writing the reference store's spare list,
and transforms them into the spare's masks and coefficients (below). Every
step is sized for the grid's block slots and runs for the device's count:
the observed pass and the forward from indirect slices, the sort's levels
recorded for the bound, a level the count does not need copying its entries
through. Keys are unique, so the same content gives the same bytes whatever
the hash table's order. A frame whose front finds no block to code is an
I-frame of none (below), with no back batch.
At `EncoderConfig::intra_period` N above 1 (default 1, 0 refused) the
`Encoder` writes P-frames (the 2026-10-14 entry): an I-frame first, then
every N coded frames, after `force_intra()`, when the grid's `voxel_size` or
`trunc_dist` is not the reference's (`coding_difference`, the writers' and
reader's rule), when the reference holds no block, and when the grid holds
none (the 2026-10-16 entry, decided by the device's count, so a P-frame
planned at record finishes as an I-frame of none); a P-frame otherwise,
against the frame returned last. Its reference is what the decoder
will hold of that frame: the forward's q and masks and the sorted list, left
in `detail::ReferenceStore`'s spare and swapped in when `finish` returns the
frame, so a refusal or `abandon` keeps the reference and the period's count.
In the front's batch, `inter_diff.comp` splits the frame against the reference
and codes it, an invocation an entry: each reference entry finds its block
by binary search (remove, skip when every word is equal, else keep or zero),
`survivor_ranks.comp` counts the survivors, and each block the reference
lacks takes its place among the new blocks from those ranks. A residual of
−32768 codes zero, and so does a block whose target payload a static price
(`emit_payload` walked at `bit_length + 1` bits a symbol, 8 a byte symbol,
plus raw bits) puts strictly below its residual's. Only the device knows the
frame's blocks and its new blocks' count, so the front counts a P-frame, at
`EncoderConfig::update_size` entries an update segment
(`EncoderConfig::resolved_update_size()`: the segment size unless the caller
sets one), over M plus the bound's blocks, and the device takes the blocks
the frame lacks and the survivors off (`counted_entries`, from two frame
words). `EncoderConfig::validate` holds the config's checks, which `create`
makes first.
The frame's rANS coding runs on the device (`detail::DeviceFrameWriter`,
built by `Encoder::create`, which fails if its kernels do not build): the
forward output stays in VRAM, the front also counts every model's symbols
and each block's coder steps and lays the steps out (`detail::StepLayout`,
whose header has the formulas: one workgroup scans the steps into each
entry's place, and the segments' slot sizes into their offsets), and its
last command (`encode_results.comp`) gathers what the host needs -- the
symbol counts, the layout's two totals, the frame words and the reject and
refusal words -- into a results slot (`core::HostRing`), from which the
host builds the tables (`frame_tables`) and sizes the coding's buffers. The
back half uploads the tables inline first, writes every block's steps in
parallel, codes one segment per invocation into its slot, scans the lengths
into the payload's offsets and a second results slot, and packs the streams
into a payload slot: in place on unified memory and lavapipe, the slot
itself; copied on a discrete GPU, the payload's likely size (the last
frame of its type's bytes per entry plus 25%), any rest gathered again into
its own output range by `finish` (a tail, `"  ..tail"`). The frame's
reference hash runs over the head and the payload unjoined (`frame_hash`'s
two-run form). Bulk buffers are device-local and retained. A segment is one
serial chain, so smaller segments give the device parallelism at a few
percent of size. A frame the device cannot hold is refused, never coded on
the host.
Both device coders work on entries, a P-frame's stream positions
(`ResidentBlocks`): its reference entries, each with a u32 mode, then its new
blocks, which alone the block list holds; an intra frame is all new blocks.
The count pass also flags what every writer refuses of the entries (a mode
past the alphabet, a written −32768), which `record_back` refuses in their
words (`check_entries`).
The `Decoder` decodes on the device (`detail::DeviceFrameReader`): the host
parses the frame (`parse_frame`) and uploads only its tables and payload.
One batch expands each table into a 4096-slot lookup, decodes one segment per
workgroup into the inverse transform's layout, judges the segments
(`rans_verdict.comp`: the smallest failing segment, by its fault or its first
coordinate against the block segment before's last) and scans the survivor
ranks (`survivor_ranks.comp`, on `scan.glsl`), and reads back 8 bytes, the
verdict and the survivor count, which `check_segment` and `check_survivors`
word exactly as the oracle's reader does. Every decoded entry stays in VRAM
for the merge, the apply and the inverse (`DctTransform::record_inverse` over
`ResidentBlocks`). One invocation
decodes a whole segment, so the format holds a segment to at most
`kMaxDeviceDecodeSegmentSize` (1,024) entries, keeping an invocation near
50 ms: every reader refuses a frame whose segment size or update size is
larger, before any device work, and every writer, `Encoder::create`
included, refuses such a size (`check_segment_size`), so every valid frame
decodes on the device. A frame past `maxStorageBufferRange` or free memory
is refused with its code (`InvalidArgument` or `OutOfMemory`); nothing
decodes on the host.
`Decoder::decode(frame, grid)` leaves the caller's grid holding exactly the
frame. It is `Decoder::record` into a batch, one submit and `finish` (the
2026-10-24 one-batch entry): the record is parsed in place in an input
slot, whose payload the reader binds at its byte offset, and the whole
decode is recorded with every count on the device. `detail::BlockLists`
merges a P-frame's surviving reference blocks with its new ones into the
frame's list and each block's sources, finds row 13, a new block on a
reference block, and diffs the hash table against the frame's list into the
blocks the grid lacks (missing) and those the frame lacks (gone), by count
and binary search (the header has the mechanism). Nothing sorts. A gate
kernel (`detail::FrameGate`) judges rows 11 to 13 and opens the grid's
removal of the gone blocks and allocation of the missing ones from those
device lists (`VoxelBlockGrid::record_remove`,
`VoxelHashMap::record_allocate`), whose rounds close it on failure; behind
it, sized by N on the device, the frame keeps shared blocks in their slots,
rewrites every voxel's `tsdf` and `weight`, and stamps `changed` only on a
block it leaves different. A frame the gate refuses moves the map's epoch
and tick and leaves the grid untouched. The counts are gathered into a
results slot, which `finish` reads. Attributes, geometry (exact
`voxel_size` / `trunc_dist`, off the header), parsing and the heap are all
checked before the grid is touched. The attributes must be `tsdf` and `weight` and
nothing else (`VoxelBlockGrid::attribute_count`), since a kept block would
carry any other one stale. A grid too small for the frame is `OutOfMemory`,
whether its heap has too few slots or its hash table cannot place the
blocks. Both are recovered by growing the grid and decoding again; the
decoder never grows one. Beside it, as `tsdf::Fuser` grows for fusion,
`decode_grid.hpp` holds a player's growth: `make_decode_grid` builds a
grid from a frame's header in `volume::block_grid_params`' layout, its heap
twice the frame's blocks, and `decode_growing` decodes into it, growing it
once on `OutOfMemory` and
decoding again twice at most on `IoError`; `record_growing` grows a heap
the frame outgrows before it records, and `finish_growing` decodes a frame
the device refused again from its own input slot. Both take the caller's ceiling,
`max_blocks`: a frame past it is refused before it is decoded, and a grow
past `make_decode_grid`'s grid for it before it is made, each as
`InvalidArgument`; the factory checks the header's block count against the
segment table as the decoder does. The examples pass the source grid's
slots. The factory contains host allocation
failures from header parsing through grid creation. The growing decode merges
only its successful attempt's metrics; if that merge runs out of host memory,
the caller's prior metrics stay unchanged and the decode still succeeds, since
its reference has committed. Lock contention that
outlasts the device rounds is `IoError`, never `OutOfMemory`, and a free heap
that refuses a removed block is `InvalidArgument`. A failure after the
grid has changed leaves it holding neither frame until a decode
succeeds. A P-frame decodes against the frame the `Decoder` decoded last,
which `detail::ReferenceStore` keeps as decoded: its quantized coefficients,
masks and block list on the device, in the inverse's layout, and its
`FrameReference` on the host, never rebuilt from a grid. The merge writes
each frame block's sources (the 2026-10-12 entry) into the store's spare,
`inter_apply.comp` makes the blocks there with integers alone, counting any
source past what is bound, and the inverse reads the spare in the same
batch; the spare becomes the reference only when the whole decode
succeeds. An intra frame takes the same apply from its payload alone (the
2026-10-13 entry). With `Decoder::set_deblock` on (off by default; the
2026-10-21 deblock entry), the same batch then smooths the frame across its
block faces (`detail::Deblock`, whose header has the filters): a line fit across
each face, and at coarser steps a Gaussian after it. The host picks the
filter per frame from the header alone: K and T, the step of DC and the
zigzag's next three coefficients at their basis peaks, in millimetres,
select a row of `kDeblockRule`, the study's rule; a fine step runs none. A
masked block, holding an unobserved voxel, keeps its exact decode (that
entry's 2026-10-23 amendment): a first dispatch, an invocation a block,
flags each fully observed block by slot in a word a slot zeroed earlier in
the batch and counts the masked blocks; every later kernel returns,
uniformly and before any probe or barrier, for a block whose own mask words
are not all ones; a line is fitted across a face only when the neighbour's
flag is set too, so neither side of a face with a masked block is
line-fitted; the Gaussian holds a masked neighbour's voxels at their
decoded values through all three passes, its unobserved ones dropping out;
and the count joins the results slot, beside the inverse's reject count.
The line passes run in place on `tsdf`, a dispatch an axis, a
64-lane workgroup a block and a lane a face line, probing the block and its
+axis neighbour through `vrResolveOctants` (`block_neighbourhood.glsl`).
The Gaussian is one dispatch a block over the block and the 1-voxel halo of
its 26 neighbours, in shared memory; it writes the interior in place and
the shell, the voxels a neighbour's halo reads, into a snapshot by list
position, which a store dispatch copies back. A neighbour's observed voxels
come from its weights, and each block is found by coordinate. Only fully
observed blocks' `tsdf` change, and a block the deblock writes is stamped
`changed`. P-frames decode from the reference's coefficients, so the switch
may change between any two frames.
`Decoder::rebuild` reconstructs the last decoded frame from the held
reference with the switch as it is now, into the grid that decode left
(the grid diff checks its blocks first, on the device, gating the rest), in
one batch and one submit, so a paused player shows a toggle;
`Decoder::last_deblock` reports the filter, T, the frame's blocks and those
the fallback kept at their exact decode. Both classes report `StageMetrics`
(`"codec encode"` / `"codec decode"`). Their breakdown rows share no name;
the encoder's compaction is `"  ..encode set"`, apart from a fuse's
`"  ..active set"`. The private pieces under
`src/volumetric_kit/recon/codec/` are the `DctTransform`, the rANS tables,
the frame's layout and checks, its device coders, the block lists and the
reference store. The transform's forward takes a device block list to
`ResidentBlocks` — K
quantized 16-bit coefficients per block in 3-D zigzag order and a 16-word
observed mask, left on the device for the writer — and its inverse takes
them back, each recorded into the caller's `CommandBatch` over device-local
buffers. The forward writes into its caller's buffers, the
encoder's reference store; the compacted observed list and the sort's two
halves are retained and grow only when a call needs more capacity, releasing
the old buffer first and taking 1.5x headroom (`ensure_device_scratch`);
descriptor ranges and transfers use the current logical sizes.
They remain allocated until their owner is destroyed. The per-call staging
inside `CommandBatch` remains transient. The SDF is normalized by
`trunc_dist` before the transform and the steps are fractions of it, so the
decoder checks a frame's `trunc_dist` against its grid's before the inverse.
`CodecParams::validate` checks the scale,
each kept basis's weight and its effective step; weights beyond K are unused.
The scale and kept weights must be normal, so a flush-to-zero host or device
agrees on validity. It refuses a step
small enough for the ±32767 clamp to engage (√512 / 32767), and one past
`kMaxStep` (64), where every coefficient is 0 and a decoded one could
overflow to infinity. The forward
never reads an unobserved voxel's `tsdf`: it fills each one from the nearest
observed voxel along x, then y, then z, since a fused block's zeros there
are a step K = 32 cannot hold, and they decoded a partially observed block
with 2.8x the error. The transform refuses any block size but 8; more
entries than the grid's heap, or a list buffer too small for them; and, on
the device with one hash probe per block, any entry whose
coord the grid does not hold, which the inverse then writes nothing into.
Each block is found by that probe and its ptr never read, so the decoder
lists blocks it has just allocated by coordinate alone. "Observed" is
`volume::kObservedWeight`, the threshold the mesher reads too. The inverse
writes weight 1.0 on observed voxels and a fresh block's zeros elsewhere.
With `EncoderConfig::true_sdf` (the 2026-10-21 entry), `dct_surface.comp`
runs twice over the active list before the observed filter, a workgroup a
block, each lane taking the 8 cells anchored on its x-line, their corners
reached across the block's +x/+y/+z faces through
`volume/shaders/block_neighbourhood.glsl`, the mesher's neighbourhood probe
and corner addressing. A cell counts when its 8 voxels are observed. The
first pass sets, in each counted cell whose sign changes, the cut bit of
every corner with |tsdf| > √3 h + 1e-6 m; the second flags each block
holding a corner of a counted cell with no corner cut, whose sign changes or
that has a corner with |tsdf| < h; the filter appends the flagged blocks.
Metres on the raw `tsdf`, strict comparisons, below 0 inside, and every
write an `atomicOr`, so the order the workgroups run in changes nothing. The
forward reads the cut set: a cut voxel is unobserved, its mask bit 0 and its
value filled. The cut set (a bit a voxel slot) and the keep flags (a word a
block slot) are retained scratch too, sized to the grid's heap, zeroed in
each batch and bound on every call, since a regrown one is a new buffer.
The v3 frame (`bitstream.hpp`, the 2026-10-02 entry) carries the K kept
weights in its parameter header, `44 + 4K` bytes (300 at K = 64); older
versions are refused. `read_frame_info` reads them without entropy decoding.
Header byte 6 is the frame type (`FrameType`, which `read_frame_info`
reports): intra (0), or a P-frame (1), coded against the frame before it
as `bitstream.hpp` lays out and the 2026-10-12 entry explains. The device
writer and reader, and the oracle's, code both, and the `Decoder` decodes
both; every reader refuses a later type as `Unsupported`.
The payload is segments of R sorted
blocks (default 64). Each segment is one independent rANS stream
(`rans.hpp`: a 32-bit state, 16-bit words and 12-bit probabilities, integer
only, which the kernels code and the oracle pins byte for byte). Every integer
is coded as a class from a fixed per-frame table plus one raw field, its
sign the low bit. A partial mask is coded a z plane at a time and a line
at a time, each against the one before it (v2, 2026-10-01: the mask had
been half of room0's bits). The models and a block's grammar, its writer
and its reader, are one GLSL text that the kernels include and the oracle
compiles as C++ (`shaders/frame_models.glsl`, which `bitstream.cpp` compiles
too, and `shaders/frame_grammar.glsl`, 2026-10-08), and golden frames in the
tests pin v3's bytes for the device writer and the oracle's on every
machine. The format
requires strictly increasing coordinates, so a decoded list is duplicate-free:
the delta code cannot step backwards within a segment, and the reader checks
each segment's raw first coordinate against the one before. A flag bit other
than `kSectionRequired` is reserved, so a known section that sets one is
refused. `parse_frame` never reads outside its buffer, and it takes the
caller's `max_blocks`, because a frame's size cannot bound its block count
(probability-one blocks cost no bits). It checks that limit only once the
count agrees with the segment table, so a sound frame past it is
`OutOfMemory` and a corrupt count stays `InvalidArgument`. Every size is
checked in 64 bits so a 32-bit build refuses rather than wraps. A segment's
end check (every word consumed, the state back at its start) is a
**consistency** check, not an integrity one. It cannot see a flipped raw
bit, nor a symbol swapped for one of equal frequency, so integrity is the
transport's: the CRC-32 of the `.vrst` record that carries the frame
(`stream`).

### eval

`MeshDistance` (point-to-surface distance up to a reach,
through a hash of cells half the reach on a side, searched nearest first
and pruned by distance, over a **copy** of the triangles),
and `ReferenceMesh`, which indexes a reference once so a sweep can judge
many meshes against it, giving accuracy, coverage and an optional F-score. They refuse, with `Status`, what would read out of
bounds, overflow or mean nothing:
- a bad reach, or indices out of range;
- a corner that is not finite or past the cell keys' range;
- a reach so small that the triangles would average more than
  `kMaxCellsPerTriangle` cells each;
- a threshold past the reach.

The surface is every triangle but one collapsed to a point. The points
measured are the vertices those triangles use. The closest point is the face projection
when it lands inside, else the nearest edge, so a degenerate triangle
counts as the segment it collapses to and a thin one is measured to float
rounding, where Ericson's region test lost it now and then. A `stride` picks
vertices by a hash of their position, so the figures reproduce whatever
order marching cubes' atomics emitted the mesh in.

### io

`io::load_color_packed` decodes JPEG/PNG to top-left row-major encoded RGB
bytes in the existing `R | G<<8 | B<<16` packing. It performs no implicit
linearization or profile conversion; the capture adapter declares the input
encoding. `io::load_depth_u16` requires a genuine 16-bit single-channel PNG
and returns its samples as stored: what a unit is, is the caller's to say
(the Replica sensor's `RgbdFrame::metres_per_unit`).
Expected dimensions are explicit so a camera cannot silently use different
intrinsics. `io::write_png_rgba8` takes already-encoded RGBA8.
`io::encode_png_u16` encodes 16-bit gray samples as stored, and
`io::write_png_u16` writes them; stb writes 8-bit only, so it encodes the
samples' big-endian bytes as two 8-bit channels and IHDR is rewritten to
16-bit gray. The writers open their output only after encoding succeeds; PLY
export converts linear mesh colors to canonical sRGB bytes (clamped, NaN as 0)
and preserves triangle indices.
Errors name the file and backend reason. Public functions validate their input
and translate backend exceptions that reach them into `Status`/`Result`
(tinyply's writer is `noexcept`); private third-party types never appear in
public headers.

`recon_mesh_io_assimp` adds `io::load_mesh`, which returns the owned
`io::TriangleMesh` position / index container of `io/triangle_mesh.hpp`.
Assimp handles format decoding and triangulation. Recon walks every node
instance with composed transforms and corrects triangle winding when the
determinant is negative. Exactly coincident positions join across UV/material
seams, with no tolerance-based merging. This is geometry import, not topology
repair: degenerate and duplicate triangles are retained, and manifold/sign
checks are the consumer's, through `io::audit_mesh_topology` if it opts in.
`io::check_mesh_file` holds the path checks `load_mesh` makes before it
reads, for a caller that checks a sequence's files before loading any.
Non-triangle primitives, animation, skinning, morph targets and invalid
transforms on mesh instances are refused; nodes without meshes are not
checked.

`io::load_textured_mesh` (`io/textured_mesh.hpp`) runs the same traversal
with each vertex keyed on its position and first UV channel; `load_mesh`
keys on the position alone, so its output is unchanged. An OBJ's diffuse map
path is relative to its material library, which the loader finds through an
`Assimp::IOSystem` that notes each `.mtl` Assimp opens. Assimp 6 already
prefixes the path with the library's directory relative to the OBJ, even an
absolute path; the loader removes that prefix and resolves the rest against
the library, as it does on earlier releases, which add none.

Mesh coordinates retain the imported scene's scale; neither a metre unit nor
a target height is guessed. No Assimp normalization or handedness flags are
enabled, and importer unit/up-axis conversions are switched off where the
installed Assimp allows: before 5.3 it still applies Collada's `<unit>`, and
5.4.x converts FBX regardless. This is a host boundary for file input and output, separate from
GPU-resident reconstruction. The default build has no Assimp dependency;
`VR_WITH_ASSIMP=ON` finds an installed Assimp and builds a separate target.
The image and PLY backends use the existing pinned stb and tinyply sources,
now private implementation dependencies of `recon_image_io` and
`recon_mesh_io`.

`io/mesh_normalization.hpp`, in `recon_mesh_io`, holds opt-in helpers for a
caller that knows its mesh's unit or up direction; nothing in recon calls
them. They serve one mesh or a temporal sequence alike:

- `up_axes` turns a head-up direction into the proper rotation that takes it
  to +Y. `mesh_bounds` measures positions along those axes, and
  `extend_bounds` joins another frame's, exactly and in any order, so a
  sequence is measured one frame at a time.
- `fit_transform` (to a height in metres) or `scale_transform` (a given
  `metres_per_unit`) makes one `MeshTransform` from the bounds, centring X/Z
  and putting the lowest Y at zero. `apply_transform` maps every frame
  through it in double precision, rounded to float, so a still point stays
  still. The fits and `apply_transform` refuse axes that are not a proper
  rotation, and a refusal leaves the mesh unchanged; the fits also refuse
  bounds they would place past float range. The source compiles
  with `-ffp-contract=off`, so a transform's doubles do not depend on the
  compiler.
- `encode_transform` and `decode_transform` are a transform's one byte
  form, bit for bit (the layout is in the header): what a `.vrst` Geometry
  track's params hold, so a player can place decoded geometry back in the
  source's coordinates.
- `audit_mesh_topology` counts the edges, triangles and components the
  signed conversion depends on, capping each hole before it takes a
  component's volume; `supports_signed(allow_holes)` gives the verdict.

### stream

`recon_stream` writes and reads `.vrst` files, version 0: a `VRST` session
header, the records appended after it in `RECD` chunks, and on `finish` a
footer index (`INDX`, `TAIL`). The byte layout, the CRC and the rules a
record follows are written down once, in `stream/session.hpp`. Payloads are
stored and returned byte for byte. `TrackKind::Geometry`, a codec frame per
record with key meaning intra, its params empty or a mesh source's
transform, is the only kind defined; the writer takes any nonzero kind and
the reader passes an unknown one through.

**Writing** (`StreamWriter`, one thread). `create` makes the file with
`O_CREAT | O_EXCL`, so nothing at the path is touched, a symbolic link
included, then writes the session header and flushes the file and its
directory. Each `append` checks the record rules, then is one `writev` with
no buffering, so a killed process loses nothing it appended. `sync` flushes
to storage (`F_FULLFSYNC` on Apple); a power loss can lose what came after
the last one. A failed write or flush sticks: every later call returns it,
and the file reads as a killed writer's. `finish` writes the footer, flushes
and closes.

All fallible stream calls contain host allocation failures, including
validation diagnostics and copies of a saved failure, as `OutOfMemory`.
Validation or preparation allocation failures leave the writer retryable;
a failed write or flush remains sticky even if its diagnostic cannot be
allocated.

**Reading** (`StreamReader`). `open` checks the session header, then takes
each track's counts (`stats`) and key records (`key`, and `key_at` for the
last key at or before a pts) from a finished file's footer index, reading no
record. The checks a footer must pass are `Recovery::finished`'s: the
offset chain, the `TAIL` naming an `INDX` that ends where the `TAIL` starts,
is what makes the two a footer, and the key count is bounded before anything
is sized by it. Any other file `open` scans, one chunk header per chunk and
never a payload. `next` returns record headers in file order from a
`Position` the caller holds, skipping other chunks, and `read` checks the
header again and the payload's CRC, into caller memory such as a decoder's
input slot (allocating nothing) or a vector it sizes. Every method is `const` over `pread`, so
threads share one reader. A payload above `ReaderOptions::max_payload_bytes`
is `OutOfMemory` before any allocation.

**Recovery.** A whole header whose body runs past the end starts the torn
tail. A header that fails its CRC or holds what no chunk can starts a
search, in 8-byte steps, for the next `RECD`, `INDX` or `TAIL` with a valid
header; the bytes skipped are a damaged span, or with no chunk after them
the torn tail. `Recovery` reports them, and whether the file was opened from
its footer (`finished`) or the scan found a `TAIL` where the footer's starts
(`footer_damaged`). `next` passes damage the same way as the scan,
returning `InvalidArgument` once; a geometry consumer resumes at the next
key record.
A session header that is cut, out of range or fails a CRC is
`InvalidArgument`, and one of another version or with a flag set
`Unsupported`.

**In memory** (`stream/chunks.hpp`, the 2026-10-21 entry). `ChunkEncoder`
makes the session chunk and frames each record by the record rules, whose
state it holds; `StreamWriter` writes through one. `ChunkParser` takes bytes
as they arrive (`feed` copies them) and returns the session, then each
record, through the checks the reader's chunks pass. Each check runs once
the bytes it needs are in, a header's before its body is needed: B is at
most `kMaxSessionBytes` for the session (`InvalidArgument`) and
`ChunkParser::kMaxBodyBytes` (64 MiB) for any other chunk (`OutOfMemory`).
Both CRCs are checked on every chunk, a skipped one's too. Other tags, the
footer included, and a record setting an unknown flag are skipped. Nothing
resyncs: every call after a failure returns it. Drained between pieces, it
holds about one chunk. A `next` with no payload holds the record, its CRC
checked, for `read` into caller memory, as `StreamClient`'s does live.

Measured on an Apple M5 Max (Release, APFS, warm page cache): CRC-32 at
2.5 GB/s, 26 times the roughly 95 MB/s of a four-camera recording; appends of
737,280-byte (depth-sized) payloads at 1.95 GB/s; reads, CRC checked, at
2.1 GB/s. Opening 864,000 records of 1 KiB on 8 tracks (an hour at 240 a
second, 934 MB) takes 0.33 to 0.36 s by the scan, and from the footer 0.5 ms
with a key in 30 records or 14 ms with every record a key (a 27.6 MB index).

### networking

`recon_networking` serves one session live over TCP and reads it
(`networking/stream_server.hpp`, `networking/stream_client.hpp`, the
2026-10-21 entry). The wire is the file: the `VRST` chunk, then `RECD`
chunks as `StreamWriter` writes them, made by `ChunkEncoder` and read by
`ChunkParser`, with no footer and no framing of its own. A viewer sends
nothing; the server reads and drops what one does send, and closes on its
end. The port is 47801 (`kDefaultPort`).

**Serving** (`StreamServer`). `start` listens on a numeric IPv4 address, or
every interface, and starts an accept thread; one thread per viewer sends,
at most `max_viewers` (4) at once, first come, and a connection past them is
closed at accept. `publish` frames the record by the writer's rules, copies
its chunk once, takes one lock to append it to the GOP log and wakes the
viewers: it never waits on a socket. A viewer takes its next chunk only once
its socket polls writable, under 128 KiB unsent (`TCP_NOTSENT_LOWAT`), and
a P-frame never goes out without the records it is predicted from. At a
chunk boundary a viewer goes on from a newer key if one has come: newest key
wins, in either mode, so a lagging viewer's seq jumps to a key. By
default (`replay_gop`) the log holds one GOP, the newest key and every
record after it, as chunks shared by the viewers still to send them. A
viewer's first record, after its `VRST` chunk, is the GOP's key, and the
log keeps every record since. A record that would take the GOP past
`max_gop_bytes` (64 MiB) drops it and every record until the next key, as
a copy that fails does (`OutOfMemory`). Without `replay_gop` (the
2026-10-25 keys-on-demand entry), a viewer joins waiting for the next key, then
gets every record, in order, but by newest key wins; one that a record would
leave more than `max_gop_bytes` behind finishes its chunk under way and
waits for the next key again, alone, as every viewer does after a lost copy.
The log keeps only the records a viewer still has to send, none before the
newest key. Each join and each fall-back counts in `key_wanted()`, an
atomic any thread reads, which a live sender answers with a key at its next
frame; a lost key counts its waiting viewers again, a key within
`max_gop_bytes` counts none, and with `replay_gop` it stays 0. A fall-back
is logged at Info, and a record past `max_gop_bytes` on its own, which no
viewer can take, at Warning. A viewer whose end acknowledges no
byte for `stall_limit` (10 s) with bytes waiting is closed: a slow reader
stays while its end acknowledges a byte within the limit (one behind a large
receive buffer may not), as does one with nothing waiting. Viewers that
join, leave or fail are logged at Info; a refusal at Warning, the first and
then one in 10 s at most, the rest at Debug. The destructor stops every
thread within a poll slice (20 ms) and the write under way. One track is
served (`Unsupported` otherwise), and `publish` is called from one thread.

**Reading** (`StreamClient`). `connect` never blocks: a numeric IPv4 address
is parsed at once, and a name is looked up on a detached thread, which the
client abandons if it is destroyed first. `next(wait, …)` finishes the
lookup and the connect, trying each address in turn, then returns each
record, both CRCs checked, its `offset` that of the bytes received, waiting
at most `wait`. The receive buffer is 2 MiB, set before the connect. No
byte for `idle_limit` (10 s), counted from `connect`, ends the connection
with `IoError`, as do a refusal at every address, a reset and the sender's
end, which comes after every whole record; a failed check ends it with the
parser's error, which every later call returns again. Every error reads
`StreamClient: <host>:<port>: <reason>`, the address once, and the idle
limit's reason names the stage it ended in (the header lists the reasons).
The four-argument `next` overload also writes whether this call made
progress: a record returned, a successful lookup/connect step, or bytes
received. It resets that output on every call and keeps it true across a
later idle step. `next(0, ...)` reads at most one receive, so a false result
alone does not mean the input is drained; callers waiting on their own
controls drain while progress is true. On an error the output covers only
earlier successful steps, and the error remains terminal. The original
three-argument overload forwards without requesting progress.

**Sockets** (private `socket.hpp`, which gives the reasons). IPv4 only:
the server listens on it, and a name is looked up for it. Every socket is
non-blocking and close-on-exec, accepted ones included, and never raises
SIGPIPE; every call is retried on `EINTR`. The sender sets `TCP_NODELAY`
and the listener `SO_REUSEADDR`. The GOP log (`gop_log.cpp`), in either
mode, has no socket, thread or clock, and is tested alone.

### sender

`recon_sender` is what a sender does with a prepared set
(`sender/set_sender.hpp`, `sender/codec_stream.hpp`, the 2026-10-26
entry). `fuse_inputs` (`sender/fuse_inputs.hpp`) hands prepared frames to
the fuser, nothing uploaded.

**The stream** (`CodecStream`). `create` takes the encoder's settings, the
session and `CodecOutputs`: a new `.vrst` file, the server's options, and
whether to decode. It refuses a session that is not one Geometry track,
then makes the server, so a port in use leaves no file, then the
file, which must be new, then the encoder and, with `decode`, the decoder,
so a refused output costs no kernel. A codec error removes the new file
again, as does destroying the stream unfinished before its first frame. The
file and the viewers get that session. `code(grid, set, pts_ns)`
encodes the grid, appends a record (a key exactly when an I-frame), serves
it once it is in the file, so no viewer gets a frame the file lacks, then
decodes it with `decode_growing` from the decoder's input buffer into a
player grid `make_decode_grid` built from the first frame, and returns a
`SentFrame`: its type, bytes, blocks, whether its payload took a tail, and
the encode's and the decode's rows. A set or pts out of order is refused
before it is encoded, so the writer never refuses one the encoder has
taken. Any other failure after the encode, of the append or the decode,
calls `force_intra()`, so the next frame never references one the file or
the player lacks; a frame the decoder refuses stays in the file and counts
in `coded()`. A record the server refuses is logged at Warning and fails no
frame, as failing would stop the recording. Before each frame's front half,
where the encoder picks I or P, the stream reads the server's
`key_wanted()`: a rise calls `force_intra()`, which stands until a frame is
returned, so joins between two frames make one key, a refused frame or a
front half recorded again keeps it, and the period counts from it.
`finish` closes the viewers and writes the footer, in one attempt.

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
not stop the set, and the frame. The front half, its device rows and the
back half are no public call: they are the internal class `CodecStream`
owns (`stream_coder.hpp`), which it hands out only as an incomplete type
(`coder()`) for `SetSender`'s source to complete.

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
`fuse_orbbec` and `rig_viewer`, whose default is `Dynamic`;
`rig_viewer`'s View panel switches it as the rig runs. The four
dataset examples poll their frames through `sensor::IRgbdSensor&` — the
fuse loop never learns what is behind it — and prepare each on its own.
The two live ones read Orbbec cameras raw as a `SensorArray` of
`OrbbecSensor`s, `fuse_orbbec` one camera or a rig and `rig_viewer` a rig,
and prepare each set in one batch (`SensorArray::process`).
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
and `fuse_orbbec`'s camera lines and counters in `fuse_orbbec_sets.hpp`.
The per-set table of `fuse_orbbec`, `codec_replica` and `rig_viewer` is
`set_log.hpp`'s.
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
**`fuse_orbbec`** (`VR_WITH_ORBBEC`) reads one `OrbbecSensor`, or with
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
fused set as one frame of a `sender::CodecStream` (the 2026-10-20
decision), decoding it too only with `output.csv` or `codec.sweep`
(`set_codec_outputs`), for their decode columns and the sweep's grid. Each
set's row is stamped published once its frame is written and served, before
any decode, or after its fusion when it is not coded. `main.cpp` opens the
stream before the cameras and refuses there a port in use and a PLY that is
the stream's or the CSV's file under another name (`check_apart`). A
record's set is the set's index in the run
and its pts the set's time; a set stamped before the last frame, as a clock
stepping back stamps it, takes that frame's time. A run that fails part-way
still finishes its stream and reports the frames it coded; one that coded
no set leaves none.
The report's bitrate is at `source.fps`, and `codec.sweep` sweeps the last
grid. A Ctrl+C or SIGTERM (`interrupt.hpp`, `rig_viewer`'s handler too,
installed first) ends the run. During the setup, the run returns before the
cameras stream, removing the empty stream and releasing the cameras. Once
they stream, it ends after the set in hand, as `run.frames` would: the
stream is finished, its viewers closed, the cameras released and the PLY
written. A second signal of the same kind ends the process.
**`rig_viewer`** (`VR_BUILD_VIEWER` with `VR_WITH_ORBBEC`, and
`VR_WITH_FFMPEG` off Apple; CI's viewer leg compiles it) is `fuse_viewer`'s live-rig
sibling: an `OrbbecSensor` per camera of the sync file, opened as
fuse_orbbec opens a rig (`open_orbbec_cameras`, at the calibration's colour
controls), read as a
`SensorArray`, its raw sets prepared, fused and textured from every camera
on the GPU, and the atlas filled by
device copies recorded in gfx's frame (the 2026-09-29 decision), fusing
depth only inside each colour camera's view unless given `fusion.depth_within_color: false`, and
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
rate–distortion table the defaults are chosen from. The codec examples
stream through the sender tier's `CodecStream` ([sender](#sender)); their
report and the sweep are `examples/common/codec_report.hpp` and
`codec_sweep.hpp`. The report (`CodecReport`) keeps a row per frame that
went out whole, from its `SentFrame` (source index, frame type, bytes,
blocks, and encode and decode host and device ms), sums each type's stage
rows, and writes `output.csv` as each row comes; the core cannot append, so
the file is the report's own. A stream that does not decode
(`CodecOutputs::decode` off) has rows with empty decode columns.
`codec_outputs` maps `output.stream` and `output.serve` to `CodecOutputs`,
and `codec_session` gives the `.vrst` one Geometry track at the source's
nominal rate (the rig's `source.fps`, else 30 a second), with no session id
or start time, so a rerun writes the same bytes; a record's set is the
source index and its pts that index at 30 fps, as Replica frames carry no
time. `codec_outputs` serves without `replay_gop`, so fuse_orbbec's viewers
that join or fall behind get a key at the next set; `serve_stream`, which
cannot make a key, keeps it.

The report opens the CSV once the stream's file is made, and refuses one
that is that file under another name, through symlinks or case-insensitive
names (`InvalidArgument`); the stream, dropped unfinished before its first
frame, then removes its file for a retry.

**`codec_mesh`** codes one mesh or a numbered sequence of them (README,
Codec evaluation, has the keys). The core
escapes literal `%` characters in the scenario's directory before joining a
relative `source.path` (0.8.0), so only the user's path carries a
frame-number pattern.
Before any GPU work it refuses a malformed
frame-number pattern, a range on a plain path and any frame file that is
missing or not a regular file, checks a shell against the truncation
(`MeshSdfParams::validate`), and refuses a band too narrow for a true SDF
(`check_true_sdf_band`). It codes with `EncoderConfig::true_sdf` (the
2026-10-21 entry), the one caller that sets it. Its first pass loads each
frame, joins its bounds along `source.up_vector` (`io::extend_bounds`),
audits it and drops it, so memory does not grow with the range. In exact
arithmetic a proper similarity changes none of the audit's counts, so a
frame signed mode cannot
take is refused there; rounding can still change the degenerate count or a
volume's sign, which the second pass's audit decides. One transform for the range follows (`fit_transform` or
`scale_transform`) and places every frame unchanged, so a still point stays
still; it is printed once and stored as the Geometry track's params
(`io::encode_transform`). The stream is created after the GPU setup and
before any frame is converted, so a rerun onto it is refused before any
conversion or output. Each frame (`code_frame`) is loaded, placed, audited again on the
placed coordinates (the audit that decides, as rounding can make a triangle
degenerate), converted into the one source grid after a `clear`, so no block
of an earlier frame stays, and coded as one record: set the frame's
number, pts that number at 30 fps, seq from 0. The last frame alone is
extracted from the source and player grids with `mesh::MarchingCubes` and
compared, conversion error against the placed input apart from codec error
against the uncompressed surface. `codec.inspect_only` runs both passes on
the host. File parsing, placement and PLY export belong to `io`; the example
keeps its sequence (`mesh_sequence.*`) and the evaluator's host mesh
(`geometry_mesh.*`).

Both examples take `codec.quantization_table` (`uniform`, `band`, `radial`) and `codec.quantization_scale` for a
single configuration. Their common `codec.sweep` compares all three table
families at K = 64, 84, 120 and 512. The middle two retain complete frequency
shells; K = 64 cuts a shell and is kept as the existing baseline. Each
configuration has a warm-up and three timed codec rounds of one grid, all
intra whatever `codec.intra_period` (`sweep_encoder_config`); mesh
extraction and quality evaluation are outside those timings. Full frame
bytes include the quantization table. These are deterministic candidate
tables, not fitted JPEG image tables: the criterion is decoded geometry at
a given total size.

**`serve_stream <in.vrst> [--port N]`** serves a recording live: a sender
with no camera or GPU, so it builds wherever the examples do. It replays one
track's records from its first key read whole, each due at its pts from its
pass's start, in a loop until Ctrl+C (`ReplayLoop`, `stream_replay.hpp`).
Each pass is restamped, seq past the passes before and pts a pass's span (the
records' pts span and one mean step) later, so seq keeps rising and key pts
never go back, as `publish` requires. A record whose payload fails its CRC
is skipped with the P-frames after it, up to the pass's next key, and
printed in the first pass. It listens on every interface, on 47801 or N (0
takes a free one, which it prints). Bad arguments exit 2 with the usage; a
failure, a missing file or no key whose payload reads whole included,
exits 1.

## Next work

**Cameras and sensors for the family (the 2026-10-06 plan).** recon is the
library calib and ios build on for cameras and sensors. The `camera` tier has
landed; the stack continues:
1. **The sensor interface.** `RgbdFrame` (each camera's `CameraModel`,
   `color_to_world` and `depth_to_color`, a sequence number, pixels it holds)
   and `IRgbdSensor` with `SensorInfo` have landed, the Femto Mega as
   `OrbbecSensor`, and `OrbbecCapture` and the SDK's host path are gone. The
   Replica source is an `IRgbdSensor` too, its colour packed host words
   (`RgbdFrame::color_packed`), and `ICameraCapture` and `CapturedFrame`
   are gone (2026-10-07). The frame gains no float depth: the iOS scanner,
   which still calls the host-depth and convention entry points kept for
   it, quantises ARKit's float depth to 16 bits when it moves onto the
   interface.
2. **`SensorArray`**, vendor-neutral, has landed: start order from the sync
   roles, trigger grouping, poses from the calibration file
   (nearest-frame and tracked members later), and `process(set)`, every
   stream of every sensor in one GPU batch. `OrbbecRig` is gone:
   `fuse_orbbec` and `rig_viewer` open a `SensorArray` of `OrbbecSensor`s on
   the SDK's global timestamps (2026-10-07), their skew measured flat over a
   20-minute `fuse_orbbec` run; sessions of hours are still to measure.
3. **Luma readback** has landed in `sensor/utils` (`PictureReadback`),
   wherever the decoder left the picture, for calib's detector.
4. **Pipelined stages**: the core's `CommandBatch` submits without waiting,
   ordered by timeline semaphores; the per-set host waits go, in
   `tsdf::Fuser` (an allocation's failure count and occupancy read a set
   late); then a pipeline over acquire, prep, the grid chain (fuse, mesh,
   texture, still serial) and consumers.
5. **Later: mixed arrays** of fixed sensors and tracked ones (iPhones):
   nearest-frame members, registration of a tracked sensor's world, a network
   sensor with an ios sender, and clock offsets.

**Incremental mesh extraction was removed** (2026-10-06): nothing live
used it and its win was never measured. It comes back only with a
measurement on a large scan (room0 replicated several times over can stand
in for one), against a full extract with view culling.
**View-culled meshing has landed** (2026-10-06): `fuse_viewer` and
`rig_viewer` publish their render camera's `view_proj` to the fuse thread,
which meshes the blocks inside it (a 0.25 m margin) and re-meshes when the
view moves. The win is correct and unmeasured — expect it roughly linear in
the visible fraction on the `ExtractTimings` rows that scale with the active
set (the dispatch, the arena; `readback_ms` and `descriptor_ms` are per-call
constants and will read flat, and the frustum compaction still scans every
hash slot), and quote nothing until a run on a large scan says so. Beside that:
first-class glTF/GLB export in `io` and the gfx-vertex converter. PLY export
has moved from the example into the validated I/O module. On `mesh`, the greppable
`TODO(mesh)`s: cross-block vertex sharing, per-vertex normals, measuring a
return of the default kernel to the per-triangle append (the per-block
reservation cost ~10% of the dispatch and was taken for incremental
extraction). On `texture`,
the `TODO(texture)`s: keeping a static keyframe set's
depth and coverage in the pass between calls, blending views at their seams,
and a per-triangle tile
index in gfx so a shared mesh can be textured from several views; and the
multi-keyframe post-scan atlas, which the multi-view path can carry.

**On `codec`**, trimming a P-frame's host overhead at small R, where its
encode stays slower than intra's on the host alone (the 2026-10-17 freeze
entry). For true-SDF streams (the 2026-10-21 entry): the step-keyed deblock
in the player; one 0.2 m comparison reach in `codec_compare_options`, which
reaches max(truncation, 2 cm) today; and a sweep ladder in metres, since
fractions of a 16-voxel band are coarse steps. For the deblock (the deblock
entry's 2026-10-23 amendment): a guard finer than the block, at mask edges,
before the masked blocks of live RGB-D frames, nearly all of theirs, are
filtered.

**On `fuse_orbbec`'s stream** (the 2026-10-20 entry): a capture-volume
crop (`fusion.max_depth_m` is the limit there is), raw RGBD recording and
replay, colour, an upright world, and the session's host clock and
calibration (`codec_session` leaves both out). With
`output.csv` or `codec.sweep` the stream still decodes each frame inside the
set loop: on
the lab rig (4 Femtos, RTX 4090, Release; 1 cm, Dynamic, period 30; 303
sets) a P-frame's decode took 2.70 ms host and 2.14 ms device, 37% and 38%
of the set's fusion and coding, and 6.49 / 4.38 ms at 5 mm. A loop slower
than the rig gets the newest set, so sets drop; that cost is unmeasured.

**On `voxelize`**, mesh → TSDF has its two modes (2026-09-27), a second way into
the grid the codec encodes: a mesh sequence converts frame by frame. A
winding-number sign would fix at the root the sign errors a true-SDF stream
cuts (the 2026-10-21 true-SDF entry), and the folds within √3 voxels that it
cannot. Both modes cost the same. On Rafa2 the band's allocation costs what
PERF.md's MESH3 row gives, and the write what the 2026-10-21 `MeshIntegrator`
entry gives. Smaller costs around them remain: writing only the blocks the
codec will code, chaining the write's submissions, and computing
`triangle_candidate_offsets` and uploading the mesh once a frame rather than
once to allocate and again to write.
Dynamic's clear is still a hard reset from one ray: clearing on evidence (a
weight decrement, an incidence term) and behind a pixel with no return are
the next steps, each judged on the lab rig (`TODO(tsdf)` in
`tsdf_integrate.glsl`, 2026-10-08).

**On `sensor`**, each a `TODO(sensor)`: the GPU pre-processing has landed for
one camera (`GpuFramePrep`, the 2026-09-28 GPU pre-processing decision: at 4K
it takes the host from 15.5 ms of undistortion and registration a frame to
none, and the run's CPU eightfold down) and for the rig's raw sets. NVDEC
and nvJPEG hand their pictures over on the device, and VideoToolbox both
kinds, and a raw Orbbec frame's colour stays on the device over either
codec, VideoToolbox's and NVDEC's plane images sampled where they are. For
H.265: the camera's encoder settings, its key-frame interval above all,
which sets what a lost frame costs (`camera_stream.cpp`). The rig's next consumer is calib's
viewer, showing its synchronised sets.

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
