# volumetric_kit_recon

The **reconstruction + compression** compute backend of the `volumetric_kit`
family — it turns posed depth / RGB-D frames into a sparse TSDF volume, extracts
geometry, compresses it, and hands the result to a renderer.

It is the standalone sibling of
[`volumetric_kit_gfx`](https://github.com/taojin-6/volumetric_kit_gfx) (the
Vulkan/MoltenVK renderer). The two are independent libraries that meet at a thin
interop seam: `recon` builds the volume, `gfx` renders it.

The voxel grid and what reads it -- voxel hashing, marching cubes, mesh files,
quality measurement, the codec and the `.vrst` stream and its live serving --
are [`volumetric_kit_grid`](grid/README.md), a package of its own in `grid/`
that recon builds on, on the core alone.

> Start with [AGENTS.md](AGENTS.md) for shared working conventions and commands.
> [DESIGN.md](DESIGN.md) describes the architecture, current implementation,
> and remaining work. [DECISIONS.md](DECISIONS.md#decision-index) holds the
> locked-decision index and dated rationale; [PERF.md](PERF.md) tracks live
> rig performance work.

## Why

`recon` is a from-scratch, production-grade rebuild that salvages the proven core
of an earlier reconstruction engine and hardens it for public release:
deterministic only (no experimental/neural code), CI-gated, and split into
clean, independently consumable tiers.

## Design at a glance

- **Tiered, like gfx:** `core` → `tsdf` → `texture` → `interop`, with
  `sensor` off `core`, on grid's tiers below them. A tier may depend only on
  tiers to its left, and grid on nothing of recon's.
- **One Vulkan path everywhere:** compute runs as Vulkan compute shaders
  (GLSL → SPIR-V), with MoltenVK on Apple — Linux / Android / macOS / iOS
  from one source, mirroring `volumetric_kit_gfx`. No Metal/CUDA split. GCC or
  Clang; Windows and MSVC are not supported.
- **Explicit errors:** fallible calls return `Status` / `Result<T>`. Build recon
  with C++ exceptions enabled for its internal error handlers.
- **Shared-device renderer handoff:** the live viewers draw recon's mesh
  buffers on one `VkDevice` shared with `volumetric_kit_gfx`. The host/file
  handoff remains available for standalone workflows; see the
  [interop contracts](DESIGN.md#the-interop-seam).

## Building

Requires CMake ≥ 3.21, a C++17 compiler, and the Vulkan toolchain (MoltenVK on
Apple). See the CI workflows for platform dependency installation.

```sh
recon_root="$(git rev-parse --show-toplevel)"
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$recon_root/build" --parallel
ctest --test-dir "$recon_root/build" --output-on-failure
```

The GPU tests follow the core's test policy (its README, "Build and test"):
`VKC_REQUIRE_VULKAN_DEVICE`, `VKC_TEST_VALIDATION` and
`VKC_TEST_SYNC_VALIDATION`.

Consume it from another CMake project with `FetchContent`, pinned by commit
(recon has no install rules), then link a tier (e.g.
`volumetric_kit::recon_core`). Fetched, recon builds its library tiers alone:
its tests, examples and `-Werror` (`VR_WARNINGS_AS_ERRORS`) default on only
when it is the top-level project. A consumer with no GPU work, such as calib's
solver, sets `VR_CAMERA_ONLY` ON to build the camera tier
(`volumetric_kit::recon_camera`) alone, with no Vulkan SDK; it refuses the
examples and the optional drivers.

A consumer's kernels include recon's exported GLSL as recon's own do,
through the core's `INCLUDE_TARGETS`: `volumetric_kit::recon_core` exports
the pinhole (`volumetric_kit/recon/core/shaders/pinhole.glsl`) with
`vkc_export_glsl`, under a root that holds it alone. The voxel grid's
contract is grid's ([its README](grid/README.md#building)).

```cmake
vkc_compile_shaders(my_kernels INCLUDE_TARGETS volumetric_kit::recon_core
                    SHADERS my.comp)
```

A consumer that needs a recon at least as new as a given version calls the
core's `vkc_require_sibling(volumetric_kit::recon <min-version>)` after
`FetchContent_MakeAvailable(volumetric_kit_recon)`: the umbrella carries
recon's `project(VERSION)` as `VKC_PACKAGE_VERSION`.

recon builds on grid (`grid/`), declared after the core and checked with
`vkc_require_sibling(volumetric_kit::grid ...)` at `VR_GRID_MIN_VERSION`
(0.1.0); in this build grid's tests, examples and `-Werror` follow recon's.
An application that fetches grid itself declares it first, by URL, SHA and
`SOURCE_SUBDIR grid`; to build against a local grid checkout, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_GRID=<path>`.

recon builds on [`volumetric_kit_core`](https://github.com/taojin-6/volumetric_kit_core),
the family's shared foundation, fetched pinned by commit on the first configure:
its error handling, and its Vulkan foundation -- device, allocator, buffers,
kernels, command batches, timers -- whose types recon uses under the core's
names (`volumetric_kit::core::Status`, `volumetric_kit::core::Device`) and which
one `VkDevice` shares with gfx.
An application that also fetches calib, which builds on the core too, declares
`volumetric_kit_core` first, so both build against one copy, and sets
`VKC_WITH_VULKAN ON` before the first `FetchContent_MakeAvailable` of it
unless recon is camera-only; to build against a local core checkout, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<path>`. recon needs the core
at `VR_CORE_MIN_VERSION` (0.29.0) or newer, with its vulkan tier unless
`VR_CAMERA_ONLY` is ON, and checks both with the core's `vkc_require_core`:
on a core older than that or missing a required tier, recon's configure
fails saying why. A core older than 0.1.0 has no `vkc_require_core`, so the
configure stops at that call as an unknown command: move the first-declared
pin to 0.29.0 or newer.

Examples and tests also need the core's config tier. That floor covers every
core version they and the viewers' pinned gfx (`VR_BUILD_VIEWER`) need.

### Asset I/O

`volumetric_kit::recon_image_io` provides encoded RGB image loading, 16-bit
grayscale depth PNG loading with explicit units, and RGBA8 and 16-bit
grayscale PNG writing (`io/image_io.hpp`); it links the core's base tier
alone, with no Vulkan. These are host file operations; they allocate no
Vulkan resources and do not download a live GPU volume. Mesh files -- PLY
export and the optional Assimp import (`VGRID_WITH_ASSIMP`) -- are grid's
([its README](grid/README.md#mesh-files)).

## Codec evaluation

`codec_replica` runs grid's codec on a fused room sequence; grid's
`codec_mesh` codes a mesh or a sequence of them, and grid's README documents
the `codec` section both read ([its codec evaluation](grid/README.md#codec-evaluation)).

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$recon_root/build" --parallel
# Edit source.path in this file to point at the installed dataset.
"$recon_root/build/examples/codec_replica/codec_replica" \
  --config "$recon_root/config/examples/codec_replica.json"
```

## Live streaming

`fuse_orbbec` serves the rig's coded frames with `output.serve`, coding the
next set as a key for a viewer that joins or falls behind, through
`volumetric_kit::recon_sender`: each set fused and its frame coded in two
batches ([the sender contract](DESIGN.md#sender)), then written and served
by grid's codec stream. The server, the client, `serve_stream` and what they
print are grid's ([its live streaming](grid/README.md#live-streaming)).

## License

MIT — see [LICENSE](LICENSE).
