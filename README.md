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
are [`volumetric_kit_grid`](https://github.com/taojin-6/volumetric_kit_grid),
a repository of its own that recon builds on, on the core alone.

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
when it is the top-level project. The camera and sensor tiers are the
in-tree sensor package (`sensor/`, `volumetric_kit::sensor_*`), which builds
and tests alone too (`cmake -S "$recon_root/sensor" ...`); recon's configure
passes it the `VS_` options, and `-DVS_BUILD_TESTS=ON` builds its tests in
recon's tree. A consumer with no GPU work, such as calib's solver, builds the
package's camera tier (`volumetric_kit::sensor_camera`) alone with
`VS_CAMERA_ONLY` ON, with no Vulkan SDK; it refuses the optional drivers.

recon exports no GLSL. A consumer's kernels include the GLSL recon's kernels
read as they do, through the core's `INCLUDE_TARGETS`: the sensor package's
`volumetric_kit::sensor_frame` exports the pinhole
(`volumetric_kit/sensor/frame/shaders/pinhole.glsl`) with `vkc_export_glsl`,
under a root that holds it alone. The voxel grid's contract is grid's
([its README](https://github.com/taojin-6/volumetric_kit_grid/blob/main/README.md#building)).

```cmake
vkc_compile_shaders(my_kernels INCLUDE_TARGETS volumetric_kit::sensor_frame
                    SHADERS my.comp)
```

A consumer that needs a recon at least as new as a given version calls the
core's `vkc_require_sibling(volumetric_kit::recon <min-version>)` after
`FetchContent_MakeAvailable(volumetric_kit_recon)`: the umbrella carries
recon's `project(VERSION)` as `VKC_PACKAGE_VERSION`.

recon builds on grid, fetched pinned by commit after the core and checked
with `vkc_require_sibling(volumetric_kit::grid ...)` at
`VR_GRID_MIN_VERSION` (0.1.0). grid's own CI tests grid, so its tests and
`-Werror` stay off here; recon's tests and examples turn on its test support
and its examples' helpers. An application that fetches grid itself declares
it first; to build against a local grid checkout, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_GRID=<path>`.

recon builds on [`volumetric_kit_core`](https://github.com/taojin-6/volumetric_kit_core),
the family's shared foundation, fetched pinned by commit on the first configure:
its error handling, and its Vulkan foundation -- device, allocator, buffers,
kernels, command batches, timers -- whose types recon uses under the core's
names (`volumetric_kit::core::Status`, `volumetric_kit::core::Device`) and which
one `VkDevice` shares with gfx.
An application that also fetches calib, which builds on the core too, declares
`volumetric_kit_core` first, so both build against one copy, and sets
`VKC_WITH_VULKAN ON` before the first `FetchContent_MakeAvailable` of it; to
build against a local core checkout, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<path>`. recon needs the core
at `VR_CORE_MIN_VERSION` (0.29.0) or newer, with its vulkan tier, and checks
both with the core's `vkc_require_core`:
on a core older than that or missing a required tier, recon's configure
fails saying why. A core older than 0.1.0 has no `vkc_require_core`, so the
configure stops at that call as an unknown command: move the first-declared
pin to 0.29.0 or newer.

Examples and tests also need the core's config tier. That floor covers every
core version they and the viewers' pinned gfx (`VR_BUILD_VIEWER`) need.

### Asset I/O

The sensor package's `volumetric_kit::sensor_image_io` provides encoded RGB
image loading, 16-bit grayscale depth PNG loading with explicit units, and
RGBA8 and 16-bit grayscale PNG writing (`volumetric_kit/sensor/io/image_io.hpp`);
it links the core's base tier alone, with no Vulkan. These are host file
operations; they allocate no Vulkan resources and do not download a live GPU
volume. Mesh files -- PLY export and the optional Assimp import
(`VGRID_WITH_ASSIMP`) -- are grid's
([its README](https://github.com/taojin-6/volumetric_kit_grid/blob/main/README.md#mesh-files)).

### Optional: Orbbec SDK

The Orbbec (Femto Mega) capture code is off by default and needs the
[Orbbec SDK](https://github.com/orbbec/OrbbecSDK_v2/releases) ≥ 2.10.6
installed. It is not fetched: install it once, outside the repo (the family
convention is `<workspace>/third_party/OrbbecSDK_v<version>`), and point the
build at its root:

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVS_WITH_ORBBEC=ON -DOrbbecSDK_ROOT=<sdk>
# on Linux, add -DVS_WITH_FFMPEG=ON -DVS_WITH_CUDA=ON
# or once, for every repo that finds it:  export OrbbecSDK_ROOT=<sdk>
```

The driver decodes each camera's colour itself, so it needs the video
decoders too (see below: every Apple build has them; on Linux, FFmpeg and
CUDA). That builds
`volumetric_kit::recon_sensor_orbbec` -- `OrbbecSensor`, one camera as an
`IRgbdSensor` (`sensor/orbbec/orbbec_sensor.hpp`), which a `SensorArray`
reads as a synced rig -- and the live example. Every example takes a core
scenario file through `--config`; `--help` prints the common entry point.
The core owns JSON decoding, defaults, units, ranges, source selection and
operation compatibility. See [example configurations](config/README.md).
Set `diagnostics.readback` to `true` to read each set's luma back to the
host through `PictureReadback`, during either streaming or fusion.

```sh
build/examples/fuse_orbbec/fuse_orbbec --config config/examples/fuse_orbbec.json
# Stream and decode alone, at 4K MJPEG and 25 fps; 60000 camera frames:
build/examples/fuse_orbbec/fuse_orbbec --config config/examples/stream_orbbec.json
# Each fused set coded as one frame of a .vrst:
build/examples/fuse_orbbec/fuse_orbbec --config config/examples/codec_orbbec.json
# The same, also served live:
build/examples/fuse_orbbec/fuse_orbbec --config config/examples/serve_orbbec.json
```

With `output.stream`, `output.csv` or `output.serve`
([Live streaming](#live-streaming)), the example codes each fused set as one
frame, as the [codec examples](#codec-evaluation) do, at the set's time (for
a rig, on the host's clock); `output.stream` records the frames in a `.vrst`.
They are geometry only, in the world `source.calibration` defines. Ctrl+C
ends a run cleanly ([Examples](DESIGN.md#examples)).

Edit `source.serial` for a single camera, or `source.sync_config` for a synced
rig. A camera wired as a secondary streams only while its primary does; choose
a primary or standalone camera for a single-camera run. With neither serial nor
sync file, the example opens the only camera answering in the discovery window
(8 s).

The sync file stays in the Orbbec SDK's format (`config/femto_mega_sync.json`),
and `source.calibration` references the family's calibration file for poses,
each camera's depth offset and the colour cameras' exposure
(`color_controls`), which every camera is opened at; without it they run at
auto exposure. Auto white balance is on either way. Colour is H.265 at
3840x2160 and 25 fps unless the scenario names another mode.
A camera whose sync settings differ is refused; `source.apply_sync: true`
explicitly writes those settings. The stream operation refuses fusion, codec
and output settings before opening any resource. Relative paths are resolved
from the scenario file's directory, including the sync, calibration and output
paths.

### Video decoders

The sensor package's HEVC and JPEG decoders (`volumetric_kit::sensor_video`)
decode on the GPU's hardware and hand their pictures out on the device only.
Every Apple build has them, on VideoToolbox with no FFmpeg; their pictures
reach Vulkan through `PixelBufferImporter` (`volumetric_kit::sensor_apple`).
On Linux they are
off by default: NVDEC through FFmpeg ≥ 6.1, found through pkg-config, and
nvJPEG, with the CUDA 13 toolkit's headers (`sensor/cmake/vs_cuda.cmake`).
`VS_WITH_FFMPEG` on Apple fails the configure. The H.265 syntax the Apple
decoder reads (`volumetric_kit::sensor_video_syntax`,
`volumetric_kit/sensor/video/hevc_syntax.hpp`) is host code, in every build.

```sh
sudo apt install pkg-config libavcodec-dev libavutil-dev
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVS_WITH_FFMPEG=ON -DVS_WITH_CUDA=ON -DVS_BUILD_TESTS=ON
# fail, rather than skip, where the decoders find no device path
# (videotoolbox on a Mac):
VS_TEST_HEVC_BACKEND=cuda ctest --test-dir build -R video
```

## Codec evaluation

`codec_replica` runs grid's codec on a fused room sequence; grid's
`codec_mesh` codes a mesh or a sequence of them, and grid's README documents
the `codec` section both read ([its codec evaluation](https://github.com/taojin-6/volumetric_kit_grid/blob/main/README.md#codec-evaluation)).

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
print are grid's ([its live streaming](https://github.com/taojin-6/volumetric_kit_grid/blob/main/README.md#live-streaming)).

## License

MIT — see [LICENSE](LICENSE).
