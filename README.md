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

- **Tiered, like gfx:** `core` → `tsdf` → `texture` → `interop`, on grid's
  tiers below them. A tier may depend only on tiers to its left, and grid on
  nothing of recon's.
- **Cameras in a package of their own:** capture, decoding and frame
  preparation are
  [`volumetric_kit_sensor`](https://github.com/taojin-6/volumetric_kit_sensor),
  a repository of its own, whose frame contract (`sensor_frame`) is all
  recon's libraries link.
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
when it is the top-level project.

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

recon builds on sensor (`volumetric_kit::sensor_*`, the camera and sensor
tiers; [its README](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/README.md#building)
covers its options, drivers and decoders), fetched pinned by commit after
the core and checked with `vkc_require_sibling(volumetric_kit::sensor ...)`
at `VR_SENSOR_MIN_VERSION` (0.1.0); recon's configure passes it the `VS_`
options. sensor's own CI tests sensor, so its tests and `-Werror` stay off
here. An application that fetches sensor itself declares it first; to build
against a local sensor checkout, pass
`-DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_SENSOR=<path>`.

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

recon reads and writes no asset files of its own: image files are the
sensor package's (`volumetric_kit::sensor_image_io`,
[its README](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/README.md#image-files)), and mesh files -- PLY export
and the optional Assimp import (`VGRID_WITH_ASSIMP`) -- are grid's
([its README](https://github.com/taojin-6/volumetric_kit_grid/blob/main/README.md#mesh-files)).

### Optional: Orbbec SDK

The live example, `fuse_orbbec`, needs the sensor package's Orbbec driver:
the installed Orbbec SDK and, on Linux, the video decoders
([the package's README](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/README.md#orbbec-driver)).

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVS_WITH_ORBBEC=ON -DOrbbecSDK_ROOT=<sdk>
# on Linux, add -DVS_WITH_FFMPEG=ON -DVS_WITH_CUDA=ON
```

Every example takes a core
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

### Optional: librealsense

`fuse_realsense` fuses one RealSense D400 camera (the D455) with its own
depth, computed on the camera with its projector on, and needs the sensor
package's RealSense driver: librealsense 2.59.1 or later, installed from the
platform's packages
([the package's README](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/README.md#realsense-driver)).

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVS_WITH_REALSENSE=ON
build/examples/fuse_realsense/fuse_realsense \
    --config config/examples/fuse_realsense.json
```

It runs `fuse_orbbec`'s fusion on its one camera, posed at the origin and
still: each frame's YUYV colour is converted and undistorted on the GPU,
fused Dynamic, and the mesh written to the PLY. `source.serial` names the
camera when several are attached; the colour and depth sizes and
`source.fps` pick its modes (1280x800 and 1280x720 at 30 fps by default).
Its colour runs at auto exposure. The camera's first second or so of
framesets, which pair depth with stale colour, are refused and counted
failed in the report at the end.

### Optional: FFS depth (NVIDIA)

With `source.depth: "ffs"` (core's FuseRealsense), `fuse_realsense` takes its
depth from Fast-FoundationStereo on the camera's infrared cameras instead,
with the projector off: each pair rectified on the GPU and matched by
TensorRT on the sensor's own thread (sensor's `StereoDepthSensor`), its depth
left on the GPU for the frame prep, so the loop is the same. It needs the
sensor package's `sensor_ffs`: `VS_WITH_FFS`, with `VS_WITH_CUDA` and
TensorRT 11 or later, on Linux with an NVIDIA GPU
([the package's README](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/README.md#building)).
A build without it refuses an FFS scenario before anything opens.

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVS_WITH_REALSENSE=ON -DVS_WITH_CUDA=ON -DVS_WITH_FFS=ON
build/examples/fuse_realsense/fuse_realsense \
    --config config/examples/fuse_realsense_ffs.json
```

`source.ffs_model` is the FFS ONNX model at the depth size (960x544 by
default), which sensor's `tools/ffs/export_onnx.py` exports, and
`source.ffs_plan_cache` the directory its TensorRT plans are kept in: the
first run at a size builds one, which takes minutes, and later runs load it
in about a second. In the run's per-set table, FFS's time is in no row of
its own, only in "wait: cameras" and arrival→published, and the per-set
counters also count the rectifier's submit and wait on the sensor's thread
(DECISIONS.md, 2026-10-29).

### Optional: a microphone

With `source.audio` ([config/README.md](config/README.md)), `fuse_orbbec`
records a mic beside the cameras into its `.vrst`, as signed 16-bit samples
on the host's clock, and serves it with the frames under `output.serve`. It
needs the sensor package's microphone driver,
`VS_WITH_PORTAUDIO`, on an installed PortAudio
([the package's README](https://github.com/taojin-6/volumetric_kit_sensor/blob/main/README.md#microphone-driver));
a build without it refuses the key before anything opens. macOS asks for the
process's microphone access the first time it opens an input.

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVS_WITH_ORBBEC=ON -DOrbbecSDK_ROOT=<sdk> -DVS_WITH_PORTAUDIO=ON
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
