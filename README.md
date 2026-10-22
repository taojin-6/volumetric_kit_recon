# volumetric_kit_recon

The **reconstruction + compression** compute backend of the `volumetric_kit`
family — it turns posed depth / RGB-D frames into a sparse TSDF volume, extracts
geometry, compresses it, and hands the result to a renderer.

It is the standalone sibling of
[`volumetric_kit_gfx`](https://github.com/taojin-6/volumetric_kit_gfx) (the
Vulkan/MoltenVK renderer). The two are independent libraries that meet at a thin
interop seam: `recon` builds the volume, `gfx` renders it.

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

- **Tiered, like gfx:** `core` → `volume` → `tsdf` → `mesh` → `texture` →
  `interop`, with `sensor` off `core`, `codec` off `volume`, and `eval`/`io` off
  `mesh`. A tier may depend only on tiers to its left.
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

A consumer's kernels include recon's GLSL pinhole as recon's own do,
`#include "volumetric_kit/recon/core/shaders/pinhole.glsl"`, under the root
that `volumetric_kit::recon_core`'s `VR_SHADER_INCLUDE_DIR` property names,
which holds the exported GLSL alone.

```cmake
get_target_property(vr_glsl volumetric_kit::recon_core VR_SHADER_INCLUDE_DIR)
vkc_compile_shaders(my_kernels INCLUDE_DIRS "${vr_glsl}" SHADERS my.comp)
```

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
at `VR_CORE_MIN_VERSION` (0.2.0) or newer, with its vulkan tier unless
`VR_CAMERA_ONLY` is ON, and checks both with the core's `vkc_require_core`:
on a core older than that or missing a required tier, recon's configure
fails saying why. A core older than 0.1.0 has no `vkc_require_core`, so the
configure stops at that call as an unknown command: move the first-declared
pin to 0.2.0 or newer.

Examples and tests require core 0.16.0 or newer with its config tier, for
`fuse_orbbec`'s `output.serve`; the pinned core supplies it.

### Asset I/O

`volumetric_kit::recon_io` provides encoded RGB image loading, 16-bit
grayscale depth PNG loading with explicit units, RGBA8 and 16-bit grayscale
PNG writing, and binary PLY mesh export. Include `io/image_io.hpp` or
`io/ply_writer.hpp` under `volumetric_kit/recon/`. These are host file
operations; they allocate no Vulkan resources and do not download a live GPU
volume.

Static mesh import is a separate optional target,
`volumetric_kit::recon_io_assimp`, with Assimp as a private implementation
dependency. Install Assimp and enable it explicitly:

```sh
brew install assimp                       # macOS
sudo apt install libassimp-dev zlib1g-dev   # Debian/Ubuntu
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVR_WITH_ASSIMP=ON
```

`io::load_mesh(path)` in `io/mesh_io.hpp` returns owned positions and triangle
indices. It triangulates polygons, applies scene-node transforms and instances,
preserves winding under reflections, and joins exactly coincident positions.
OBJ, PLY and glTF are covered by tests; other formats depend on the installed
Assimp build. Materials/textures are discarded and animated/skinned/morphed
assets are refused. Import does not impose physical units or a target height:
normalization such as Rafa2's 1.7 m convention is a caller operation, with
the opt-in helpers of `io/mesh_normalization.hpp` (in `recon_io`).

The default build does not need Assimp. Cross-compiles must supply Assimp for
the target platform, not a host package.

### Optional: Orbbec SDK

The Orbbec (Femto Mega) capture code is off by default and needs the
[Orbbec SDK](https://github.com/orbbec/OrbbecSDK_v2/releases) ≥ 2.10.6
installed. It is not fetched: install it once, outside the repo (the family
convention is `<workspace>/third_party/OrbbecSDK_v<version>`), and point the
build at its root:

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVR_WITH_ORBBEC=ON -DVR_WITH_FFMPEG=ON -DOrbbecSDK_ROOT=<sdk>
# on Linux, add -DVR_WITH_CUDA=ON
# or once, for every repo that finds it:  export OrbbecSDK_ROOT=<sdk>
```

The driver decodes each camera's colour itself, so it needs the video
decoders too (see below; on Linux, CUDA as well). That builds
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
and `source.calibration` references the family's calibration file for poses.
A camera whose sync settings differ is refused; `source.apply_sync: true`
explicitly writes those settings. The stream operation refuses fusion, codec
and output settings before opening any resource. Relative paths are resolved
from the scenario file's directory, including the sync, calibration and output
paths.

### Optional: FFmpeg

The HEVC and JPEG decoders (`volumetric_kit::recon_sensor_video`,
`sensor/video/`) are off by default and need FFmpeg ≥ 6.1 installed, found
through pkg-config. They decode on the GPU's hardware and hand their pictures
out on the device only: VideoToolbox on macOS, NVDEC and nvJPEG on Linux,
which need the CUDA 13 toolkit's headers as well (`cmake/vr_cuda.cmake`):

```sh
brew install ffmpeg pkgconf        # macOS
sudo apt install pkg-config libavcodec-dev libavutil-dev
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVR_WITH_FFMPEG=ON              # on Linux, and -DVR_WITH_CUDA=ON
# fail, rather than skip, where the decoders find no device path:
VR_TEST_HEVC_BACKEND=cuda ctest --test-dir build -R video
```

VideoToolbox's pictures reach Vulkan through `PixelBufferImporter`
(`volumetric_kit::recon_sensor_apple`, `sensor/apple/`), which every Apple
build has, iOS included, with or without FFmpeg.

## Codec evaluation

`codec_replica` runs the grid codec on a fused room sequence. `codec_mesh`
codes one triangle mesh or a numbered sequence of them: it loads each through
`io::load_mesh`, uses `tsdf::MeshIntegrator` to convert it to a TSDF,
compresses and decodes that grid, then extracts the decoded surface with
marching cubes. Build the mesh example with `VR_WITH_ASSIMP=ON` after
installing the [asset I/O prerequisites](#asset-io).
Their `codec` section selects `quantization_table` (`uniform`, `band`,
`radial`), `quantization_scale`, `segment_size`, `intra_period` (an I-frame
every N coded frames, zero-motion P-frames between; 1, the default, codes
every frame intra), `update_size` at a period above 1 (a P-frame's entries
per update segment; 0, the default, takes the segment size), and the shared
rate-distortion `sweep`, which codes intra.
`output.stream` records every coded frame in a new `.vrst` file, each I-frame
a key (see [the stream container](DESIGN.md#stream)), and `output.csv` writes
a row per decoded frame: its type (`I` or `P`), bytes, blocks, and encode and
decode host and device ms.

A mesh scenario gives the head-up direction (`source.up_vector`) and either
the input's unit (`source.metres_per_unit`) or a height to fit
(`source.height_m`). The example rotates head-up to +Y, centres X/Z and puts
the lowest point at Y=0, with one transform fitted to every frame it codes,
printed once and stored in the stream; the files are left unchanged.
Rafa2's OBJs declare no unit and its person is tilted, so its scenarios give
the visually checked long principal axis and fit 1.7 m: a height
normalization, not an assertion about the source's unit.

A sequence's `source.path` holds a printf-style frame number, such as
`Frame_%05d_textured.obj` (`%d` or `%0Nd` once; `%%` for a literal `%`), and
`run.first` and `run.frames` select an exact range, every file checked
before any work. Each frame is converted alone and coded as one record whose
set is its number, at 30 fps. `codec_mesh_sequence.json` codes Rafa2's
frames 1–150 at 5 mm in an 8-voxel band; its frames 151–300 replay 150–1,
so stop at 150. Frames 74–99 and 128–131 have holes, so it sets
`source.allow_holes: true`: signed mode then takes a mesh whose only defect
is boundary edges, each hole growing a skirt to the truncation past its rim.

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
  -DVR_WITH_ASSIMP=ON
cmake --build "$recon_root/build" --parallel
# Edit source.path in these files to point at the installed datasets.
"$recon_root/build/examples/codec_replica/codec_replica" \
  --config "$recon_root/config/examples/codec_replica.json"
"$recon_root/build/examples/codec_mesh/codec_mesh" \
  --config "$recon_root/config/examples/codec_mesh.json"
"$recon_root/build/examples/codec_mesh/codec_mesh" \
  --config "$recon_root/config/examples/codec_mesh_sequence.json"
```

The mesh example's comparisons, sweep and `output.prefix` PLYs are the last
frame's: `_input.ply` (normalized original), `_source.ply` (uncompressed
TSDF surface) and `_decoded.ply`. Conversion error, codec-only error and
total error are reported separately. `codec.inspect_only` checks every
frame's input and normalization without a GPU.
The signed mode checks indexed topology and winding; it does not establish
freedom from self-intersections. `source.sdf_mode: "shell"` supports open or
inconsistently wound meshes, with an intentional surface offset.

Use a Release build for measurements. The current format is v3, with a
global scale and the K kept bases' quantization weights in each frame; older
versions are refused. See [the codec contract](DESIGN.md#codec) and
[the measurement record](DECISIONS.md#2026-10-02--per-basis-quantization-and-a-normalized-mesh-codec-fixture).

## Live streaming

`volumetric_kit::recon_networking` serves a session to viewers over TCP as
it is recorded (`networking/stream_server.hpp`) and reads it as it arrives
(`networking/stream_client.hpp`), byte for byte the `.vrst` file's chunks;
see [the networking contract](DESIGN.md#networking). `fuse_orbbec` serves
the rig's coded frames with `output.serve`. `serve_stream` serves a
recording live, paced by its pts and looping until Ctrl+C, with no camera or
GPU; a viewer connects to the sender's `host:port`:

```sh
"$recon_root/build/examples/serve_stream/serve_stream" rafa2.vrst  # port 47801
"$recon_root/build/examples/serve_stream/serve_stream" rafa2.vrst --port 0
```

Both listen on every interface, with no authentication or encryption: any
host that reaches the port can watch, and the first four to connect keep
their slots while they stay (`max_viewers`).

## License

MIT — see [LICENSE](LICENSE).
