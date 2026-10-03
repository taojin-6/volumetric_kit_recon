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
deterministic only (no experimental/neural code), installable and packaged,
CI-gated, and split into clean, independently consumable tiers.

## Design at a glance

- **Tiered, like gfx:** `core` → `volume` → `tsdf` → `mesh` → `texture` →
  `interop`, with `sensor` off `core`, `codec` off `volume`, and `eval`/`io` off
  `mesh`. A tier may depend only on tiers to its left.
- **One Vulkan path everywhere:** compute runs as Vulkan compute shaders
  (GLSL → SPIR-V), with MoltenVK on Apple — Linux / Android / macOS / iOS /
  Windows from one source, mirroring `volumetric_kit_gfx`. No Metal/CUDA split.
- **Exception-free:** fallible calls return `Status` / `Result<T>`; mobile builds
  with `-fno-exceptions` are first-class.
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

Consume it from another CMake project via `find_package(volumetric_kit_recon)`
or `FetchContent`, then link a tier (e.g. `volumetric_kit::recon_core`).

### Asset I/O

`volumetric_kit::recon_io` provides encoded RGB image loading, 16-bit
grayscale depth PNG loading with explicit units, RGBA8 PNG writing, and
binary PLY mesh export. Include `io/image_io.hpp` or `io/ply_writer.hpp` under
`volumetric_kit/recon/`. These are host file operations; they allocate no
Vulkan resources and do not download a live GPU volume.

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
normalization such as Rafa2's 1.7 m convention is a caller operation.

The default build does not need Assimp. An installed package built with
`VR_WITH_ASSIMP=ON` also needs Assimp's CMake package at consumption time.
Cross-compiles must supply Assimp for the target platform, not a host package.

### Optional: Orbbec SDK

The Orbbec (Femto Mega) capture code is off by default and needs the
[Orbbec SDK](https://github.com/orbbec/OrbbecSDK_v2/releases) ≥ 2.9.3
installed. It is not fetched: install it once, outside the repo (the family
convention is `<workspace>/third_party/OrbbecSDK_v<version>`), and point the
build at its root:

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVR_WITH_ORBBEC=ON -DOrbbecSDK_ROOT=<sdk>
# or once, for every repo that finds it:  export OrbbecSDK_ROOT=<sdk>
```

That builds `volumetric_kit::recon_sensor_orbbec`, an `ICameraCapture` over one
camera (`sensor/orbbec/orbbec_capture.hpp`), and the live example:

```sh
build/examples/fuse_orbbec/fuse_orbbec --serial <serial> --frames 300
# H.265 colour instead of MJPEG (a build with -DVR_WITH_FFMPEG=ON too):
build/examples/fuse_orbbec/fuse_orbbec --serial <serial> --hevc
# the driver's hardware test opens only the camera you name:
VR_ORBBEC_TEST_SERIAL=<serial> ctest --test-dir build -R orbbec
```

A camera wired as a sync secondary streams only while its primary does; name
a primary or standalone camera for a single-camera run. Without `--serial` the
example opens the only camera that answers, after waiting out the whole
discovery window (8 s).

A synced rig runs from its sync configuration (the lab rig's is
`config/femto_mega_sync.json`, in the Orbbec SDK's layout) and a calibration
file (`sensor/rig_calibration.hpp`) for the poses. The rig refuses cameras whose sync settings differ from the file;
`--apply-sync` writes it to them:

```sh
build/examples/fuse_orbbec/fuse_orbbec --rig config/femto_mega_sync.json \
    --calibration calib.json --frames 300
VR_ORBBEC_TEST_RIG=$PWD/config/femto_mega_sync.json \
    ctest --test-dir build -R orbbec_rig
```

### Optional: FFmpeg

The HEVC decoder (`volumetric_kit::recon_sensor_video`,
`sensor/video/hevc_decoder.hpp`) is off by default and needs FFmpeg ≥ 4.4
installed, found through pkg-config:

```sh
brew install ffmpeg pkgconf        # macOS
sudo apt install pkg-config libavcodec-dev libavutil-dev libswscale-dev
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVR_WITH_FFMPEG=ON
# require a back end (cuda, videotoolbox, vaapi) in the test:
VR_TEST_HEVC_BACKEND=cuda ctest --test-dir build -R video
```

It decodes on the first hardware back end that works (NVIDIA ahead of an
integrated GPU on Linux), else in software.

## Codec evaluation

`codec_replica` runs the grid codec on a fused room sequence. `codec_mesh`
loads static triangle geometry through `io::load_mesh`, uses
`tsdf::MeshIntegrator` to convert it to a TSDF, compresses and decodes that
grid, then extracts the decoded surface with marching cubes. Build the mesh
example with `VR_WITH_ASSIMP=ON` after installing the
[asset I/O prerequisites](#asset-io).
Both support `--quant-table uniform|band|radial`, `--step` for the global
quantization scale, and `--sweep` for the shared rate-distortion study.

Rafa2's supplied OBJ has no declared physical unit and its person is tilted
in the stored coordinates. The explicit head-up vector below is the
visually checked long principal axis. The example rotates it to +Y,
centres X/Z, puts the feet at Y=0, and scales the projected height to 1.7 m.
It prints the original and normalized bounds and leaves the asset unchanged.
This is a height normalization, not an assertion about the source's unit.

```sh
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
  -DVR_WITH_ASSIMP=ON
cmake --build "$recon_root/build" --parallel
dataset_root=/absolute/path/to/datasets
"$recon_root/build/examples/codec_replica/codec_replica" \
  "$dataset_root/replica_room0/room0" --max-frames 400 \
  --voxel 0.01 --encode-every 0 --preload --sweep

"$recon_root/build/examples/codec_mesh/codec_mesh" \
  "$dataset_root/Rafa2/Frame_00001_textured.obj" \
  --height 1.7 --up-vector -0.9120591159,0.0661250017,-0.4046920474 \
  --voxel 0.005 --mode signed --sweep -o "$recon_root/build/rafa2"
```

The mesh example writes `_input.ply` (normalized original), `_source.ply`
(uncompressed TSDF surface), `_decoded.ply`, and the compressed `.vrtc`
frame. Conversion error, codec-only error and total error are reported
separately. `--inspect-only` checks the input and normalization without a
GPU run. The signed mode checks indexed topology and winding; it does not
establish freedom from self-intersections. `--mode shell` supports open or
inconsistently wound meshes, with an intentional surface offset.

Use a Release build for measurements. The current format is v3, with a
shared 512-entry quantization table and global scale in each frame; older
versions are refused. See [the codec contract](DESIGN.md#codec) and
[the measurement record](DECISIONS.md#2026-10-02--per-basis-quantization-and-a-normalized-mesh-codec-fixture).

## License

MIT — see [LICENSE](LICENSE).
