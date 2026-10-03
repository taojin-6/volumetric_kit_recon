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
  `interop`, with `sensor` off `core`, `codec` off `volume`, and `eval` off
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

### Benchmark a recorded room

The default examples include `fuse_replica`, a headless replay of a posed
Replica-SLAM scene (`results/` and `traj.txt`, with `../cam_params.json`).
Use a Release build and an explicit truncation distance when comparing voxel
sizes; otherwise the example defaults to `4 * voxel_size`.

```sh
"$recon_root/build/examples/fuse_replica/fuse_replica" /absolute/path/to/room0 \
    --voxel 0.01 --trunc 0.04 --max-frames 400 --preload \
    --device-extract --mesh-every 1 \
    --timings-csv "$recon_root/build/room0-10mm.csv" \
    --out "$recon_root/build/room0-10mm.ply"
```

`--timings-csv` adds one row per fused frame and prints p50/p95/p99 timing
summaries. `pipeline_host_ms` is the sum of the wall-clock calls for fusion
and that frame's scheduled extraction. It excludes input polling/decoding,
preloading, the optional dirty-block survey, reporting, and final PLY export.
The CSV is buffered in memory and written after the run. The existing `done`
fps still includes final mesh extraction/export and is not online latency.

Allocation, integration and active-set rows distinguish host wall time
(including fence waits) from device timestamps. The active-set host row is
inside integration's host row; do not add it twice. Mesh phase columns are
host wall times; `mesh_device_ms` is blank because the extractor exposes no
device timestamps. Other blank fields mean an absent stage or an unavailable
device measurement, not zero elapsed time. Frames without a scheduled mesh
have `mesh_present=0`, blank mesh timing fields and zero mesh counters.
`mesh_retry` marks more than one extraction dispatch; `mesh_incremental`
reports what `--incremental` actually did, including fallback to full meshing.
`mesh_remeshed_blocks` counts all active blocks for a full extract.

Percentiles use nearest ranks over measured samples, including startup and
retry frames. Mesh summaries include only frames with a scheduled extract;
the pipeline summary includes every fused frame. The CSV's `frame` is the
one-based fused-frame ordinal (after `--stride`). Keep frame selection,
meshing cadence, truncation, flags and hardware identical for A/B runs, run
them interleaved on a shared machine, and follow [PERF.md](PERF.md) before
claiming a performance improvement.

### Adaptive room reconstruction (draft)

`fuse_replica_hierarchical` runs GPU root allocation, bounded refinement,
TSDF fusion and dual-cell mesh extraction on a posed Replica-SLAM sequence.
It uses a separate cell-centered field with 8³ samples per leaf; level zero
is finest and each following level doubles the spacing. The uniform
`VoxelBlockGrid` path remains independent. Finest spacing is configurable.
Defaults remain 5/10/20 mm; the comparison below explicitly selects
7.5/15/30 mm and 65,536 total node slots, with a common 40 mm physical
truncation band.

The scene directory contains `results/frameNNNNNN.jpg`, matching
`depthNNNNNN.png`, and `traj.txt`; intrinsics default to `../cam_params.json`
or can be supplied with `--cam-params`.

```sh
recon_root="$(git rev-parse --show-toplevel)"
room0="/absolute/path/to/Replica/room0"
cmake -S "$recon_root" -B "$recon_root/build" -DCMAKE_BUILD_TYPE=Release \
    -DVR_BUILD_EXAMPLES=ON -DVR_WARNINGS_AS_ERRORS=ON
cmake --build "$recon_root/build" --parallel --target \
    fuse_replica fuse_replica_hierarchical compare_mesh_quality
mkdir -p "$recon_root/build/room-validation"
"$recon_root/build/examples/fuse_replica_hierarchical" "$room0" \
    --voxel 0.0075 --levels 3 --trunc 0.04 --depth-jump 0.04 \
    --buckets 2048 --max-nodes 65536 --max-splits 64 --max-merges 64 \
    --merge-stability 8 --refine-every 4 --surface-error 0.002 \
    --noise-floor 0.0005 --pixel-stride 4 --max-frames 400 \
    --mesh-every 1 --preload --device-extract \
    --timings-csv "$recon_root/build/room-validation/adaptive.csv" \
    --out "$recon_root/build/room-validation/adaptive.ply"
```

Fusion and extraction run every frame in this experiment. `--refine-every 4`
classifies and applies topology budgets on frames 1, 5, 9, …; excess split
requests are reported as deferred and can be requested by later classifications.
Split then merge share one bounded `update_topology` submission and completion
wait. `topology_host_ms` / `topology_device_ms` report that combined stage;
standalone split/merge timing columns remain blank, while their separate
event counters remain populated. The stage is included in `fuse_host_ms`,
so do not add it to fusion again.

`--max-nodes` includes fixed root slots (`8 * --buckets`), internal parents
and child slots, not just active leaves. Logs and CSV report per-level leaf
counts, deferred and exhausted requests. `--support-coarsening` enables an additional conservative
support pass and is off by default. Preloading this 400-frame fixture uses
about 2.5 GB of host RAM.

The 7.5/15/30 mm comparison met the local M5 Max mean-time gates at the
measured revisions (`d08e4f9` baseline and `73c1d3b` candidate, both based
on `e4db453`):
adaptive +7.93% and the uniform path with adaptive code present −0.73%,
using medians of three interleaved 400-frame run means. Adaptive p95/p99
latency is slower; this does not establish a tail-latency or cross-GPU win.
The stack rebased through PRs #146 and #145 onto `2424f40` has not been retimed.
Those recorded outputs improve detail-region p95 distances to the 5 mm
proxy, while overall/planar distances and the 5 mm F-score are worse. See
[H2 measurements and reproduction](PERF.md#h2--online-hierarchical-room-experiment-draft)
for the paired baseline, proxy quality, timing boundaries and remaining
validation. Static replay does not establish moving-body reconstruction or
live rig performance; adaptive codec transport is not implemented.

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

## License

MIT — see [LICENSE](LICENSE).
