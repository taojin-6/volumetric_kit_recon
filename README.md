# volumetric_kit_recon

The **reconstruction + compression** compute backend of the `volumetric_kit`
family — it turns posed depth / RGB-D frames into a sparse TSDF volume, extracts
geometry, compresses it, and hands the result to a renderer.

It is the standalone sibling of
[`volumetric_kit_gfx`](https://github.com/taojin-6/volumetric_kit_gfx) (the
Vulkan/MoltenVK renderer). The two are independent libraries that meet at a thin
interop seam: `recon` builds the volume, `gfx` renders it.

> **Status: early.** The repository is conventions scaffolding plus the `core`
> foundation tier. The `volume` / `tsdf` / `mesh` / `interop` tiers are landing
> next, as Vulkan compute (MoltenVK on Apple). See [DESIGN.md](DESIGN.md) for the
> architecture, [CLAUDE.md](CLAUDE.md) for the conventions and the
> locked-decision index, and [DECISIONS.md](DECISIONS.md) for the dated record
> behind each decision.

## Why

`recon` is a from-scratch, production-grade rebuild that salvages the proven core
of an earlier reconstruction engine and hardens it for public release:
deterministic only (no experimental/neural code), installable and packaged,
CI-gated, and split into clean, independently consumable tiers.

## Design at a glance

- **Tiered, like gfx:** `core` → `volume` → `tsdf` → `mesh` → `interop`. A tier
  may depend only on tiers to its left.
- **One Vulkan path everywhere:** compute runs as Vulkan compute shaders
  (GLSL → SPIR-V), with MoltenVK on Apple — Linux / Android / macOS / iOS /
  Windows from one source, mirroring `volumetric_kit_gfx`. No Metal/CUDA split.
- **Exception-free:** fallible calls return `Status` / `Result<T>`; mobile builds
  with `-fno-exceptions` are first-class.
- **Trivial renderer handoff:** because the renderer is *also* Vulkan, geometry
  goes to `volumetric_kit_gfx` as glTF/mesh data today and, later, as a shared
  `VkBuffer`/`VkImage` on a common device — the easy same-API, same-device case,
  with no cross-API memory translation.

## Building

Requires CMake ≥ 3.21 and a C++17 compiler (plus the Vulkan SDK / MoltenVK once
the Vulkan core lands).

```sh
cmake -B build
cmake --build build
ctest --test-dir build
```

Consume it from another CMake project via `find_package(volumetric_kit_recon)`
or `FetchContent`, then link a tier (e.g. `volumetric_kit::recon_core`).

### Optional: Orbbec SDK

The Orbbec (Femto Mega) capture code is off by default and needs the
[Orbbec SDK](https://github.com/orbbec/OrbbecSDK_v2/releases) ≥ 2.9.3
installed. It is not fetched: install it once, outside the repo (the family
convention is `<workspace>/third_party/OrbbecSDK_v<version>`), and point the
build at its root:

```sh
cmake -B build -DVR_WITH_ORBBEC=ON -DOrbbecSDK_ROOT=<sdk>
# or once, for every repo that finds it:  export OrbbecSDK_ROOT=<sdk>
```

That builds `volumetric_kit::recon_sensor_orbbec`, an `ICameraCapture` over one
camera (`sensor/orbbec/orbbec_capture.hpp`), and the live example:

```sh
build/examples/fuse_orbbec/fuse_orbbec --serial <serial> --frames 300
# the driver's hardware test opens only the camera you name:
VR_ORBBEC_TEST_SERIAL=<serial> ctest --test-dir build -R orbbec
```

A camera wired as a sync secondary streams only while its primary does; name
a primary or standalone camera for a single-camera run. Without `--serial` the
example opens the only camera that answers, after waiting out the whole
discovery window (8 s).

A synced rig runs from its sync configuration (`femto_mega_sync.json`, the
Orbbec SDK's layout) and a calibration file (`sensor/rig_calibration.hpp`) for
the poses. The rig refuses cameras whose sync settings differ from the file;
`--apply-sync` writes it to them:

```sh
build/examples/fuse_orbbec/fuse_orbbec --rig femto_mega_sync.json \
    --calibration calib.json --frames 300
VR_ORBBEC_TEST_RIG=femto_mega_sync.json ctest --test-dir build -R orbbec_rig
```

## License

MIT — see [LICENSE](LICENSE).
