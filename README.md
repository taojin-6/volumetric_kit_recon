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

## License

MIT — see [LICENSE](LICENSE).
