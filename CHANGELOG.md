# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to
follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial repository scaffolding: tiered layout, MIT license, `.clang-format`,
  `.cmake-format.yaml`, `.pre-commit-config.yaml`, `.gitignore`.
- `CLAUDE.md` — the living source of truth (objective, locked decisions, tier
  architecture, interop contract, salvage + exclusion policy, gotchas).
- `core` tier foundation: backend-neutral `Status` / `Result<T>` error handling
  (`VR_TRY` / `VR_ASSIGN`), `VR_CHECK` contract checks, a pluggable log handler,
  the version API, and portable POD math types.
- Tiered CMake with install/export and package config
  (`find_package(volumetric_kit_recon)` / `FetchContent`), warnings-as-errors,
  and sanitizer support.
- Direction set to a single **Vulkan compute** path (MoltenVK on Apple),
  mirroring `volumetric_kit_gfx`, so the reconstruction backend and renderer
  share one cross-platform API and a trivial same-device interop seam.
