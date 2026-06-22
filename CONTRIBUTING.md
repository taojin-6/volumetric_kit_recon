# Contributing

Thanks for helping build `volumetric_kit_recon`. A few conventions keep the
codebase consistent with its sibling `volumetric_kit_gfx`.

## Read first

[CLAUDE.md](CLAUDE.md) is the source of truth for the project's objective,
locked decisions, tier architecture, interop contract, and the exclusion policy
(no experimental/neural code ships). Skim it before proposing a change; update
it in the same change that alters a decision.

## Setup

```sh
pip install pre-commit && pre-commit install
cmake -B build -DVR_BACKEND=Metal   # or -DVR_BACKEND=CUDA on Linux/NVIDIA
cmake --build build
ctest --test-dir build
```

## Conventions

- **C++17**, no compiler extensions. Namespace `volumetric_kit::recon` (`vr::`),
  nested per tier. Public headers live under `include/volumetric_kit/recon/<tier>/`.
- **Error handling:** return `Status` / `Result<T>` — never throw across the API.
  Use `VR_TRY` / `VR_ASSIGN` to propagate. Programmer errors use `VR_CHECK`.
- **Formatting** is enforced by pre-commit (clang-format, cmake-format). CI runs
  the same hooks; the `lint` check is a merge gate.
- **Warnings are errors** (`VR_WARNINGS_AS_ERRORS=ON`). Keep first-party code
  clean under `-Wall -Wextra -Wpedantic`.
- **Doxygen** on every public class/function, matching
  `include/volumetric_kit/recon/core/result.hpp`.
- **Tiers are strict:** a tier may depend only on tiers to its left.

## Workflow

- Develop in a **git worktree** under `.worktrees/` (e.g.
  `git worktree add .worktrees/my-change -b my-change`), not by checking out a
  branch in the primary tree. Remove it once merged.
- **Conventional Commits:** `feat(volume): …`, `fix(tsdf): …`, `build: …`,
  `refactor(core): …`, `docs: …`, `test: …`.
- Mark deferred work with a greppable `// TODO:` comment.
- Add tests with the change; move-only resource types need move-construct,
  move-assign-over-live, and self-move tests (the sanitizer CI job makes them
  real leak/double-free detectors).

## What not to add

Experimental, learned/neural, or paper-experiment code does not belong here (see
the exclusion list in CLAUDE.md). It stays in the upstream research repos.
