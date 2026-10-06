# Shared repository instructions

This is the canonical working guide for all agents, including Codex and Claude
Code. Keep shared instructions here. `CLAUDE.md` imports this file; it must not
duplicate these rules or send the reader through a circular reference.

## Project and scope

`volumetric_kit_recon` is the production reconstruction and compression backend
of the `volumetric_kit` family: posed depth/RGB-D frames → sparse TSDF volume →
geometry and compressed bitstreams → renderer handoff.

- Vulkan compute (GLSL → SPIR-V) is the baseline on every platform, with
  MoltenVK on Apple. Optional native CUDA work must preserve baseline numerics.
- `volumetric_kit_gfx` is an independent sibling. Work in this repository;
  changes to sibling repositories need their own task and conventions.
- recon builds on [`volumetric_kit_core`](https://github.com/taojin-6/volumetric_kit_core),
  the family's shared foundation (fetched, pinned by commit): its error
  handling and log sink, and its Vulkan foundation (device, allocator,
  buffers, kernels, batches, timers), used under the core's names
  (`core::Status`, `core::Device`).
  Code two or more siblings need belongs there, as a task in that repository.
- The prior `implicit_world_reconstruction` implementation is an algorithm
  reference for ports; `implicit_surface_compression` is research reference
  only. Never build or write in either prior repository.
- Production only: no triplane/neural code, tiny-cuda-nn, KLT, iQuantizer, or
  Python research/evaluation harnesses. The production C++ `eval` tier is allowed.
- A sensor driver belongs here only if this repository can build and test it.
  Platform-only drivers such as ARKit belong with their downstream application.
- The tiers work on any data. Dataset-specific preprocessing (units,
  orientation, scale) belongs to the caller, as an example does for its demo
  asset, never to a tier or its tests.

## Read what the task needs

Use this map to find the relevant sections; do not read every document for
every change. Detailed contracts and measurements remain authoritative in the
linked documents.

| Task | Read |
| --- | --- |
| Build, dependencies, optional drivers | [README.md](README.md#building), [CONTRIBUTING.md](CONTRIBUTING.md), relevant CMake files |
| Architecture, tier APIs, current implementation | [DESIGN.md](DESIGN.md#tiered-architecture), then the affected tier under [Implementation reference](DESIGN.md#implementation-reference) |
| Vulkan, shader ABI, device ownership, memory | [Key gotchas](DESIGN.md#key-gotchas-verified) and [The interop seam](DESIGN.md#the-interop-seam) |
| Color or camera conventions | [Color space](DESIGN.md#color-space), the affected sensor/texture section, and its public headers |
| Changing a contract or understanding a constraint | [Decision index](DECISIONS.md#decision-index), then the relevant dated entries; newest context wins |
| Live rig performance | [PERF.md](PERF.md), including its measurement rules and the item's row |
| Remaining work or an unmeasured claim | [Next work](DESIGN.md#next-work) and [Measured lessons](DECISIONS.md#measured-lessons) |

Before changing incremental mesh extraction, read [mesh](DESIGN.md#mesh) and
[Next work](DESIGN.md#next-work), plus the 2026-08-11 span/dispatch decisions
and the 2026-10-01 changed-stamp decision they cite. The second 2026-08-11
dispatch entry supersedes the first one's `share_vertices` restriction.

## Architecture and conventions

Dependencies flow left to right:

```text
core → volume → tsdf → mesh → texture → interop
  │        └→ codec     ├→ eval
  │                    └→ io
  └→ sensor ← camera          (later: track, stream)
```

- No upward includes. `camera` depends on the family core's base tier and GLM
  only, `sensor` on `camera` and `core`, `codec` on `volume`, and
  `eval` and `io` on `mesh`; driver/decoder/pre-processing targets stay separate.
  Asset I/O is host-side at file boundaries; Assimp is an optional private
  backend of `recon_io_assimp`, never a dependency of GPU kernels.
- C++17, no compiler extensions. Namespace `volumetric_kit::recon` (`vr::` in
  docs), nested per tier except `core`, which stays in `volumetric_kit::recon`:
  recon declares no `core` namespace, so `core::` always means the core's.
  Headers: `include/volumetric_kit/recon/<tier>/…`.
- recon's own macros use `VR_`; `VK_` belongs to Vulkan. CMake targets use
  `volumetric_kit::recon_<tier>`; see [package targets](DESIGN.md#naming-conventions-use-these-consistently).
- Fallible APIs return the core's `core::Status` / `core::Result<T>` and
  propagate with `VKC_TRY` / `VKC_ASSIGN`. No exceptions cross the API
  boundary. Programmer errors use `VKC_CHECK`. recon does not re-export core
  names into `vr::`; it logs with source `"vr"` (`core/log.hpp`).
- Include Vulkan through the core's `volumetric_kit/core/vulkan/vulkan.hpp`.
  Keep host PODs and GLSL `layout(scalar)` definitions byte-identical; host
  assertions alone cannot validate the shader ABI. A `create` that builds
  kernels first calls `check_device_requirements`.
- Kernel memory is device-local, reached through `CommandBatch`. Preserve
  the documented exception for small parameters; the host reads results
  back through the batch, never through a mapping.
  Require `DEVICE_LOCAL` for bulk allocations and verify borrowed input
  metadata; `HOST_VISIBLE` may coexist with device locality on UMA or BAR.
- Zero-copy recon/gfx interop uses one shared `VkDevice`. The mesh ring is
  retired by the host's release report; preserve the documented queue and
  resource lifetime contracts.
- 8-bit color is encoded; float color is linear. The working space is linear
  BT.709/D65, with exact sRGB as its canonical encoded form. Convert at the
  sensor boundary and encode at presentation.
- Full Doxygen on public classes/functions, matching
  `include/volumetric_kit/recon/sensor/camera_capture.hpp`. Deleted
  copy/defaulted move declarations already convey ownership; do not repeat
  "move-only" in prose. `@ref` only recon's own names; write the core's in
  backticks (`Status::Code::Unsupported`).
- No `friend` declarations. Test through public APIs with behavior-level tests.
- Mark deferred work with a greppable `TODO:` comment (existing tier-tagged
  `TODO(mesh)` / `TODO(sensor)` comments remain useful task pointers).
- Keep first-party code warning-clean with `VR_WARNINGS_AS_ERRORS=ON`.

## RAII resource types

Every Vulkan/VMA handle owner, including a wrapper owning a deleter, follows
these rules:

- Delete copy operations and provide the move pair.
- On move construction, move assignment, and `destroy()`, reset every owned
  member: handle, metadata, and deleter. The empty object's accessors must
  agree with `valid()`.
- Guard self-move; move assignment destroys its current state before adoption.
- Type-erase backend cleanup with `std::function<void()>` so VMA stays out of
  public headers. Reset a moved-from deleter to `nullptr`.
- Validate sizes, extents, usage, and other inputs before creating resources;
  return non-OK `Status` for invalid inputs.
- Test move construction and the emptied source, assignment over a live object,
  and self-move through a pointer to avoid `-Wself-move`. Sanitizer CI checks
  these for leaks and double frees.
- An internal class that only aggregates such owners skips these rules: it
  deletes copy and move, and `create()` returns `Result<std::unique_ptr<T>>`,
  as the codec's transform, frame writer and reader do. With no moved-from
  state, there is no move list to keep in step with its members. A public
  aggregate with a member that is unsafe to self-move (a `std::` container)
  defaults its move constructor and deletes move assignment, as
  `VoxelBlockGrid` does.

## Working with Git

- Implement each task in its own branch and **git worktree under `.worktrees/`**.
  Concurrent Claude/Codex tasks use separate worktrees. Preserve unrelated
  local changes and other tasks' worktrees; remove your worktree after merge.
- Use **absolute paths** for `cmake -S/-B`, `git -C`, and file operations;
  sibling repositories share the workspace.
- Use Conventional Commits, e.g. `feat(volume): …`, `fix(tsdf): …`, `docs: …`.
- When opening a PR, assign the authenticated user: `gh pr create --assignee @me`.
- Keep shared decisions and handoff context in committed project documentation
  so either agent can continue the work.

## Build and validation

Run from the task's worktree. The default build needs CMake ≥ 3.21, a C++17
compiler, and the Vulkan toolchain (MoltenVK on Apple); see README and CI for
platform dependencies.

```sh
recon_root="$(git rev-parse --show-toplevel)"
cmake -S "$recon_root" -B "$recon_root/build" \
  -DCMAKE_BUILD_TYPE=Release -DVR_BUILD_TESTS=ON
cmake --build "$recon_root/build" --parallel
ctest --test-dir "$recon_root/build" --output-on-failure
pre-commit run --all-files --show-diff-on-failure
git -C "$recon_root" diff --check
```

- Always set the build type. Use Release for performance measurements and
  report the build type, hardware, workload, and host/device timing separately.
- For code changes, build the affected targets and run relevant tests; add
  regression coverage for changed behavior. Run the full suite when shared
  contracts or broad changes warrant it. CI also covers Debug and sanitizers.
- For documentation-only changes, check formatting, links, and consistency;
  a build is unnecessary. The hooks can be scoped with `pre-commit run --files`.
- `VR_BUILD_VIEWER`, `VR_WITH_ORBBEC`, `VR_WITH_FFMPEG`, `VR_WITH_CUDA`, and
  `VR_WITH_ASSIMP` enable optional paths. Orbbec, FFmpeg and Assimp are
  installed prerequisites, never fetched by this repository.
- Tests use no real hardware: a camera is checked by hand through its example
  (`fuse_orbbec`). Report such a check as run or not run, never as a test.
- For batching/barrier changes, run synchronization validation
  (`VK_KHRONOS_VALIDATION_VALIDATE_SYNC=true`). Measure phases before choosing
  an optimization; follow PERF.md for live rig work and discrete-GPU evidence.

## Keeping the guidance current

- Keep this file concise (roughly 100–200 lines). Put implementation detail
  in DESIGN.md, decision evidence in DECISIONS.md, and active rig work in PERF.md.
- When changing a locked decision, update its index entry and dated rationale
  in DECISIONS.md in the same commit. Update the corresponding DESIGN.md
  contract and any essential rule here that changes with it.
- Update a PERF.md item's row in the PR that implements it. Preserve the
  distinction between landed behavior, provisional choices, open work, and
  claims awaiting measurement.
- `CLAUDE.md` stays an import of this file; any future Claude-only instruction
  belongs below that import. Shared rules are edited here only.
