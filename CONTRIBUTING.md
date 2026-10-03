# Contributing

Thanks for helping build `volumetric_kit_recon`. A few conventions keep the
codebase consistent with its sibling `volumetric_kit_gfx`.

## Read first

[AGENTS.md](AGENTS.md) is the canonical working guide for contributors and
coding agents. It contains the scope, conventions, Git workflow, resource
ownership rules, and validation commands. `CLAUDE.md` imports that same guide.

Use its task map to read the relevant contracts in [DESIGN.md](DESIGN.md),
dated decisions in [DECISIONS.md](DECISIONS.md#decision-index), and live rig
work in [PERF.md](PERF.md). Keep the affected documentation in the same change
as the implementation; the [update policy](AGENTS.md#keeping-the-guidance-current)
describes which file owns each kind of guidance.

## Setup

```sh
pip install pre-commit
pre-commit install
```

For platform dependencies and optional drivers, see [Building](README.md#building).
Run the commands in [Build and validation](AGENTS.md#build-and-validation) from
your task's worktree. Set the build type explicitly; performance reports use
Release.

CI runs the same formatting hooks as the local pre-commit installation;
`lint` is a merge gate. Build and test coverage also includes Debug, Release,
and sanitizer jobs. Documentation-only changes use the formatting and link
checks described in AGENTS.md.
