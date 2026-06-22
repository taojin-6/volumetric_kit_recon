# Self-hosted CI runners

Scripts: `.github/setup-linux-runner.sh` (Linux), `.github/setup-mac-runner.sh`
(macOS), `.github/teardown-runners.sh` (decommission/migrate any host).

The three Ubuntu legs of `ci.yml` run on a self-hosted runner labelled
`vk-linux-gpu`, each in an OS-matched container (`ubuntu:22.04` / `:24.04` /
`:26.04`) with the host GPU passed through via `--gpus all`, so the Vulkan
compute tests exercise the real driver. The `lint` and `sanitizers` jobs stay on
GitHub-hosted runners (the sanitizers job deliberately runs on lavapipe —
ASan/LSan against a proprietary driver report driver-internal allocations as
false leaks). The macOS leg runs natively on a self-hosted Mac (label `mac`) for
the native Apple toolchain + MoltenVK.

> recon is GPU-optional — its tests also run on lavapipe (software Vulkan), so if
> you'd rather not stand up self-hosted runners, switch the `ci.yml` matrix
> entries from `runner: vk-linux-gpu` / `runner: mac` to the GitHub-hosted images
> `ubuntu-24.04` / `macos-26` (set `container: ''` and drop `runner`). `_build.yml`
> already installs lavapipe / MoltenVK on hosted legs.

A self-hosted runner takes **one job at a time**, so to run the legs in parallel
you register **several runner instances** on the box, all sharing the
`vk-linux-gpu` label; GitHub then dispatches the queued legs across them. The
build matrix emits 6 Linux jobs (3 OSes × Debug/Release), so 6 instances give
full parallelism — fewer just means some legs queue.

## Host prerequisites (one-time)

1. **Docker Engine** — <https://docs.docker.com/engine/install/ubuntu/>.
2. **NVIDIA Container Toolkit**, so containers can use the GPU:
   ```bash
   # add the toolkit apt repo first (see NVIDIA docs), then:
   sudo apt-get install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   # verify the GPU is visible inside a container:
   docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all ubuntu:24.04 nvidia-smi
   ```

## Register N parallel runners

The quickest path is `bash .github/setup-linux-runner.sh`, which does everything
in this section (it mints the token via `gh` if you're authed, else prompts for
one). The manual loop below is what it automates, kept for reference.

The registration token and the exact tarball URL come from the repo →
**Settings → Actions → Runners → New self-hosted runner (Linux x64)**. One token
registers all N (valid ~1h).

Runner dirs/names are scoped with a `recon-` prefix so this repo's runners
coexist with another repo's on the same box (e.g. `volumetric_kit_gfx`, whose
scripts use the unprefixed `actions-runner-<i>`) instead of clobbering them.

```bash
TOKEN="<REGISTRATION_TOKEN>"
RUNNER_VERSION="2.330.0"          # whatever the runner page shows
URL="https://github.com/taojin-6/volumetric_kit_recon"
N=6                                # one per Linux build leg (3 OS x Debug/Release)
THREADS=4                          # N*THREADS ~= core count -> no oversubscription

curl -o ~/actions-runner.tar.gz -L \
  "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz"

for i in $(seq 1 "$N"); do
  dir=~/actions-runner-recon-$i   # recon-scoped so it won't collide with other repos' runners
  mkdir -p "$dir" && tar xzf ~/actions-runner.tar.gz -C "$dir"
  ( cd "$dir"
    # Loaded into every job on this runner -> caps cmake/ctest fan-out so the
    # parallel legs share the cores instead of each grabbing all of them.
    printf 'CMAKE_BUILD_PARALLEL_LEVEL=%s\nCTEST_PARALLEL_LEVEL=%s\n' "$THREADS" "$THREADS" > .env
    ./config.sh --unattended --url "$URL" --token "$TOKEN" \
      --labels vk-linux-gpu --name "$(hostname)-recon-$i" --work _work   # do NOT sudo config.sh
    sudo ./svc.sh install "$USER"   # one systemd service per runner name
    sudo ./svc.sh start )
done
```

Each runner becomes its own `actions.runner.*` systemd service that auto-starts
on boot. **Until at least one `vk-linux-gpu` runner is online the Ubuntu legs
stay _pending_** and the `ci / required` gate waits on them — bring the runners
up before merging.

> Membership in the `docker` group is root-equivalent; fine for a personal box,
> reconsider for a shared one.

## macOS runners

macOS can't be containerised, and the macOS leg's point is the native Apple
toolchain + MoltenVK, so the Mac runs jobs **natively, no Docker**. On a
**private** repo this also saves real money: hosted macOS minutes bill at 10×.

Prereqs on the Mac: `xcode-select --install` + Homebrew. Then:

```bash
bash .github/setup-mac-runner.sh        # registers 2 runners labelled `mac`
sudo pmset -a sleep 0 disablesleep 1    # keep it awake (also enable auto-login)
```

It must be an **always-on, auto-logged-in** Mac — the runner is a launchd
LaunchAgent that only runs in the user session, so a laptop that sleeps makes the
macOS leg flaky.

## Disabling / migrating a runner host

Runners on a host share one label (`vk-linux-gpu` or `mac`) and GitHub routes
jobs to whichever host is online, so swapping machines needs **no `ci.yml`
change**: bring the new host up, then tear the old one down.

- **Pause** (go offline, stay registered) — scoped to this repo's `recon-`
  runners so another repo's stay up:
  ```bash
  # Linux (systemd service, runs as root):
  for d in ~/actions-runner-recon-*/; do ( cd "$d" && sudo ./svc.sh stop ); done
  # macOS (per-user LaunchAgent — drop the sudo):
  for d in ~/actions-runner-recon-*/; do ( cd "$d" && ./svc.sh stop ); done
  ```
  Resume with `svc.sh start` (sudo on Linux, no sudo on macOS).
- **Fully remove** (decommission, or before handing the machine on):
  ```bash
  bash .github/teardown-runners.sh   # stop + uninstall service + deregister + delete dirs
  ```
  Run it on the *old* host. Deregistering matters — otherwise it lingers as an
  offline runner, and a still-registered runner on a machine you give away is a
  security exposure.
