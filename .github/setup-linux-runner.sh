#!/usr/bin/env bash
# Register N self-hosted GitHub Actions runners on THIS Linux box, all labelled
# `vk-linux-gpu`, so ci.yml's 6 Ubuntu legs (3 OSes x Debug/Release) run in
# OS-matched containers against the real GPU driver instead of hosted lavapipe.
#
# Prereqs: Docker Engine + NVIDIA Container Toolkit (so containers see the GPU) —
# see .github/self-hosted-runners.md, 'Host prerequisites'.
# Run as your login user:  bash .github/setup-linux-runner.sh
#   Installs a systemd service per runner, so it calls sudo — you'll be prompted
#   for your password. Safe to re-run: already-registered runners are left as-is
#   (no fresh token needed) and only their services are (re)installed + started.
# Tear down later (e.g. rebuilding the box) with: bash .github/teardown-runners.sh
#
# Runner dirs/names are scoped with a `recon-` prefix so this repo's runners
# coexist with another repo's on the same machine (e.g. volumetric_kit_gfx, whose
# scripts use the unprefixed `actions-runner-<i>`) instead of clobbering them.
set -euo pipefail

REPO="taojin-6/volumetric_kit_recon"
SLUG="recon"                         # dir/name scope so repos don't collide
LABEL="vk-linux-gpu"
N=6                                  # one per Linux build leg (3 OS x Debug/Release)
BASE="$HOME"

case "$(uname -m)" in
  x86_64)  PKG_ARCH="x64"   ;;
  aarch64) PKG_ARCH="arm64" ;;
  *) echo "unsupported arch: $(uname -m)"; exit 1 ;;
esac

command -v docker >/dev/null 2>&1 || {
  echo "install Docker Engine first: https://docs.docker.com/engine/install/ubuntu/"; exit 1; }
# GPU passthrough is the whole point of the `vk-linux-gpu` label; warn (don't fail)
# if the NVIDIA Container Toolkit isn't wired into Docker — legs would still run,
# but on CPU/lavapipe inside the container rather than the real driver.
if ! command -v nvidia-ctk >/dev/null 2>&1 || ! docker info 2>/dev/null | grep -qi nvidia; then
  echo "WARNING: NVIDIA Container Toolkit / nvidia runtime not detected — GPU won't"
  echo "  pass into containers. See .github/self-hosted-runners.md. Continuing anyway."
fi

CORES="$(nproc)"; THREADS=$(( CORES / N )); [ "$THREADS" -lt 1 ] && THREADS=1

# Only fetch a token + tarball if at least one runner still needs registering, so
# re-runs that just (re)start services don't demand a fresh registration token.
need_register=0
for i in $(seq 1 "$N"); do
  [ -f "${BASE}/actions-runner-${SLUG}-${i}/.runner" ] || need_register=1
done

if [ "$need_register" -eq 1 ]; then
  # Mint a registration token via gh if it's authed; otherwise prompt for one.
  if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    VER="$(gh api repos/actions/runner/releases/latest --jq .tag_name | sed 's/^v//')"
    TOKEN="$(gh api -X POST "repos/${REPO}/actions/runners/registration-token" --jq .token)"
  else
    echo "gh not authed — get a token at https://github.com/${REPO}/settings/actions/runners/new"
    VER="2.335.1"
    read -r -p "Paste registration token: " TOKEN
  fi
  TAR="${BASE}/actions-runner-linux-${PKG_ARCH}-${VER}.tar.gz"
  [ -f "$TAR" ] || curl -fsSL -o "$TAR" \
    "https://github.com/actions/runner/releases/download/v${VER}/actions-runner-linux-${PKG_ARCH}-${VER}.tar.gz"
fi

for i in $(seq 1 "$N"); do
  dir="${BASE}/actions-runner-${SLUG}-${i}"
  echo "==> [${i}/${N}] ${dir}"
  if [ ! -f "${dir}/.runner" ]; then
    mkdir -p "$dir"; tar xzf "$TAR" -C "$dir"
    # Loaded into every job -> caps cmake/ctest fan-out so the parallel legs share
    # the cores instead of each grabbing all of them.
    printf 'CMAKE_BUILD_PARALLEL_LEVEL=%s\nCTEST_PARALLEL_LEVEL=%s\n' "$THREADS" "$THREADS" > "${dir}/.env"
    ( cd "$dir" && ./config.sh --unattended --url "https://github.com/${REPO}" \
        --token "$TOKEN" --labels "$LABEL" --name "$(hostname -s)-${SLUG}-${i}" --work _work --replace )
  else
    echo "    already registered — ensuring its service is installed + running"
  fi
  # One systemd service per runner (auto-starts on boot); needs root. 'install'
  # errors if the unit already exists, so tolerate that and always (re)start.
  ( cd "$dir"
    sudo ./svc.sh install "$USER" 2>/dev/null || true
    sudo ./svc.sh start )
done

echo "Done — ${N} runners labelled '${LABEL}'. Verify they came online:"
echo "  systemctl list-units 'actions.runner.*' --no-pager"
