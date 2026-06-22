#!/usr/bin/env bash
# Register N self-hosted GitHub Actions runners on THIS Mac, all labelled `mac`,
# so ci.yml's macOS Debug+Release legs run on real Apple-GPU MoltenVK instead of
# (10x-billed, on a private repo) hosted runners.
#
# Prereqs: Xcode Command Line Tools (`xcode-select --install`) + Homebrew.
# Run as your login user in Terminal:  bash .github/setup-mac-runner.sh
# Tear down later (e.g. when switching Macs) with: bash .github/teardown-runners.sh
#
# Runner dirs/names are scoped with a `recon-` prefix so this repo's runners
# coexist with another repo's on the same machine (e.g. volumetric_kit_gfx, whose
# scripts use the unprefixed `actions-runner-<i>`) instead of clobbering them.
set -euo pipefail

REPO="taojin-6/volumetric_kit_recon"
SLUG="recon"                         # dir/name scope so repos don't collide
LABEL="mac"
N=2                                  # Debug + Release in parallel
BASE="$HOME"

case "$(uname -m)" in
  arm64)  PKG_ARCH="arm64" ;;        # Apple Silicon
  x86_64) PKG_ARCH="x64"   ;;        # Intel
  *) echo "unsupported arch: $(uname -m)"; exit 1 ;;
esac
xcode-select -p >/dev/null 2>&1 || { echo "run 'xcode-select --install' first"; exit 1; }
command -v brew >/dev/null 2>&1   || { echo "install Homebrew first: https://brew.sh"; exit 1; }

# Mint a registration token via gh if it's authed; otherwise prompt for one.
if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
  VER="$(gh api repos/actions/runner/releases/latest --jq .tag_name | sed 's/^v//')"
  TOKEN="$(gh api -X POST "repos/${REPO}/actions/runners/registration-token" --jq .token)"
else
  echo "gh not authed — get a token at https://github.com/${REPO}/settings/actions/runners/new"
  VER="2.335.1"
  read -r -p "Paste registration token: " TOKEN
fi

CORES="$(sysctl -n hw.ncpu)"; THREADS=$(( CORES / N )); [ "$THREADS" -lt 1 ] && THREADS=1
TAR="${BASE}/actions-runner-osx-${PKG_ARCH}-${VER}.tar.gz"
# Reuse a cached tarball only if it's a valid archive; an interrupted earlier
# download leaves a truncated file that the next `tar xzf` would choke on.
tar tzf "$TAR" >/dev/null 2>&1 || curl -fsSL -o "$TAR" \
  "https://github.com/actions/runner/releases/download/v${VER}/actions-runner-osx-${PKG_ARCH}-${VER}.tar.gz"

for i in $(seq 1 "$N"); do
  dir="${BASE}/actions-runner-${SLUG}-${i}"
  echo "==> [${i}/${N}] ${dir}"
  mkdir -p "$dir"; tar xzf "$TAR" -C "$dir"
  # Loaded into every job -> caps cmake/ctest fan-out so the parallel legs share
  # the cores instead of each grabbing all of them.
  printf 'CMAKE_BUILD_PARALLEL_LEVEL=%s\nCTEST_PARALLEL_LEVEL=%s\n' "$THREADS" "$THREADS" > "${dir}/.env"
  (
    cd "$dir"
    ./config.sh --unattended --url "https://github.com/${REPO}" \
      --token "$TOKEN" --labels "$LABEL" --name "$(hostname -s)-${SLUG}-${i}" --work _work --replace
    ./svc.sh install && ./svc.sh start   # launchd LaunchAgent; no sudo on macOS
  )
done

echo "Done — ${N} runners labelled '${LABEL}'. Keep this Mac awake + auto-logged-in:"
echo "  sudo pmset -a sleep 0 disablesleep 1   # + enable auto-login in System Settings"
