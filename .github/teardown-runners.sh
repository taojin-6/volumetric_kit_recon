#!/usr/bin/env bash
# Stop, deregister, and delete THIS REPO's self-hosted runners on THIS machine.
# Use when decommissioning or migrating a runner host (e.g. switching Macs, or
# rebuilding the Linux box). Works on macOS and Linux. Run as the user that owns
# ~/actions-runner-recon-*.  Best-effort: keeps going if a single step fails.
#
# Scoped to this repo's `recon-` prefixed dirs only, so it leaves another repo's
# runners (e.g. volumetric_kit_gfx's unprefixed `actions-runner-<i>`) untouched.
set -o pipefail

REPO="taojin-6/volumetric_kit_recon"
SLUG="recon"
# Linux runs the runner as a systemd service (root); macOS as a per-user LaunchAgent.
if [ "$(uname -s)" = "Linux" ]; then SUDO=(sudo); else SUDO=(); fi

shopt -s nullglob
dirs=("$HOME"/actions-runner-"${SLUG}"-*/)
if [ "${#dirs[@]}" -eq 0 ]; then
  echo "No ~/actions-runner-${SLUG}-* runner dirs on this machine — nothing to remove."
  exit 0
fi

for dir in "${dirs[@]}"; do
  [ -f "${dir}config.sh" ] || continue
  echo "==> Removing runner in ${dir}"
  (
    cd "$dir" || exit 1
    "${SUDO[@]}" ./svc.sh stop      2>/dev/null || true
    "${SUDO[@]}" ./svc.sh uninstall 2>/dev/null || true
    # Deregister from GitHub so it doesn't linger as an offline runner.
    if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
      RM_TOKEN="$(gh api -X POST "repos/${REPO}/actions/runners/remove-token" --jq .token)"
      ./config.sh remove --token "$RM_TOKEN" \
        || echo "   config.sh remove failed — delete it under Settings -> Actions -> Runners"
    else
      echo "   gh not authed — service stopped, but the runner is still registered."
      echo "   Remove it under Settings -> Actions -> Runners (or run 'gh auth login' and re-run)."
    fi
  )
  rm -rf "$dir"
done

echo "Done. Runners still registered on the repo:"
if command -v gh >/dev/null 2>&1; then
  gh api "repos/${REPO}/actions/runners" --jq '.runners[].name' || true
fi
