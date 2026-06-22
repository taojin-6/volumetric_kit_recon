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
  # Delete the runner dir only once it is deregistered: the dir holds the
  # .credentials needed to deregister, so removing it after a failed or skipped
  # `config.sh remove` would strand the runner as a lingering (security-exposing)
  # offline registration with no way to retry. Keep it for a retry instead.
  if (
    cd "$dir" || exit 1
    "${SUDO[@]}" ./svc.sh stop      2>/dev/null || true
    "${SUDO[@]}" ./svc.sh uninstall 2>/dev/null || true
    # Deregister from GitHub so it doesn't linger as an offline runner.
    command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1 || {
      echo "   gh not authed — service stopped, but the runner is still registered."
      echo "   Remove it under Settings -> Actions -> Runners (or run 'gh auth login' and re-run)."
      exit 1
    }
    RM_TOKEN="$(gh api -X POST "repos/${REPO}/actions/runners/remove-token" --jq .token)" \
      && [ -n "$RM_TOKEN" ] || { echo "   could not mint a remove-token from GitHub"; exit 1; }
    ./config.sh remove --token "$RM_TOKEN"
  ); then
    rm -rf "$dir"
  else
    echo "   Kept ${dir} (not deregistered) — fix the cause and re-run, or remove"
    echo "   it under Settings -> Actions -> Runners, then: rm -rf ${dir}"
  fi
done

echo "Done. Runners still registered on the repo:"
if command -v gh >/dev/null 2>&1; then
  gh api "repos/${REPO}/actions/runners" --jq '.runners[].name' || true
fi
