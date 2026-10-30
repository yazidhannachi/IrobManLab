#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
export LAB_HOSTNAME="$(hostname)"
export LAB_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ ! -f "$LAB_XAUTHORITY" ]]; then
  echo 'Set XAUTHORITY to your desktop session authority file.' >&2
  exit 1
fi
compose=(docker compose -f docker-compose.yml)
for arg in "$@"; do
  case "$arg" in
    --nvidia) compose+=(-f docker-compose.nvidia.yml) ;;
    # Real FR3 driver image; does not move the robot until a launch file is started.
    --real) compose+=(-f docker-compose.real.yml) ;;
    # Also start the ZED X Mini stream receiver next to the lab container.
    --zed) compose+=(-f docker-compose.zed.yml) ;;
    *) echo 'Usage: scripts/run.sh [--nvidia] [--real] [--zed]' >&2; exit 1 ;;
  esac
done
"${compose[@]}" up -d
# The entrypoint builds before starting sleep; wait for the environment.
until "${compose[@]}" exec -T lab test -f /tmp/lab-ready; do
  if [[ "$("${compose[@]}" ps --status running -q lab)" == '' ]]; then
    "${compose[@]}" logs lab
    exit 1
  fi
  sleep 1
done
"${compose[@]}" exec lab bash
