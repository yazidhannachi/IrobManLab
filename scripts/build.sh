#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
compose=(docker compose -f docker-compose.yml)
for arg in "$@"; do
  case "$arg" in
    --real) compose+=(-f docker-compose.real.yml) ;;
    # ZED X Mini stream receiver (ZED SDK and ROS 2 wrapper, NVIDIA GPU).
    --zed) compose+=(-f docker-compose.zed.yml) ;;
    *) echo 'Usage: scripts/build.sh [--real] [--zed]' >&2; exit 1 ;;
  esac
done
"${compose[@]}" build
