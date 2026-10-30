#!/bin/bash
# Install and start the CPU performance service for the real FR3 (run with sudo).
set -euo pipefail
cd "$(dirname "$0")"
if [[ $EUID -ne 0 ]]; then
  echo 'Usage: sudo scripts/install_cpu_performance.sh' >&2
  exit 1
fi
install -m 644 franka-cpu-performance.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now franka-cpu-performance.service
echo "power profile: $(powerprofilesctl get 2>/dev/null || echo 'n/a')"
echo 'CPU governors:'
sort /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | uniq -c
