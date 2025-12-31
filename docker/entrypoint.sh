#!/usr/bin/env bash

set -euo pipefail

print_usage() {
  cat <<'EOF'
Available apps (run as CMD/args):
  rplidar_publisher <usb_device>
  mid360_publisher
  sim_publisher
  icm20948_publisher
  ads1115_publisher
  camera_publisher
  all_publisher

Example:
  docker run --rm <image> rplidar_publisher /dev/ttyUSB0
EOF
}

# If no args, just show help and exit; if args, exec so signals (Ctrl+C) reach
# the child process directly instead of being swallowed by PID 1.
if [[ $# -eq 0 ]]; then
  print_usage
  exit 0
fi

echo "Running: $*"
exec "$@"