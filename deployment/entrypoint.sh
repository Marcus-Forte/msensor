#!/usr/bin/env bash

set -euo pipefail

print_usage() {
  cat <<'EOF'
Available apps (run as CMD/args):
  sensor_publisher [config.json]
  sim_publisher
  remote_recorder -a <host:port> [-o output.pbscan]
  playback_publisher -f <file> [-s <speed>] [-p <port>] [--autoplay]
  scan_checker -f <file>

Example:
  docker run --rm <image> sensor_publisher /usr/local/etc/publisher_config.json
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