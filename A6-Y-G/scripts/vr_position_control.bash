#!/usr/bin/env bash
# Explicit local permission/stop command for the native-position backend.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
a6y_load_install
export ROS_DOMAIN_ID="${A6Y_REAL_ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/a6y-vr-ros-log}"
mkdir -p "$ROS_LOG_DIR"
exec python3 "$A6Y_HANDOFF_ROOT/tools/vr_control.py" --controller position "$@"
