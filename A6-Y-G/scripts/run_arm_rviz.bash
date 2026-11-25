#!/usr/bin/env bash
set -euo pipefail
A6Y_RVIZ_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Only ROS base packages are needed; no controller or old workspace overlay.
source "$A6Y_RVIZ_ROOT/scripts/environment.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${A6Y_REVIEW_DOMAIN_ID:-194}"
export ROS_LOG_DIR="$A6Y_RVIZ_ROOT/../diagnostics/arm_rviz_logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$A6Y_RVIZ_ROOT/tools/arm_visualization/arm_view.py" "$@"
