#!/usr/bin/env bash
set -euo pipefail
A6Y_GRAVITY_ENTRY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$A6Y_GRAVITY_ENTRY/scripts/environment.bash"
source "$A6Y_GRAVITY_ENTRY/scripts/build_gravity_stack.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${A6Y_GRAVITY_DOMAIN_ID:-197}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$A6Y_GRAVITY_ENTRY/scripts/gravity_dds.xml}"
export ROS_LOG_DIR="$A6Y_GRAVITY_ENTRY/../diagnostics/arm_gravity_logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$A6Y_GRAVITY_ENTRY/tools/arm_gravity/gravity_view.py" "$@"
