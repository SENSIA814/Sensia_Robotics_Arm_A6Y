#!/usr/bin/env bash
set -euo pipefail
A6Y_MOVEIT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$A6Y_MOVEIT_ROOT/scripts/environment.bash"
source "$A6Y_MOVEIT_ROOT/scripts/build_moveit_stack.bash"
source "$A6Y_MOVEIT_ROOT/scripts/build_moveit_panel.bash"
# This host's Fast DDS intermittently aborts controller_manager while sending
# service replies. Keep the upstream motion stack, use the installed Cyclone RMW.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${A6Y_MOVEIT_DOMAIN_ID:-196}"
export ROS_LOG_DIR="$A6Y_MOVEIT_ROOT/../diagnostics/arm_moveit_logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$A6Y_MOVEIT_ROOT/tools/arm_moveit/moveit_view.py" "$@"
