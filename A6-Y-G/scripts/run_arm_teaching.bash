#!/usr/bin/env bash
set -euo pipefail
A6Y_TEACHING_ENTRY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$A6Y_TEACHING_ENTRY/../diagnostics/arm_teaching_logs"
export A6Y_TEACHING_APP_LOG="$A6Y_TEACHING_ENTRY/../diagnostics/arm_teaching_logs/ui_$(date +%Y%m%d_%H%M%S)_$$.log"
exec > >(tee -a "$A6Y_TEACHING_APP_LOG") 2>&1
source "$A6Y_TEACHING_ENTRY/scripts/environment.bash"
source "$A6Y_TEACHING_ENTRY/scripts/build_gravity_stack.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${A6Y_GRAVITY_DOMAIN_ID:-197}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$A6Y_TEACHING_ENTRY/scripts/gravity_dds.xml}"
export ROS_LOG_DIR="$A6Y_TEACHING_ENTRY/../diagnostics/arm_teaching_logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 "$A6Y_TEACHING_ENTRY/tools/arm_gravity/teaching_view.py" "$@"
