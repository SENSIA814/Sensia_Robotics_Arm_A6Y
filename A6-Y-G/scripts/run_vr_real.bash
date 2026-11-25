#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
a6y_load_install
cd "$A6Y_HANDOFF_ROOT"
mode=readonly
for arg in "$@"; do
  case "$arg" in mode:=*) mode="${arg#mode:=}" ;; esac
done
case "$mode" in readonly|execute|commission) export ROS_DOMAIN_ID="${A6Y_REAL_ROS_DOMAIN_ID:-0}" ;; mock) export ROS_DOMAIN_ID=86 ;; *) echo 'mode:=readonly|mock|commission|execute' >&2; exit 2 ;; esac
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/a6y-vr-ros-log}"
mkdir -p "$ROS_LOG_DIR"
# Own-launch exclusion plus graph check for older entry points.
exec 9>"/tmp/a6y-vr-domain-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo '该域中已有 VR 实机入口。' >&2; exit 2; }
exec ros2 launch a6_y_vr_teleop a5_vr_real.launch.py "$@"
