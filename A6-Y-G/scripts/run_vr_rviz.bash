#!/usr/bin/env bash
set -eo pipefail
A6Y_VR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$A6Y_VR_ROOT/scripts/environment.bash"
a6y_load_install
export ROS_DOMAIN_ID="${A6Y_VR_DOMAIN_ID:-84}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-$A6Y_VR_ROOT/validation/vr_rviz/ros_logs}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch a6_y_vr_teleop a5_vr_rviz.launch.py "$@"
