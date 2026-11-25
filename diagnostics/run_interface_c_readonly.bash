#!/usr/bin/env bash
# View seven real motor states. All joints must remain read_only=true.
set -eo pipefail
A6Y_DIAG_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A6Y_RUNTIME_DIR=/home/taoqiu/a6y_handoff_20261124
source "$A6Y_RUNTIME_DIR/scripts/environment.bash"
a6y_load_install
export ROS_DOMAIN_ID="${A6Y_REVIEW_DOMAIN_ID:-194}" ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$A6Y_DIAG_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"
A6Y_PORT=/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7-dirty_github.com_normaldotcom_canable2-fw.git_209B3971534B-if00
A6Y_CONFIG="$A6Y_DIAG_DIR/motors_209B3971534B_readonly.yaml"
[[ -e "$A6Y_PORT" ]] || { echo 'R2 adapter 209B3971534B is absent.' >&2; exit 2; }
/usr/bin/python3 - "$A6Y_CONFIG" "$A6Y_PORT" <<'PY'
import sys, subprocess, yaml
from pathlib import Path
joints = yaml.safe_load(Path(sys.argv[1]).read_text())['joints']
if len(joints) != 7 or not all(j.get('read_only') is True for j in joints.values()):
    raise SystemExit('Refusing: all seven joints must be read-only')
if subprocess.run(['fuser', sys.argv[2]], capture_output=True).returncode != 1:
    raise SystemExit('Refusing: serial port occupied or occupancy check failed')
PY
exec ros2 launch a6_y_controllers a5_gravity_comp.launch.py \
  dry_run:=false auto_enable:=false can_channel:="$A6Y_PORT" \
  motor_config_file:="$A6Y_CONFIG" use_rviz:="${A6Y_REVIEW_RVIZ:-true}"
