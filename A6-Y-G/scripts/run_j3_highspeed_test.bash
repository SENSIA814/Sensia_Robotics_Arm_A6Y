#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.bash"
a6y_load_install
cd "$A6Y_HANDOFF_ROOT"
mode=mock
args=("$@")
for ((i=0;i<${#args[@]};i++)); do
  if [[ "${args[i]}" == --mode ]]; then mode="${args[i+1]:-}"; fi
  if [[ "${args[i]}" == --mode=* ]]; then mode="${args[i]#--mode=}"; fi
done
case "$mode" in
  mock) export ROS_DOMAIN_ID=91 ;;
  real) export ROS_DOMAIN_ID="${A6Y_REAL_ROS_DOMAIN_ID:-0}" ;;
  *) echo '--mode mock|real required' >&2; exit 2 ;;
esac
export ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/a6y-j3-highspeed-ros-log
mkdir -p "$ROS_LOG_DIR"
exec python3 tools/test_j3_highspeed.py "$@"
