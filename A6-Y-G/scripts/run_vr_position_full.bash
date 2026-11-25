#!/usr/bin/env bash
# Full native-position control for the operator-requested physical trial.
# Starts DISABLED; use vr_position_control.bash prepare, then arm.
set -euo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_vr_position.bash" \
  mode:=commission stage:=follow motion_range:=full \
  operator_trial:=true "$@"
