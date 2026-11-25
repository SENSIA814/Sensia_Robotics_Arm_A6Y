#!/usr/bin/env bash
# May be sourced by the launcher; source the ROS environment first.
set -euo pipefail
A6Y_PANEL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
A6Y_PANEL_SRC="$A6Y_PANEL_ROOT/tools/arm_moveit/rviz_panel"
A6Y_PANEL_BUILD="$A6Y_PANEL_ROOT/build/arm_moveit_ui"
mkdir -p "$A6Y_PANEL_BUILD"
A6Y_PANEL_HASH="$(find "$A6Y_PANEL_SRC" -maxdepth 1 -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d ' ' -f 1)"
if [[ ! -f "$A6Y_PANEL_BUILD/source.sha256" ]] || [[ "$(cat "$A6Y_PANEL_BUILD/source.sha256")" != "$A6Y_PANEL_HASH" ]] || [[ ! -f "$A6Y_PANEL_BUILD/install/local_setup.bash" ]]; then
  echo '正在构建 RViz 规划与夹爪面板（仅首次或代码更新后）…'
  colcon --log-base "$A6Y_PANEL_BUILD/log" build --base-paths "$A6Y_PANEL_SRC" \
    --build-base "$A6Y_PANEL_BUILD/build" --install-base "$A6Y_PANEL_BUILD/install" \
    --packages-select a6y_rviz_panel --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  printf '%s\n' "$A6Y_PANEL_HASH" > "$A6Y_PANEL_BUILD/source.sha256"
fi
set +u
source "$A6Y_PANEL_BUILD/install/local_setup.bash"
set -u
