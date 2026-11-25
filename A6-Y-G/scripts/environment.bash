#!/usr/bin/env bash
# Source from handoff entry scripts; isolate this child process from old overlays.
A6Y_HANDOFF_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PYTHONNOUSERSITE=1
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
unset VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo '需要先安装 ROS 2 Humble：/opt/ros/humble/setup.bash 不存在。' >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
a6y_load_install() {
  if [[ ! -f "$A6Y_HANDOFF_ROOT/install/local_setup.bash" ]]; then
    echo '请先按 README 的构建步骤运行 colcon build。' >&2
    exit 2
  fi
  set +u
  source "$A6Y_HANDOFF_ROOT/install/local_setup.bash"
  set -u
  local package prefix
  for package in a5_description a6_y_ros2_control a6_y_controllers; do
    prefix="$(ros2 pkg prefix "$package")"
    if [[ "$prefix" != "$A6Y_HANDOFF_ROOT/install/"* ]]; then
      echo "加载了交接目录外的包：$package $prefix" >&2
      exit 2
    fi
  done
}
