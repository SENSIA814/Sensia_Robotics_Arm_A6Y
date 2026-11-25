#!/usr/bin/env bash
# Isolated overlay of the user's archived ROS packages. Never use the old install/.
set -euo pipefail
A6Y_NATIVE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
A6Y_NATIVE_SRC="$A6Y_NATIVE_ROOT/tools/arm_moveit/ros2_stack/src"
# rosidl resolves symlinks and cannot parse the Chinese source/build path.
# This regenerable ASCII cache contains only copies of this task's packages.
A6Y_NATIVE_BUILD="/tmp/a6y-moveit-build-$(id -u)"
if [[ -L "$A6Y_NATIVE_BUILD" ]] || [[ -e "$A6Y_NATIVE_BUILD" && ! -O "$A6Y_NATIVE_BUILD" ]]; then
  echo '构建缓存目录不属于当前用户或是符号链接，拒绝使用。' >&2; exit 2
fi
mkdir -p "$A6Y_NATIVE_BUILD"
A6Y_NATIVE_HASH="$(find "$A6Y_NATIVE_SRC" -type f -not -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d ' ' -f 1)"
if [[ ! -f "$A6Y_NATIVE_BUILD/source.sha256" ]] || [[ "$(cat "$A6Y_NATIVE_BUILD/source.sha256")" != "$A6Y_NATIVE_HASH" ]] || [[ ! -f "$A6Y_NATIVE_BUILD/install/local_setup.bash" ]]; then
  echo '正在构建用户原版 MoveIt / ros2_control 移植包…'
  /usr/bin/python3 -c 'import shutil,sys; shutil.copytree(sys.argv[1], sys.argv[2], dirs_exist_ok=True)' "$A6Y_NATIVE_SRC" "$A6Y_NATIVE_BUILD/src"
  colcon --log-base "$A6Y_NATIVE_BUILD/log" build --base-paths "$A6Y_NATIVE_BUILD/src" \
    --build-base "$A6Y_NATIVE_BUILD/build" --install-base "$A6Y_NATIVE_BUILD/install" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  printf '%s\n' "$A6Y_NATIVE_HASH" > "$A6Y_NATIVE_BUILD/source.sha256"
fi
set +u
source "$A6Y_NATIVE_BUILD/install/local_setup.bash"
set -u
if [[ "$(ros2 pkg prefix a6_y_ros2_control)" != "$A6Y_NATIVE_BUILD/install/a6_y_ros2_control" ]]; then
  echo '硬件插件没有来自本次移植目录，已拒绝启动。' >&2
  exit 2
fi
