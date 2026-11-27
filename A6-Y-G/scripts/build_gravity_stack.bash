#!/usr/bin/env bash
set -euo pipefail
A6Y_GRAVITY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
A6Y_GRAVITY_BUILD="/tmp/a6y-gravity-build-$(id -u)"
if [[ -L "$A6Y_GRAVITY_BUILD" ]] || [[ -e "$A6Y_GRAVITY_BUILD" && ! -O "$A6Y_GRAVITY_BUILD" ]]; then
  echo '重力控制器构建缓存身份无效。' >&2; exit 2
fi
mkdir -p "$A6Y_GRAVITY_BUILD"
A6Y_GRAVITY_HASH="$(find "$A6Y_GRAVITY_ROOT/src/A5" "$A6Y_GRAVITY_ROOT/src/a6_y_ros2_control" "$A6Y_GRAVITY_ROOT/src/a6_y_controllers" "$A6Y_GRAVITY_ROOT/src/a6_y_teleop_msgs" "$A6Y_GRAVITY_ROOT/src/a6_y_teleop" -type f -not -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d ' ' -f 1)"
a6y_gravity_python_matches() {
  /usr/bin/python3 - "$A6Y_GRAVITY_ROOT" "$A6Y_GRAVITY_BUILD" <<'PY'
from pathlib import Path
import sys
root, build = map(Path, sys.argv[1:])
source = root/'src/a6_y_teleop/a6_y_teleop'
destinations = list((build/'install/a6_y_teleop/lib').glob('python*/site-packages/a6_y_teleop'))
if len(destinations) != 1:
    raise SystemExit(1)
dest = destinations[0]
for path in source.rglob('*.py'):
    installed = dest/path.relative_to(source)
    if not installed.is_file() or installed.read_bytes() != path.read_bytes():
        raise SystemExit(1)
PY
}
if [[ ! -f "$A6Y_GRAVITY_BUILD/source.sha256" ]] || [[ "$(cat "$A6Y_GRAVITY_BUILD/source.sha256")" != "$A6Y_GRAVITY_HASH" ]] || [[ ! -f "$A6Y_GRAVITY_BUILD/install/local_setup.bash" ]] || ! a6y_gravity_python_matches; then
  echo '正在构建原有重力补偿控制器与配套 MIT 驱动…'
  /usr/bin/python3 - "$A6Y_GRAVITY_ROOT" "$A6Y_GRAVITY_BUILD" <<'PY'
import pathlib,shutil,sys
root, build = map(pathlib.Path, sys.argv[1:])
# setuptools uses mtimes: a restored source can be older than a different
# cached version. Regenerate only this Python package's build/install outputs.
cached = build/'build/a6_y_teleop/build'
if cached.exists(): shutil.rmtree(cached)
for installed in (build/'install/a6_y_teleop/lib').glob('python*/site-packages/a6_y_teleop'):
    shutil.rmtree(installed)
for name in ('A5','a6_y_ros2_control','a6_y_controllers','a6_y_teleop_msgs','a6_y_teleop'):
    dest=build/'src'/name
    if dest.exists(): shutil.rmtree(dest)
    shutil.copytree(root/'src'/name, dest, ignore=shutil.ignore_patterns('__pycache__'))
PY
  CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base "$A6Y_GRAVITY_BUILD/log" build --base-paths "$A6Y_GRAVITY_BUILD/src" \
    --build-base "$A6Y_GRAVITY_BUILD/build" --install-base "$A6Y_GRAVITY_BUILD/install" --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  if ! a6y_gravity_python_matches; then
    echo '实际安装的跟随脚本与当前源码不一致，已禁止启动。' >&2; exit 2
  fi
  printf '%s\n' "$A6Y_GRAVITY_HASH" > "$A6Y_GRAVITY_BUILD/source.sha256"
fi
set +u
source "$A6Y_GRAVITY_BUILD/install/local_setup.bash"
set -u
for A6Y_GRAVITY_PKG in a6_y_ros2_control a6_y_controllers a5_description a6_y_teleop; do
  if [[ "$(ros2 pkg prefix "$A6Y_GRAVITY_PKG")" != "$A6Y_GRAVITY_BUILD/install/$A6Y_GRAVITY_PKG" ]]; then
    echo "重力补偿包路径不符：$A6Y_GRAVITY_PKG" >&2; exit 2
  fi
done
