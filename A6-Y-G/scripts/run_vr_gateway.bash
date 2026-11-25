#!/usr/bin/env bash
set -euo pipefail
A6Y_VR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
A6Y_VR_PYTHON="${A6Y_VR_PYTHON:-$A6Y_VR_ROOT/.venv-vr/bin/python}"
if ! "$A6Y_VR_PYTHON" -c 'import vuer, numpy, scipy' >/dev/null 2>&1; then
  echo 'VR 网关环境缺失，请按 docs/VR遥操作/虚拟机械臂/VR_RVIZ_USAGE.md 安装依赖。' >&2
  exit 2
fi
exec "$A6Y_VR_PYTHON" "$A6Y_VR_ROOT/vr_gateway/quest_gateway.py" "$@"
