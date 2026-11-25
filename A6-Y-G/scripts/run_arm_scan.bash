#!/usr/bin/env bash
set -euo pipefail
A6Y_SCAN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Diagnostic tools use the existing source driver; ROS build/source is unnecessary.
export PYTHONNOUSERSITE=1
unset PYTHONPATH
if [[ "${1:-}" == "--cli" ]]; then
  shift
  exec /usr/bin/python3 "$A6Y_SCAN_ROOT/tools/arm_diagnostics/scanner.py" "$@"
fi
if [[ "${1:-}" == "--web" ]]; then
  shift
  exec /usr/bin/python3 "$A6Y_SCAN_ROOT/tools/arm_diagnostics/server.py" "$@"
fi
exec /usr/bin/python3 "$A6Y_SCAN_ROOT/tools/arm_diagnostics/desktop.py" "$@"
