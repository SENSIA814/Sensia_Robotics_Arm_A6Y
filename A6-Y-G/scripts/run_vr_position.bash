#!/usr/bin/env bash
set -euo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_vr_real.bash" "$@" control_backend:=position
