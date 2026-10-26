#!/usr/bin/env bash
# Existing ROCm environment; no package installation or driver changes.
set -euo pipefail
TERRAIN_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
TERRAIN_PYTHON=${TERRAIN_PYTHON:-/home/ian/miniforge3/envs/rover-yolo/bin/python}
[[ -x "$TERRAIN_PYTHON" ]] || { echo "ROCm Python not found: $TERRAIN_PYTHON" >&2; exit 2; }
export PYTHONPATH="$TERRAIN_ROOT/src/rover_navigation${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1
exec "$TERRAIN_PYTHON" -m rover_navigation.terrain_service "$@"
