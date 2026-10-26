#!/usr/bin/env bash
# Host NPU inference by default; explicit torch rollback keeps the GPU environment.
set -euo pipefail
NAV_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
NAV_NPU_PROJECT=${NAV_NPU_PROJECT:-/home/ian/capstone-training/mars-pilot50-v2}
NAV_DETECTOR_MODEL="$NAV_NPU_PROJECT/runs/detect/coverage500_fourclass_yolov8s_npu_960/weights/best.pt"
NAV_DETECTOR_MANIFEST="$NAV_NPU_PROJECT/experiments/npu/resume_20261016T221548Z/candidate/manifest.json"
NAV_DETECTOR_BACKEND=npu
NAV_DETECTOR_PORT=8775
while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend|--model-manifest|--port)
      [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --backend) NAV_DETECTOR_BACKEND=$2 ;;
        --model-manifest) NAV_DETECTOR_MANIFEST=$2 ;;
        --port) NAV_DETECTOR_PORT=$2 ;;
      esac
      shift 2 ;;
    --help|-h)
      echo 'Usage: bash scripts/halo_navigation_detector.sh [trained-best.pt] [--backend npu|torch] [--model-manifest manifest.json] [--port 8775]'
      exit 0 ;;
    --*) echo "Unknown option: $1" >&2; exit 2 ;;
    *) NAV_DETECTOR_MODEL=$1; shift ;;
  esac
done
NAV_DETECTOR_MODEL=$(realpath "$NAV_DETECTOR_MODEL")
[[ -f "$NAV_DETECTOR_MODEL" ]] || { echo 'Trained checkpoint not found' >&2; exit 2; }
case "$NAV_DETECTOR_BACKEND" in
  npu)
    # Vendor activation scripts assume unset optional variables are allowed.
    set +u
    source "$NAV_NPU_PROJECT/scripts/halo_npu_env.sh"
    set -u
    NAV_DETECTOR_PYTHON=${NAV_DETECTOR_PYTHON:-$VIRTUAL_ENV/bin/python}
    NAV_DETECTOR_OPTIONS=(--backend npu --model-manifest "$NAV_DETECTOR_MANIFEST") ;;
  torch)
    NAV_DETECTOR_PYTHON=${NAV_DETECTOR_PYTHON:-/home/ian/miniforge3/envs/rover-yolo/bin/python}
    NAV_DETECTOR_OPTIONS=(--backend torch) ;;
  *) echo 'Choose --backend npu or --backend torch; no automatic fallback' >&2; exit 2 ;;
esac
[[ -x "$NAV_DETECTOR_PYTHON" ]] || { echo 'The selected detector Python environment is missing' >&2; exit 2; }
export PYTHONPATH="$NAV_PROJECT/src/rover_perception:$NAV_PROJECT/src/rover_training${PYTHONPATH:+:$PYTHONPATH}"
exec "$NAV_DETECTOR_PYTHON" -m rover_perception.detector_service --model "$NAV_DETECTOR_MODEL" \
  "${NAV_DETECTOR_OPTIONS[@]}" --port "$NAV_DETECTOR_PORT"
