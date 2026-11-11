#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SECONDS_TO_VIEW=${1:-60}
[[ $# -le 1 && "$SECONDS_TO_VIEW" =~ ^[0-9]+$ ]] || { echo 'Usage: view_autonomous_demo.sh [5..90 seconds]'; exit 2; }
python3 -B "$ROOT/autonomous_demo_assess.py" --check
eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"
CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")
exec python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/application-v9/management/phase2_desktop_launch.py \
 "$ROOT/demo_exterior_view.py" --kit "$SIM_KIT" --case "$CASE" --seconds "$SECONDS_TO_VIEW" --attended
