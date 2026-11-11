#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108
APP=/home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/application-v9
REP=${1:-1}
[[ $# -le 1 && "$REP" =~ ^[123]$ ]] || { echo 'Choose 1 (sparse), 2 (standard) or 3 (cluttered)'; exit 2; }
python3 -B "$ROOT/capture5000_assess.py" --check
eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"
CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")
if [[ -f "$CASE/functional-parity.json" ]]; then
  python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/terrain-phase2-20261105T031610Z/verify-parity-reuse.py "$SIM_KIT" "$CASE"
else
  bash "$SIM_KIT/trial.sh" "$SIM_LABEL" parity
fi
echo 'Experimental 40 m model assessment; inspect launch clearance before enabling manual drive.'
exec python3 "$APP/management/phase2_desktop_launch.py" "$ROOT/capture5000_assess.py" \
  --kit "$SIM_KIT" --case "$CASE" --representative "$REP" --seconds 1800 --attended
