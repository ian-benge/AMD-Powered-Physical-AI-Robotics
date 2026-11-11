#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REP=${1:-1}
PROFILE=${2:-rehearsal}
SPEED=${3:-0.05}
MODE=${4:-mission}
[[ $# -le 4 && "$REP" =~ ^[123]$ && "$PROFILE" =~ ^(rehearsal|full)$ && "$SPEED" =~ ^0\.(05|10)$ && "$MODE" =~ ^(mission|contact)$ ]] || {
  echo 'Usage: start_autonomous_demo.sh [1|2|3] [rehearsal|full] [0.05|0.10] [mission|contact]'; exit 2;
}
EXTRA=()
if [[ "$MODE" == contact ]]; then
  [[ "$SPEED" == 0.05 ]] || { echo 'Contact qualification requires the 0.05 configuration'; exit 2; }
  EXTRA=(--fixture flat)
fi
python3 -B "$ROOT/autonomous_demo_assess.py" --check --speed "$SPEED" "${EXTRA[@]}"
eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"
CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")
if [[ -f "$CASE/functional-parity.json" ]]; then
  python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/terrain-phase2-20261105T031610Z/verify-parity-reuse.py "$SIM_KIT" "$CASE"
else
  bash "$SIM_KIT/trial.sh" "$SIM_LABEL" parity
fi
if [[ "$MODE" == contact ]]; then
  echo 'Stationary contact fixture: use view_autonomous_demo.sh for inspection, confirm clearance, and keep stopped. Restart after the probe.'
else
  echo 'Browser profile: Gazebo 3D window disabled; Ogre RGB-D still uses the connected desktop.'
  echo 'Keep stopped. Use view_autonomous_demo.sh for brief launch-clearance inspection before confirmation.'
fi
exec python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/application-v9/management/phase2_desktop_launch.py \
 "$ROOT/autonomous_demo_assess.py" --kit "$SIM_KIT" --case "$CASE" --representative "$REP" --profile "$PROFILE" --speed "$SPEED" "${EXTRA[@]}" --attended
