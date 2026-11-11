#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108
APP=/home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/application-v6
python3 -B "$ROOT/capture5000_assess.py" --check
eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"
CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")
exec python3 "$APP/management/phase2_desktop_launch.py" "$ROOT/capture5000_terrain.py" \
  "$APP/CANDIDATE_CONFIGURATION.json" --kit "$SIM_KIT" --case "$CASE" --module rover_navigation.terrain_service
