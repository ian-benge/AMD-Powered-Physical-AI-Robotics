#!/usr/bin/env bash
# Host-only scene restarts; never expose the Docker socket to ROS or the browser.
set -euo pipefail
if [[ $# != 2 ]]; then
  echo 'Usage: bash scripts/halo_navigation_scene_control.sh /absolute/scene-bundle existing-session-name' >&2
  exit 2
fi
NAV_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$NAV_PROJECT/src/rover_navigation${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m rover_navigation.scene_control --project "$NAV_PROJECT" --bundle "$1" --session "$2"
