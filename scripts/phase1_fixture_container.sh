#!/usr/bin/env bash
# Isolated camera fixture, no bridge to any ROS or navigation node.
set -euo pipefail
[[ "${P1_SUPERVISED:-}" == 1 ]] || { echo 'Use phase1_run_fixture.py with current halo-rover capture gates'; exit 2; }
[[ $# == 4 ]] || { echo 'Usage: phase1_fixture_container.sh NAME IMAGE ABS_EVIDENCE FIXTURE_DIR'; exit 2; }
P1_NAME=$1
P1_IMAGE=$2
P1_EVIDENCE=$(realpath "$3")
P1_FIXTURE=$4
[[ "$P1_NAME" =~ ^rover-phase1-[a-z0-9-]+$ && "$P1_FIXTURE" =~ ^fixture[0-9]+$ ]] || exit 2
[[ "$P1_IMAGE" =~ ^sha256:[a-f0-9]{64}$ ]] || { echo 'Immutable image ID required'; exit 2; }
[[ -f "$P1_EVIDENCE/$P1_FIXTURE/world.sdf" ]] || exit 2
while IFS='=' read -r P1_KEY P1_VALUE; do
  case "$P1_KEY" in DISPLAY|XAUTHORITY) export "$P1_KEY=$P1_VALUE" ;; esac
done < <(systemctl --user show-environment)
[[ -n "${DISPLAY:-}" && -r "${XAUTHORITY:-}" ]] || { echo 'Current desktop display authentication required'; exit 2; }
P1_XAUTH=$(mktemp /tmp/phase1-xauth.XXXXXX)
trap 'rm -f -- "$P1_XAUTH"' EXIT
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$P1_XAUTH" nmerge -
[[ -s "$P1_XAUTH" ]] || exit 2
docker run --rm --init --name "$P1_NAME" --label rover.phase1.fixture=true --network host --shm-size=1g \
  --user "$(id -u):$(id -g)" --device /dev/dri --group-add "$(stat -c %g /dev/dri/renderD128)" \
  -e HOME=/tmp -e DISPLAY -e XAUTHORITY=/tmp/fixture.xauth -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e GZ_PARTITION="$P1_NAME" -e ROS_DOMAIN_ID=89 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$P1_XAUTH:/tmp/fixture.xauth:ro" \
  -v "$P1_EVIDENCE:/evidence" "$P1_IMAGE" bash -c \
  'source /opt/ros/jazzy/setup.bash; exec gz sim -r -s --render-engine-server ogre "/evidence/$1/world.sdf"' fixture "$P1_FIXTURE"
