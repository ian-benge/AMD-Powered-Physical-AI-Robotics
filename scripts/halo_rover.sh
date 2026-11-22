#!/usr/bin/env bash
set -euo pipefail

CAPSTONE=/home/ian/capstone-training
SIM_STAGE=$CAPSTONE/rover-sim
DEMO_ROOT=$CAPSTONE/rover-navigation/navigation_sessions/autonomous-demo-20261109
DEMO_RUNTIME=$DEMO_ROOT/runtime-v50
TERRAIN_STARTER=$DEMO_ROOT/gpu-repair-20261117/terrain-sdma-off-v3/start_terrain.sh

usage() {
  cat <<'USAGE'
Halo V50 survey commands

  capture       Start a fresh capture
  temps         Monitor temperatures
  npu           Qualify and start the NPU service
  demo-terrain  Check cooldown and start terrain compute
  demo          Start V50 survey at 0.15 m/s
  view          Inspect the stopped scene for 10 seconds
  rviz          Open the live navigation map
  gazebo        Open the live Gazebo GUI
  stop          Stop the workloads
  finish        Record the protected closeout
  status        Show the current case status

Use a separate Halo terminal for each service. Optional arguments:
  halo-rover demo [1|2|3] [survey|rehearsal|full] [0.05|0.10|0.15]
  halo-rover view [5..90 seconds]
USAGE
}

live_view() {
  exec docker exec -it rover-navigation bash -c '
    source /opt/ros/jazzy/setup.bash &&
    source /work/install/setup.bash &&
    exec "$@"
  ' halo-rover "$@"
}

case "${1-}" in
  capture)
    if [[ $# -eq 1 ]]; then
      exec /usr/bin/bash "$SIM_STAGE/capture-92c-20261122/capture.sh"
    fi
    ;;
  demo-terrain)
    [[ $# -eq 1 ]] || { echo 'Usage: halo-rover demo-terrain' >&2; exit 2; }
    context=$(/usr/bin/python3 -I "$SIM_STAGE/manage.py" context)
    eval "$context"
    /usr/bin/python3 -I "$SIM_KIT/trial.py" "$SIM_LABEL" cooldown --cooldown-phase terrain
    exec /usr/bin/bash "$TERRAIN_STARTER"
    ;;
  demo)
    [[ $# -le 4 ]] || { usage >&2; exit 2; }
    exec /usr/bin/bash "$DEMO_RUNTIME/start_autonomous_demo.sh" "${2:-1}" "${3:-survey}" "${4:-0.15}"
    ;;
  view)
    shift
    exec /usr/bin/bash "$DEMO_RUNTIME/view_autonomous_demo.sh" "$@"
    ;;
  rviz)
    [[ $# -eq 1 ]] || { echo 'Usage: halo-rover rviz' >&2; exit 2; }
    live_view rviz2 -d /work/install/rover_navigation/share/rover_navigation/config/navigation.rviz \
      --ros-args -p use_sim_time:=true
    ;;
  gazebo)
    [[ $# -eq 1 ]] || { echo 'Usage: halo-rover gazebo' >&2; exit 2; }
    live_view gz sim -g
    ;;
  help|-h|--help)
    usage
    exit 0
    ;;
  '')
    usage >&2
    exit 2
    ;;
esac

exec /usr/bin/bash "$SIM_STAGE/rover-sim.sh" "$@"
