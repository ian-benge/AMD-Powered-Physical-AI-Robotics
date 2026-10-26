#!/usr/bin/env bash
# A separate simulator/domain. No training or existing live container is stopped.
set -euo pipefail
if [[ $# -lt 2 ]]; then
  echo 'Usage: bash scripts/halo_navigation.sh /absolute/scene-bundle scene_050000 [session-name] [extra ROS launch arguments...]' >&2
  exit 2
fi
NAV_BUNDLE=$(realpath "$1")
NAV_SCENE="$2"
NAV_SESSION=${3:-"$(date -u +%Y%m%dT%H%M%SZ)"}
if [[ ! "$NAV_SCENE" =~ ^scene_[0-9]+$ || ! "$NAV_SESSION" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo 'Invalid scene or session name' >&2; exit 2
fi
[[ -f "$NAV_BUNDLE/scenes/$NAV_SCENE/scene.json" ]] || { echo 'Scene not found' >&2; exit 2; }
[[ -f install_navigation/setup.bash ]] || { echo 'Build install_navigation first; see docs/NAVIGATION.md' >&2; exit 2; }
if [[ $# -ge 3 ]]; then shift 3; else shift 2; fi
NAV_TERRAIN_BACKEND=rocm_http
for NAV_ARG in "$@"; do
  [[ "$NAV_ARG" != terrain_backend:=* ]] || NAV_TERRAIN_BACKEND=${NAV_ARG#terrain_backend:=}
done
if [[ "$NAV_TERRAIN_BACKEND" == rocm_http ]]; then
  python3 - <<'PY'
import json,urllib.request
try:
    status=json.load(urllib.request.urlopen('http://127.0.0.1:8782/status',timeout=3))
    if not status.get('ready') or status.get('backend')!='rocm':
        raise RuntimeError(status.get('error') or 'AMD GPU service is not ready')
except Exception as error:
    raise SystemExit(f'GPU terrain unavailable: {error}. Start bash scripts/halo_terrain_gpu.sh in another terminal, or explicitly pass terrain_backend:=cpu.')
PY
fi
NAV_DEVICES=()
NAV_DEFAULT_SOFTWARE=1
if [[ -d /dev/dri ]]; then
  NAV_DEFAULT_SOFTWARE=0
  NAV_DEVICES+=(--device /dev/dri)
  for NAV_RENDER in /dev/dri/renderD* /dev/dri/card*; do
    [[ ! -e "$NAV_RENDER" ]] || NAV_DEVICES+=(--group-add "$(stat -c '%g' "$NAV_RENDER")")
  done
fi
NAV_DISPLAY=()
for NAV_ARG in "$@"; do
  if [[ "$NAV_ARG" == gui:=true || "$NAV_ARG" == rviz:=true ]]; then
    if [[ -z "${DISPLAY:-}" ]] && command -v systemctl >/dev/null; then
      while IFS='=' read -r NAV_KEY NAV_VALUE; do
        case "$NAV_KEY" in
          DISPLAY) export DISPLAY="$NAV_VALUE" ;;
          XAUTHORITY) export XAUTHORITY="$NAV_VALUE" ;;
        esac
      done < <(systemctl --user show-environment 2>/dev/null)
    fi
    [[ -n "${DISPLAY:-}" ]] || { echo 'GUI/RViz requires DISPLAY; launch from the Halo desktop terminal.' >&2; exit 2; }
    if command -v xrandr >/dev/null && xrandr --query 2>/dev/null | head -n 1 | grep -q 'current 0 x 0'; then
      echo 'The desktop has no active monitor (0 x 0). Connect the Ubuntu remote desktop, or use gui:=false rviz:=false.' >&2
      exit 2
    fi
    command -v xauth >/dev/null || { echo 'Install xauth to use GUI/RViz.' >&2; exit 2; }
    NAV_XAUTH=$(mktemp /tmp/rover-navigation-xauth.XXXXXX)
    trap 'rm -f -- "$NAV_XAUTH"' EXIT
    xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$NAV_XAUTH" nmerge -
    [[ -s "$NAV_XAUTH" ]] || { echo 'No desktop X11 cookie found; check XAUTHORITY.' >&2; exit 2; }
    NAV_DISPLAY=(-e DISPLAY -e XAUTHORITY=/tmp/navigation.xauth -e QT_X11_NO_MITSHM=1 -e QT_QPA_PLATFORM=xcb
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$NAV_XAUTH:/tmp/navigation.xauth:ro")
    break
  fi
done
NAV_DOCKER=(docker)
docker info >/dev/null 2>&1 || NAV_DOCKER=(sudo docker)
# NAV_FASTDDS_PROFILE=default compares against Fast DDS's built-in transports.
NAV_FASTDDS=(-e FASTRTPS_DEFAULT_PROFILES_FILE=/work/src/rover_navigation/config/fastdds.xml)
[[ "${NAV_FASTDDS_PROFILE:-navigation}" != default ]] || NAV_FASTDDS=()
# Keep the shell alive so its EXIT trap removes the temporary display cookie.
"${NAV_DOCKER[@]}" run --rm --init --name rover-navigation --network host --shm-size=1g \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID="${NAV_ROS_DOMAIN_ID:-87}" -e GZ_PARTITION="rover-navigation-$(id -u)" \
  -e LIBGL_ALWAYS_SOFTWARE="${NAV_SOFTWARE_RENDERING:-$NAV_DEFAULT_SOFTWARE}" \
  "${NAV_FASTDDS[@]}" "${NAV_DEVICES[@]}" "${NAV_DISPLAY[@]}" -v "$PWD:/work" -v "$NAV_BUNDLE:/scenes:ro" -w /work \
  rover-navigation bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /work/install_navigation/setup.bash
    exec ros2 launch rover_navigation navigation.launch.py \
      scene:="/scenes/scenes/$1/scene.json" session:="/work/navigation_sessions/$2" "${@:3}"
  ' navigation "$NAV_SCENE" "$NAV_SESSION" "$@"
