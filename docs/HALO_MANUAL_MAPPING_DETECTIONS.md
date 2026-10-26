# Manual mapping with live detections

Drive from a localhost page, see trained object classifications, and retain
depth-based detection candidates beside the terrain map. This workflow uses
scene 50000 and the host YOLOv8s NPU detector. The separate autonomous
waypoint/return milestone remains unfinished. Arms and sample collection are
outside this workflow.

The [October 20 verification report](HALO_MANUAL_MAPPING_REPORT_20261020.md)
records the actual checks, failures and detector limitations. The
[October 21 NPU switch report](HALO_NAVIGATION_NPU_SWITCH_20261021.md) records
the current detector deployment and live verification.

Object mapping now uses [Phase Three confirmation and revisit tracking](NAVIGATION_PHASE_THREE.md).
The default map shows confirmed rocks; raw camera detections remain immediate.
Enable **Show tentative candidates** to inspect pending observations and reasons.

## Build and launch on Halo

Use `/home/ian/capstone-training/rover-navigation`, branch
`navigation/mapping-and-exploration`. Build the five navigation packages using
the [navigation build instructions](NAVIGATION.md). Fresh SSH logins can use
Docker directly through the user's Docker group.

Start the trained detector in a Halo terminal, leaving it running:

```bash
cd /home/ian/capstone-training/rover-navigation
bash scripts/halo_navigation_detector.sh
```

The default is the verified YOLOv8s A16W8 NPU deployment, in the separate
`rover-npu` environment, listening on `127.0.0.1:8775`. It uses the preserved
four-class source checkpoint and a static 576x960 ONNX artifact. No model runs
inside the ROS container. Do not launch another instance if `/status` already
reports the correct NPU model:

```bash
curl -fsS http://127.0.0.1:8775/status
```

Require `ready: true`, the four reviewed class names, `model.backend: npu`,
`executed_device: npu`, `proof_state: npu_executed`, and verified compilation/
compute-offload evidence. The browser identifies the model as
`YOLOv8 · AMD NPU · a16w8`. The source checkpoint SHA-256 is
`89bbcf4ff124996fb6a520c5bf751287c2b5ea47bbd1d7e2adaf7f530ee44ca3`.
No model weights are copied into this repository or the ROS container.

The NPU launcher sources the existing matched XRT setup without installing or
upgrading it. Startup verifies checkpoint/artifact hashes, warms up the model
and checks real NPU execution before listening. An inference failure marks the
service unready and requires an explicit backend restart; it does not switch
to GPU automatically.

For an explicit GPU rollback, stop only the detector service currently owning
8775, then run:

```bash
cd /home/ian/capstone-training/rover-navigation
bash scripts/halo_navigation_detector.sh --backend torch
```

Use `--model-manifest /absolute/manifest.json` to select another verified NPU
artifact, with its matching positional source checkpoint. Use `--port` only
when the navigation client's `detector_url` is configured for that port.

In a second Halo terminal, launch a new session name:

```bash
cd /home/ian/capstone-training/rover-navigation
MODEL=/home/ian/capstone-training/mars-pilot50-v2/runs/detect/coverage500_fourclass_yolov8s_npu_960/weights/best.pt
bash scripts/halo_navigation.sh \
  /home/ian/capstone-training/mars-pilot50-v2/dataset/live_eval_scenes \
  scene_050000 manual-mapping-001 \
  gui:=true rviz:=true preview:=true mission_controls:=true \
  controls_port:=8777 mapping_on_confirm:=true \
  detector_model:="$MODEL"
```

The wrapper rejects existing session directories and an occupied container name.
Keep the full Ubuntu remote desktop connected for Gazebo/RViz. For headless use,
set `gui:=false rviz:=false`. Omit `detector_model` to run camera/manual mapping
without inference; the page explicitly reports that no detector is configured.
`detector_backend:=remote_http` is the default when a model is supplied.

In a third Halo terminal, keep the scene-control gateway running:

```bash
cd /home/ian/capstone-training/rover-navigation
bash scripts/halo_navigation_scene_control.sh \
  /home/ian/capstone-training/mars-pilot50-v2/dataset/live_eval_scenes \
  manual-mapping-001
```

The session name must match the already launched session. If the gateway is
already running, use its existing page instead of starting another instance.
This host process serves the page on 8774 and keeps restart progress/errors
available while the ROS container restarts. Its private supervisor listens on
8776; the ROS controls listen on 8777. Only the public local page needs forwarding.
Launches without this gateway can keep the original controls port 8774, but do
not provide a persistent Restart button.

## Open the page

On the Halo desktop open **http://127.0.0.1:8774**. The separate raw preview is
still **http://127.0.0.1:8773/frame.png** and has no driving controls.

For a Windows browser, keep this PowerShell command running:

```powershell
& C:\Windows\System32\OpenSSH\ssh.exe -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -i "$env:USERPROFILE\.ssh\halo_ed25519" -o IdentitiesOnly=yes -L 127.0.0.1:8874:127.0.0.1:8774 -L 127.0.0.1:8873:127.0.0.1:8773 ian@halo-strix
```

Open **http://127.0.0.1:8874** on Windows. For the raw preview there, use
**http://127.0.0.1:8873/frame.png**. The raw-preview link inside the page uses
the Halo port 8773; use the forwarded URL explicitly on Windows.

1. Inspect the physical launch pad in Gazebo and confirm its existing 1.1 m
   clearance with **Confirm clear launch area**. Confirmation cannot erase
   measured hazards or make unseen terrain outside that disk traversable.
2. Wait for fresh sensors and **ready**, then click **Enable manual drive**.
3. Hold **W / Up** for 0.05 m/s forward motion. Hold **A / Left** or **D / Right**
   for 0.3 rad/s turns. Pointer buttons provide the same hold behavior. Reverse
   is unavailable. Releasing a key stops that input and sends zero velocity when
   no drive keys remain held. Manual mode stays selected between commands.
   The active page continues sending zero commands at rest; the guard's
   0.35-second wall-time command lease remains unchanged.
4. Press **Space** or **Stop** for an explicit stop. Losing page focus, hiding
   the tab or losing fresh status also releases the browser input. A delayed
   request cannot revive an expired/released input token.
5. Watch guard mode, reason, latched fault and final command. Unknown ground,
   unsafe terrain, stale sensors and localization loss still stop the rover.
   Inspect the cause before selecting manual mode again.

**Restart scene** stops motion, returns the simulated rover to launch, and
starts a new session with a fresh SLAM database, terrain memory, candidate list,
mission/home state and clock. Previous session folders are retained. The new
terrain map and candidate list stay empty until **Confirm launch area & start
mapping** is pressed. Camera and localization initialize in the background.
After confirmation, wait for ready and select manual mode. A new session never
reuses a clearance confirmation or motion command from the old one.

An `unsafe_or_unobserved_ground` stop means the proposed motion's full rover
footprint reaches an unsafe or unknown cell. The page now names that cause.
Selecting manual mode cannot make that route safe; turn toward mapped ground
or restart at the inspected launch area. This check remains active in manual mode.

The **Manual drive enabled** button confirms that the page still owns its
manual session. Stop ends that session. Leaving the page or losing its fresh
connection releases control; a connection loss while moving still causes a
latched `command_timeout`. Restoring the connection cannot restart movement.

Startup can record `invalid_odometry` before RGB-D localization initializes.
It can remain visible while the rover shows stopped/ready with fresh sensors.
Explicit manual selection clears it once readiness is
established; a loss after driving begins remains a latched safety stop.

Only `navigation_motion_guard` publishes `/cmd_vel`. The page publishes only
`/navigation/manual_cmd_vel`; it never starts the older browser/Gazebo driving
bridge. Existing waypoint controls remain collapsed and require their existing
explicit preview/Start workflow.

## Detections and saved candidates

The trained classes are `rock`, `pale_soil_patch`, `yellow_crystal_rock` and
`mudstone_slab`. The page draws class/confidence on the exact inferred image,
then shows the map, current rover pose and candidate IDs. Raw-camera fallback
is labeled when inference is unavailable; old boxes are not painted on a newer
raw frame. The detector uses bounded latest-frame buffers and one inference
worker. It does not accumulate an inference queue.

The mapper requires matching RGB detection/depth/CameraInfo timestamps and
optical frames, robust valid depth inside each box, and image-time transforms
to `odom`, with image-time odometry covariance. Missing alignment, depth or
transforms prevents evidence updates. Tracks use uncertainty-aware one-to-one
association, class evidence, appearance and physical extent. Positions remain
in `odom` and are reprojected using the current `map -> odom` correction,
including RViz markers. A changing class label alone does not create another ID.
An odometry reset starts a new archived track epoch rather than mixing frames.

Confirmed rocks persist when they leave view. Tentative candidates expire after
30 simulated seconds without observation; eligible visible contradictions can
demote and retire confirmed tracks. State is saved every second to
`navigation_sessions/SESSION/sample-sites.json`, with all tracks under `tracks`.
The append-only `sample-sites-events.jsonl` records evidence and transitions.
Persistence does not
mean automatic restoration into a new odometry session. The mapper rejects an
existing output file rather than silently reusing an old coordinate system.

Confirmed markers are published durably on `/navigation/sample_site_markers`.
Version-two `/navigation/sample_sites` has confirmed-only `objects` and separate
`tentative_objects`. Optional tentative RViz markers use
`/navigation/tentative_site_markers`. Confirmation requires three translated
viewpoints and raised depth geometry; soil patches and slabs remain tentative.
These topics do not change terrain costs, RTAB-Map or Nav2 goals. Depth gives
surface locations, not precise collection poses. Tracking reduces false
placements; it does not make detector classifications infallible.

Red terrain is separate from detector candidates: calibrated depth is
back-projected into 3D, transformed at its observation time and classified on
2.5 cm cells. A measured slope above 12 degrees or height span above 3 cm marks
a hazard. Two depth-frame confirmations retain it for the session; later free
readings cannot erase it. The page displays a conservative 5 cm projection.
Gray is unknown and pale ground is considered traversable. A red patch can be
rough or steep ground rather than a rock, and depth/localization errors can
also create conservative markings.

## Repeatable verification

Inside the running container, source ROS and the navigation install:

```bash
source /opt/ros/jazzy/setup.bash
source /work/install_navigation/setup.bash
python3 scripts/halo_navigation_detection_check.py \
  --output navigation_sessions/SESSION/checks/detections-001.json --duration 5
```

This observer cannot move the rover. It saves raw JSONL, an annotated image,
alignment counts, model identity, durable-marker receipt and publisher ownership.
It checks plumbing, not detector accuracy. Use a new output name each time.

The separate `halo_navigation_manual_check.py` can move the rover and is
restricted to fresh `flat` or `flat_edge` fixtures. It records truth independently
for the one-second browser-input/release test or the bounded known-ground-edge
test (`--edge`). Historical smoke-helper restrictions are unchanged.

For UI verification from Windows through port 8874:

```powershell
node scripts/navigation_browser_check.cjs navigation_sessions/SESSION/checks/browser-001
```

The optional `--drive` starts with a 750 ms forward hold; `--turn-right` starts
with a three-second turn. Both then check two short turns separated by idle
intervals, explicit Stop, and a bounded connection-loss stop during a short
forward command. These options select manual mode and require an already
inspected, confirmed launch area. They use a separate Chrome test profile and
save a screenshot and UI-state report. They do not use a simulator truth feed.

Stop the rover before ending the launch terminal. Stop the dedicated host
detector with Ctrl+C in its own terminal when finished. Leave other Halo/NPU
services untouched.
