# Rover mapping and navigation

This branch implements a separate Jazzy/Harmonic navigation stack. It includes
RGB-D mapping/localization, terrain grids, guarded Nav2 waypoint missions,
frontier exploration and return-home services. Arms and sample collection are
outside this package. **Full Gazebo/Halo acceptance is pending.** Passing the
offline tests is not a claim of autonomous driving or hardware readiness.
See [the implementation verification report](NAVIGATION_TEST_REPORT.md) for
completed checks and the remaining runtime gates.
For the connected manual browser, live detector and saved sample-site markers,
use [Manual mapping with live detections](HALO_MANUAL_MAPPING_DETECTIONS.md).
The [October 20 Halo smoke report](HALO_NAVIGATION_SMOKE_REPORT_20261020.md)
records the first measured manual-drive, command-lease, clock-pause and planner
checks; it does not establish autonomous mission acceptance.

## Build on the Halo

Use this branch in its own checkout. Keep existing training, live inference and
NPU acceptance artifacts in their current checkout. From the navigation root:

```bash
sudo docker build -f docker/Dockerfile.jazzy -t rover-jazzy .
sudo docker build -f docker/Dockerfile.navigation -t rover-navigation .
sudo docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/work" -w /work rover-navigation bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    colcon --log-base log_navigation build --base-paths src \
      --build-base build_navigation --install-base install_navigation \
      --packages-up-to rover_navigation
  '
```

Require every selected package to build before launching. The navigation image
adds RTAB-Map, Nav2, GridMap messages and depth-processing dependencies. Inference
is optional and stays in its existing host Conda environment.

## Start an isolated navigation session

Reuse an existing generated live-evaluation bundle, mounted read-only:

```bash
bash scripts/halo_navigation.sh \
  "$HOME/capstone-training/mars-pilot50-v2/dataset/live_eval_scenes" \
  scene_050000 first-mapping
```

This starts headless, with a separate ROS domain (87), Gazebo partition and
container. The source bundle is never rewritten. Each new session saves its
world, robot description, resolved configuration, provenance and RTAB-Map database
under `navigation_sessions/<session>`. An existing session directory is rejected.
Stop with Ctrl+C. Do not run training/other simulators during performance acceptance.

The navigation camera is RGB-D 960x540 at a nominal 30 Hz simulation time, with
no bounding-box camera. Its default pitch is 0.4 rad downward to reduce the
near-field blind gap from the original high camera. `camera_pitch:=0.1` reproduces
the original pitch for comparison. Capture profiles and RGB-only live profiles
remain unchanged. Camera geometry is provisional until physically calibrated;
existing detector accuracy does not validate the navigation camera viewpoint.

## Motion ownership and operator controls

In a second Halo terminal enter the same running container:

```bash
sudo docker exec -it rover-navigation bash
source /opt/ros/jazzy/setup.bash
source /work/install_navigation/setup.bash
```

Inspect RGB/depth and map output in RViz or ROS tools before enabling motion.
The launcher does not connect the existing browser/native teleop publishers to
the wheel command topic. Only `motion_guard` publishes `/cmd_vel`; the existing
Twist stamper converts to the Jazzy wheel controller's stamped interface.
Navigation opts into an additional 0.35-second wall-clock watchdog at that
adapter, so loss of the motion-guard process also clears commands during a
simulation pause. The watchdog is disabled by default for existing workflows.

Starting mode is stopped. Confirm a physically clear **1.1 m radius launch area**
around the rover, then explicitly seed that clearance once for this session:

```bash
ros2 service call /navigation/confirm_clear_start std_srvs/srv/Trigger '{}'
ros2 service call /navigation/manual std_srvs/srv/Trigger '{}'
```

This operator confirmation connects the camera's visible ground to the rover's
initial footprint; it is not a terrain/scene ground-truth feed. Measured hazards
are not cleared by the seed. Check the pitch/near-ground coverage before driving.

Manual teleop must publish continuously to `/navigation/manual_cmd_vel`; lease
expiry is 0.35 wall seconds. Example low-speed test, release with Ctrl+C:

```bash
ros2 topic pub --rate 20 /navigation/manual_cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}, angular: {z: 0.0}}'
```

Stop at any time:

```bash
ros2 service call /navigation/stop std_srvs/srv/Trigger '{}'
```

Select manual/autonomous mode again after a sensor/command fault to resume; old commands are
discarded. Manual selection cancels any Nav2 mission. Stale depth, stale or invalid
odometry, lost map transforms, paused/reset simulation clocks and unsupported
ground latch a stop. Reverse commands are clamped to zero. Defaults are 0.20 m/s,
0.60 rad/s; simulated time drives navigation and wall time drives the safety gate.
The present wall deadlines assume the simulator can supply fresh data at least
once per second. Slow software rendering may intentionally prevent motion.
After a simulation clock rewind, restart the navigation stack in a new session;
previous mapping/localization history is not eligible for automatic reuse.
Harmonic's chassis-contact sensor emits contact events rather than empty
heartbeats. After a chassis-contact stop, inspect the cause and restart in a new
session: silence does not demonstrate that contact has cleared.

## Mapping, localization and missions

RGB-D odometry owns `odom -> base_link`; RTAB-Map owns `map -> odom`. Ground-truth
odometry is bridged only for the independent evaluator, with no ground-truth
relay in the navigation graph. The legacy EKF is not launched: its inputs and
frames must be validated before future wheel/IMU fusion.

The terrain adapter classifies each depth frame on its own (10 m window,
2.5 cm cells): height span and slope are never computed across frames, because
small pose inconsistencies between frames otherwise appear as 3–8 cm steps on
flat ground. Classifications are fused into a 20 m odom-frame memory. A hazard
is retained once two distinct frames detect it (`hazard_confirmations`) and is
never cleared by later free readings; a single-frame hazard still blocks that
update. `/navigation/local_ground` (6 m, 2.5 cm, odom) is cut from this memory.
`/navigation/terrain_map` (20 m, 5 cm, map) resamples it through the latest
`map -> odom` transform: a cell is hazardous if any sub-cell is, and known only
if all are. A loop closure moves the memory as a whole; drift within it is not
redistributed. `/navigation/terrain_layers` still publishes GridMap elevation,
slope, height-span, roughness and observation-count layers from RTAB-Map's merged
cloud for inspection only; they are not used for navigation. Missing
measurements remain unknown. The costmaps use updated terrain StaticLayers plus inflation; obstacles
and terrain hazards are encoded in those grids. Initial simulation limits are
12 degrees slope and 3 cm vertical span; these are not validated hardware limits.

Depth samples wait in a bounded 20-frame queue for camera and base transforms
at their original image timestamp. Each update selects the newest transformable
sample within the existing 0.6-second simulation and one-second receipt deadlines;
it does not wait inside the executor for TF delivery. Samples keep their original
timestamps, and processed/expired samples cannot refresh the guard. RGB/depth
pairing in odometry/SLAM and depth/CameraInfo pairing in terrain are limited to
15 ms to reject adjacent 30 Hz frames when transport drops or bursts messages.

If `ready` becomes false with increasing `depth_age_seconds`, compare raw depth
and terrain-grid arrival rates (run each command separately, stop with Ctrl+C):

```bash
ros2 topic hz /oak/depth/image_raw --wall-time
ros2 topic hz /navigation/local_ground --wall-time
ros2 topic echo /navigation/status --full-length
```

Martian navigation sessions collide with the scene's visual terrain mesh
(`terrain_collision:=visual`, the default; recorded in `session.json`). Depth
matched the visual height grid within 1.4 mm RMS on scene 50000, while the
scene's coarser collision mesh differed by 14 mm RMS and was smoother, so the
wheels would not meet slopes that navigation avoids. `terrain_collision:=scene`
keeps the generator's collision mesh for comparison. The source bundle is not
modified; only the session world changes.

If visual odometry is lost before any mode has ever been selected (its first
registration can fail while Gazebo is still loading textures, and it never
resets itself), the guard requests `/rgbd_odometry/reset_odom` after 2 seconds,
at most every 5 seconds. The rover cannot have moved, so the restarted odometry
is consistent. `odometry_resets` in guard status counts these. Once motion has
been enabled, lost odometry remains a latched fault with no automatic reset.

Navigation sessions load `controllers_navigation.yaml` (saved in the session)
with `wheel_separation_multiplier: 1.0`. With the shared 1.5 value, Harmonic
turned this rover at exactly 1.5 times the commanded rate, because in-place
turns pivot on the front axle without wheel slip; the guard's swept-footprint
check assumes the commanded rate. Other Harmonic workflows keep the shared
value. `scripts/halo_navigation_turn_check.py` measures the ratio in a flat
session (at most 0.3 rad/s for 3 seconds per direction, through the guard);
`--arcs` adds four 0.05 m/s arcs. It also fits the body point that does not
slide sideways: 0.130 m ahead of `base_link` in every measured motion. That
point is the `drive_center` frame (`DRIVE_CENTER_X` in `core.py`, recorded in
`session.json`), which navigation plans, controls and checks. Hardware needs
its own measured value.

The launcher gives the container a Fast DDS profile
(`src/rover_navigation/config/fastdds.xml`) with 16 MB shared-memory segments:
960x540 RGB and depth frames exceed the default 512 KB segment. Set
`NAV_FASTDDS_PROFILE=default` to compare with the built-in transports; each
session records the profile in `session.json`. To measure delivered camera and
odometry rates without commanding motion:

```bash
python3 /work/scripts/halo_navigation_sensor_timing.py \
  --output /work/navigation_sessions/SESSION/checks/sensor-timing-001.json --duration 120
```

The status field named `depth_age_seconds` measures time since the guard last
accepted a local ground grid, not raw camera-image age. Check simulator logs for
TF extrapolation or RGB/depth synchronization warnings before selecting motion.
Keep the one-second sensor timeout in place while diagnosing missing grids.

Nav2 uses differential-drive State Lattice primitives, full footprint collision
checking and Regulated Pure Pursuit. Its behavior tree keeps the current path
until a lethal cell blocks it or the goal changes, then replans, without
unobserved backup/spin recoveries. A failed replan cancels the mission. The independent guard checks the swept
chassis/wheel envelope against known ground before sending each moving command.

For a waypoint trial, start a NEW session with a route in map coordinates:

```bash
bash scripts/halo_navigation.sh /absolute/live_eval_scenes scene_050000 waypoint-01 \
  'waypoints:=[[1.5,0.0,0.0],[1.5,1.5,1.5708]]'
```

After manual map inspection and clear-start confirmation:

```bash
ros2 service call /navigation/enable_autonomy std_srvs/srv/Trigger '{}'
ros2 service call /navigation/start_waypoints std_srvs/srv/Trigger '{}'
# Or, after waypoint acceptance:
ros2 service call /navigation/start_exploration std_srvs/srv/Trigger '{}'
# Request an early return:
ros2 service call /navigation/return_home std_srvs/srv/Trigger '{}'
```

Home is captured on the first mission start, not at every takeover. Goals must
be on observed traversable ground. Exploration chooses reachable observation
positions set back from unknown cells, excludes attempted frontiers, and returns
when no candidates remain or the default 600-second simulation budget reaches
estimated return time plus 50% and a ten-second settling allowance. An unavailable
return route stops the mission. Status topics expose control state, fault reason,
mission state and home. Full coverage is not promised by frontier exhaustion.

To localize against a saved map, start a new session with
`database:=/work/navigation_sessions/first-mapping/rtabmap.db`. The source database
is copied into the new session, and RTAB-Map incremental mapping is disabled.

## Acceptance and evidence

Run the recorder in a separate terminal/container process:

```bash
ros2 run rover_navigation evaluate --ros-args \
  -p output:=/work/navigation_sessions/waypoint-01/evaluation.jsonl
```

It records independent truth/estimated trajectories, home error, chassis contacts
and control/mission events. A navigation-only chassis contact sensor also latches
the motion guard. Output paths must be new. Start before motion. Confirm the
native contact publisher and bridge subscriber use the same topic, exercise a
controlled contact probe, and retain RViz/Gazebo observations for wheel/obstacle
interactions; the chassis sensor does not measure every possible collision.
Also capture `docker stats`, simulation real-time
factor, ROS topic rates, and sensor age with detector inference disabled/enabled.

After stopping the recorder, summarize the run with:

```bash
ros2 run rover_navigation summarize /work/navigation_sessions/waypoint-01/evaluation.jsonl
```

The summary interpolates the truth position at mission start (after any manual
mapping), reports final home error and aligned odometry error, and explicitly
reports contact observations only when the contact stream was actually recorded.
An absent contact stream does not establish zero collisions. Start the
recorder before requesting the mission; missing start evidence cannot pass.

### Bounded flat-fixture checks

Inside the sourced navigation container, the manual smoke script defaults to
read-only preflight. Output paths must be new. It checks the flat fixture,
command ownership, event-only contact transport, and fresh pose/truth/ground.

```bash
python3 /work/scripts/halo_navigation_smoke.py \
  --session /work/navigation_sessions/agent-manual-01 \
  --output /work/navigation_sessions/agent-manual-01/checks/preflight-002.json
```

Adding `--drive` allows at most one wall second at 0.05 m/s through the manual
guard input; `--stop-method lease` checks the existing command timeout. Use a
different output name for each run. `--pause-check` instead tests stationary
pause/resume and restores Gazebo to running in cleanup. No arm or motor hardware
is controlled by these fixture-restricted checks. Raw event streams accompany
the summaries; physical contact and autonomous mission acceptance remain separate.

The separate planner inspector never sends a NavigateToPose goal or publishes
motion. It saves a terrain preview and checks a 35 cm proposed path:

```bash
python3 /work/scripts/halo_navigation_map_check.py \
  --output /work/navigation_sessions/agent-manual-01/checks/map-plan-002.json
```

For an optional read-only RGB preview, append `preview:=true` to the simulator
launcher. Open `http://127.0.0.1:8773/frame.png` on the Halo (refresh for a new
frame), or tunnel port 8773 over SSH. `/status` reports frame age. This reuses the
existing camera bridge with controls disabled and does not start inference or
another command publisher. The raw RTAB-Map occupancy grid is also available on
`/map` for RViz; the terrain-enhanced navigation map is `/navigation/terrain_map`.

Offline source checks can run without ROS:

```bash
PYTHONPATH=src/rover_navigation:src/mars_training_environment:src/rover_training:src/rover_perception \
  python -m pytest src/rover_navigation/test -q
bash -n scripts/halo_navigation.sh
```

These tests require NumPy, PyYAML, xacro and pytest. The ROS adapter tests use
controlled message doubles; the Docker build and actual Jazzy/Gazebo execution
remain separate acceptance gates.

Acceptance remains pending until all of the following are measured on the Halo:

1. Fresh aligned RGB/depth, continuous odometry, a single TF/command owner,
   visible near-ground coverage and a manual stop that works while Gazebo pauses.
2. Manual mapping loop, saved/reloaded database and relocalization.
3. Each existing evaluation scene (50000-50002), three consecutive waypoint and
   return runs: no collisions, no unsupported-ground travel, truth-measured final
   error <=0.15 m and <=10 degrees.
4. Separate fixtures for slopes, steps, drop-offs, narrow gaps, unknown regions
   and blocked return routes; faults for missing depth, invalid odometry,
   localization loss, clock pause/reset, stale commands and manual takeover.
5. Frontier exploration/return and full-stack timing/resource measurements.

The scene generator uses finer visual terrain than physical collision terrain;
compare both when interpreting traversability and contact behavior. Do not feed
either mesh or object locations to navigation. NPU replay/accuracy acceptance is
separate and does not establish any of these navigation outcomes.

Dedicated deterministic worlds are available with `fixture:=flat`, `slope`,
`step`, `dropoff`, `narrow_gap` or `blocked_return` appended to the launcher.
They replace only the session world. The 35 cm narrow gap is smaller than the
padded rover width; the drop-off has a real 1 m gap and lower pit floor.
`blocked_return` starts its `return_gate` away from the route. After crossing
x=1.5, move that model to x=0.8, y=0, z=0.35 using Gazebo's `/world/mars_sampling/set_pose`
service and verify that the mission stops if no route home remains. Rendered
fixture behavior, tracking quality and collision instrumentation still need
verification on the actual Gazebo runtime.

Hardware trials require measured camera/footprint calibration, visibility and
stopping-distance checks, wheel/IMU validation and a hardware clock/control
adapter. This simulation launcher is intentionally not a physical-rover launcher.
