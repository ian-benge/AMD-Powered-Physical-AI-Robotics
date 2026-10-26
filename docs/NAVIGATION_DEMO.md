# Martian waypoint and return demo

The demo uses RGB-D mapping, terrain-aware Nav2 planning and the independent
motion guard. Pick a destination in RViz, inspect both preview paths, then press
**Start and return**. The mission returns to its original home and stops.
Arms and sample collection are outside this demo.

## Launch on the Halo

Build the navigation packages as described in [NAVIGATION.md](NAVIGATION.md).
From the Halo checkout, use a new session name every time:

```bash
bash scripts/halo_navigation.sh \
  /home/ian/capstone-training/mars-pilot50-v2/dataset/live_eval_scenes \
  scene_050000 martian-demo-001 \
  gui:=true rviz:=true preview:=true mission_controls:=true \
  navigation_speed:=0.10
```

The launcher can read `DISPLAY` and `XAUTHORITY` from the user's desktop
environment when invoked over SSH. A full Ubuntu remote desktop must have an
active display. A camera page alone is not a desktop connection. A 0-by-0
desktop is rejected because OGRE cannot initialize its video modes there.
RViz defaults to software rendering (`rviz_software_rendering:=1`); Gazebo's
renderer remains controlled by `NAV_SOFTWARE_RENDERING`.

For headless acceptance, set `gui:=false rviz:=false`; the control page, camera,
goal topic, planner and recording still work. Add `fixture:=flat` for the initial
flat tests. Omit `fixture` to load the actual Martian scene, terrain and rocks.

## Map, select, preview, start

1. Inspect the launch pad in Gazebo before calling `/navigation/confirm_clear_start`.
   It confirms only the existing 1.1 m launch disk; it does not mark the wider
   scene clear. Use the existing guarded manual input at 0.05 m/s to map a short
   corridor. Stop and stage back at the launch pad.
2. In RViz, use **2D Goal Pose**, click the desired location, and drag its heading.
   Its configured topic is `/navigation/selected_goal`, not `/goal_pose`.
3. Review the cyan outward path and yellow return path. Green arrows identify
   home; orange identifies the destination. Unknown terrain remains blocked.
4. Open `http://127.0.0.1:8774`. When the destination is verified and the guard is
   stopped and ready, press **Start and return**. Start computes both routes
   again using current mapping and localization. Stop remains available during
   validation and motion.
5. The first Start establishes the session's home. Later trials return to that
   same home. Select destinations 1.8–3 m away for the Martian acceptance trials,
   beyond the seeded launch disk and entirely within mapped terrain.
6. In Harmonic this rover turns and drives as a differential drive about its
   front axle, 0.13 m ahead of `base_link` (measured for in-place turns and
   0.05 m/s arcs). Navigation therefore plans, controls and checks the
   `drive_center` frame at that point: Nav2's robot frame, the guard's swept
   footprint, mission home and previews all use it, and the footprint extends
   0.355 m behind it (padded). In-place turns at home no longer move it.
   After the return goal succeeds, the mission still re-approaches home up to
   twice if its estimated error exceeds 8 cm (`home_attempts` in mission
   status). Smac lattice paths end on the nearest of 16 uneven headings, up to
   about 13 degrees from home, so the state then becomes `aligning` when the
   heading error exceeds 5 degrees: a goal inside the rover's own planner cell
   rotates it in place to the exact home heading. The rotation sweep must be
   known clear, or the mission stops. Destinations need about 0.4 m of known
   clear ground behind the drive center for the return turn.

New fixed hazards update the local safety grid and global planning map.
Costmaps update at 5 Hz. Nav2 checks the current path at 2 Hz and replans only
when a lethal cell touches its remaining footprint or the goal changes;
replanning a valid path from a moving pose alternated between near-equal
turnarounds. Nav2's check ignores unknown cells, so the guard's swept-footprint
check and the mission's return-route check remain the gates for unknown ground.
The planner prefers turning in place to loops (`rotation_penalty: 1.0`, curve
shortcuts limited to 1 m). The controller may hold zero velocity for up to
3 seconds while a safe path becomes available.
A blocked route, missing return route or safety fault cancels the mission.
Explicit Start is required for another attempt; faults never automatically rearm.
Fixed-hazard observations are retained conservatively for the session. Removing
a test obstacle does not clear it automatically; restart the session afterward.

## Repeatable acceptance commands

Inside the running container, source ROS and the navigation installation:

```bash
source /opt/ros/jazzy/setup.bash
source /work/install_navigation/setup.bash
```

First run the physical probe in a **fresh flat session**. It drops a tiny physical
box onto the stationary chassis and records native and ROS contacts:

```bash
python3 scripts/halo_navigation_mission_check.py \
  --session navigation_sessions/CONTACT_SESSION \
  --output navigation_sessions/CONTACT_SESSION/checks/contact-001.json \
  --contact-probe
```

Restart with a new flat session after contact, then test the complete mission:

```bash
python3 scripts/halo_navigation_mission_check.py \
  --session navigation_sessions/FLAT_SESSION \
  --output navigation_sessions/FLAT_SESSION/checks/waypoint-001.json \
  --start --confirm-flat-start --goal-forward .5 --timeout 120
```

The flat fixture confirmation option is restricted to that fixture. The original
bounded manual smoke helper retains its original restrictions. For obstacle
trials, first manually map a corridor with clear detours, then use a fresh flat
session per obstruction trial with `--goal-forward 2 --obstruction detour` or
`--goal-forward 2 --obstruction blocked`. These options introduce a physical
obstruction after departure; no simulator geometry enters navigation maps.

For a Martian goal already selected in RViz, omit goal arguments:

```bash
python3 scripts/halo_navigation_mission_check.py \
  --session navigation_sessions/MARS_SESSION \
  --output navigation_sessions/MARS_SESSION/checks/waypoint-001.json \
  --start --min-outward 1.4 --timeout 300
```

Repeat with distinct output names for three trials. The evaluator starts with
`mission_controls:=true`, before any mission. Each trial requires destination
success, a nontrivial outward trip, return within 0.15 m and 10 degrees of the
first mission's truth home, no safety interruption and a final zero command.
Outward displacement is measured only until the destination is reached; travel
during the return does not count. Any guard `fault` raised during the trial
fails it. The runner waits up to 20 seconds for discovery of every stream and
service it checks before preflight, and records guard faults in its report.
The runner always requests Stop on exit. Interrupted runs remain evidence and
are not passes.

## Evidence and interfaces

Each session preserves the generated world, robot and Nav2 configuration,
`mission-events.jsonl` (truth, odometry, selected goals, preview/current paths,
guard and mission events), and exclusive-name JSON/JSONL check reports. Camera
captures are saved when port 8773 is available. Keep Gazebo/RViz captures with
the session to review wheel and terrain interactions.

- `/navigation/selected_goal`: `geometry_msgs/PoseStamped`, finite planar pose in `map`.
- `/navigation/start_selected_goal`: `std_srvs/Trigger`, validates then starts an out-and-return mission.
- `/navigation/preview_outward`, `/navigation/preview_return`: `nav_msgs/Path`, latched previews.
- `/navigation/mission_markers`: `visualization_msgs/MarkerArray`, home/destination.
- `/navigation/mission_status`: existing JSON plus selection result, trial number and outbound success.
- `/navigation/status`: existing JSON plus `stop_generation`, fencing pending Starts after Stop/takeover,
  `sequence`, and `fault`. `fault` keeps the first non-operator stop (reason, simulation time and
  sensor/footprint detail) until the next successful mode selection, so a later Stop cannot mask it.
  Status is also published immediately whenever the mode or reason changes.
- `/navigation/enable_autonomy`: successful response text contains JSON with mode, stop generation and
  `status_sequence`; the mission ignores guard statuses published before that arming.
- Loopback HTTP port 8774: `GET /`, `GET /status`, `POST /start`, `POST /stop`.

Only the motion guard publishes `/cmd_vel`. Simulator truth is confined to
evaluation and physical fixture placement. A chassis contact probe verifies
that instrumented surface; it cannot establish wheel collision coverage or
hardware readiness.
