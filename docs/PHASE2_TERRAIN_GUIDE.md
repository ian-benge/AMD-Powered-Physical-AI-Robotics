# Phase 2 terrain operator guide

Phase 2 adds `mixed_reachable` version 1. Fifty seeded layouts passed CPU
geometry/connectivity/replay checks, and all three attended manual traversal
checks passed; see the [acceptance report](PHASE2_TERRAIN_ACCEPTANCE_20261104.md)
for the final monitoring limitation. No dataset capture, training or autonomous
sample-site approach is part of these commands.

## Candidate and preserved baseline

The Phase 2 source/install is separate:

```bash
PHASE2=/home/ian/capstone-training/rover-navigation/navigation_sessions/terrain-phase2-20261105T031610Z
APP="$PHASE2/application-v5"
```

It reuses the accepted Phase 1 renderer image
`sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa`.
The baseline `~/bin/halo-rover sim` and Phase 1 application/pins remain available.
Use the Phase 2 supervisor below for this terrain. Its configuration pins the
new source/install and scene assets; do not change expected hashes to bypass a
failure. Existing scene manifests are preserved, including the old terrain.

## Fresh capture and retained services

First inspect the current selection:

```bash
~/bin/halo-rover status
```

If the previous case is closed, start these in **separate Halo terminals**.
Do not paste them as a single sequential script:

```bash
~/bin/halo-rover capture
```

Wait for two health events and leave capture open through protected finish.

```bash
~/bin/halo-rover temps
```

Keep temperatures visible throughout. Then:

```bash
~/bin/halo-rover npu
```

Wait for detector readiness before starting:

```bash
~/bin/halo-rover terrain
```

Require the retained isolated ROCm 7.14.1/Torch 2.12 environment, float64 and HIP
blocking-wait requested/observed flags **4**. Keep Quiet/Automatic fans and Linux
power-saver. Stop workloads at **Tctl >=90 C**, a capture/monitor/fan failure or
any GPU incident. The temperature display itself only warns.

## Launch a checked representative

Inside a terminal in the **Halo graphical desktop**, run:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/terrain-phase2-20261105T031610Z/start-phase2-qualification.sh 1
```

That prepared helper runs parity once per fresh capture. On later launches in
the same case, it checks the existing passing record against the same boot,
terrain PID/start time and saved input hashes. A changed terrain process cannot
reuse it. The protected parity record is never overwritten. The expanded commands
below show a first launch in a fresh case; use the checked helper for repeats:

```bash
PHASE2=/home/ian/capstone-training/rover-navigation/navigation_sessions/terrain-phase2-20261105T031610Z
APP="$PHASE2/application-v5"
eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"
CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")
bash "$SIM_KIT/trial.sh" "$SIM_LABEL" parity
python3 "$APP/management/phase2_desktop_launch.py" \
  "$APP/management/phase2_supervise_live.py" "$APP/CANDIDATE_CONFIGURATION.json" \
  --kit "$SIM_KIT" --case "$CASE" --representative 1 --seconds 600 --attended
```

Wait for `RENDERER_PROVENANCE_READY`. The supervisor continuously checks capture,
labelled Tctl and fan health, repeats renderer provenance for each replacement
container and stops on a guard failure. It never confirms clearance or commands
motion. The ten-minute bound is a development limit, not an endurance trial.
The desktop parent forwards Ctrl+C to the supervisor without interrupting its
current health-query subprocess; cleanup still stops the candidate container.

Open **http://127.0.0.1:8774/** on Halo for the persistent scene-control page.
The underlying dashboard remains on 8777 and also forwards scene actions.
For Windows, keep a separate PowerShell tunnel open:

```powershell
ssh -N -T -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -L 127.0.0.1:8874:127.0.0.1:8774 halo
```

Open **http://127.0.0.1:8874/** on Windows. Once the scene is fully ready, inspect
the launch area and click **Confirm clear launch area**. Set and verify
**0.05 m/s** and turn speed **at most 0.30 rad/s**, then click **Enable manual
drive**. This qualification profile caps manual commands at **0.05 m/s** and
**0.30 rad/s** on the server and advertises those bounds to the page, including
after scene changes. Verify the displayed speed before driving. Normal launch
defaults remain unchanged. Release controls to command zero; **Stop**
disarms manual driving. Retain every sensor, localization and obstacle guard.
The baseline `drive-ready` command checks the baseline's container identity;
use this candidate's provenance-ready signal and browser clearance confirmation.

| Representative | Scene seed | Evaluator route map |
| --- | --- | --- |
| 1 | 2601136 | `qualified-layouts/scene_2601136-routes.png` |
| 2 | 2601113 | `qualified-layouts/scene_2601113-routes.png` |
| 3 | 2601104 | `qualified-layouts/scene_2601104-routes.png` |

The map files are in `$APP`. They show terrain, obstacles, three reachable
staging areas and evaluator routes. They are never supplied to the rover's
mapper/controller. Use the live camera/map for guarded driving. Each layout
must demonstrate gentle-slope travel, turns and an obstacle passage at 0.05 m/s,
then Stop. Record the session, actual route, rejected commands, camera defects
and peak temperature. A guard rejection is evidence to review, not permission
to override a limit. The first layout's crystal route is a long detour; its soil
and ordinary-rock routes are shorter options for the initial traversal.

For representative 2 or 3: Stop, Ctrl+C the supervisor and wait for its result,
then rerun the checked helper with argument `2` or `3`. Keep the same
healthy capture/temperatures/services running. Every launch starts a fresh map
and requires clearance confirmation again. Use a new capture if its two-hour
bound would expire before protected finish.

## Restart, new scene and shutdown

**Restart scene** reuses the current scene and starts a fresh map.
**Generate new scene** uses `mixed_reachable` v1, checks all objects and three
staging routes, retries deterministically up to 24 layouts and saves the retry
reasons. It writes a new bundle and preserves old scenes/assets. Both operations
stop the rover and require a new launch-clearance confirmation. Wait for the
replacement container's provenance-ready signal before driving. Generated
layouts do not replace the three recorded representative acceptance checks.

At the end: **Stop**, Ctrl+C the candidate supervisor and inspect its result,
then:

```bash
~/bin/halo-rover stop
~/bin/halo-rover finish
```

Keep capture and temperatures running through protected cooldown and final
journal capture. Review `MANUAL_SESSION_FINAL.json` and candidate evidence;
only then Ctrl+C the capture/temperature terminals. A GPU incident requires
evidence preservation and review before another workload on that boot.

Rollback uses a fresh capture and the original baseline launch sequence in
[the Phase 1 guide](PHASE1_RENDERING_GUIDE.md). No original image tag, source
pin, driver/runtime, power policy or fan setting was replaced.

## Repeat CPU qualification

With the environment and training packages on `PYTHONPATH`, use a fresh output:

```bash
python3 scripts/phase2_validate_layouts.py \
  --template /path/to/retained/scene.json --assets /path/to/bundle/assets \
  --output /new/phase2-qualification --seed-start 2601100 --count 50 --materialize 3
```

This generates no camera dataset and sends no motion. It checks raw saved
triangles, shared surfaces, the full clear launch disk, actual visual/collision
object bounds, continuous one-metre corridors, turning clearance and staging
range. Every seed is generated twice from scratch. Three full scene bundles
must match their geometry hashes. On Halo, run CPU qualification under the same
capture and thermal supervision as the candidate build.
