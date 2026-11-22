# Autonomous exploration and simulated sampling

For the current Halo demo, use the
[V50 survey commands](AUTONOMOUS_PLANNER_REHEARSAL_20261117.md#v50-thermal-cutoff-and-run-commands).

V50 is staged for attended survey testing. Prior rehearsals verified home
arrival, and installed ROS replay verified simulated sample delivery. The
next live milestone is a full survey, sample trip and verified second home
stop. [Survey results](AUTONOMOUS_PLANNER_REHEARSAL_20261117.md#v49-long-survey-and-sampling-demo).

**November 10 incident:** the first v11 autonomous rehearsal froze during its
first return. Its shutdown was unclean and the initiating cause is unresolved.
See the [freeze investigation](AUTONOMOUS_DEMO_FREEZE_20261110.md) before using
the historical launch instructions below. Later bounded movement results
appear in the [planner rehearsal report](AUTONOMOUS_PLANNER_REHEARSAL_20261117.md).

**November 10 thermal isolation:** a separate
[browser profile with the Gazebo window disabled](AUTONOMOUS_DEMO_BROWSER_PROFILE.md)
uses application-v13 / runtime-v15 and a bounded inspection viewer. Its stationary
check peaked at 83.125°C with worst reported publication p95 165.65 ms. The
below-80°C target was not met; no matched temperature reduction or autonomous
acceptance is established. Use its checked inspection procedure rather than
opening a continuous exterior window during platform isolation.

## Behavior

Start with a fresh scene and empty localization/maps. Confirm only the inspected
launch-clearance area. Start exploration explicitly in the browser. The rover
captures its estimated `drive_center` home in the odometry epoch, observes the
launch area, selects reachable frontiers on known ground, then reserves time for
its first return. No simulator geometry or truth poses feed mission decisions.

At home the guard stops and disarms. Select a confirmed pale-soil site or click
measured safe ground. A map click means **operator-selected**, without a material
claim. Preview the outward/return routes and stopping pose; approve within 30 s.
Approval starts travel to the checked stopping pose. The rover records ten
seconds of stationary simulated collection, then waits for **Return home**.
Delivery is recorded at the verified second home stop.

Stop, takeover, contact, stale localization/sensors, browser heartbeat loss or a
navigation failure cancels the mission. Restart establishes a new mission and
clears approvals. A stopped mission cannot silently resume.

### Object-aware rehearsal on Halo

The staged `runtime-v28` adds confirmed rocks and crystals to the local and
global navigation grids. The obstacle margin includes estimated size, position
uncertainty and 10 cm of extra clearance. Nav2 plans around these cells, and the
motion guard stops if the rover's predicted footprint enters them. Selecting an
object for sampling plans to a nearby safe stopping pose. Collection is simulated
and still requires the round-trip preview and explicit approval.

Supporting views can be 18 cm apart in the demo, down from 20 cm. Confirmation
still requires three views plus the existing class and depth checks. The map
legend lists the class colors. Hover or focus an object marker to see its class,
ID and supporting view count. Enable **Tentative objects** to see hollow markers
while evidence is still accumulating.

Start each of these commands in a separate Halo desktop terminal. Wait for
repeating capture health records before starting temperatures and NPU. Wait for
the NPU to report ready before starting terrain.

```bash
# Terminal 1
~/bin/halo-rover capture
# Terminal 2
~/bin/halo-rover temps
# Terminal 3
~/bin/halo-rover npu
```

Start terrain after the native prelaunch temperature check passes (at or below
70°C for 30 seconds).

```bash
# Terminal 4
halo_context=$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context) &&
eval "$halo_context" &&
python3 -I "$SIM_KIT/trial.py" "$SIM_LABEL" cooldown --cooldown-phase terrain &&
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/gpu-repair-20261117/terrain-sdma-off-v2/start_terrain.sh
```

Wait for `ENVIRONMENT_V2_TERRAIN_READY`, then launch the sparse rehearsal at
0.05 m/s. Open the [Halo dashboard](http://127.0.0.1:8774) and wait for the
live camera before running the ten-second exterior inspection.

```bash
# Terminal 5
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v28/start_autonomous_demo.sh 1 rehearsal 0.05
# Terminal 6, after the live camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v28/view_autonomous_demo.sh 10
```

Once the viewer closes, confirm the clear launch area. Start exploration as soon
as the dashboard enables **Start autonomous exploration**. The v28 rehearsal
uses live navigation readiness in place of the five-minute stationary mapping
hold. Keep the dashboard visible for the operator heartbeat.

Watch for a confirmed object and a route that passes it with clearance. A run
without a relevant object encounter verifies exploration and return only. Click
**Return now** after a frontier visit or after roughly 90 seconds for a short
retest. After verified home arrival, select a confirmed object and inspect the
stopping pose in **Preview sample round trip**. If the preview shows safe
clearance, **Approve sample trip & automatic return** tests the simulated
collection and second home return.

Press **Stop**, then Ctrl+C the simulation and terrain terminals. Run the
following in a spare terminal before closing capture or temperatures.

```bash
~/bin/halo-rover stop && ~/bin/halo-rover finish
```

After protected finish and its cooldown complete, Ctrl+C the capture and
temperature terminals.

The Halo finish command displays `MANUAL_SESSION_CLOSED_CLEANLY` for a clean
short rehearsal. The protected endurance receipt remains in the case directory.
See the dashboard or demo event log for the mission result.

The staged release passed the navigation tests, ROS service smoke probe and all
three scene launch checks. Live avoidance and a complete object sampling trip
are awaiting attended verification.

### V50 survey workflow

The survey aims for 25 minutes of measured-ground mapping, with 32 minutes
available through the first home arrival. The planner evaluates reachable
frontiers and returns earlier when the safe return reserve is needed or no
checked frontier remains. The dashboard reports measured coverage and the
reason for ending exploration.

At the first home stop, select a confirmed sample site and review the
outbound and return routes. Approve the drive, wait for the simulated
collection result, and choose **Return home**. The checked departure
includes a 0.50 m reverse before the rover turns.

## Retained profile and limits

| Setting | Candidate behavior |
|---|---|
| Environment | Frozen 40 × 40 m sparse scene; standard/cluttered variations |
| Camera | V50 Ogre RGB-D at 960 × 540 and 8 Hz, with 0.40 rad downward tilt and 8 m usable depth |
| Detector | Frozen epoch-22 A16W8 NPU candidate; baseline retained separately |
| Terrain | Float64, HIP flags 4, unchanged hazard/clearing rules |
| Movement | V50 on Halo defaults to 0.15 m/s. The turn cap is 0.30 rad/s. |
| Full profile | 30 min overall, exploration and first return up to 20 min, selection 2 min, sampling round trip 8 min |
| Rehearsal | 12 min overall, exploration and first return up to 6 min, selection 1 min, sampling round trip 4 min |
| Survey | 42 min overall, mapping up to 25 min, first home stop by 32 min, selection 2 min, sampling round trip 8 min plus unused first-return time |
| Starter | 45 min including setup and shutdown |
| Browser lease | Operator page sends 200 ms heartbeats; >1 s without renewal requests Stop |
| Temperature | V50 uses a 92°C Tctl cutoff for demo workloads and native capture. |

Coverage is depth-supported area inside the operating map. Launch-only clearance,
unknown space and hazards are reported separately. It is not complete coverage
merely because frontiers run out. Evidence scores and detector confidence are not
calibrated accuracy.

## Installation and initial platform session

The build helper creates a new pinned application, reuses the qualified image and
CPU/thread launcher settings, and runs source suites without GPU devices. Failed
snapshots/checks remain in separate directories. A checked runtime release is
created only after the build/import checks pass.

On Halo, start fresh native capture and temperature terminals. Wait for two health
events and keep the temperature display visible. Start the native NPU reference,
then the corrected `capture5000-training-20261108/start_capture5000_terrain.sh`
starter. Do not start training, image collection or another simulator.

The dedicated starter, once installed in the candidate runtime directory, accepts:

```bash
bash start_autonomous_demo.sh 1 rehearsal 0.05
# Separate, stationary safety fixture; never an accepted demonstration:
bash start_autonomous_demo.sh 1 rehearsal 0.05 contact
# Later, after movement qualification:
bash start_autonomous_demo.sh 1 full 0.10
```

Representatives 1/2/3 are sparse/standard/cluttered. Run from the connected Halo
graphical desktop. Wait for model and renderer provenance readiness. The first
attended session is stationary platform qualification; do not select Start
exploration until those gates have been reviewed. With the current GUI-off
profile, inspect clearance using its bounded stationary viewer, close that view,
then confirm clearance in the browser. Do not pre-map by driving manually.

At closeout, Stop in the browser and Ctrl+C the supervisor. Use the retained
protected finish procedure with monitoring active throughout cooldown and final
journal capture. Preserve any gap explicitly. Separately pinned services may need
their own terminal Ctrl+C before native finish; do not weaken an identity guard.

## Windows recording

The companion Windows deployment uses OBS. Configure a scene that captures the actual browser
window, verify the preview and make a short recording/playback test. Enable its
WebSocket server on loopback. If authenticated, set `OBS_WEBSOCKET_PASSWORD` in the
recording terminal; it is not written to the evidence files.

Keep the existing dashboard SSH tunnel, or use:

```powershell
ssh -N -L 8874:127.0.0.1:8774 halo
```

In a second Windows terminal, before exploration:

```powershell
node scripts/record_autonomous_demo.mjs --output navigation_sessions/demo-video-check --preflight
node scripts/record_autonomous_demo.mjs --output navigation_sessions/demo-video-run --minutes 35
```

Use a new output directory each time. Recording observes GET `/status`; it never
sends controls or renews the operator lease. Keep the controlling browser visible
and connected. Continue recording/stopped mapping to 30 healthy minutes if the
mission finishes early. Ctrl+C finalizes the OBS original and records its path.

For short exterior clips, use the actual running Gazebo session through the Halo
desktop and switch the Windows OBS scene briefly. The current inspection viewer
is stationary-only; moving exterior footage requires a separately qualified
procedure. Type a marker in the recording terminal before and after each clip.
Qualify that temporary GUI load first. Do
not add a second simulator camera or replace a failed accepted run with clips
from another attempt.

The original must be preserved. The future 3–5 minute edit must caption time
compression/cuts and simulated collection. No edited video exists before an
accepted continuous mission has been recorded.

## Exports

Preserve the authoritative `demo-events.jsonl`/state JSON and observer JSONL from
the same mission. On a Python environment with Matplotlib and NumPy:

```bash
python3 export_autonomous_demo.py --events PATH/demo-events.jsonl \
  --observer PATH/observer.jsonl --final PATH/final-status.json \
  --map PATH/map.png --output PATH/new-export
```

This creates route CSV (explicit odometry frame), mission/event JSON, sample
record, coverage/latency/temperature/RTF PNG/SVG/PDF plots, an annotated map when
provided, a concise JSON report and hashes. A complete state-machine result does
not substitute for independent acceptance. Missing gates stay pending.

## Acceptance matrix

| Gate | Required evidence | Current status |
|---|---|---|
| Source behavior and compatibility | Mission, coverage, soil, Stop races; existing navigation suites | v11: 674 tests ran across four suites; 4 GPU-only checks skipped. Five browser/JavaScript checks and three export/video/fixture-policy tests passed |
| Installed ROS service and launch | Generated typed service import; all three actual launch graphs | Six-package build, actual ROS/HTTP smoke probe and three representative launch graphs passed |
| Platform | p95 publication ≤200 ms; median RTF ≥0.8 and ≥80% matched 20 m baseline; freshness with recording | v9 stopped check passed: worst steady rolling p95 173.5 ms, RTF about 1.0, no freshness failures. Matched baseline and recording remain pending |
| Safety | Isolated positive contact wiring; unknown/hazard/stale/release stops; CPU/ROCm parity | Positive stationary chassis-contact/rearm-rejection fixture passed. All 14 outer-map CPU/ROCm cases have exact publication/clearing parity; remaining autonomous fault rehearsals pending |
| Geometry/localization | Phase 3 synchronized review ≤15 mm; drift ≤0.10 m / 5° | Launch-area ground p95 4.17 mm passed; travel, outer-area and home review pending |
| Movement | Progressive autonomous 0.05 then 0.10 m/s | Pending |
| Diagnostics | 100 manually reviewed images, five unseen groups, both frozen detectors | Reservation tool prepared; no images collected |
| Primary cycles | Three fresh successful cycles, both target sources; ≥3 frontiers, ≥10 m exploration, ≥20 m² added ground | Pending |
| Variations | One shortened complete cycle per standard/cluttered scene | Pending |
| Home returns | Both ≤0.15 m / 10°, verified final zero commands, no contact/fault/manual recovery | Pending |
| Recording | Full-budget primary run; 30 min continuous healthy operation; raw and captioned edit | Windows helper prepared; live recording pending |
| Archive | Exact source/model/config identities and verified off-host archive | Source snapshots preserved; final archive pending |

Plan three or four attended sessions: platform/contact/timing and 0.05 checks;
exploration/return and 0.10 qualification; sampling/variation cycles; then the
fresh primary acceptance recordings. Extend the schedule if these gates fail.

## First stationary platform check

The v8 preview remained stopped and failed sensor freshness. A 25-second raw
probe received RGB at 1.24 Hz, depth at 2.05 Hz and odometry at 0.25 Hz while
camera-info remained at 15.15 Hz. Real-time factor was about 1.0 and sampled Tctl
was about 59–61.5°C. This was not a successful platform or autonomous check.

The separately packaged launcher's DDS profile path referenced `/work/src`,
which this application does not contain. The qualified profile exists under
`/work/source/src`; its 16 MiB shared-memory transport is needed for the large
image messages. v9 corrects that path while retaining the exact profile contents,
CPU/thread settings and rendering settings. Its build now requires a separate
multi-process 960 × 540 RGB/depth throughput fixture. The failed v8 preview and
logs are preserved; live qualification must be repeated after this repair.

The v9 fixture passed with 90/90 frames on both streams at about 15.006 Hz and a
maximum gap below 69 ms. v9 also passed the six-package build, real ROS/HTTP probe
and all 669 source tests (four GPU-only skips). A later power/reboot interruption
left the native monitoring session without protected teardown; its cause is
unknown. Protected review subsequently passed for all three boot journals and a
60-second current-boot cooldown. The missing teardown and historical monitoring
gap remain recorded; the review permits a fresh session without declaring the
old session clean.

The fresh v9 preview passed a five-minute stationary check with 298 snapshots,
including 268 after a 30-second settling interval: no freshness failures, worst
steady rolling publication p95 173.5 ms and peak steady Tctl 82.9°C. The initial
snapshot reached 85°C; the native and 87°C qualification protections stayed intact.
A separate 25-second sensor probe measured about 15.05 Hz RGB/depth/odometry,
with maximum arrival gaps below 79 ms and RTF 0.993. These are stopped measurements
without recording, autonomous travel or a matched 20 m baseline.

Six exact-stamp RGB-D/map captures passed independent publication-hash and ground
review. Worst world-ground absolute-error p95 was 4.17 mm; stationary drift over
the 25-second capture span was 0.16 mm / 0.0042°. These values do not establish
travel drift or outer-area acceptance. The evaluator correctly leaves its overall
result `REVIEW_REQUIRED` because no outer-area frames were captured. The preview
then closed with no supervisor failure or cleanup errors.

The follow-up candidate connects display-only native Tctl telemetry to the
browser and exports. Readings expire after two seconds and must match the boot;
missing data stays unavailable. Motion authorization still comes exclusively from
the existing guard. Home-transform preflight precedes arming, and any subsequent
initialization failure explicitly stops/disarms. A separate ten-minute flat
contact-fixture mode disables the autonomous mission and requires a fresh launch
after the isolated physical-contact check. Its checks are not mission evidence.

The v10 contact fixture passed: the guard published zero within 1.057 ms of the
bridged chassis-contact receipt and rejected rearming while contact remained
active. Native Gazebo evidence identifies the physical probe and chassis. Wheel
coverage is not established by this fixture. It closed cleanly, and a fresh scene
is used for the subsequent autonomous rehearsal.

The 40 m replay repeated fourteen declared outer-coordinate/boundary cases on
CPU and ROCm, including weak-artifact clearing, protected hazards and sensor or
localization loss. Published maps and clearing evidence matched exactly. The
v10 mapping/guard source hashes equal the tested v9 components; the separate
component-identity record binds this reuse to both applications and input hashes.

The first v10 mission Start was safely rejected before arming. The mission's
odometry subscription requested reliable delivery while the qualified publisher
offers best-effort sensor data; ROS logged the incompatible reliability policy.
The guard's existing sensor subscription was unaffected. The v11 correction
matches that sensor QoS, retains covariance and freshness validation, publishes
individual mission-health checks and records rejected commands without exposing
operator tokens. Start stays disabled until those inputs are healthy. The real
ROS probe now requires reception from a best-effort odometry publisher, and a
combined-page browser test exercises both shipped scripts together. A separate
browser compatibility repair uses cryptographic random bytes when secure-context
`randomUUID` is unavailable. No autonomous travel occurred in the rejected v10
attempt; its logs and measurements remain preserved.
