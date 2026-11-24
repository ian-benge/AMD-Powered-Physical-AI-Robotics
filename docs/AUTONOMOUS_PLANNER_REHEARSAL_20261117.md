# Autonomous planner rehearsal — November 17, 2026

The [GPU repair investigation](HALO_GPU_REPAIR_20261117.md) records the later
V26 result: bounded 0.05 m/s exploration/home return passed after a separate
Nav2 progress-check correction, and protected finish found no new incidents.
A reviewed exact-configuration 0.05 m/s receipt is issued; 0.10 m/s, endurance
and full sampling acceptance remain separate tests. The original v22 failure
and historical gates below remain preserved.

The November 17 work adds frontier alternatives, route-decision evidence, a
map-correction timestamp repair and a separate recipe with fewer special targets.
The first live rehearsal stopped safely before its home return. The corrected
rare-site candidate passed its build and geometry checks; live qualification and
full autonomous acceptance are pending.
The November 11 AMD evaluation tag remains the published snapshot.

## Planner changes

- Rank observation poses using the mission's selected linear and angular speed.
  The previous heuristic assumed 0.10 m/s even during a 0.05 m/s rehearsal.
- Keep up to four observation poses per connected frontier, at least 0.65 m apart,
  with at most 16 returned goals. A rejected nearest pose can have another safe
  alternative on the same boundary. Deterministic ordering is retained.
- Preserve known-ground reachability, full turning clearance and independent
  outward/return path validation. These choices never mark unknown cells free.
- Record each search, selected route, estimated gain, route distances, travel
  reserves and explicit budget rejection in the mission event log. Estimated gain
  remains a heuristic; measured coverage remains depth-supported.

The previous rehearsal recorded one frontier visit before returning because no
remaining trip fit its six-minute exploration/return budget. The new choices do
not extend that budget or establish the cause of the subsequent host freeze.

The November 17 v18 rehearsal completed its observation sweep and one frontier
visit: about 3.23 m of estimated travel and 21.80 m² of additional measured ground.
It stopped when a map/odometry lookup requested simulation time 655.216 s while
the latest transform was at 655.214 s. Halo remained responsive and the guard
published zero. No successful home return or sample was recorded.

The first repair selected the newest map/odometry edge and applied a freshness
check. V21 then refused three exploration-start requests because that edge failed
the check; the rover remained stopped. A subsequent live read-only probe found
the newest SLAM edge 52 ms ahead of the clock, while the full rover-pose chain was
66 ms behind it. V22 selects the latest common **rover-pose chain** time, verifies
the existing −0.05 to 0.60 s freshness envelope, and queries the correction at that
valid time. It retains interpolation failure and stale-chain rejection. Home, targets
and frontier history remain anchored to the odometry epoch and reprojected into
the current map. New regressions cover the observed 2 ms lag, stale/future data
and both conversion directions. A real tf2 buffer fixture and the stopped live
read-only reprojection probe passed in v22; neither sends control commands.

## Rare sample-site recipe

`mixed_reachable_v2_demo_rare.json` selects `sample_site_recipe=rare_demo_v1`.
Ordinary-rock density presets, object sizes, burial, camera and terrain settings
are retained. Existing scenes and training profiles remain available.

| Export target | Standard recipe count | Rare demo count |
|---|---|---|
| Pale soil, including lookalike exposures | 12–20 | 3–6 |
| Yellow crystal rocks | 8–12 | 2–4 |
| Mudstone slabs, both appearance variants | 8 | 2 |
| Ordinary rocks | Sparse 76–110; standard 152–220; cluttered 228–330 | Unchanged |
| Pale-stone rock controls | 8–16 | Unchanged |

The three checked frozen scenes contain the following exported objects. The rock
column includes ordinary rocks and pale-stone controls. Bounded layout failures
are retained; the original scene seeds were not replaced.

| Density / seed | Rock | Pale soil | Yellow crystal | Mudstone | Special-target reduction | Successful layout attempt |
|---|---:|---:|---:|---:|---:|---:|
| Sparse / 4311000 | 109 | 6 | 3 | 2 | 68.6% | 2 |
| Standard / 4311001 | 205 | 4 | 2 | 2 | 80.0% | 4 |
| Cluttered / 4311002 | 253 | 6 | 3 | 2 | 69.4% | 11 |

The configured expectation falls from 34 to 9.5 special objects per populated
scene, a **72.1% reduction** after integer rounding. Actual counts vary by seed.
Near/outer assignment alternates across each exported target class so the two
remaining slabs can support both staging bands. Conservative corridors and
independent route audits remain required. Generate new scene retains the recipe;
Restart retains the frozen scene and establishes fresh localization/maps.

The requested autonomous speed is **0.10 m/s**, with turn speed at most 0.30 rad/s.
Its launcher still requires a reviewed successful 0.05 m/s qualification bound to
that exact application/configuration. The interrupted v18 run supplies no such
receipt. Camera/depth rates remain 15 Hz.

## Checks completed before rendering

| Check | Result |
|---|---|
| Windows navigation source suite | 261 tests; four GPU-only skips |
| Linux navigation source suite | 261 tests; four GPU-only skips |
| Windows environment source suite | 243 tests; five skips |
| Linux environment source suite | 243 tests; no skips |
| Rare-scene evidence helper | Two regressions passed on Windows and Linux |
| Installed navigation and environment build | Passed |
| Actual ROS/HTTP command and best-effort odometry probe | Passed |
| Multi-process 960 × 540 RGB/depth fixture | 90/90 frames each; approximately 15 Hz; largest gap 67.6 ms |
| Sparse, standard and cluttered rare-scene launch graphs | All three passed without starting graphics |
| Saved shared meshes, launch clearance and eight class/band corridors per rare scene | Independent audit passed for all three scenes |
| Rare-scene deterministic geometry regeneration | Passed for all three original seeds; bounded failures retained |
| Installed guard, terrain, perception and other retained components | Original pinned file hashes unchanged |
| Parent candidate integrity and new runtime pin check | Passed |

Build attempts v16 and v17 remain preserved: the CPU test wrapper first omitted
the other source packages, then replaced ROS's Python path instead of extending
it. V18 fixes the wrapper environment; the tested planner bytes are unchanged
across these attempts. No rendering or movement occurred during those failures.

The short stopped v18 prerequisite measured worst sampled rolling publication
p95 **163.67 ms**, median reported RTF **0.99637**, and peak sampled Tctl **81.875°C**
over 16:14:22–16:15:22 UTC. All 13 snapshots were stopped, disarmed and healthy.
This is not a matched baseline, travel-drift or endurance qualification.

V20's scene-audit helper was superseded before completion because scene objects
gain render-only material fields after the geometry hash is computed. Its partial
outputs are preserved. The corrected helper reads the saved pre-material layout,
checks its hash, independently audits the saved meshes/routes and compares a
deterministic geometry regeneration. These checks passed in v21 before rendering.
This three-scene demo check does not replace the earlier 50-layout qualification
or establish a new 50-layout qualification for the rare recipe.

The stopped v21 prerequisite recorded seven healthy, disarmed snapshots over
16:51:40–16:52:10 UTC: worst sampled rolling publication p95 **177.48 ms**,
median reported RTF **0.99295**, and peak sampled Tctl **82.875°C**. Commands and
estimated travel remained zero. This is a short launch prerequisite, with no
movement, return, drift, matched-baseline or endurance acceptance implied.

V21 subsequently reached the retained **87°C qualification stop** during a second
exterior inspection at 16:55:43–16:55:49 UTC. The supervisor stopped the scene and
candidate detector, with no supervisor cleanup errors. The viewer's concurrent
cleanup found its container already exiting; a later retry correctly found no
container. No autonomous movement occurred in this attempt. The failure archive
has 32 independently verified off-host members; native session closeout is separate.

## Lower-load candidate after the thermal stop

V22 changes requested RGB-D rendering from **15 to 10 Hz** and caps exterior
inspection at **10 seconds**, with a ten-second default. Image resolution,
mount/FOV, clipping, renderer, rare-scene geometry, 40 m maps, detector, float64,
HIP flags 4, physics and motion/thermal safeguards are retained. Previous candidates
remain available. This does not change training-image capture profiles.

The reduced rate requests one third fewer RGB-D frames. It does not establish a
particular power or temperature saving. Keep the exterior view and RViz closed
during normal browser operation. A short inspection does not qualify exterior
video during autonomous travel. Check that physical vents and exhaust space are
clear; fan and power settings are unchanged.

Windows and Linux navigation suites passed **262 tests**, with four GPU-only skips
on each. Eleven tool/viewer tests passed on Linux; Windows skipped the two Linux
process checks. The rebuilt package, real ROS/HTTP probe, real tf2 interpolation,
all three launch graphs and runtime pins passed. The 960 × 540 transport fixture
received **60/60 RGB and depth frames at 10.00 Hz**, with maximum gaps of 100.9 and
101.4 ms respectively. The actual launch record also resolved ten Hz, and its
ten-second exterior inspection closed without cleanup errors.

The five-minute stopped check (17:06:50–17:11:50 UTC, 61 snapshots) passed:
mean Tctl **78.79°C**, sampled peak **80.5°C**, worst sampled rolling mapping
p95 **121.25 ms**, median reported RTF **0.99799**, maximum camera age **0.544 s**
and maximum accepted-depth age **0.106 s**. All sampled navigation, localization,
mapping and stopped-command checks passed. The below-80°C peak target remains
unmet; movement and endurance remain separate gates.

For context, equal three-minute windows 90–270 seconds after the two fresh
launches, both with the exterior window closed and no travel, measured:

| Measurement | V21 / 15 Hz | V22 / 10 Hz |
|---|---:|---:|
| Mean sampled Tctl | 81.22°C | 78.54°C |
| Peak sampled Tctl | 83.375°C | 80.625°C |
| Mean native reported PPT | 71.27 W | 67.26 W |
| Median rolling publication p95 | 169.43 ms | 120.32 ms |
| Median reported real-time factor | 0.99644 | 0.99799 |

Each window contains 180 rover snapshots and 35 native health records. These
sequential runs have different inspection durations and initial thermal states;
they support the combined lower-load profile without isolating the causal effect
of frame rate. This is a 40 m to 40 m comparison, not the required matched 20 m
baseline. Rolling latency summaries are not a newly computed aggregate percentile.

### Movement attempt and GPU incident

The v22 mission started at approximately 17:12:26 UTC. The initial observation
sweep and frontier selection succeeded. After the operator requested early return,
the rover began its planned home route. Estimated travel reached **2.79 m** before
loss of clock/sensor delivery stopped the mission; no home arrival or sample was
verified. The observer recorded a moving peak Tctl of **84.375°C**, below the 87°C
qualification stop. Before the failure, rolling publication p95 remained near
121 ms; stale data and degraded RTF after the stall are failure evidence.

At **17:14:19.579625 UTC**, the native journal recorded `ring sdma0 timeout`,
followed by GPU reset, repeated MES queue-removal failures and `VRAM is lost due
to GPU reset`. The reset reported success at 17:14:37. The terrain worker aborted,
the supervisor refused the incident-marked case and stopped its containers, and
the graphical desktop/RDP session ended. The native monitor terminals also ended;
their gap is retained. A separate read-only temperature logger records later idle
cooldown without replacing the missing native monitoring.

Protected finish completed at 17:19:56 UTC with `INCIDENT_REVIEW_REQUIRED` and
`post_teardown_clean=false`. No scene, NPU or terrain listeners remained. The
31-member GPU-incident archive, including protected teardown, was independently
verified off-host. The separate temperature log ran 17:20:04–17:23:05 UTC; it
does not cover the earlier protected cooldown. The durable observer was then
intentionally stopped by verified PID/start-time identity.

This is a confirmed GPU incident with an unresolved initiating cause. It does not
establish a thermal cause, implicate a particular application solely because its
queue could not be suspended during reset, or explain the separate November 10
hard freeze. SSH remained available. The Windows RDP tunnel remained listening,
but no user graphical session or Halo RDP listener remained.

**Movement qualification failed.** No 0.05 m/s movement qualification receipt was
issued, and 0.10 m/s remains gated. Keep workloads off, preserve protected closeout
and review the GPU incident before another hardware attempt. Future isolation
should distinguish rendering alone from simultaneous rendering/ROCm mapping under
fresh monitoring; a CPU-reference experiment would need its own candidate and
performance qualification, with no silent fallback or safety-policy relaxation.

## Attended rehearsal sequence

Hardware testing is stopped after the GPU incident. The instructions below are
for a future reviewed, freshly monitored session; the incident-marked November 17
case cannot be reused to resume the mission.

Use a fresh native capture/temperature session, current NPU qualification and the
retained float64/HIP-flags-4 terrain starter. Keep the desktop connected and
temperatures visible. Confirm current-session CPU/ROCm parity before rendering.

After the rare-site build, geometry audits and runtime pins pass, use the checked
candidate path supplied with its build receipt. After the thermal stop, use v22
for the lower-load qualification:

```bash
DEMO=/home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109
bash "$DEMO/runtime-v22/start_autonomous_demo.sh" 1 rehearsal 0.05
```

After renderer readiness, inspect launch clearance from another terminal:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v22/view_autonomous_demo.sh 10
```

Close the exterior view before confirming launch clearance in the browser.
Initially keep the rover stopped for fresh sensor, publication, temperature and
off-host observer checks. An initial invalid-odometry fault remains recorded until
explicit mode selection with healthy inputs; it is not automatically cleared.

The first movement check is a short 0.05 m/s autonomous exploration and early
return. Keep the controlling browser visible. Proceed to map-click sample preview
and approval only after a verified home arrival, with a safe route and sufficient
time. Rejections, faults or thermal stops require review. Do not enable manual
drive to rescue an accepted autonomous attempt.

Keep native 90°C protection and the 87°C qualification stop. The November 10
stationary observation reached 83.125°C, so the previous below-80°C target remains
unmet. Full autonomous acceptance, 0.10 m/s qualification, localization drift,
variation cycles and recording gates remain open until measured.

At closeout, press browser Stop and interrupt the demo and separately pinned
terrain/NPU services. Run protected `halo-rover finish` with capture and temperature
monitoring active through cooldown and final journal capture; then stop monitors.

Private build receipts, overlays and observations are retained under
`navigation_sessions/autonomous-demo-20261109/rehearsal-20261117/` and omitted from
Git. See the [demo guide](AUTONOMOUS_DEMO_GUIDE.md),
[browser profile](AUTONOMOUS_DEMO_BROWSER_PROFILE.md) and
[original freeze investigation](AUTONOMOUS_DEMO_FREEZE_20261110.md).

## November 19 follow-up at frontier F

V26 passed five minutes of stationary mapping in the representative 1 scene at
0.10 m/s. The first exploration goal then ended with Nav2 error 105
(`FAILED_TO_MAKE_PROGRESS`). As the rover entered the 6 cm goal tolerance,
linear commands went to zero for final heading alignment. It continued to
receive turn commands of about 0.05 rad/s as its estimated distance from the
goal grew from 5.9 cm to 14.6 cm. Simulator truth also recorded backward drift.
The guard recorded no fault and protected finish was clean with zero kernel
incident matches.

Nav2 Jazzy Regulated Pure Pursuit keeps its final-turn decision latched by
default after reaching the XY tolerance. V27 configures the demo controller to
recheck XY during final alignment and configures the stopped goal checker to
verify position again before accepting arrival. It retains the 6 cm XY
tolerance, heading and stopped-velocity tolerances, the 20-second progress
watchdog, and the existing speed and collision checks. An isolated physics
replay reached yaw tolerance at the original turn acceleration, so V27 uses
the original acceleration.

The installed Nav2 replay used 84 recorded distance samples. The updated
controller released final-turn mode on all 21 post-entry samples outside
tolerance and could resume alignment on reentry. The installed goal checker
rejected a drifted pose and a moving rover. The navigation suite ran 265 tests
with four GPU-only skips, and the three actual launch graphs passed. V27 is
installed as a separate candidate on Halo. Attended retest and speed
qualification for V27 are pending. Its configuration SHA-256 is
`3b28bd8325754311cbd94ecc765997ca1005a74df2a98e74879d6fc059f94eef` and runtime
release SHA-256 is
`485dfdd29cb76be6a95aec6586b012a349c4ca627d0e4ad5e9b139c61cc2c01b`.

Use the following launcher for the attended V27 rehearsal after starting a
fresh native capture, qualifying the NPU and starting the SDMA-off terrain
worker for that case. The native `halo-rover capture` launcher can prepare a
new case after the previous clean finish and capture closure are recorded.
The November 19 reproduction helper remains tied to its earlier case.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v27/start_autonomous_demo.sh 1 rehearsal 0.05
```

Use the corrected viewer packaged with V27 from a separate Halo terminal.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v27/view_autonomous_demo.sh 10
```

The original V27 retry included five minutes of stationary mapping before
exploration. The [current V28 rehearsal](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
starts exploration once the dashboard enables Start after clearance
confirmation. Observe the first frontier through alignment and check for an
accepted visit, then verify autonomous return. Record actual coverage and
preserve protected finish. The 0.10 m/s gate requires a receipt for the
candidate's own configuration.

The requested longer-term operator workflow is recorded in the
[autonomous demo guide](AUTONOMOUS_DEMO_GUIDE.md#v50-survey-workflow).

## November 19 V28 turn failure and V29 wheel traction

V28 reached its frontier XY tolerance, then drifted back downhill during
alignment. The controller alternated between turning and moving forward to
reacquire position. The Nav2 progress checker aborted with error 105. Protected
finish found zero new kernel incidents.

The rendering-free Gazebo fixture reproduced error 105 with the original wheel
model and the installed Nav2 controller. V29 sets both rear wheel contact
friction coefficients to 0.35 in demo scenes. Across two repeats of each turn
direction on the recorded slope, Nav2 reached the goal with its original 6 cm
position and 0.087 rad heading tolerances. The final repeated recorded turn
finished 4.3 cm from its target. The opposite turn finished 5.9 cm away.
The flat 90° turn displaced the front-axle reference by less than 0.1 mm, and
the flat straight-drive check reached its target.

The pinned V29 build passed the navigation suite, the ROS smoke check and all
three launch graph checks. All three graphs generated the same Nav2 and drive
controller configuration as V28. The replay used simulator pose feedback with
rendering disabled. The next attended rehearsal tests the camera-based
localization, terrain grid and live motion guard together.

Reuse the [capture, NPU and terrain startup procedure](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
with the V29 starter and viewer. Once the viewer closes and clearance is
confirmed, start as soon as the dashboard enables exploration. Let the rover
finish its frontier turn before requesting the return.

```bash
# Simulation terminal
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v29/start_autonomous_demo.sh 1 rehearsal 0.05
# Separate inspection terminal after the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v29/view_autonomous_demo.sh 10
```

### V29 attended result

V29 reached its first frontier and measured 14.8 m² of additional ground.
The remaining exploration budget could not cover another frontier and its
return, so the mission started home automatically. It stalled during the
opening return turn, with about 3 m still to travel. At the stall, the rover
repeatedly switched between in-place rotation and a short-radius forward arc.
Estimated pose and simulator truth showed the same stall. The motion guard
recorded no fault. Nav2 aborted for lack of progress with error 105.

Protected finish recorded clean teardown and zero new kernel incidents.
`INCOMPLETE_OR_FAILED_REVIEW_REQUIRED` is the lower-level endurance result for
this case because no timed measurement was recorded. The final
`MANUAL_SESSION_CLOSED_CLEANLY` confirms clean shutdown of the rehearsal.

### V30 directional rear traction

The rendering-free replay reproduced the return stall with the V29 path and
wheel model. Stronger turn commands and altered path alignment thresholds
also stalled. V30 assigns the rear tires 0.25 friction along the axle
(sideways) and 0.80 in the direction of rolling. This allows the rear tires to
scrub sideways while retaining rolling grip.

The original return path reached home in two isolated replays. It stopped
5.85 cm and 5.78 cm from the goal, within the original 6 cm position
tolerance. The earlier frontier turn passed in both directions. Flat pivot
and straight travel also passed. The replay used simulator pose feedback.
V30 passed the ROS smoke check, navigation suite and all three launch checks.
Attended verification with camera localization is pending.

For the next rehearsal, use the same capture, temperature, NPU and terrain
commands with the V30 starter and viewer.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v30/start_autonomous_demo.sh 1 rehearsal 0.05
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v30/view_autonomous_demo.sh 10
```

### V31 return display, sample timing, and scene changes

V30 visited the frontier and completed a verified home return. The recorded
home goal stayed at launch. The dashboard now shows only the active Nav2 route
during motion. Candidate routes are used for planning, and sample round-trip
routes appear during preview.

The sample planner found clear routes but several exceeded the 240-second
sampling allocation. V31 makes unused exploration time available to sampling
within the original overall mission duration. In the V30 recording, the early
home arrival left about 372 seconds for the sample trip. Several recorded
options needed about 292–353 seconds. Preview reports the required and available
times when a target is too far and still requires approval before departure.

The original scene generation was stopped by the 87°C qualification limit.
V31 waits for 30 seconds at or below 70°C before starting the two-CPU generator
and again before relaunching the scene. The same seed generated successfully in
isolation and peaked at 77.25°C. Its content, reachability contract and actual
launch graph passed. The protected finish command displays the clean shutdown
verdict for a clean rehearsal and keeps the endurance receipt on disk.

The navigation suite, browser tests, ROS smoke probe and launch graph checks
passed. The attended sampling trip and browser scene replacement are pending.
Use the same [platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
with these V31 launchers.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v31/start_autonomous_demo.sh 1 rehearsal 0.05
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v31/view_autonomous_demo.sh 10
```

### V32 stable navigation frame

V31 completed the frontier visit. About six seconds into its return, the
RTAB-Map correction moved the map frame by 2.8 m. Visual odometry and Gazebo
truth remained continuous. The active Nav2 goal retained its old map
coordinates, so the rover drove toward a point near the frontier and
eventually returned error 105.

V32 fixes the demo navigation map to the visual odometry origin. Terrain,
tracked objects, the home marker, the path display and Nav2 all use that
frame. RTAB-Map keeps its graph in a separate diagnostic frame. The motion
guard checks the fixed transform and stops if it changes unexpectedly.

The isolated Gazebo replay followed the recorded return with the injected map
shift and reported success 2.79 m from the real home. With the fixed transform
it finished 5.9 cm from home. The real ROS static publisher, RTAB-Map
configuration, stale odometry check, guard regression and all three launch
graphs passed. The attended camera-based return remains to be verified.

Use the same [platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and verify that the home marker stays at launch through the frontier turn.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v32/start_autonomous_demo.sh 1 rehearsal 0.05
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v32/view_autonomous_demo.sh 10
```

### V33 sample approach and map refresh

The attended V32 rehearsal completed the frontier visit and returned within
5.5 cm of home. The two pale soil previews stopped 2.29 m and 2.41 m from
their targets because the planner added each patch's extent to the obstacle
clearance.

V33 tries closer poses for measured ground and soil. The two recorded sites
have preferred distances of about 1.20 m and 1.22 m. Each pose must have
clearance through a full turn and feasible, budgeted routes in both
directions. The planner can choose a wider stop if the closer alternatives
fail. The dashboard shows the planned distance next to the selected target.

The map retains the current image while a new image loads and after a failed
fetch. The automatic crop expands as the mapped area grows and resets for a
new scene. Full map mode stays fixed on the whole map.

The navigation suite, browser regression tests, Chromium image-load fixture,
ROS smoke probes and three launch graph checks passed. The V33 attended
sampling run is pending. Use the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo),
then use these launchers.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v33/start_autonomous_demo.sh 1 rehearsal 0.05
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v33/view_autonomous_demo.sh 10
```

Use the [Halo dashboard](http://127.0.0.1:8774) during scene changes. Generating
a scene stops the current camera while the terrain generator checks candidate
layouts. The first V33 generation tried four layouts and took about three
minutes to return the live camera. When it returns, use the V33 viewer command
to inspect the new launch area before confirming clearance.

### V34 crystal selection and sample departure

V33 selected yellow crystal rock `0:67` but refused its preview because the
cell at the center of the rock was unmeasured. The pale soil preview placed a
stop 1.20 m from site `0:4`. Nav2 ended its planned return 7.8 cm from the
requested home pose. Approval compared that path endpoint to home and treated
the offset as a map shift even though the home pose had stayed fixed.

V34 allows confirmed sample markers with unmeasured centers and searches
multiple approach distances. The stopping pose, turn and travel corridor must
still be measured and safe. Approval checks the requested home pose for
changes and separately checks the whole return route. The motion guard uses
a protected area around the selected site throughout the sample trip.

The rover checks its bearing to the target at the stop, aligns in place if
needed and holds still for simulated collection. It then backs up 0.50 m
along a checked corridor before starting the return turn. The preview
includes the reverse and its travel time. If clearance or localization fails,
the rover stops. The ROS and launch checks passed. The installed motion guard
and drivetrain reversed 48.5 cm and settled in an isolated flat-ground Gazebo
test. The live end-to-end sampling rehearsal is pending.

Use the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and these launchers. The dashboard remains at port 8774.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v34/start_autonomous_demo.sh 1 rehearsal 0.05
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v34/view_autonomous_demo.sh 10
```

### V36 sample authorization and Mars survey

V34 measured 105.03 m², or 6.56% of the 40 m × 40 m map. After two
productive frontier stops, every shortlisted round trip exceeded the
remaining exploration budget. The subsequent sample approval reused a
heartbeat from 31.4 seconds earlier. Every navigation input was healthy
when the watchdog stopped the sample departure.

V36 treats the verified owner's sample approval as fresh contact and
continues heartbeats while the guard acknowledges arming. It dispatches
only after a successful arm with a current lease. A selection timeout
during arming stops the guard. The heartbeat limit is still one second.

The frontier search considers observation stops farther back from the
unmeasured edge, spreads candidates across directions, and ranks them
using known-ground travel cost from the rover and back to home. Nav2's
A*-based state-lattice planner checks both actual routes. The round trip
has one 30-second contingency in addition to both legs' travel margins.
The `full` profile allows up to 20 minutes for mapping and the first
return within the 30-minute total mission budget. The `rehearsal` profile
still uses six minutes for mapping and first return.

Representative 1 and newly generated demo scenes use patchy wind
ripples and irregular low ridges on the same rendered and collision
surface. The audited representative's median ordinary obstacle scale
increased from 0.30 m to 0.35 m. All four classes and all eight
reachability routes passed. The terrain was inspired by
[Perseverance's observations of wind ripples](https://science.nasa.gov/blog/reading-the-ripples-at-observation-mountain/)
and [layered, rocky slopes at Jezero](https://science.nasa.gov/photojournal/nasas-perseverance-rover-provides-sweeping-view-of-broom-point/).

Use the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and these launchers for the longer mapping test. Watch measured
coverage at the first home arrival, then approve a sample preview to
test the new heartbeat transition. The achieved coverage and complete
sample trip are pending attended verification.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v36/start_autonomous_demo.sh 1 full 0.05
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v36/view_autonomous_demo.sh 10
```

### V37 frontier arrival and operator resume

V36 was approaching the third frontier when its routine check of the
route home failed. The plan's initial turn crossed an unsafe or
unmeasured footprint at `(-1.594, -0.783)` m. The guard stopped at
about `(-1.705, -0.884)` m. Navigation health was fresh. The recorded map
origin and dimensions stayed fixed throughout exploration.

On a route-planning failure, V37 stops and offers **Resume with a new
route** in the browser that owns the mission. Resume uses the remaining
mission time and requires healthy navigation, a stopped rover, and a
checked route home. If the direct turn is blocked, it can check a short
forward move on measured ground to a spot with more turning room. A
guard fault or lack of a safe route keeps the rover stopped.

Frontier scoring prefers outward viewpoints into larger unmeasured
patches. The rover observes from measured ground and advances as its
camera measures more terrain. Frontier candidates and their planned
arrival and departure poses need room for a full turn and an extra
12 cm of clearance. The active frontier stop is checked as the map updates.

The dashboard draws decoded map images into a persistent canvas and
retains the visible image and markers through failed updates. Chromium
verified canvas pixel retention through delayed and corrupt image
responses. The attended Halo route recovery and display check are pending.

Use the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and these launchers.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v37/start_autonomous_demo.sh 1 full 0.05
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v37/view_autonomous_demo.sh 10
```

### V38 sample refinement, contact recovery and 0.10 m/s

V37 visited four frontiers and mapped 144.39 m² before the operator
requested return. The pale soil trip was approved and drove for about
eight seconds. As it approached site `0:4`, the estimated radius grew
from 0.675 m to 0.715 m and the center moved about 4 cm. The previous
25 mm envelope-change limit stopped the trip. No chassis contact was
reported around the stop.

V38 plans a larger protected area for soil from the start. Refinements
to the confirmed site can be used while they fit inside that checked
area. The rover uses the latest accepted center to face the sample.
Growth beyond the protected area still calls for a new preview.

After a chassis-contact stop, **Back up 0.5 m & replan** becomes
available when contact has cleared and the sensors are healthy. The
rover checks its recent drive trace and follows it in reverse at up to
0.05 m/s. It stops before checking the next route and excludes the
previous contact location from that route. Incomplete history, blocked
ground, or a missing alternate route keeps the rover stopped.

The attended simulation launcher now defaults to 0.10 m/s forward with
the existing platform and motion checks. The stationary contact fixture
keeps its 0.05 m/s configuration. The launch checks verify the requested
speed in the generated Nav2 controller configuration. In isolated
Gazebo, the installed guard and drivetrain held an average 0.099 m/s
forward. A synthetic contact stopped the rover and blocked retreat
arming until contact cleared. The rover then backed 0.482 m along the
curved trace. The full camera-based 0.10 m/s rehearsal is pending.

Use the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and these launchers.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v38/start_autonomous_demo.sh 1 full 0.10
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v38/view_autonomous_demo.sh 10
```

### V39 alternate routes and sample approaches

The attended V38 rehearsal at 0.10 m/s measured 229.62 m². One frontier
stalled 6.10 cm from its requested stop while Nav2 made small heading
corrections. V39 uses a 9 cm arrival tolerance for frontier goals, inside
the turning clearance checked during planning. A replay of the stopped
pose through the installed Nav2 checker passed at 9 cm and failed at
6 cm. The frontier checker also rejected the replay pose while moving
or facing the wrong way. Precision home and sample goals use their
6 cm checker.

When exploration pauses, **Try a different route** searches through a
measured waypoint at least 65 cm outside the failed route. It checks
both legs and rules out rejoining the failed corridor on the next
frontier departure. The rover stays stopped if it cannot verify a
distinct route and a return reserve.

After a sample preview, **Preview another angle** looks for a stop at
least 35° apart from the approaches already shown for that target.
The replacement round trip needs a fresh approval. When the search
finds no alternative, the previous preview stays available for its
original validity window.

V38 stopped a second pale-soil trip when its tracked radius grew from
1.00 m to 1.13 m. V39 calculates the radius from measured boundary
points and the current center. Across 26 archived independent
observations, the corrected geometry stayed inside the allowance for
the approved preview. If a new measurement exceeds that allowance
during travel, the rover stops and accepts a fresh preview and
approval from its current position within the original mission budget.

The browser tests, navigation suite and three 0.10 m/s launch graphs
passed. The live alternate route and full sample trip remain pending.
Use the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and these launchers.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v39/start_autonomous_demo.sh 1 full 0.10
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v39/view_autonomous_demo.sh 10
```

### V40 automatic route recovery and sample return

V39 paused on two Nav2 105 failures and one blocked return route. V40
automatically looks for a distinct, checked route after the guard
confirms a stop. It requires healthy navigation and a live operator
heartbeat. It allows up to two automatic attempts before the next
successful frontier visit. If it cannot verify a route with enough
time to return, it stays stopped and shows the reason.

The V39 rover reached the pale soil sample site and held position while
fresh observations briefly dropped out. In V40, a brief evidence gap
pauses the simulated collection timer. After collection, or if evidence
stays unavailable, the dashboard offers **Return home**. The rover
waits for that command and checks its reverse departure before turning
toward home. An incomplete collection is recorded as unconfirmed.

The V39 map joined exploration points across intervening repositioning
moves. V40 renders the recorded trail as separate segments across
motion phases and telemetry gaps, so the map does not draw straight
lines between those segments.

The browser and navigation test suites, installed ROS smoke test, and
all three 0.10 m/s launch checks passed. The attended end-to-end V40 run
is pending. Follow the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and launch V40 with these commands.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v40/start_autonomous_demo.sh 1 full 0.10
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v40/view_autonomous_demo.sh 10
```

### V41 frontier arrivals, new coverage and perimeter objects

V40 recovered automatically from three Nav2 105 errors. The recorded
poses before the failures were 9.0, 9.3 and 10.3 cm from their frontier
targets, with heading errors up to 14.4°. V41 uses a 16 cm and 16° arrival
tolerance for frontier observations. The planner checks the full turn
envelope across the wider arrival area. The installed Nav2 checker
accepted the three poses in replay and rejected poses outside the
tolerance and poses with movement. Home and sample drives use the
existing precision checker.

Frontier scoring uses the recorded driven trail. It prefers useful views
away from that trail and gives a route that retraces earlier travel a
lower score. It checks up to two more candidates when the first feasible
route substantially overlaps the trail. Footprint checks and the return
reserve still apply when selecting a route.

Each side of the 40 m scene gains two rocks and a sample site in the
16–18.2 m perimeter band. The sample sites have connected staging areas.
The supplemental placements use independent random streams, so the
original objects and sample regions retain their seeded placement.
**Generate a new scene** uses the same perimeter placement.

The V40 supervisor ended the run at the 87°C qualification cutoff.
V41 requests a checked home return during exploration at a fresh 85°C
reading or a sustained 20-second average of at least 83°C. In replay
this requested return about four minutes before the V40 teardown. The
87°C supervisor cutoff and 90°C native protection still apply, and the
launcher prints the supervisor's specific stop cause.

The installed ROS probe, browser tests, Nav2 arrival replay, and three
0.10 m/s launch checks passed. Each scene audit matched all original
placements and checked the new perimeter. The attended V41 run is pending.

Use the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and launch V41 at 0.10 m/s.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v41/start_autonomous_demo.sh 1 full 0.10
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v41/view_autonomous_demo.sh 10
```

### V42 demo temperature cutoff

The starter and exterior viewer share an 89°C cutoff. The dashboard
displays this cutoff from telemetry. V42 removes the 85°C and rolling
83°C triggers for an early return. Halo's native protection is 90°C.

Follow the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and use the V42 launcher and viewer.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v42/start_autonomous_demo.sh 1 full 0.10
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v42/view_autonomous_demo.sh 10
```

### V43 waypoint arrivals and 0.15 m/s

V42's two failed frontier approaches ended about 16.6 cm from the
target while the rover continued making small heading corrections.
A recovery waypoint failed at 6.8 cm. V43 uses a 22 cm frontier radius
and a 12 cm recovery radius. Nav2 ends those goals when the pose enters
the configured position and heading tolerances. The mission waits
for fresh odometry and a zero guard command before advancing from
each waypoint. The arrival areas have turning clearance checks.
The installed Nav2 plugins accepted all three archived approaches
within the new tolerances.

At the final V42 stop, the mission node's odometry and guard data aged
out while both publishers continued sending. V43 puts the frontier
search in a separate process and batches full-footprint path checks
over the same spatial and angular samples. In an isolated ROS replay,
the median check time for the recorded 276-point return path dropped
from about 26 ms to 4 ms. The mission kept receiving fresh inputs during
repeated searches and stopped when the replay withheld odometry.

Autonomous cruise is set to 0.15 m/s. The manual speed slider allows
0.15 m/s and starts at 0.05 m/s. All three launch checks passed at the
new speed. The attended V43 run is pending.

Follow the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and use these launchers.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v43/start_autonomous_demo.sh 1 full 0.15
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v43/view_autonomous_demo.sh 10
```

### V44 waypoint handoff and recovery

The repeated V43 stall held about 23.5 cm from a frontier with a 22 cm
arrival radius. V44 checks a 40 cm frontier region and a 22 cm recovery
region in the mission node. The Nav2 controller aims farther inside
each region. When the estimated drive center enters the region with
the required heading and turning clearance, the mission requests
the active action to stop. It waits for the terminal action result
and fresh stationary feedback before dispatching the next goal.
A progress abort at a clear waypoint can use the same arrival check.

A route failure outside the arrival region starts automatic rerouting
after the guard confirms the stop. Recovery uses the existing operator
heartbeat, measured terrain and return reserve. If it cannot verify
a distinct route, it checks for a safe route home within the mission
budget. F shows the active destination during the recovery drive and
then moves to the next frontier. The dashboard clears a recovered
error and enables Start once the map publication timings qualify.

Three isolated ROS runs exercised a near-goal handoff, an out-of-region
progress abort, automatic recovery, the next frontier, and a verified
home stop. The near-goal stop request took at most 0.21 s in the
fixture, and automatic recovery began within 0.08 s of its scripted
abort. The browser checks verified both F markers and the recovery
messages. The three launch graphs and the input freshness replay
passed.

Follow the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
and launch the V44 candidate at 0.15 m/s.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v44/start_autonomous_demo.sh 1 full 0.15
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v44/view_autonomous_demo.sh 10
```

### V45 frontier routing and 8 Hz RGB-D

V44 drove the loop visible on the map during exploration. On the first
southbound leg, its 85 cm overlap check counted only 7% of the path as
near the earlier trail. V45 weights proximity out to 3 m and compares up
to three candidate routes after it finds a feasible one. It favors
expected new coverage per second of checked outbound travel. Candidates
require turning clearance and a checked route home within the budget.
V44 also planned a 2.19 m route to reach a recovery waypoint 1 m away.
V45 skips alternate recovery routes longer than twice the direct distance.

V44's visible monitor recorded 89.5°C before the native thermal guard
stopped the run. V45 captures RGB and depth at 8 Hz, down from 10 Hz,
to reduce rendering and image processing work. It uses 960 × 540 images
at 0.15 m/s. An attended run is needed to measure the temperature change.

The route regressions and installed ROS waypoint and freshness checks
passed. The transport fixture received all 48 RGB and 48 depth frames
at 8 Hz. All three scene launch checks and the flat fixture check passed.

Start with `~/bin/halo-rover capture` on Halo. The repaired launcher can
reuse a verified kit if the earlier start stopped before protected
preflight created a case. Wait for the health records before continuing
with the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo).

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v45/start_autonomous_demo.sh 1 full 0.15
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v45/view_autonomous_demo.sh 10
```

### V46 operator heartbeat

V45 stopped on its fourth frontier when its last dashboard heartbeat was
1.013 seconds old. Odometry, depth, detector and motion guard data were
healthy at the stop. V46 gives each heartbeat reply 250 ms before the
browser can retry. A recent acknowledgement from the same mission can
bridge a delay in the full dashboard status up to 1.5 seconds. Periodic
status requests run one at a time. Stop and scene changes release the
browser's authority immediately.

The installed ROS test held a status request open and delayed one
heartbeat reply past its timeout. Subsequent heartbeats kept the mission
active. After the final heartbeat, the mission stopped in 1.007 seconds.
A different operator token and a heartbeat sent after Stop were both
rejected. Browser tests and all three scene launch checks passed.

If the V45 closeout is pending, complete `~/bin/halo-rover finish` before
starting a fresh capture. Use the
[Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
with these V46 launchers.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v46/start_autonomous_demo.sh 1 full 0.15
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v46/view_autonomous_demo.sh 10
```

### V47 complete mapping and sampling rehearsal

On November 22, V46 ran autonomously for 9 minutes 41 seconds, traveled 62.4 m,
and measured 345.6 m² (21.6% of the map). It completed 17 frontiers
and automatically recovered from three changes in arrival clearance.
Navigation inputs stayed healthy and the operator heartbeat held.
Halo stopped the simulation at 89.375°C, before the first return home.
The protected teardown was clean and the kernel review found zero
incident matches.

One 21 m frontier approach spent almost two minutes retracing earlier
travel before it lost arrival clearance. V47 skips an approach if its
trail overlap is at least 80% and the estimated retrace exceeds 60
seconds. It checks other frontiers and proceeds to a checked home
return if the search exhausts its options.

The demo uses RGB-D visual odometry and the depth-based terrain and
object maps. V47 removes the separate analysis-only RTAB-Map graph
and secondary camera preview process from demo launches. The graph
alone wrote a 734 MiB database in V46. The dashboard renders display
rasters once per second and waits 125 ms between camera requests.
Visual odometry and mapping use 8 Hz RGB-D input. Halo's demo cutoff
is 89°C. Temperature savings need an attended measurement.

The isolated installed ROS test ran both home returns, selected a
confirmed rock with an unmeasured center, verified the standoff and
round trip, collected a simulated sample, and backed away before the
return turn. It reported the sample delivered at home with zero motion.
Browser, heartbeat, navigation, and launch checks passed.

For the first recording attempt, use `rehearsal` at 0.15 m/s. It budgets
six minutes for exploration and the first return, then four minutes for
the sample trip plus any unused exploration time. Start a fresh capture
and follow the [Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo).

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v47/start_autonomous_demo.sh 1 rehearsal 0.15
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v47/view_autonomous_demo.sh 10
```

1. Close the exterior inspection, confirm the clear launch area, and
   choose **Start autonomous exploration**. Keep the dashboard visible.
2. Let the rover return and stop at home. Select a nearby confirmed
   rock or pale soil patch. The V46 scene had a confirmed rock 1.7 m
   from home. The set of confirmed targets depends on the route.
3. Choose **Preview sample round trip**, review both routes, and click
   **Approve drive to sample**. Use **Preview another angle** if you need
   a different approach to that object.
4. Wait for simulated collection to finish, then choose **Return home**.
   Verify **Mission complete at home** and **Simulated · delivered home**.

Use [OBS on the Windows PC](AUTONOMOUS_DEMO_GUIDE.md#windows-recording)
to record the remote dashboard with video encoding on Windows. Leave
native capture and temperatures running through `halo-rover finish`.

### V48 frontier recovery and soil sampling

V47 measured 159.2 m² and verified both home arrivals. During mapping,
a frontier lost turning clearance. The rover spent 14 seconds moving
to a recovery waypoint, then rejected routes that shared the failed
corridor. Its remaining exploration budget also ruled out distant
frontiers, so it started the checked return home.

After a frontier arrival loses clearance, V48 stops and excludes that
waypoint. It checks the home reserve and searches for a new frontier
from the stopped position when there is a direct safe route home.
It checks full route and arrival clearance before moving. A blocked
route or controller failure uses the distinct-route recovery.

The selected pale soil patch moved 15 cm in the tracker and its
estimated radius grew by 12 cm. V48 includes a 30 cm refinement allowance
when it plans a soil route. A refinement can continue if the same
confirmed target's updated footprint fits within the approved keepout.
The preview checks the approach, stopping pose, reverse departure, and
route home against that larger area. A measured change beyond the
checked area stops the rover for a new preview and approval.

V47 offered **Return home** during collection. That request ended the
timer after 1.8 seconds of credited collection. V48 offers return after
the collection result. Stationary soil collection allows up to 0.85
seconds for supporting RGB-D evidence to reach the mission node. An
evidence loss can still leave the sample unconfirmed, in which case the
dashboard offers **Return without a sample**.

The installed ROS replay recovered from a blocked frontier without a
reposition, accepted the recorded soil refinement during the approved
trip, and delivered the simulated sample at the second home stop.
The browser tests and rock sample replay also passed. The attended V48
rehearsal reached its first home stop, as described below.

V47's protected closeout was clean. Use the
[Halo platform startup commands](AUTONOMOUS_DEMO_GUIDE.md#object-aware-rehearsal-on-halo)
for the next rehearsal.

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v48/start_autonomous_demo.sh 1 rehearsal 0.15
# In a separate terminal once the camera is ready
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109/runtime-v48/view_autonomous_demo.sh 10
```

### V49 long survey and sampling demo

On November 22, V48 began its first return 2 minutes 12 seconds after
autonomy started. The six-minute rehearsal budget covered exploration and
the drive home. Of the final 24 candidates, 13 exceeded the remaining
round-trip budget and 11 exceeded the retrace limit. The rover verified its
home arrival at 4 minutes 7 seconds and had measured 159.7 m² of the
1,600 m² map. The first return was a planned response to those route
filters. The last candidate needed about 238 seconds for its approach
and return, with 228 seconds left in rehearsal.

V49's `survey` profile aims for 25 minutes of mapping, including the opening
sweep. It budgets up to 32 minutes from autonomy start through the first
home arrival. The planner searches up to 80 viewpoints, favors routes that
reach new ground with less retrace, and considers longer retraces when
needed to reach a checked viewpoint. It returns when no checked frontier
fits, when the return reserve is reached, or when the mapping timer ends.
The dashboard shows the remaining mapping time during exploration.

The recorded V48 path replay reproduced the budget rejection in rehearsal
and selected that frontier in survey using a measured-clear test grid.
The isolated ROS survey test completed frontier recovery, both verified
home stops, and simulated sample delivery. The 321-case navigation suite
passed with four skips, and all three scene launch checks passed.

Two attended long surveys on November 22 ended at the V49 thermal cutoff.
The first measured 536.1 m² (33.5%) over 14 minutes 25 seconds, traveled
99.6 m, and visited 30 frontiers before the supervisor stopped at 89.625°C.
The second measured 506.7 m² (31.7%) over 12 minutes 29 seconds and traveled
82.3 m before the supervisor stopped at 89.375°C. Both supervisors reported
clean teardown. The first home return and sampling were still ahead in both
missions. In a short V47 rehearsal, the rover verified both home stops and
returned without a confirmed sample.

After the first home stop, there are two minutes to select and preview a
confirmed sample site. The sample trip has eight minutes plus any unused
first-return time within the 42-minute overall limit. Review the approach
and return, approve the drive, then request home after the simulated
collection result appears.

### V50 thermal cutoff and run commands

V50 stops the Halo survey at AMD Tctl ≥92°C. Native capture, the
temperature display, terrain supervision, the scene supervisor, and the
exterior viewer use that cutoff. The dashboard displays it as a single
limit. In offline checks, the guards accepted 91.999°C and stopped
at 92°C.

### V51 sample return handoff

On November 23, an attended V50 run mapped for 25 minutes, verified the
first home stop at 27:51, and measured 875.7 m² (54.7%) by the end of
the run. The operator reviewed a larger soil envelope after an outbound
stop, approved the revised route, and reached the sample pose. The mission
logged 10.19 seconds of stationary simulated collection. It reported a
ready return for 3.7 seconds before Stop or takeover ended the mission.
The audited capture closed cleanly with zero kernel incident matches.

V51 keeps **Return home** in the sticky top toolbar from sample approach
through the collection result. It explains why the rover is waiting,
enables when the stopped rover has healthy navigation, and shows a refused
return request where the operator can retry it. The operator authorizes
the return explicitly. The rover checks the reverse departure and route
home before moving.

Browser checks exercised the control with the map scrolled and the camera
expanded. The installed ROS replay used V50's recorded sample pose,
keepout and 390-pose home preview on a measured-clear fixture. It backed
0.50 m, stopped at home and recorded delivery. The full attended V51
survey was stopped during mapping to investigate a distant frontier.

### V52 closer survey frontiers

In the stopped V51 run, the planner selected a 25.46 m outward leg across
the map. Its estimated overlap with earlier travel was 70.5%. The old
80% threshold kept the leg in its preferred group while it deferred
shorter approaches with higher overlap. The rover had visited 21
frontiers and measured 24.2% when the operator stopped.

V52 gives viewpoints within 6 m of reachable driving their own place in
the candidate search, even when the same map edge extends much farther.
It checks nearby destinations first and measures the actual planned
drive around obstacles. Coverage gain and trail novelty rank the
checked routes within 2 m of the shortest usable drive. The search can
expand to a farther viewpoint when closer approaches fail clearance or
return checks. Viewpoints sit on measured ground and face into unmapped
terrain.

The recorded-route replay used both V51 plans and chose the 12.5 m
approach on a measured-clear test grid. A separate corridor test verified
that close observations were generated along a connected map edge. The
installed sample-return replay also passed.

### V53 frontier turns and planning time

Two frontiers in the attended V52 run produced loops in the driven
trail. After entering the 40 cm arrival radius, the rover drove 5.48 m
around one frontier while Nav2 tried to match the viewing angle. It
drove another 3.09 m around a later frontier before the operator stopped.
One frontier search took 10.94 seconds after the map supplied candidates,
with 64 paths published while checking outbound and return routes.

V53 ends the Nav2 drive on arrival within 40 cm when the actual rover pose
has clear turning space. After Nav2 acknowledges the stop and fresh
feedback confirms the rover is stationary, the rover turns in place to
face the frontier. The next search starts after fresh feedback confirms
the turn has stopped. The operator can request an early return during
the turn.

The survey planner compares three checked nearby round trips and
continues checking while another candidate could offer a shorter
drive. Coverage and earlier travel still guide the choice among routes
of similar length. Both legs are checked against the latest map and the
estimated time needed to get home.

The installed replay used both recorded arrival poses and verified
zero forward motion commands during the turn. In the slow search replay,
the planner made 10 route requests from the archived candidate paths
and selected a checked 1.18 m leg. The replay used a measured-clear test
grid and simulated motion feedback. Live timing and traction still
need an attended run.

### V54 map edge clearance

The V53 survey stopped during a northern frontier approach. The selected
viewpoint was at `(7.475, 17.825)` on the 40 m map. The rover got within
44 cm of it and came to about 2.6 m from the map edge. Nav2's remaining
route grew from 0.91 m to 3.08 m as it changed its approach. The guard
recorded an operator stop after the detour.

V54 keeps the rover's footprint 3 m inside each edge during the 40 m
demo. The mission and Nav2 plan on the interior 34 m square. The local
motion guard checks the same boundary. The camera and dashboard use the
full 40 m map.

A frontier normally uses the 40 cm arrival radius. If a fresh Nav2 plan
takes a long detour with the rover within 65 cm, the mission can stop
at the current pose when it has full turning clearance. It waits for
Nav2 and the motion guard to confirm the stop before aiming in place.

The installed replay received both map sizes over ROS and rejected
the recorded V53 route at the setback. It shifted the same approach
inland on a measured-clear test grid and verified the stop at 54 cm
with simulated motion feedback. The local sweep tests covered all four
edges. The next attended survey will verify operation with live terrain.

### V55 recovery turns and scene generation retries

V54 measured 658.6 m² (41.2%) and drove 104.5 m before the motion guard
stopped it 19:10 into the survey. The return route failed a footprint
check. Recovery found a connector that started with a clockwise turn,
but the controller turned counterclockwise and its predicted sweep
reached a tracked rock. The guard stopped the rover before contact.

V55 checks the recovery turn against the measured map and fresh local
ground. The rover performs the checked turn in place, waits for a
stationary reading, then rechecks the route before handing control back
to Nav2. Recovery stays stopped if the current terrain blocks the turn
or a safe departure cannot be checked.

The next scene request also failed when the generator exhausted the
perimeter placement attempts for one layout. V55 lets the generator
try the next layout for the same seed within its retry limit. The
failed seed `277760684` generated a scene successfully on layout 15.

The recovery replay used the recorded pose, 197° clockwise turn and
rejected rock cell. It verified zero translation through the turn and
one fresh route check after the stop. The replay used simulated motion
feedback. The next attended run will verify the complete recovery on
live terrain.

### V56 automatic mapping recovery

V55 completed both home stops and delivered the simulated sample at
26:33. The operator resumed mapping three times and requested an early
return at 17:34. The final map measured 621.0 m² (38.8%).

V56 counts automatic attempts near each stopped position. Moving to
new ground lets the rover recover again even before completing a
frontier. If the rover runs out of attempts at the same spot, it
checks a route home. When a return-route check causes a pause, the
rover first replans from the stopped pose, then checks a different
departure if needed. The owner’s live heartbeat and a checked route
are required before it resumes.

If Nav2 asks for a small heading correction after a recovery turn,
V56 checks the new turn against the terrain, performs it in place,
and waits for a full stop before checking the departure again. Two
corrections can fit within the existing recovery turn deadline.

The installed replay completed all three recorded heading corrections
and automatically resumed the two later pauses using the recorded
routes and the existing heartbeat. It used a measured-clear test map
and simulated motion feedback.

### V57 longer survey legs

An attended V56 survey reached its mapping time limit at 25:01 and
verified home arrival at 29:04 with 53.0% of the map measured. It
resumed exploration automatically seven times. The median planned
frontier leg was 1.81 m, and 80 of 85 were under 3 m.

V57 gives viewpoints 3.5–6 m away their own places in the frontier
shortlist. The survey compares up to three such candidates first.
A preferred leg needs a checked outbound path of at most 6 m, limited
overlap with the driven trail, enough room to turn at the frontier,
and a checked return within the time budget. If none passes, the
planner checks the nearby options.

In replay, V57 selected a 5.2 m option from the same recorded batch
where V56 had selected 2.6 m. A separate replay selection used the
shorter leg when the longer option retraced earlier travel. These
checks used recorded path geometry and trail history on a clear test
grid. The next V57 survey selected 11 preferred legs out of 39, with a
longest leg of 5.41 m. It mapped 28.6% before the motion guard stopped a
recovery turn near a tracked rock.

### V58 edge soil and mudstone tracking

The three demo scenes now have six, five, and six perimeter pale soil
patches, each up from two. The generator places added patches near a
connected staging pose inside the 3 m driving setback. The regenerated
scenes preserve the original terrain, rocks, and interior regions.

Two depth-supported views can confirm a rock, crystal, mudstone slab, or
pale soil patch when they span at least one second and both the rover's
drive center and camera have moved at least 18 cm. A mudstone slab can
qualify with shallow relief and a measured rim above the local ground.
The archived V57 slab mesh passed all six two-view depth replays. Bare
ground at the same location failed the geometry check in all 12 views.
These replays used rendered depth and projected boxes.

Short recovery checks now skip a path whose driven length exceeds 1.8
times the displacement to its waypoint. In replay this rejected the
0.97 m detour toward V57's 0.50 m recovery waypoint and checked another
connector. Live mapping and recovery in V58 still need a full run.

Finish any prior capture with `~/bin/halo-rover finish` and complete its
sudo prompt. Run each command below in a separate Halo desktop terminal.
Wait for repeating capture health before temperatures and NPU. Wait
for NPU readiness before starting terrain. The terrain command runs the
cooldown check before it launches the worker.

```bash
~/bin/halo-rover capture
~/bin/halo-rover temps
~/bin/halo-rover npu
~/bin/halo-rover demo-terrain
# Once terrain reports ready
~/bin/halo-rover demo
# Once the dashboard camera is ready
~/bin/halo-rover view
```

`demo` opens V58 scene 1 with the survey profile at 0.15 m/s. `view` opens a
ten-second inspection. After it closes, confirm the clear launch area on
the [Halo dashboard](http://127.0.0.1:8774) and choose **Start autonomous
exploration**. Keep the dashboard visible and let the planner begin its
own return. The simulator uses 8 Hz RGB-D.

The following commands open live views in the running scene. Use a
separate Halo terminal for each window.

```bash
~/bin/halo-rover rviz
~/bin/halo-rover gazebo
```

RViz opens the top-down navigation map. In Gazebo, open **Plugins →
View Angle** and select **View from the top**. Keep the dashboard
visible to maintain the operator heartbeat while the rover drives.

At the end of the run, press **Stop** on the dashboard and run the
following in a spare terminal. Keep capture and temperatures open
until finish completes.

```bash
~/bin/halo-rover stop
~/bin/halo-rover finish
```
