# Phase Two: camera coverage and reliable manual mapping

Phase Two measures what the current camera can observe near the rover, compares
navigation-only camera pitches, and validates guarded manual mapping. Camera
angles are evaluated before changing the launch default. The 1.1 m operator
clearance disk, 12 degree slope limit, 3 cm height-span limit, hazard retention,
unknown-space blocking and all watchdogs retain their settings.

## Coverage visible to the operator

The mapping-health panel reports the first fully observed strip beyond the
padded front and the observed sample fraction in a 2.5 m forward corridor.
**Show fresh camera coverage** draws this corridor over the map:

- Teal strips contain only freshly measured traversable samples.
- Red strips include a measured hazard.
- Hatched strips include unobserved ground.

The corridor spans the padded rover width. It samples at half the terrain-cell
resolution and uses only the current, timestamp-aligned depth classification
before memory fusion or launch seeding. It can therefore show unseen ground
inside a pale launch disk. It does not represent a path, clearance authorization
or stopping distance. The existing guard still checks the full retained terrain
and its motion sweep. Old coverage is hidden when the depth stamp or mapping
receipt is stale. The corridor moves through the same current map correction
as the rover and fault overlays.

Space invokes Stop even when a diagnostic checkbox has keyboard focus. The page
continues to require an explicit manual selection after a latched fault.
The camera-detail indicator samples image brightness variation twice per second.
It highlights nearly uniform views as a clue when visual tracking is unavailable;
it is not a confidence estimate and never authorizes movement. Malformed image
inputs cannot crash the controls through this optional diagnostic.

## Reproducible camera comparisons

`camera_pitch:=0.4`, `0.6` and `0.8` specify radians downward in navigation
sessions. Each session records the actual camera parameters and a predicted
flat-ground near/far interval in `session.json`. The prediction uses the camera
height, aspect ratio, configured horizontal field of view and optical-depth
clip planes. Actual depth coverage and localization determine acceptance; the
flat-plane prediction does not mark terrain as observed. The configured field
of view remains provisional for a future physical OAK-D calibration.

Use a fresh session for each profile and keep scene, resolution, camera rate,
runtime and motion command identical. Earlier sessions retain their recorded
pitch on Restart. Capture a stationary baseline before any movement, and retain
failed runs separately. The existing terrain audit saves synchronized camera,
depth, calibration, terrain and pose evidence.

Inside the container, after sourcing ROS and the navigation install:

```bash
python3 scripts/halo_navigation_coverage_check.py \
  --session navigation_sessions/NEW_SESSION \
  --output navigation_sessions/NEW_SESSION/checks/coverage-001.json \
  --settle 30 --drive-seconds 20 --confirm-clear-launch
```

This command authorizes a bounded simulation trial at 0.05 m/s through the
existing manual HTTP API. It never publishes `/cmd_vel`, a ROS manual command or
a navigation goal directly. Use the confirmation option only for an inspected
clear launch area in an allowed fixture or scene 50000. `--drive-seconds 0`
performs a stationary observation. Settling is bounded to 5–45 wall seconds;
driving to 0–30 seconds. The rover receives an explicit Stop in cleanup.

`--expect-known-edge` is restricted to the existing `flat_edge` fixture. The
independent evaluator aborts before truth x exceeds 1.28 m if the guard has not
already stopped. This test limit is not a navigation input or a steering rule.
The physical steep-ramp fixture should be observed without clearance seeding:
its ramp intersects the fixed 1.1 m disk. The helper rejects automated launch
confirmation for that fixture.

The report contains raw status/pose events, camera configuration, initial/final
coverage, independently observed displacement and pose disagreement. Relative
position is compared in each start pose's body axes to account for different
coordinate origins and initial headings. The short-trial comparison limits are
0.10 m and 5 degrees; these are evaluator criteria, not changes to the guard.
Failures, odometry loss and interrupted movement remain failed trial results.
Truth is consumed only by this evaluator.

The UI check supports `--coverage` alongside its existing `--diagnostics` and
bounded `--drive` checks. With driving enabled it also verifies Space-to-Stop
while the coverage checkbox has focus.

The host page uses persistent HTTP connections through SSH. The browser sends
the next heartbeat immediately when an earlier round trip consumed its 50 ms
period, keeping only one request outstanding. This avoids adding another timer
wait on a slower link. The input, guard and wheel-command watchdogs keep their
existing durations; a link that exceeds their limits still stops control.
Rejected HTTP requests close their connection so unread bodies cannot become
subsequent requests.
