# Navigation implementation verification — October 19, 2026

Branch: `navigation/mapping-and-exploration`, in `AMD-Capstone-navigation`,
based on `npu-testing` at `14d616ea`. The original checkout remains unchanged.

## Implemented

- Separate RGB-D navigation camera and isolated session/world output; source
  capture/live profiles, scene bundles and localization databases are preserved.
- RTAB-Map RGB-D odometry/SLAM and persistent database configuration; no
  ground-truth odometry relay in the navigation graph.
- Local/global terrain occupancy grids plus GridMap elevation, slope, vertical
  span, roughness and observation-count layers. Global geometry follows the
  optimized SLAM cloud. Operator-confirmed launch clearance is anchored in map
  coordinates and reprojected into odom after loop corrections.
- Nav2 differential State Lattice and Regulated Pure Pursuit configuration,
  explicit manual/autonomous modes, latched safety faults and independent wall
  watchdogs at the motion guard and wheel-command adapter.
- Waypoint, exploration and return services, return-time reserve, independent
  trajectory/contact recording and an offline run summarizer.
- Physical slope, step, drop-off, narrow-gap and blocked-return fixtures;
  chassis contact instrumentation and optional read-only camera preview.

## Local verification

- **40 navigation tests passed**, including depth units/stride/endianness,
  unsupported ground, slope/height hazards, full/swept footprint checks,
  reachable frontiers and viewing direction, return budgeting, mode takeover,
  stale sensors/commands, clock pause/reset, tracking-loss covariance,
  contact stops, downstream watchdog, map-anchor corrections and run metrics.
- The navigation launch tests expand the real Xacro with controlled launch
  doubles and check sensor profiles, isolated output, database copies, command
  ownership and absence of the ground-truth relay. ROS adapter checks use
  controlled message/TF doubles; they do not execute ROS middleware.
- **151 existing perception/live/NPU Python tests passed.** The existing browser
  control JavaScript harness passed. No real NPU inference was run for this change.
- The broader environment/training run passed **152 tests** and hit one
  Windows `PermissionError` in unchanged concurrent review-page replacement code.
  That test passed separately on the original checkout and on the navigation
  checkout. This platform-specific flaky result was not repaired in this branch.
- The navigation Python wheel builds, the Bash launcher passes syntax checks,
  runtime modules parse, and `git diff --check` passes. These are source/package
  checks, not a successful `colcon` build against installed ROS dependencies.

## Halo startup correction - October 20, 2026

The operator reports the flat-world camera preview working on the Halo and
the ROS graph containing terrain, mission, RTAB-Map, Nav2 and wheel-controller
nodes. However, `/navigation_motion_guard` is absent and `/navigation/status`
does not publish. Source inspection identified an import of `qos_profile_clock`,
which is absent from Jazzy's `rclpy.qos` API. This prevents the guard from loading.

The guard now constructs its clock QoS explicitly (depth 1, best-effort
reliability). The adapter tests limit QoS exports to the supported Jazzy names
and exercise guard construction, so nonexistent imports are no longer silently
accepted by an unrestricted mock. The stricter test reproduced the import error
before the fix. After the fix, **41 navigation tests and 36 subtests pass**.
The operator subsequently rebuilt on the Halo and confirmed `/navigation/status`
publishes stopped mode with a zero command. That verifies corrected guard startup;
no motion test has passed. A latched `invalid_odometry` fault was reported, while
the later captured odometry sample satisfies the guard's current validation.

## Halo depth/TF timing correction - October 20, 2026

The operator measured raw depth delivery at roughly 25-28 Hz (maximum observed
gap 0.230 seconds), while local-ground delivery was roughly 1.5-2.1 Hz with gaps
up to 2.383 seconds. Status includes accepted-ground ages above two seconds and
`ready: false` despite fresh pose/clock data. The terrain logs repeatedly show
future extrapolation: depth stamps were 66-198 ms ahead of available odometry
TF. RGB-D odometry also reported RGB/depth pairs separated by 32-34 ms.

The terrain adapter now retains up to 20 depth/CameraInfo pairs and selects the
newest fresh pair with camera and base transforms at the image's own timestamp.
Lookup is nonblocking, so the same executor can receive TF between retries.
New camera arrivals no longer discard every frame that was waiting for TF.
The existing 0.6-second simulation freshness bound and one-second receipt bound
remain; processed/expired frames cannot be republished as fresh measurements.
Clock rewind clears pending history and operator clearance. Approximate-sync
intervals are limited to 15 ms in odometry, SLAM and terrain calibration pairing
to reject adjacent 30 Hz frames.

Terrain counts/sums/squares use `bincount` in place of repeated indexed updates;
minima, maxima, roughness, unknown cells and hazard thresholds are retained.
One Windows synthetic 3,456,000-point local-grid check improved from 1.064 to
0.730 seconds. This is not a Halo or full-pipeline latency measurement.

**49 navigation tests and 39 subtests pass locally.** New checks cover a delayed
newest frame with an older transformable frame, retry without another image,
missing base transforms, expired/processed frames, bounded buffering, future
clock stamps, clock rewind and repeated-point terrain statistics. Real TF
transport and corrected end-to-end grid timing still require the Halo retest.

## Remaining acceptance

Docker has no running engine in this Windows session; ROS/Gazebo are unavailable
locally. Direct SSH testing on the Halo subsequently completed the first bounded
manual and planner checks. See the [measured Halo smoke report](HALO_NAVIGATION_SMOKE_REPORT_20261020.md)
for results, the contact-topic correction, raw evidence and limits. Five packages
built on the Halo and 51 source checks passed there. Manual forward movement,
explicit stop, command-lease stop and stationary clock-pause/resume passed.
A read-only planner request also passed; it did not drive the path.

Physical collision sensing, autonomous waypoint/return missions, map reload,
terrain fixtures, exploration and sustained timing acceptance remain open. The
earlier timing-fix section describes observations before this direct smoke run;
the smoke report records the newer evidence without treating it as full acceptance.

Follow [NAVIGATION.md](NAVIGATION.md) for build/run commands and the ordered
acceptance gates. Require the three-scene repeated waypoint/return trials,
fault/terrain fixtures, map reload/localization, exploration and full-stack
resource measurements before treating this as a validated autonomous rover.
Chassis-contact logging alone does not establish zero wheel/obstacle collisions.
Hardware operation needs its own measured sensor, clock and actuator adapters.
