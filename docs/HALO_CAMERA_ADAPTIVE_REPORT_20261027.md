# Camera outage investigation and adaptive terrain — October 27, 2026

The camera outage was reproduced in Gazebo's RGB-D sensor output while the
rover was stationary. The first adaptive-terrain implementation is built on
Halo and source-tested. Switching the active session to Ogre eliminated the
sampled outages at the reproduced stationary failure pose. Driving validation
and live terrain-clearing acceptance are recorded separately below.

## Camera and odometry findings

At the failing scene-50000 pose, RGB periodically became the uniform sky color
while depth contained no finite samples. Evaluator-only simulator poses showed
the stationary rover had not turned toward the sky. During a 20-second capture,
17 of 34 sampled RGB images were uniform; 270 of 534 odometry messages had
invalid tracking covariance. Lost odometry used diagonal covariance values of
9999. The guard correctly latched `invalid_odometry` and zeroed motion.

This establishes a sensor-rendering problem rather than an explanation based
solely on browser image delivery. It does not identify the exact renderer or
driver defect. A [Gazebo upstream report](https://github.com/gazebosim/gz-sim/issues/2708)
describes similar intermittent Ogre2 images; it is supporting context, not proof
that the same defect caused this run. No drivers, firmware or runtime packages
were upgraded during this investigation.

The browser also removed its current image after any failed HTTP request or
decode. The new loop decodes a replacement before displaying it, retains the
last good image during an interruption, displays its stale age, retries, and
discards late frames from a previous scene. An interrupted image is not labeled
live. Guard fault evidence now includes the rejected odometry covariance.

## Rendering checks

`sensor_headless:=true` selects EGL rendering for the sensor while Gazebo and
RViz can remain visible. At launch, a 20-second EGL recording observed 598 RGB
frames, 599 odometry messages and no lost/invalid odometry. A separate stopped
pose-placement test at the earlier failure location still recorded three
uniform samples among 37 sampled RGB images and 46 invalid messages among 584
odometry messages. EGL alone therefore did not eliminate the fault. These
captures are not a controlled performance comparison or moving-rover accuracy
qualification.

The explicit `sensor_render_engine` launch option supports `ogre2` and `ogre`.
The default remains Ogre2. An Ogre comparison requires
`sensor_render_engine:=ogre sensor_headless:=false` and an active display.
The engine selection and EGL setting are saved in the session and preserved by
Restart. At the same failure pose, a 20-second Ogre recording observed 541 RGB
frames, 537 odometry messages, no invalid/lost odometry, no uniform images among
34 sampled images, and no sampled empty-depth output. This is evidence for a
workaround at that pose, rather than proof that every sensor outage is fixed.

| Stopped failure-pose check | RGB frames | Invalid odometry / total | Uniform sampled images |
| --- | ---: | ---: | ---: |
| Original Ogre2 session | 539 | 270 / 534 | 17 / 34 |
| Ogre2 with EGL | 579 | 46 / 584 | 3 / 37 |
| Ogre with display | 541 | 0 / 537 | 0 / 34 |

Image quality was sampled approximately twice per wall second; odometry counts
cover received messages. Depth-health snapshots are not an exact synchronized
RGB/depth evaluation. The short captures do not establish all-frame outage
rates, localization accuracy or comparative throughput.

The current Halo session explicitly selects Ogre with a visible desktop. Source
defaults remain Ogre2, so training and capture profiles are not silently
changed. The renderer can change image appearance; earlier detector accuracy
measurements do not automatically qualify images from this alternate profile.
The deployed NPU model and runtime were retained.

## Operator driving observation

A subsequent 60-wall-second read-only recording captured the operator driving
and turning, then stopping: 1,727 RGB frames, 1,811 odometry messages, no invalid
odometry, no lost-odometry reports, no uniform images among 109 sampled images,
and no sampled empty-depth output. The guard ended stopped with
`operator_stop`; no safety interruption was recorded during this window.
Evaluator-only truth moved as far as 5.21 m from the first recorded position.
Observed commands reached 0.30 m/s and 0.30 rad/s, so this was not the requested
0.05 m/s bounded acceptance trial. It does not qualify higher-speed operation,
localization accuracy or full collision handling.

No terrain cells cleared in that run. The new clearing check had interpreted
RTAB-Map's latest future-dated transform as stale. A read-only timing probe
measured latest stamps 0.048-0.100 s ahead; requesting the current simulation
time returned a bracketed transform. Clearing now requests that current-time
correction, as the object mapper already does. Missing current-time transforms
still suspend clearing while fresh local hazard output continues. RTAB-Map's
[broadcast implementation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/src/CoreWrapper.cpp)
includes this forward tolerance. The fix does not enlarge freshness limits.

## Adaptive terrain and verification

The [adaptive policy](ADAPTIVE_TERRAIN.md) requires three closer translated views
and dense safe ground. It protects established hazards, suspends clearing on
localization/depth failures, records evidence, updates both maps from the same
memory, and leaves safety stops latched. Detector labels cannot clear terrain.

- Windows navigation: 182 tests run successfully, with four real-ROCm cases
  skipped. The Halo Linux/ROS run had the same result.
- `rover_navigation` rebuilt successfully on Halo.
- The JavaScript speed-controls and camera-recovery harnesses passed.
- A separate Chrome profile verified the restored camera, speed defaults/caps,
  Reset and stopped state without sending control requests.

The stationary placement probe used simulator truth only to reproduce a known
failure pose. It did not send velocity commands or supply truth to navigation.
The scene was restarted afterward with launch clearance unconfirmed. The
renderer-switch helper refused a later restart when operator manual mapping
was active, preserving that running scene.

## Evidence and open acceptance

Raw observations, uniform-frame images, before/after status, pose-placement
configuration and browser captures are retained in ignored session directories:

- `navigation_sessions/adaptive-area-20261027/`: original sensor recordings,
  source backup/manifest and first Linux build/test log.
- `navigation_sessions/adaptive-resume-20261027/`: restored host-service logs,
  EGL/Ogre recordings, stationary pose probes, operator observations and browser
  evidence.
- `navigation_sessions/adaptive-renderer-20261027/`: renderer-option source
  backup/manifest and repeated Linux build/test log.
- `navigation_sessions/adaptive-current-tf-20261027/`: current-time correction
  fix, preserved pre-restart map/status, source backup and Linux build/test log.
- Every newly launched scene: `terrain-evidence-events.jsonl` and its saved
  launch configuration.

Live motion and annotated positive/negative terrain sequences are still needed
to qualify clearing. Source checks do not demonstrate that a particular red
area is physically safe. The fixed-pose Ogre check passed; broader driving and
terrain acceptance remain necessary.

## Final handoff

The corrected package was built and tested on Halo, then loaded in the fresh
Ogre session `restart-20261027T201435Z-289202`. Its final captured command was
zero, camera and localization were fresh, and mapping awaited explicit launch
confirmation. The initial odometry fault remains latched until explicit mode
selection after readiness recovers; the deployment did not rearm it.

The read-only Chrome check passed against this session: live camera, stopped
rover, 0.20 m/s and 0.30 rad/s defaults, 0.50 m/s and 0.60 rad/s widget limits,
increment/decrement and Reset, with no control POSTs. Refresh the operator page
at `http://127.0.0.1:8774`, inspect and confirm the launch area, then explicitly
enable manual drive. Use 0.05 m/s for the next terrain-evidence trial.

Raw evidence was copied back to the local checkout under
`navigation_sessions/adaptive-recovered-20261027/`, alongside
`navigation_sessions/adaptive-20261027-evidence.tar.gz`. The archive SHA-256 is
`0dc5948ae25b996f83ed8c485780599ae5e7b31d9b179710ecd719dea1263117`.
It contains the comparison recordings, configuration, source manifests/backups,
Linux build/test logs and preserved map/status. The final browser capture is
`navigation_sessions/adaptive-resume-20261027/browser-ogre-final-001.png`.
These artifacts remain ignored by Git; source and documentation were not pushed.
