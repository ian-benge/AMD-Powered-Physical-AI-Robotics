# Phase Two coverage and manual mapping — October 21, 2026

Fresh-depth coverage diagnostics and the manual-control transport improvements
are implemented and deployed. **Camera-profile promotion remains unaccepted.**
The default stays at **0.4 radians**: larger downward pitches exposed nearer
ground when tracking was available, but failed localization checks in this
runtime. Edge drift and steep-ramp initialization remain open gates.

## Implementation

The page adds **Show fresh camera coverage**, a sampled forward corridor showing
current clear, hazardous and unobserved strips independently of retained maps
and the operator-confirmed launch disk. The health panel reports where the first
fully observed strip begins beyond the padded rover front. Stale coverage is
hidden, map corrections move the overlay consistently, and Restart clears the
old session's health/coverage labels. Nearly uniform RGB views receive a
diagnostic low-detail indication; it is not a tracking confidence or safety input.

Camera pitches are validated before creating a session, and session metadata
records the actual camera configuration and an explicitly labelled flat-ground
coverage prediction. The new bounded independent evaluator records raw truth,
visual odometry, guard state, coverage and relative pose disagreement. Navigation
never consumes truth. [Operator and evaluation guide](NAVIGATION_PHASE_TWO.md).

The browser test found a separate control-link problem: HTTP requests took
150–200 ms over the SSH forward, after which the heartbeat could wait another
timer tick and expire the existing 0.25 s input lease. The host now reuses HTTP
connections, and an overdue heartbeat runs immediately after the prior response.
Only one control request remains outstanding. Rejected requests close their
connections to isolate unread bodies. The 0.25 s input lease, 0.35 s guard/wheel
leases and all sensor/clock limits retain their values. Space invokes Stop even
when a diagnostic checkbox has focus.

## Recorded tests

Source regressions passed **124 tests on Windows and 124 on Halo**: 121 navigation,
one detector-service and two camera tests. The navigation package built on Halo.
Tests cover raw-frame coverage, tiny unknown holes, hazard preservation, map
corrections, pitch validation, independent pose scoring, optional diagnostic
failure isolation, connection reuse and rejected-request isolation.

Camera comparisons used fresh headless sessions, identical 960×540 RGB-D settings,
the existing host GPU detector and guarded 0.05 m/s HTTP input. Flat runs settled
for 15 wall seconds and drove for up to 25; Martian runs settled for 30 seconds
and drove for up to 20. Their short-trial evaluator limits were 0.10 m relative
endpoint disagreement and 5 degrees relative yaw disagreement. These limits
apply to evaluation, not the motion guard.

| Scene/profile | Recorded outcome | Interpretation |
| --- | --- | --- |
| Flat, 0.4 rad | 1.248 m truth displacement; 0.489 m relative endpoint disagreement | Failed localization comparison |
| Flat, 0.6 rad | No fresh valid visual odometry after settling | Failed startup; no drive |
| Flat, 0.8 rad | Tracking loss after about 0.005 m | Failed; guard stopped |
| Martian 50000, 0.4 rad | 1.002 m truth displacement; **0.00182 m** endpoint disagreement; 0.014° relative yaw disagreement | Passed this bounded straight trial |
| Martian 50000, 0.5 rad | No fresh valid visual odometry after settling | Failed startup; no drive |
| Martian 50000, 0.6 rad | No fresh valid visual odometry after settling | Failed startup; no drive |
| Martian 50000, 0.8 rad | Tracking loss after about 0.020 m | Failed; guard stopped |
| Physical edge, 0.4 rad | `unsafe_or_unobserved_ground` stop after 1.228 m; 0.215 m relative endpoint disagreement | Guard boundary stop recorded; localization gate failed |
| Physical ramp, 0.4 rad | No fresh valid visual odometry after 30 s; no clearance seeding or movement | Startup gate still failed |

At 0.4 rad, the first fully observed strip was typically **0.75–0.80 m beyond
the padded front**, with about 64–66% of the sampled 2.5 m corridor observed.
During valid 0.8 rad observations that first strip was about 0.30 m beyond the
front, but total corridor observation was only about 55% and tracking failed.
Near-ground visibility alone is therefore insufficient to choose a camera angle.
Saved failed 0.6 rad views were nearly uniform; their rendered inputs and visual
features need investigation before changing odometry settings. This is an
observation, not an established root cause of every failed run.

The final desktop session ran Gazebo and RViz visibly at 0.4 rad. Its ten-second
stationary capture delivered global terrain at 4.96 Hz and local ground at
5.05 Hz, with maximum observed gaps of 0.281 s and 0.285 s. RGB/depth delivery
was approximately 30 Hz. These short captures do not establish sustained
timing, inference-performance or hardware qualification.

One browser run before the transport fix lost its input lease and is retained
as a failure. **Two consecutive browser runs after the fix passed** repeated
forward/turn commands, release-to-idle, explicit Stop, network-loss stopping,
fresh-coverage rendering and Space-to-Stop with the checkbox focused. The
measured run recorded 108 successful drive requests, p95 round trip **82.4 ms**
and maximum **131.5 ms**. Network conditions can still exceed the unchanged
leases and correctly stop control.

## Evidence and operator state

Evidence is retained in new ignored session directories on Halo and copied into
the local checkout: **237 evidence/configuration files**, with all **56 recorded
source hashes matching** between Windows and Halo. Old populated maps, the operator's pre-change session
`restart-20261021T140148Z-84dadf`, source assets and unrelated worktrees were
preserved. Source changes remain on `navigation/mapping-and-exploration`.

- [All nine profile/fixture comparisons](../evidence/phase2-validation-20261021/comparison-summary.json)
- [Passing Martian 1 m trial](../evidence/phase2-mars-p04-20261021/checks/coverage-001.json)
- [Its synchronized sensor/terrain audit](../evidence/phase2-mars-p04-20261021/checks/audit-001.json)
- [Edge trial and disagreement](../evidence/phase2-edge-p04-20261021/checks/coverage-001.json)
- [Ramp startup failure](../evidence/phase2-slope-p04-20261021/checks/coverage-001.json)
- [Final GUI-session timing](../evidence/phase2-mars-final-20261021/checks/audit-001.json)
- [Pre-fix browser failure](../evidence/phase2-validation-20261021/browser-final-drive-001-failure.json)
- [First post-fix UI pass](../evidence/phase2-validation-20261021/browser-final-drive-002.json)
- [Repeated UI pass and round-trip timing](../evidence/phase2-validation-20261021/browser-final-drive-003.json)
- [Coverage UI screenshot](../evidence/phase2-validation-20261021/browser-final-drive-003.png)
- [Restart regression](../evidence/phase2-validation-20261021/browser-restart-004.json)
- [Fresh-session coverage view](../evidence/phase2-validation-20261021/browser-fresh-006.png)
- [Final live status](../evidence/phase2-validation-20261021/final-status.json)
- [Halo build/navigation checks](../evidence/phase2-validation-20261021/final-build-tests-002.log)

Each test trial ended with explicit Stop. Scene 50000 remains running with the
baseline camera; the live page subsequently had manual mode selected and a zero
command, so that selection was preserved. Refresh the existing page and select
**Show fresh camera coverage**. The final status record identifies the session
and current mode. Arms, sampling, target tracking quality,
exploration, autonomous waypoint acceptance and physical hardware were outside
this implementation.

Further promotion requires diagnosing the uniform/failed steeper-camera views,
reducing the measured edge/flat localization disagreement and establishing
repeatable ramp startup. Keep the existing safety thresholds while doing so.
