# Halo manual mapping and detections — October 20, 2026

The requested manual page, live trained detection and persistent map-marker
workflow ran on Martian scene 50000. This establishes a bounded simulation
integration check. It does not establish detector accuracy, collection-pose
accuracy, hardware readiness or the separate autonomous waypoint milestone.

Source is the uncommitted `navigation/mapping-and-exploration` checkout. The
[operator guide](HALO_MANUAL_MAPPING_DETECTIONS.md) contains exact launch,
viewing and test commands. Historical navigation evidence remains intact.

## Implemented behavior

- Localhost port 8774 now provides guarded hold-to-drive input, explicit manual
  selection, confirmation, Stop, live annotated camera and a terrain/candidate map.
  Inputs are bounded to 0.05 m/s and 0.3 rad/s with expiring sequence tokens.
  Release stops publication; the independent guard retains its 0.35 s lease.
- Optional `rock_detector` inference uses the existing host `rover-yolo` GPU
  environment through loopback port 8775. Actual predictor execution was verified
  on `cuda:0`, AMD Radeon 8060S. No ROS-container GPU stack or NPU change was needed.
- The four-class checkpoint SHA-256 is
  `89bbcf4ff124996fb6a520c5bf751287c2b5ea47bbd1d7e2adaf7f530ee44ca3`.
  Model identity and class contracts are checked by the ROS client.
- Image-time depth and optical-frame transforms place tracks in odom; current
  map corrections reproject them. Tracks merge by class within 0.3 m, persist
  out of view, publish durable RViz markers and save to `sample-sites.json`.
  Detection positions never enter the traversability map or Nav2 goals.

## Recorded checks

| Check | Result |
| --- | --- |
| `manual-flat-01`, one-second page-input forward interval | Passed: 0.0650 m truth displacement; first zero guard command 0.3134 wall seconds after release; latched `command_timeout` |
| Actual browser key hold/release, flat and Martian scene | Passed: observed manual command, then stopped/ready with `command_timeout`; explicit re-selection required |
| `manual-edge-03`, physical platform ending at x=1.5 m | Passed: truth displacement 1.2450 m; guard latched `unsafe_or_unobserved_ground`, final command zero, no reported chassis contact |
| Scene 50000 stationary detection/map observer | Passed: matching RGB/depth/CameraInfo/detection/annotated stamps, durable markers and guard-only `/cmd_vel` ownership |
| Scene 50000 turn/persistence observer | Passed: 378 aligned detection frames in 15.008 wall seconds; rock, yellow-crystal and slab predictions; earlier candidates retained |
| Out-of-view retention | Candidate #2 stayed at 3,246 observations and stamp 109.132 while new detection frames reached 122.432; its marker remained present |
| Detector-service disconnect | Passed: detector unavailable, camera explicitly raw (`X-Annotated: false`), candidates retained and command remained zero; dedicated service restarted |
| Halo package build | All five packages built |
| Linux source checks | 94 tests passed in navigation and detector-service suites; final deployment checks are saved with its session |
| Broader Windows run | 397 tests and 137 subtests passed; one unrelated review-page concurrent-file-replace test failed with WinError 5 and passed on focused repeat (8 tests, 7 subtests) |
| Final deployment regression | 96 tests passed on Linux; 96 tests and 49 subtests passed on Windows |

The turn recording delivered approximately 25 aligned detection frames per wall
second with Gazebo/RViz running. This is a short integration observation, not an
inference benchmark or sustained performance qualification.

## Failures and limits

`manual-edge-01` lost visual odometry shortly after moving on a feature-poor
platform. `manual-edge-02` initially stopped before motion because DDS had not
yet resolved the command publisher's node name; a bounded graph-discovery wait
was added to the observer. Its repeat stopped at unknown ground but had a large
pose error and only 0.097 m truth displacement, so it remained a failed trial.
Adding distinctive near-ground visual features to the physical fixture produced
the `manual-edge-03` pass. No guard threshold or launch-clearance limit changed.

The flat checkerboard produced false-positive classifications, including
high-confidence slab predictions. In the Martian turn, some objects produced
both rock and yellow-crystal candidates because merging is intentionally
class-specific. A one-frame classification can persist as a candidate; no
confidence or repeated-observation qualification is claimed for sample selection.

Offline nearest-scene-object comparisons for the first three rock markers were
approximately 6.3 cm, 3.7 cm and 3.2 cm in the horizontal plane. These are
surface-point estimates compared with object origins, not one-to-one scoring
or a localization accuracy result. Scene positions were used only in this
separate saved evaluation, never by the detector/mapper/guard.

Loop-correction reprojection and rejection of missing image-time transforms
were exercised in focused tests. A real loop closure, map reload, long mapping
drive and physical OAK-D calibration remain future validation. The original
motion watchdogs, no-reverse rule, slope/step limits and 1.1 m clearance disk
remain unchanged. Absence of reported chassis contact is not a claim of
complete wheel/rock collision instrumentation.

## Evidence

New session directories contain JSON reports, raw JSONL and captures. Copies
are retained in this local navigation checkout; session data remain Git-ignored.

- [Flat movement and release](../evidence/manual-flat-01/checks/manual-lease-001.json)
- [Flat browser capture](../evidence/manual-flat-01/checks/browser-001.png)
- [Failed first edge trial](../evidence/manual-edge-01/checks/known-edge-001.json)
- [Failed second edge repeat](../evidence/manual-edge-02/checks/known-edge-002.json)
- [Passing physical edge trial](../evidence/manual-edge-03/checks/known-edge-001.json)
- [Stationary Martian detector evidence](../evidence/manual-mars-01/checks/stationary-001.json)
- [Martian browser capture](../evidence/manual-mars-01/checks/browser-001.png)
- [Turn and marker persistence](../evidence/manual-mars-01/checks/turn-persistence-001.json)
- [Detector disconnect](../evidence/manual-mars-01/checks/detector-disconnect-001.json)
- [Offline nearest-object evaluation](../evidence/manual-mars-01/checks/offline-nearest-objects.json)
- [Linux source tests](../evidence/manual-mars-01/checks/linux-source-tests.txt)

The final deployed session is `manual-mars-02`. It is left stopped; its
post-deployment check and final status are saved in that session's `checks/`.
The dedicated detector remains available on loopback port 8775.

- [Final deployment alignment check](../evidence/manual-mars-02/checks/post-deploy-001.json)
- [Final deployment source tests](../evidence/manual-mars-02/checks/linux-source-tests.txt)
- [Final stopped status](../evidence/manual-mars-02/checks/final-status.json)

## Manual-selection correction after operator feedback

The first page revoked its input token on every key release and relied on
command-lease expiry to stop. This also left manual mode, requiring repeated
selection. The page now sends zero velocity when drive keys are released and
keeps its manual session active between commands. An active page sends zero
commands while idle. Stop, lost control, and safety faults still revoke control;
the independent guard and all of its timeouts remain unchanged.

Only the control service was rebuilt/restarted. Gazebo, odometry, SLAM, terrain
memory and the user's accumulated candidate list stayed in the same session.
The updated browser check covers three command intervals separated by idle
pauses longer than the lease, explicit Stop, and lost connection while moving.
It passed on the existing Martian session: each idle pause retained manual mode
at zero command, explicit Stop left manual mode, and a disconnected moving
browser latched `command_timeout`. The 34 accumulated candidates remained.
The package rebuild and 96 Windows source tests (49 subtests) also passed.
The evidence is saved as
[manual-persistence-001](../evidence/manual-mars-02/checks/manual-persistence-001.json).

## Scene restart and readiness feedback

The next operator stop was `unsafe_or_unobserved_ground`: the forward sweep
reached an unknown cell near the launch-clearance boundary. The generic manual
readiness error obscured it. The page/API now display that actual motion rejection
and keep the independent terrain limits unchanged.

Added Restart scene with a host gateway that remains available when ROS stops.
Each restart uses a new session and database, resets the rover/world, and retains
older session evidence. Terrain/candidate mapping waits for explicit launch-area
confirmation, giving a genuinely empty navigation map on restart. No old motion,
clearance, home or detections are restored.

The initial restart tests found an empty fixture launch argument and Docker's
image/container name ambiguity when no container remained. Both were fixed;
container inspection is now explicit, and restart errors remain visible on the
host page. Tests verify checkout ownership, duplicate rejection, fresh database
selection, blank-map gating and page availability while ROS is down.

The restart-and-drive recording verified zero known cells, zero candidates,
successful confirmation, three manual command intervals, explicit Stop and a
latched connection-loss stop. A final button restart leaves mapping paused and
empty for the operator. The expanded source suite passes 102 tests (51 Windows
subtests), with Linux results saved in the final session.

- [Restart and manual-drive evidence](../evidence/manual-mars-02/checks/restart-and-drive-002.json)
- [Final button restart](../evidence/manual-mars-02/checks/restart-final-001.json)

Final session: `restart-20261021T021310Z-1c77fc`. Read-only truth evaluation
confirmed the rover returned to x/y zero, and only the motion guard publishes
`/cmd_vel`. The terrain map and candidates remained empty while waiting for
confirmation. The gateway and dedicated GPU detector remain available.
