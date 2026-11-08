# Halo dashboard recovery and manual speed controls — October 27, 2026

Scene 50000, the NPU detector, GPU terrain service and browser dashboard were
restored after the Halo reboot. The initial recovery scene was left stopped
with an empty map, waiting for operator launch-area confirmation. Historical
sessions were retained. The operator subsequently restarted the scene and
began mapping; the recovery observations below describe the recorded UI check.

## Operator behavior

| Control | Default | Range | Increment |
| --- | --- | --- | --- |
| Forward | 0.20 m/s | 0.01–0.50 m/s | 0.01 m/s |
| Turn magnitude | 0.30 rad/s | 0.05–0.60 rad/s | 0.05 rad/s |

Sliders and minus/plus buttons change the requested speeds. Reset, page reload
and scene Restart restore the defaults. A speed change releases held movement
and requires a fresh direction press; manual mode remains selected. Repeated
keyboard events cannot restart a released direction. Selecting speeds while
disarmed sends no control request.

The forward guard cap was raised from 0.20 to 0.50 m/s at the operator's request.
Input validation and the guard share the maximum values. Command leases,
readiness checks, terrain thresholds, footprint sweeps and command ownership
are unchanged. Autonomous launch speed remains 0.10 m/s. These checks do not
qualify driving at the new maximum speed.

## Verification and deployment

- Windows navigation suite: 167 tests run, successful with four actual-ROCm
  tests skipped.
- Halo Linux/ROS navigation suite: 167 tests run, successful with the same four
  skips. The `rover_navigation` package rebuilt successfully against the
  existing installed dependencies.
- The JavaScript harness passed checks for defaults, limits, held-input
  release, retained manual selection, repeated keys, Reset, Stop and Restart.
- A separate real Chrome profile verified the restored page, fresh annotated
  camera, adjustments, caps and Reset. No control POST requests were sent.
  The rover remained stopped with zero command.

Runtime changes were copied selectively into the existing Halo navigation
workspace after baseline hash checks; previous files were backed up. The local
source changes are in `AMD-Powered-Physical-AI-Robotics`. Other local worktrees
were preserved. NPU execution proof and the ROCm terrain backend were checked
without changing their environments or models.

Recovery session: `restart-20261027T184803Z-6f31cd`.
Evidence directory on Halo:
`navigation_sessions/dashboard-recovery-20261027/`, including the build/test log,
source manifest and pre-change backups. The local directory of the same name
contains `speed-browser-001.json` and `speed-browser-001.png`. Session evidence
is Git-ignored. The historical driving browser helper now explicitly selects
0.05 m/s and 0.30 rad/s; its motion trials were not rerun during this recovery.

## Adaptive terrain audit

`TerrainMemory.remember()` still retains hazards after distinct depth-frame
confirmations. Later free observations and launch seeding cannot clear a
confirmed hazard. The SLAM-cloud terrain layers remain diagnostic; red cells
come from depth geometry independently of semantic detector boxes.

Revisable terrain evidence is still planned in [Phase Four](ROADMAP.md).
Better independent observations would need to positively demonstrate safe
supporting ground before reconsidering weak hazards. Missing depth, absent
detections or elapsed time must not clear terrain or rearm a stopped guard.

Open the restored dashboard at `http://127.0.0.1:8774` through the active local
SSH forward. Inspect the launch pad in Gazebo, select **Confirm launch area &
start mapping**, wait for readiness, then select **Enable manual drive**.
