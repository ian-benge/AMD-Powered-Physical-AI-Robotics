# Phase 3 mapping and adaptive-clearing validation

November 5, 2026. **Accepted for bounded mapping and adaptive-clearing validation.**
Controlled encoded-depth replay, all ten actual ROS cases on both CPU and ROCm,
the attended rendered-depth review and the guard review passed. Protected
manual-session closure is verified, with the final monitoring gap below.
Phase 3 authorization does not extend to dataset expansion or retraining.

## Retained policy

The ordinary hazard limits remain **12 degrees and 3 cm**. Weak retained hazards
require three independent translated views, at least 0.20 m apart and spanning
two simulated seconds. Every view must be within 3 m, at least 0.25 m and 20%
closer than the hazard observation, with dense measured safe ground throughout
the surrounding 10 cm neighborhood. The stricter clearing bounds remain
8 degrees, 1.5 cm span/elevation agreement and three samples per cell.

Unknown ground, missing depth, elapsed time and detector labels cannot clear
terrain. Established hazards stay protected. Uncertain/stale localization or
missing map correction suspends clearing. A clearing decision never selects a
control mode or releases a latched safety stop.

## Implementation and audit changes

- Expose weak, pending and protected hazard counts and supporting-view progress.
  The dashboard distinguishes insufficient measured ground in the complete
  predicted turn sweep from measured slope/height hazards.
- Record input depth hashes, camera-to-odometry and actual projection transforms,
  thresholds and hashes of both successfully published planning maps. Qualified
  clearing views retain measured range/elevation evidence.
- Extend synchronized audit bundles with local cause grids and mapping status.
- Evaluate saved terrain using its actual anti-diagonal triangles. Retain fixed
  world-orientation residuals and bias. Alternative orientations remain diagnostic
  results and cannot conceal a registration error by choosing the best fit.
- Reuse the retained bounded supervisor for a separate Phase 3 candidate and
  automatic read-only RGB-D/map recording. The accepted Phase 2 tree is preserved.

No clearing decision rule or threshold was relaxed by these changes.

## Controlled optical-depth evidence

`scripts/phase3_validate_mapping.py` intersects calibrated optical rays with
analytic planes, a 6-degree slope, a 6 cm box step, a sphere and a ridge. Only
encoded depth, calibration and camera transforms enter the production CPU mapper.
Geometry labels remain evaluator-only. The script sends no ROS or motion commands.

Local evidence:
`navigation_sessions/phase3-mapping-20261105/controlled-replay-v3/result.json`.
All **10 cases passed**:

| Case | Result |
| --- | --- |
| Flat ground and 6-degree slope | No frame hazards |
| Step, rock and ridge | Retained hazards with protected evidence |
| Weak false hazard on flat ground | 24 cells cleared after three qualified views |
| Same inputs with missing depth | No clearing; pending evidence reset |
| Same inputs with uncertain localization | No clearing; pending evidence reset |
| Same inputs with missing map correction | No clearing; pending evidence reset |
| Stationary repeated views | No clearing |

The weak false hazard is an explicitly injected 6% short-return patch. It is a
controlled measurement-error case, not a claim that a natural sensor defect was
reproduced. In the earlier sparse fixture, dense-support views occurred at
camera x=0.65, 0.90 and 1.10 m. The earlier x=0.40 m view lacked sufficient dense
support and was not counted; the first fixture therefore stopped at two
qualified views. Adding a third qualified viewpoint completed that test
without changing the policy.
The final fixture supplies continuous 5 Hz depth between viewpoints, preserving
the production 0.6-second depth-freshness requirement. The 24 cells clear in
two batches at simulated times 4.2 and 4.8 seconds.
The continuous fixture records the corresponding independent camera positions
as approximately (0.45, 0.65, 0.94) m and (0.55, 0.75, 1.06) m.

Both map projections updated after clearing, and the source safety gate stayed
latched stopped. Raw depth, map/layer arrays, decisions and input hashes are
retained, including the initial incomplete fixture and density probe.

## Actual ROS publication and ROCm parity

Candidate and evidence root on Halo:
`/home/ian/capstone-training/rover-navigation/navigation_sessions/mapping-phase3-20261105T162405Z`.
The current separate candidate is `application-v5`; the accepted Phase 2
application and image aliases remain untouched.

- All ten CPU and all ten ROCm ROS replay cases passed. Each case has 21 input
  records; the missing-depth case correctly has only 15 current-stamp map pairs.
- All **408 published local/global cost arrays** match exactly between CPU and
  ROCm. These NPZ records contain cost grids; synchronized live captures provide
  the separate cause-grid consistency check.
- The weak artifact clears from both actual planning maps. Step, rock and ridge
  hazards remain protected. Missing depth, uncertain localization, missing map
  correction and stationary views produce no clearing.
- ROS domain 219 and `/phase3/bench_cmd_vel` isolate the replay. The actual guard
  remains stopped and every recorded output is zero. No mode or motion request
  is issued. The simulator is absent from these controlled cases.
- ROCm retains float64 geometry and requested/observed HIP scheduling flags 4.
  The unchanged functional parity check also passed in the current capture.

Earlier replay attempts are preserved. Their failures exposed fixture defects:
an absent initial TF bracket, a remapped graph-query assumption, sparse depth
that correctly triggered freshness suspension, and a dropped covariance bracket
in the best-effort test publisher. The final publisher confirms receipt of both
covariance samples before advancing simulated time. Production safety rules
were not weakened to accommodate the fixture.

The Linux build passed all five packages. Linux source suites passed navigation
(199 tests, four skips), environment (164 tests) and perception (157 tests).
All three representative ROS launch setups passed inspection without executing
nodes. Windows retains the unrelated dataset-review concurrent file-replacement
failure (`WinError 5`); the same environment suite passed on Linux.

The final source archive SHA-256 is
`2be9f57854cae6660846a6b561fb7ad1c5ef4402006ef76cf68ffb537a777da8`.
The candidate configuration pins 1,860 files and has SHA-256
`f5520b7880ba402b6bd6e606b65214463eac258245df4f8c48437e945a8e49e2`.
`CPU_ROCM_REPLAY_REVIEW.json`, its scope note and `FINAL_PREFLIGHT.txt` retain
the comparison and readiness evidence.

## Independent rendered-depth evaluator

The evaluator reconstructs measured world points using image-time brackets
from `/odom_gt` and the static camera extrinsic. Ground truth remains confined
to evaluation. It compares against the saved terrain's exact triangles without
fitting away a height or orientation offset. A single first-pose alignment is
fixed throughout the separate VIO-drift diagnostic.

Local metrology evidence under
`navigation_sessions/phase3-mapping-20261105/rendered-review-metrology-v1`
includes a calibrated flat positive case, an exact triangle saddle check and
a deliberately biased 3 cm world-pose case. The positive passes and the biased
case fails; separate immutable input variants preserve reproducibility.

## Attended rendered-depth review

Representative 3 (seed **2601104**) ran in the separate Phase 3 candidate.
The operator completed the requested manual ground/turn/rock-passage check and
reported no defect. Supervisor
`application-v5/supervisor-sessions/phase3-control-20261105T171347Z`
completed renderer provenance, synchronized recording and cleanup without an
error. The corresponding navigation session is
`restart-20261105T171347Z-09ba6d`.

All **107 exact-stamp captures**, spanning **119.196 simulated seconds**, passed:

- Raw encoded depth hashes match the successfully published mapper inputs.
- Both published map hashes match the captured grids, and both cause grids
  agree with their hazard cells.
- Replaying every actual depth frame through the CPU mapper reproduces the
  ROCm current-frame classification exactly.
- All 107 captures have valid image-time world-pose brackets. Measured background
  ground agrees with the saved terrain triangles: the worst per-frame 95th
  percentile absolute error is **4.242 mm**, below the retained **15 mm** review
  bound. This excludes conservative rock envelopes and the rover footprint.
- With one fixed initial alignment, the maximum sampled VIO diagnostic error is
  **6.514 mm** in position and **0.155 degrees** in orientation. This short
  simulator recording does not establish general VIO accuracy.

The saved six-frame camera contact sheet shows textured terrain, rocks and pale
patches without a sampled uniform/blank-camera defect. The live source trace
contains 627 successful terrain publications and no clearing event. Natural
false-hazard clearing was therefore not observed in this drive; the declared
short-return artifact replay supplies that acceptance case.

The guard review covers 700 status records and approximately **2.629 m** of
truth-measured travel. Maximum commanded speeds were **0.05 m/s** and
**0.30 rad/s**. Every stopped output was zero, the final state was operator Stop,
and no autonomous mode was selected. Startup covariance of 19998 correctly
latched `invalid_odometry` before readiness; 85 status records retain that fault.
The operator explicitly enabled manual mode at simulated time 16.786 seconds.
No fault occurred after that enable. The first overbroad all-session no-fault
review is preserved alongside the corrected review, which distinguishes startup
gating from the attended drive.

Supervisor temperature samples ranged from **35.75 to 70.125 C**. Candidate pins
still matched after the drive, and no container remained running. Evidence:
`RENDERED_CAPTURE_REVIEW.json`, `ATTENDED_GUARD_REVIEW_v2.json`,
`ATTENDED_VISUAL_REVIEW.json` and `RENDERED_RGB_CONTACT_SHEET.png` under the
Phase 3 evidence root. An initial offline review command failed because a Docker
mount option followed the image; its corrected command and successful review
remain separate records. This did not alter or repeat the live drive.

## Phase 2 follow-up

Representative 3 stopped four times on **unknown terrain** within the predicted
turn sweep. Every latched output was zero, and the operator confirmed stopping
before contact. These were insufficient-observation stops, not red hazard cells
eligible for adaptive clearing. Preserve that distinction during live review.

## Protected closure and evidence

`MANUAL_SESSION_FINAL.json` records **MANUAL_SESSION_CLOSED_CLEANLY** at
17:27:05 UTC. The final journal/teardown record at 17:27:04 UTC contains no
kernel incident matches, no remaining workload containers or service listeners,
and no KFD owner. The generic teardown result is
`INCOMPLETE_OR_FAILED_REVIEW_REQUIRED` because no timed-hour measurement was
requested; the separate protected manual-session result records the bounded
manual-use closure. No endurance result is inferred from either file.

Capture ended at **17:22:07 UTC**, and temperatures ended at **17:22:31 UTC**,
before final closure. All attended rendering and offline-review workload
containers had stopped before capture ended, but continuous final-cooldown
monitoring is not established. The full temperature log reached **75.375 C**;
the attended supervisor's maximum was **70.125 C**. Preserve this monitoring gap
without upgrading the session to an endurance or stability pass.

`PROTECTED_CLOSURE_REVIEW.json`, `protected-session-case/` and `operator-kit/`
retain the final records and the exact operator workflow. `PHASE3_ACCEPTANCE.json`
records the functional acceptance and its scope. Source archives, earlier
fixture/review failures, raw synchronized captures and derived reviews remain
preserved alongside the accepted candidate.

The final archive identity and per-file verification receipt are retained
separately from the frozen evidence tree to avoid modifying archived evidence.

Off-host verification completed for **11,907 content files, the manifest and
three symlinks**, including hardlink content checks without extraction.
Archive: `mapping-phase3-20261105T162405Z-final-evidence.tar.gz`,
**679,025,605 bytes**, SHA-256
`6df78b140fba54b3a106113097d1ff63a6bc41a7748336dc53a4cf9af35b59ce`.
Local [archive verification receipt](../evidence/development-20261107/phase3-mapping-20261105/final-archive-verification.json)
and [archive identity](../evidence/development-20261107/phase3-mapping-20261105/final-evidence.json)
are stored beside the archive. The archived report predates this verification
addendum; the frozen archive remains unchanged.

## Next phase and limits

Phase 3 does not authorize dataset expansion or retraining. The next planned
phase is a **200-image training pilot**, audited for rendering, annotations,
coverage, storage and capture time before scaling to the reviewed 10,000-image
dataset. Scene/asset split isolation and the accepted deployed camera/material
profile must carry forward.

These bounded checks do not establish field VIO accuracy, field traversability,
continuous stability or autonomous navigation acceptance.
