# Phase Three rock tracking acceptance — October 21, 2026

Confirmed-rock tracking is implemented and deployed on `halo-strix`, branch
`navigation/mapping-and-exploration`. **Three fresh scene-50000 manual
translation/360-degree/revisit trials passed**, including independent pose
comparison, fresh re-sightings under existing IDs, and recorded-evidence checks
for false and duplicate confirmations. These are bounded simulation results,
not autonomous approach, hardware or general detector-accuracy qualification.

## Resulting behavior

Camera boxes remain immediate raw detections. The default object map contains
only confirmed rocks, requiring three translated viewpoints, at least two
simulated seconds, confidence >=0.50, >=70% class support and raised RGB-D
geometry. Repeated stationary frames and a rotation alone cannot confirm an
object. Soil patches and slabs remain tentative. **Show tentative candidates**
reveals hollow markers, evidence scores, view counts and rejection reasons.

Association uses position covariance, bounded spatial/uncertainty gates,
one-to-one matching, soft class evidence, color and extent checks. Uncertain
matches remain diagnostic and do not update confirmed tracks. Visible,
independent contradictions can demote and retire objects; leaving view or an
occlusion cannot. The version-two confirmed-only API, optional RViz layer,
atomic JSON snapshots and append-only event stream are described in the
[Phase Three guide](NAVIGATION_PHASE_THREE.md).

The NPU detector, camera pitch 0.4 rad, RGB-D profile, RTAB-Map, terrain costs,
launch seed and motion-guard thresholds retain their configurations. Red terrain
hazards remain separate from yellow object markers. Simulator reference positions
are used only by evaluators and never passed into the live tracker or control.

## Recorded acceptance

Each fresh trial used guarded HTTP manual commands at 0.05 m/s, translated
approximately 0.45 m, turned at <=0.30 rad/s through a complete revolution, and
ended with an explicit Stop in approximately 47.8 wall seconds. No guard safety
interruption occurred during these trials; all final guard commands were zero.

| Trial | Truth displacement | Final pose disagreement | Maximum disagreement during movement | Result |
| --- | ---: | --- | --- | --- |
| 003 | 0.4570 m | 3.66 mm / 0.263° | 6.70 mm / 1.639° | Passed |
| 004 | 0.4552 m | 2.33 mm / 0.166° | 6.59 mm / 1.283° | Passed |
| 005 | 0.4581 m | 5.20 mm / 0.031° | 5.20 mm / 1.299° | Passed |

The established rock ID `1:2` was associated with a fresh observation after the
turn in every trial. Separate revisit verification requires an increased last
observation timestamp and <=0.6 simulated seconds of age; persisted markers
alone cannot pass. The two final confirmed tracks match `pale_stone_000` (whose
assigned detection class is rock) and `rock_010` in the independent reference.
Depth measures visible surfaces, so reference matching uses the object's planar
radius plus 0.10 m rather than claiming a precise object-center/collection pose.

Full replay evaluated **1,330 recorded frames**, with **zero false-confirmation
events and zero duplicate-confirmation events**, including tracks later retired.
It included 3,847 positive reference-matched observations and 65 negative
observations without the required raised support. These are observation counts,
not distinct objects or independent confirmation views. Thresholds stayed fixed;
scene labels were applied only after tracking. Replay uses saved projected RGB-D
evidence; it does not rerun inference or independently remeasure raw depth.

There were 35, 38 and 39 tentative/unverified entries at the respective trial
ends. They were not displayed on the default confirmed map. The evaluator also
records unconfirmed scene references; this includes unobserved objects and is
not visibility-qualified recall. Live yellow-crystal confirmation, class-specific
soil/slab validation and additional scenes still need their own recorded trials.

All three synchronized terrain audits passed, saving **87 raw RGB/depth/calibration/
terrain/pose bundles**. Sampled tracker-processing p95 was 20.4, 22.0 and 20.1 ms;
sampled maxima were 24.4, 24.5 and 22.2 ms. These measurements come from status
samples, not an inference-performance or sustained scheduling qualification.

The final browser regression verified actual pending candidates hidden by
default, show/hide rendering, repeated forward/turn commands, manual selection
at rest, release-to-idle, explicit Stop, connection-loss stopping, coverage and
Space-to-Stop with a checkbox focused. Its 99 successful drive requests had
p95 round-trip 101.4 ms and maximum 122.7 ms, within the unchanged input lease
for this recording. The deliberate connection-loss case latched
`command_timeout`; recovery required explicit test-operator selection and Stop.

## Source checks and retained failures

| Environment | Check | Result |
| --- | --- | --- |
| Windows / Halo ROS container | Navigation suite | 166 tests each; passed with 4 actual-ROCm execution tests skipped |
| Windows / existing Halo NPU environment | Perception suite | 157 tests each passed |
| Windows / Halo ROS container | Camera-profile suite | 2 tests each passed |
| Halo | Navigation package build | Passed |

The first attempt at the full perception suite in the ROS container failed
because that image lacks `aiohttp`, used by the legacy browser tests. The full
suite was then run successfully in the already installed NPU environment, with
no dependency changes. A navigation-checkout copy of the NPU environment helper
also failed shell parsing; the verified detector deployment's existing helper
in the NPU checkout was used instead. Neither environment helper was modified.

Two preliminary live trials are retained as failures. They completed movement
within pose limits but could not establish revisit acceptance because the new
display freshness check hid confirmed tracks. The fixes use this node's
simulation clock and request `map -> odom` at the current simulation time,
handling RTAB-Map's intentionally forward-dated transform broadcasts without
relaxing freshness bounds. The three accepted trials used that corrected tracker.
Fresh-revisit and every-frame replay checks were strengthened afterward and
applied to the saved recordings in separate verification files; the original
live reports were not rewritten.

## Evidence and final state

The evidence transfer interrupted on October 21 was completed on **October 23,
2026**. All paths below now resolve in both navigation checkouts. The recovered
archive's SHA-256 and all **639 file checksums** matched, with no differing local
files overwritten. The export contains approximately 343 MB of evidence before
compression and excludes SQLite databases. Configuration, source fingerprints,
raw event streams, synchronized bundles, browser captures and Gazebo/RViz
captures are included. Historical sessions, assets and unrelated worktrees were
preserved. No source changes were committed or pushed.

After the outage, several recently written files were empty. The acceptance
report and copied Windows/browser evidence were restored from the verified
Windows archive. All 69 original source fingerprints then matched on Halo.
The three frozen replay inputs were reconstructed from complete records in the
surviving event logs, using their original 309/315/706-frame limits. All three
replays passed again with the same positive/negative counts and no false or
duplicate confirmation events. Empty originals are preserved under the recovery
session's `outage-originals`; the empty historical `source-sha256-halo.json` is
retained as damaged evidence, with a separate recovery verification replacing
its role. These checks do not constitute new driving trials.

- [Verified three-trial summary](../evidence/phase3-validation-20261021/verified-trials-summary.json)
- [Timing and synchronized capture summary](../evidence/phase3-validation-20261021/timing-capture-summary.json)
- [Final Halo build/navigation/camera checks](../evidence/phase3-validation-20261021/halo-build-tests-final.log)
- [Full Halo NPU-environment perception checks](../evidence/phase3-validation-20261021/halo-npu-perception-tests.log)
- [Final Windows navigation checks](../evidence/phase3-validation-20261021/windows-navigation-tests-final.txt)
- [Browser control/tracking regression](../evidence/phase3-validation-20261021/browser-final-003.json)
- [Browser screenshot](../evidence/phase3-validation-20261021/browser-final-003.png)
- [Command publisher ownership](../evidence/phase3-validation-20261021/final-command-ownership-002.json)
- [Trial 003](../evidence/restart-20261021T212626Z-06f471/checks/tracking-001.json), [fresh revisit](../evidence/restart-20261021T212626Z-06f471/checks/revisit-verification-001.json), [full replay](../evidence/restart-20261021T212626Z-06f471/checks/replay-full-001.json)
- [Trial 004](../evidence/restart-20261021T212834Z-6fa6de/checks/tracking-001.json), [fresh revisit](../evidence/restart-20261021T212834Z-6fa6de/checks/revisit-verification-001.json), [full replay](../evidence/restart-20261021T212834Z-6fa6de/checks/replay-full-001.json)
- [Trial 005](../evidence/restart-20261021T212947Z-150fe2/checks/tracking-001.json), [fresh revisit](../evidence/restart-20261021T212947Z-150fe2/checks/revisit-verification-001.json), [full replay](../evidence/restart-20261021T212947Z-150fe2/checks/replay-full-001.json)
- [Final stopped/ready snapshot](../evidence/phase3-validation-20261021/final-status.json)
- [Verified Windows evidence transfer](../evidence/phase3-resume-20261023/local-transfer-verification.json)
- [Restored source verification](../evidence/phase3-resume-20261023/recovery-verification-001.json)
- [Recovered replay verification](../evidence/phase3-resume-20261023/recovered-replay-verification.json)
- [Post-reboot navigation checks](../evidence/phase3-resume-20261023/linux-navigation-tests.txt)
- [Installed tracker verification](../evidence/phase3-resume-20261023/installed-tracker-check.json)
- [Fresh stopped scene](../evidence/phase3-resume-20261023/fresh-scene-status.json)
- [Restored dashboard check](../evidence/phase3-resume-20261023/browser-restored-001.json)
- [Restored dashboard screenshot](../evidence/phase3-resume-20261023/browser-restored-001.png)

The October 21 final snapshot remains historical: session
`restart-20261021T212947Z-150fe2`, stopped/ready with two confirmed rocks and one
tentative candidate. After the reboot, services were restored in **fresh session
`restart-20261023T190246Z-796854`**, with Gazebo and RViz visible, zero guard
commands, zero mapped cells and zero stored objects. NPU execution was verified
again and the existing GPU terrain service is ready. The 166 navigation tests
passed again, with the same four actual-ROCm tests skipped. Installed tracker
files match the accepted source, and command publishers retain guard/stamper
ownership.

Open or refresh `http://127.0.0.1:8774` on the Halo desktop or through the restored
Windows SSH forward. Mapping is deliberately paused and manual drive disabled
until the operator inspects the launch pad in Gazebo and clicks **Confirm launch
area & start mapping**. Then explicitly select **Enable manual drive**. No
historical map or candidate state was imported, and no new motion was commanded
during recovery.

Three repeated trials in one scene cannot establish general semantic accuracy.
Persistent raised terrain confusers, occlusions and unreported localization drift
remain risks requiring broader labeled evaluation. Earlier flat/edge drift and
steep-ramp startup gates remain open. This phase does not establish autonomous
target approach, broader physical collision coverage, arms or sample collection.
