# Full-collection capture recovery - November 6, 2026

The first full-collection batch stopped at an annotation-quality rejection in
`scene_4320002/view_006`. Ten scenes had finished generation and two scenes
(40 images) had committed successfully. Three attempts at the same third-scene
view failed measured-depth support checks; every attempt and diagnostic remains
preserved.

The saved RGB and depth timestamps match. Example incidental surfaces had
29/35 and 233/262 supported pixels, below the unchanged 90% requirement at
3 cm tolerance. The primary target met its framing and depth checks. Offline
reproduction matched the saved projected surfaces and did not attribute the
discrepancy to back-face culling. The exact renderer/geometry discrepancy is
not established; the correction addresses capture recovery from this localized
rejection rather than changing annotation geometry or declaring the frame valid.

## Change

The capture node can propose at most four alternative yaw offsets (approximately
plus/minus 2 and 4 degrees) for a localized incidental depth-support rejection.
The same scene, target, class quota, normal/difficult policy and 20-view count
remain in use. Each proposed pose retains map bounds, collision clearance,
camera clearance, target distance and slope checks. A newly synchronized frame
must pass all normal annotation, framing, depth and box checks before export.

Recovery applies only to the version-2 Ogre capture profile. It requires a valid
primary target and mostly agreeing measured depth on a small minority of
incidental objects. Missing sensors, poor target checks, widespread/large depth
disagreement and unsafe poses stop capture. Exhausting four alternatives also
stops. No label is dropped to make the checks pass.

The final proposal limit scales with the number of visible objects (at most
one quarter), instead of a fixed cap of three. A saved cluttered-scene fixture
had four localized failures among 41 objects; it may now try its remaining
bounded alternatives. Each proposed replacement still requires 90% measured
depth support for every exported object. Proposal eligibility is separate from
frame acceptance.

The bounded check then exposed a separate normal-view framing rejection in
`scene_4320012/view_012`: the target's top edge was pixel 53 against the required
54-pixel margin. A second candidate extends the existing closer-pose correction
to small top-margin failures with otherwise valid checks. It retains the same
margin, target-distance, collision and slope rules, and the existing three-pose
correction limit. Large clipping, difficult-view changes and compound failures
do not trigger this correction. The live replacement frame's target top is
pixel 84 and all acceptance checks pass.

Every rejected image/depth diagnostic is retained in its attempt directory.
Accepted metadata records `view_recoveries` and the measured pose; recovered
frames receive a `capture_view_recovered` review flag. They require visual
review before training export.

## Deployment and checks

The capture-only candidate is
`navigation_sessions/bulk10000-capture-recovery-20261106/application-v3/` on Halo.
Both earlier candidates are retained separately.
Its 789-file manifest inherits the original camera application's 786 pins,
allowing only capture orchestration, bounded framing and review flag changes
plus their helper/tests. Geometry, evaluator thresholds, renderer libraries,
camera profile and runtime image retain their original identities.

The full-collection starter pins `CAPTURE_RUNTIME.json`, verifies the candidate
and records its identity in every new invocation. The original application,
pilot, scene reservations, generated bundle and original 40 captured images
are retained. Resume uses the existing campaign:
`navigation_sessions/bulk10000-20261106T163426Z/`.

Verification includes 195 Linux environment tests (including seven recovery
cases and eight framing cases), 15 controller/cancellation tests, both exact
saved failure fixtures,
and static checks of both original and capture-candidate pins. Rendering uses
the existing attended capture/temperature session and unchanged native guards.

## Rendered acceptance

The bounded check completed the first batch at 2026-11-06 18:18:51 UTC. The
independent audit passed after verifying its receipt, content hashes, per-scene
renderer proof, cleanup and original preservation snapshots.

| Check | Result |
| --- | --- |
| Frames | 200: 160 populated and 40 backgrounds |
| Primary export-class targets | 40 views per class |
| Dataset validation | Zero errors |
| Measured-depth box review | 200/200 frames agree; 2,753 object appearances checked |
| Geometry-only appearances beyond usable depth | 1,032, retained |
| Accepted depth-recovered frames | Two, each after three proposals |
| Accepted top-margin correction | One; top edge moved from pixel 53 to 84, minimum 54 |
| Renderer cleanup | All 20 records, including rejected attempts, have zero remaining processes |
| Guard-recorded peak Tctl | 83.875 degrees C |
| Existing-frame preservation | Original 40-, 120- and 160-frame snapshots all hash-verified |

Recovered frames are `scene_4320002/view_006` and
`scene_4320016/view_002`. The reframed target is
`scene_4320012/view_012`. All are flagged for review. Earlier failed guarded
invocations remain recorded as failures; only the final successful guarded
completion produced the first-batch receipt. No saved rejection was promoted
into accepted data.

The starter reports **200/10,000 images and 1/50 batches**. It is paused after
the bounded check; the next normal `--run` continues with batch 2. It reuses the
completed first batch and does not regenerate its scenes:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py --run
```

Capture and temperatures remain running for operator continuation. The evidence
contains a live monitoring snapshot, not a protected final closeout. When
ending this monitoring session, run `~/bin/halo-rover finish`, retain both
monitors through cooldown and final journal capture, then stop them. Fresh
monitoring remains required after a session ends or expires.

## Evidence and limits

The independent review (`navigation_sessions/bulk10000-collection-20261106/capture-failure-20261106/FINAL_RECOVERY_REVIEW.json`, local-only)
records exact counts, preservation checks and retained guarded outcomes. The
deployment receipt (`navigation_sessions/bulk10000-collection-20261106/capture-failure-20261106/DEPLOYMENT.json`, local-only)
identifies the final controller and capture runtime.

The off-host
evidence archive (`navigation_sessions/bulk10000-collection-20261106/capture-failure-20261106/bulk-capture-recovery-20261106-evidence.tar.gz`, local-only)
contains all 200 canonical images/previews/labels/metadata, rejected diagnostics
and logs, runtime proofs, source patches, tests, bundle JSON and the monitoring
snapshot. Large original scene assets and partial attempt image copies remain
on Halo. Its receipt (`navigation_sessions/bulk10000-collection-20261106/capture-failure-20261106/ARCHIVE_RECEIPT.json`, local-only)
and off-host verification (`navigation_sessions/bulk10000-collection-20261106/capture-failure-20261106/OFFHOST_VERIFICATION.json`, local-only)
record the archive hash and all 2,732 member hashes.

The remaining 9,800 images have not been collected. The first batch still needs
visual review and four-class export; detector training has not started. These
checks do not establish endurance stability or qualify all future layouts.
