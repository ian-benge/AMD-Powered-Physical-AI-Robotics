# Pilot process cleanup fix - November 6, 2026

For the current 10,000-image collector's framing, depth-recovery and monitoring
fixes, see the [November 7 reliability report](BULK_CAPTURE_RELIABILITY_FIX_20261107.md).

The cleanup fix is installed and the complete pilot passed automated review:
**200 images across ten scenes, zero validation errors, and zero remaining
owned processes in all 11 rendered cleanup records.** Protected finish completed
with monitoring through cooldown. Image review and four-class export remain
pending; detector training has not started.

The previous pilot generated all ten scenes but committed only 40 images.
Detached Gazebo servers survived ROS launch shutdown. The renderer checker
could observe an earlier scene's server and later rejected multiple servers.
Original captures, failed attempts and their proof observations are preserved.

## Implementation

Each launch already receives a unique `GZ_PARTITION`. Cleanup now follows that
inherited partition across detached children, even after the launcher exits.
It checks UID and process start identity and uses Linux pidfds to avoid PID
reuse when signalling. Bounded interrupt, terminate and kill stages must leave
zero live owned processes. Unrelated processes are excluded.

Renderer acceptance also requires the current scene's exact world argument,
an owned process, a current renderer log, the qualified Ogre library hash and
Radeon/Mesa evidence. Every accepted attempt records a version-2 proof and
cleanup result. Resume verifies their hashes and the original mapping
observation in addition to existing image/label/metadata hashes.

The shared cleanup module is used by the repository capture implementation.
The checked pilot starter also ships an identical pinned module alongside its
worker, allowing the fix to run with the unchanged qualified camera application.
The accepted simulation launchers, image, renderer, hardware policy and capture
thresholds are unchanged. The earlier starter is retained under
`pilot200-collection-20261106/versions/before-process-cleanup-v2/` on Halo.

## Evidence compatibility

Resume retains the old dataset directory as
`pilot200-20261106T144029Z/dataset-before-process-cleanup-fix/`, including the
40 original committed frames and the three rejected scene-3 attempts. Its
weaker renderer proof is not promoted into the new capture. The ten generated
scenes and their bundle hashes are reused; image capture runs again into a new
`dataset/` under the same collection directory. Existing viewing/progress
commands continue to refer to the current capture.

## CPU verification

The Linux environment and training source suites pass (194 and 24 tests).
Fifteen focused lifecycle, proof and resume tests pass, including real detached
children, an already-exited launcher, escalation to SIGKILL, an unrelated live
process, PID start-time mismatch, survivors, old/wrong-scene proof and changed
saved evidence. Four starter cancellation tests pass. Windows runs the eleven
portable focused cases successfully and skips the four Linux process cases.

The checked starter verifies the unchanged 786 camera-application pins, 80
prepared assets and the separate starter/worker/cleanup manifest. Deployment
identities are in `process-cleanup-fix-20261106/DEPLOYMENT.json` on Halo.

## Completed pilot

The rerun used the original hashed scene bundle in
`pilot200-20261106T144029Z/`, with invocation `invocation-20261106T154634Z/`.
Capture took 554.14 seconds, including retries. Independent review verified all
1,186 bundle pins, 800 accepted image/label/metadata/preview hashes and the 40
preserved original frames.

| Check | Result |
| --- | --- |
| Populated / background frames | 160 / 40 |
| Primary target views for each of four export classes | 40 per class |
| Measured-depth box review | 200/200 frames agree; 2,807 objects checked |
| Geometry-only visible object appearances | 914, retained despite unavailable usable depth |
| Rendered cleanup | 11 records, all with zero remaining owned processes |
| Dataset validation | Zero errors |
| Guard / temperature-log peak Tctl | 80.75 / 80.875 degrees C |
| Final kernel incident matches | None |

The two background scenes have 40 empty label files. Source annotations retain
their three-class meaning; the four-class counts above describe planned primary
targets using appearance metadata, not a completed export. The contact sheet
received a spot check; full image/label review remains necessary.

Two failed attempts are preserved alongside the ten accepted scene attempts.
`scene_4312009/attempt-0001` rejected one view for insufficient measured-depth
support. Its retry passed without changing annotation thresholds.
`scene_4312013/attempt-0001` failed closed before capture because the launch's
ownership environment was temporarily unavailable; its retry passed.

The latter prompted a final startup-readiness change: wait up to one second for
the launch environment while requiring the same UID and process start identity.
The final 15 Linux focused tests include this case. This small follow-up was CPU
tested after closeout and was **not rendered again**. The accepted pilot used
the preceding cleanup module; both identities are retained:

| Cleanup module | SHA-256 |
| --- | --- |
| Rendered 200-image run | `888d8903a1460c7c761480f02f4b2dcb6f74f42aa926f549752f2ded16523f22` |
| Installed final readiness update | `87336db7c1cf20df04a1d0d4360e6e327ab82592c381dfeee88e50536b071d6c` |

## Protected closeout

The fresh session's `MANUAL_SESSION_FINAL.json` records
`MANUAL_SESSION_CLOSED_CLEANLY` at 2026-11-06 15:59:08 UTC, with
`post_teardown_clean: true`. Capture and temperature records extend past that
finish, covering cooldown and final journal capture. Workload containers are
stopped. The generic `POST_TEARDOWN.json` result remains
`INCOMPLETE_OR_FAILED_REVIEW_REQUIRED` because this manual session did not
complete a timed endurance trial; it is preserved alongside the manual final.
These results establish capture/cleanup acceptance, not endurance stability or
autonomous navigation.

## Evidence

On Halo, `process-cleanup-fix-20261106/FINAL_CAPTURE_REVIEW.json` contains the
independent audit. `DEPLOYMENT.json`, `READINESS_UPDATE.json`, test logs and
`release-final/STARTER_SHA256.json` distinguish the rendered and final releases.
The original scene bundle, both datasets and every attempt remain on Halo.

The off-host archive is
`process-cleanup-fix-20261106-evidence.tar.gz` (`navigation_sessions/process-cleanup-fix-20261106/process-cleanup-fix-20261106-evidence.tar.gz`, local-only),
479,485,757 bytes, SHA-256
`ea1bc64dad8c1a00f2186a8d156af4526811f699fc7ce6e0ac7251872e5e5a31`.
It includes complete capture outputs, preserved attempts, runtime proofs, source,
test logs, native closeout and bundle JSON. Bulky scene geometry/assets remain
on Halo; their hashes are recorded in the archived `BUNDLE_PINS.json`.
The [local verification receipt](../evidence/development-20261107/process-cleanup-fix-20261106/OFFHOST_VERIFICATION.json)
records archive and member hash verification.

The pilot remains `PILOT200_CAPTURE_COMPLETE_PENDING_REVIEW`. Next is visual
review and metadata-based four-class export. No training or larger collection
was performed.
