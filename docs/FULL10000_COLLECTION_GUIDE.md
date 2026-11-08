# Full 10,000-image collection

This guide describes the installed Halo campaign. Operator kits, pinned runtime
bundles, models and saved outputs are deployment artifacts outside GitHub.
For a fresh checkout, begin with [Getting started](GETTING_STARTED.md).

The separate starter is installed at
`/home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/`.
It reserves 500 new scenes and captures 20 views per scene in 50 resumable
200-image batches. The completed pilot remains separate and unchanged.

| Split | Scenes | Populated images | Background images | Total images |
| --- | ---: | ---: | ---: | ---: |
| Train | 400 | 6,400 | 1,600 | 8,000 |
| Validation | 50 | 800 | 200 | 1,000 |
| Test | 50 | 800 | 200 | 1,000 |
| Total | 500 | 8,000 | 2,000 | 10,000 |

Every populated scene targets five views of each export class. The source
annotations retain their three-class schema and appearance metadata. Four-class
conversion, human review and training approval are later steps.

For automatic monitoring renewal through campaign completion, use the separate
[continuous collection supervisor](BULK_CONTINUOUS_COLLECTION_GUIDE.md). It manages
native capture/temperature processes and protected finish from one local sudo
launch. The manual monitoring workflow below remains available.

The [November 7 acceptance record](BULK_CONTINUOUS_ACCEPTANCE_20261107.md)
documents 61 passing Linux tests, two protected monitoring renewals and live
batch-10 startup under the continuous supervisor.

## Start on Halo

Leave NPU, terrain and the manual rover simulation off. Use the graphical
desktop and attend collection with temperatures visible.

Terminal 1:

```bash
~/bin/halo-rover capture
```

Terminal 2, leave visible:

```bash
~/bin/halo-rover temps
```

After two capture health events and updating temperatures, run in a graphical
desktop terminal:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py --run
```

The first invocation creates `navigation_sessions/bulk10000-<UTC>/`. Subsequent
invocations resume that collection through `LATEST_COLLECTION.path`. All 500
scene groups, splits, background assignments, density presets and sunlight
recipes are reserved before generation. Existing split hashing and held-out
assets are preserved, and seeds are separate from previously inspected scenes.

The starter requires at least 120 GiB free initially and stops before another
batch if less than 15 GiB remains. It uses the existing qualified camera
application, Ogre image, renderer provenance, measured-depth checks and cleanup
module. Each batch retains the existing 90-minute guarded ceiling; native
capture expiry can stop it sooner. No monitoring limit is extended.

## Progress, interruption and resume

The current v9 candidate adds geometry-checked target-framing preflight and
recovery, pixel-localized incidental-depth recovery, fresh pose-feedback
settling, partial-batch progress, clean monitoring renewal and a vanished-session scan fix;
see the [collection reliability report](BULK_CAPTURE_RELIABILITY_FIX_20261107.md).
Quality thresholds, reserved scenes and previously accepted images are retained.

The earlier batch-6 depth rejection was fixed in capture v4.
It completed that batch with the original depth thresholds and the failing
object still labeled; see the
[November 7 recovery report](BULK_DEPTH_EDGE_FIX_20261107.md). The latest reliability
check completed eight batches (1,600 images). Use `--status` for the live count;
the same `--run` command resumes the next unfinished batch.

If the interruption was followed by a reboot before protected finish, normal
capture/finish commands intentionally refuse the old session. Use the
[reviewed post-reboot recovery procedure](BULK_RENDERER_CRASH_RECOVERY_20261106.md)
for the November 6 batch-3 interruption. Keep the graphical desktop/RDP
connection active during capture: its virtual monitor disappeared shortly
before the observed Ogre startup crash.

In another terminal:

```bash
watch -n 5 python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py --status
```

Progress shows recorded images, completed batches and the current detailed log
path. Counts advance after a complete, validated and cleanly stopped batch.
`current_batch` also shows committed images, completed scenes and the active
attempt's saved views. These partial counts are separate from completed-batch
`recorded_images`. `SESSION_RENEWAL_REQUIRED` means no further batch was started
because the remaining native capture duration was below the estimated budget;
perform protected finish, start fresh capture/temps, then repeat `--run`.
Current scene-generation progress is also saved in the current batch's
`GENERATION_PROGRESS.json`; per-scene capture logs are retained under
`dataset/attempts/`.

Ctrl+C waits for guarded container cleanup. Monitoring expiry also stops the
job. Close the protected session as described below. After starting fresh
capture and temperature terminals, repeat the same `--run` command. It verifies
completed file hashes, frame completeness, scene-bound renderer proofs, clean
teardown and successful guard receipts before reuse. Completed generation
checkpoints are verified; partial generation and failed capture attempts remain
preserved. Layout failures retain their declared seed and bounded attempt log;
investigate faults instead of substituting another seed.

To deliberately run at most one batch in a session, append `--batches 1`.
An explicit prior directory can be selected with
`--collection /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-<UTC>`.

## Close every monitoring session

After the collection command exits, run:

```bash
~/bin/halo-rover finish
```

Keep capture and temperatures running through the complete cooldown and final
journal capture. Review `MANUAL_SESSION_FINAL.json`, then Ctrl+C the monitors.
Repeat this closeout when a session expires, even if the full dataset is still
in progress.

## Results and verification

Each batch stores its own `bundle/`, `dataset/review/index.html`, immutable
attempt directories, renderer proofs, `BATCH_PINS.json` and `BATCH_RESULT.json`.
Top-level `batch-receipts/` binds completion to the native guarded session.
Resume detects missing/changed outputs and identical RGB across splits.
After all batches pass, the campaign writes
`COLLECTION_RESULT.json: FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW`.

The full collection is not automatically a reviewed training dataset. Visually
review all validation/test images, flagged training frames and a stratified
10% training sample; complete the metadata-based four-class export before
training. The starter performs no detector inference, retraining or navigation.

Static checks without starting a container:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py --check
```

Initial deployment checks passed on November 6: ten reservation/resume/startup tests, four
existing cancellation tests, 786 camera-application pins and 80 prepared assets.
The new completeness/proof verifier passed against all 200 existing pilot
frames, and all 500 reserved split assignments matched the accepted generator.
These were CPU/static checks; no new scene generation or rendered capture was
started during installation. See the
deployment receipt (`navigation_sessions/bulk10000-collection-20261106/DEPLOYMENT.json`, local-only)
and pinned release (`navigation_sessions/bulk10000-collection-20261106/RELEASE.json`, local-only).

The initial desktop invocation exposed an import-order error before any
collection was created: `remote_session` belongs to the retained operator kit,
whose verified loader must run first. The installed fix loads and verifies that
kit before importing its modules. Regression tests cover a clean Python import
path and rejection of a changed kit; the deployed loader also resolved the real
current kit successfully. The operator acknowledgement remains part of the
desktop starter, before the unchanged native capture gates.

The paused first batch also received a
[bounded capture-view recovery fix](BULK_CAPTURE_RECOVERY_FIX_20261106.md)
for localized incidental depth-support rejections. The starter verifies a
separate capture-only candidate and keeps the original application and saved
scene plan. Recovered frames retain full acceptance checks, rejected
diagnostics and an explicit visual-review flag.
The same candidate also makes bounded closer-pose corrections for small
top-margin failures, retaining the original framing requirements.
That earlier recovery release passed 195 Linux environment tests and 15 controller
tests, and the first 200-image batch completed rendered validation with zero
errors. The November 7 reliability release supersedes it; the linked report
records the current candidate and verified collection count. Use the same
`--run` command to resume the next unfinished batch.
