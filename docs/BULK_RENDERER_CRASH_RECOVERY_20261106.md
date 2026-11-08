# Bulk capture interruption and post-reboot recovery

The campaign has **580 hash-verified saved images**: batches 1 and 2 are complete
(400 frames), and nine scenes in batch 3 have 180 accepted frames. Batch 3's
1,255 scene-bundle pins and its 720 accepted evidence files passed independent
checks. All eleven renderer cleanup records have zero survivors. No capture
application, original launcher, scene reservation or saved dataset was changed
during this investigation.

## Observed failure

On November 6 at 13:51:43 CST (19:51:43 UTC), the desktop journal records an RDP
transport failure, session disconnect and removal of virtual monitor `Meta-0`.
About 16 seconds later, the final background scene, `scene_4320056`, launched.
Gazebo crashed during Ogre OpenGL plugin initialization, with a null-address
segmentation fault in `RenderSystem_GL.so` and exit 139. No images were saved
by that scene's first attempt. The later missing-renderer-provenance error is
a consequence of the crash; the ROS shutdown errors are secondary.

Display removal is a likely trigger, based on this timing. The underlying
OpenGL fault has not been proved. Tctl was about 49 C near the launch and below
90 C throughout the saved temperature stream (peak 84.375 C). This does not
establish that no GPU/kernel incident occurred: the SSH account cannot read the
protected system journal, and the ordinary streamed journal contained no kernel
records. The existing desktop crash files date to November 3, not this failure.

The original monitoring capture ended at its retained two-hour limit at
20:39:19 UTC. Temperatures continued later, but no protected teardown record was
created. Halo subsequently rebooted. Normal `finish` correctly refuses the old
boot, and normal new-capture preparation correctly refuses a missing prior
teardown. Neither guard was removed.

## Separate recovery helper

The installed helper is
`/home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-renderer-crash-20261106/bulk10000_reboot_recovery.py`.
It passed twelve CPU-only tests on Windows and Linux, a static check against the
actual old selection, and the original case-owner check. No rendering was
started during verification. The protected review and next capture remain
operator steps.

`--review` requires a local sudo terminal and explicit acknowledgement of the
missing teardown. It preserves the original session files, reads both boots'
protected kernel journals, screens the retained incident patterns, records a
new 60-second idle cooldown and verifies no workload or KFD owner remains.
Missing journals, kernel incidents, changed identities and failed checks block
recovery. The root-owned result explicitly records `post_teardown_clean: false`,
`monitoring_gap: true` and that original protected teardown is unavailable.
It does not manufacture a clean old-session result.

Only a successful protected review permits `--capture`. This creates a fresh kit
with the unchanged native builder and preparation checks, preserves the old
selection in the recovery evidence, publishes the new selection and invokes
the ordinary protected native capture helper. Existing kits, old case evidence,
native launcher source and collection data stay in place. Normal `temps`,
`finish` and later `capture` commands apply to the successor session.

## Commands for this recovery

Keep the Halo graphical desktop/RDP connection active, with temperatures
visible. Leave NPU, terrain and the manual rover simulation off.

First, in a Halo terminal:

```bash
RECOVERY=/home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-renderer-crash-20261106
sudo python3 -I "$RECOVERY/bulk10000_reboot_recovery.py" --review --acknowledge-missing-teardown
```

Continue only if the result is `REVIEWED_REBOOT_WITH_MISSING_TEARDOWN` with
`fresh_session_preparation_permitted: true`. A blocked report remains evidence;
do not delete it to retry blindly.

In that same terminal, start the fresh capture and leave it running:

```bash
python3 -I "$RECOVERY/bulk10000_reboot_recovery.py" --capture
```

In a second terminal, leave temperatures visible:

```bash
~/bin/halo-rover temps
```

After two capture health events, prepare the retained temporary drivers and
read-only fan monitor in a third terminal. This loads the reviewed modules;
it does not start NPU inference or terrain processing:

```bash
sudo python3 -I /home/ian/capstone-training/rover-sim/manage.py modules
```

Wait for `MODULES_READY`, then finish just the interrupted batch:

```bash
COLLECTOR=/home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106/start_bulk10000_collection.py
python3 "$COLLECTOR" --run --batches 1
```

The starter verifies existing evidence and resumes the missing final scene in a
new attempt directory. It preserves the failed first attempt. Expected success
is **600 images / 3 completed batches**. If successful, continue the remaining
batches in the same attended monitoring session:

```bash
python3 "$COLLECTOR" --run
```

A repeated renderer crash requires investigation; no automatic crash retries,
software-renderer fallback or quality-threshold relaxation was added. The
original capture duration remains two hours. When stopping, run
`~/bin/halo-rover finish` while monitors remain running through cooldown and
final journal capture, then stop them. Subsequent sessions use normal fresh
`capture`/`temps`, the retained `modules` action if needed, and the same collector
`--run` command.

## Completed review and permission repair (November 7 UTC)

The operator completed the protected review at 00:46:19 UTC. Both protected
kernel journals had no matches for the retained incident patterns. The report
records the original monitoring gap and does not claim clean original teardown.
Capture then stopped before preparing a successor: `mkdtemp` had left the
root-owned `protected-review-knxiq09v` directory at mode `0700`, preventing the
normal user from verifying the evidence hashes.

The installed version-2 helper publishes completed snapshots with the original
operator's group: directory `0750`, files `0640`, with root ownership and file
hashes preserved. Fourteen Linux tests passed, including real group/mode checks
and rejection of symlinks or outside paths. The static check passed. The
successful protected report and native selection hashes remained unchanged.

For this already completed review, run the following once in a Halo terminal.
It changes only the existing evidence directory's group and traversal mode:

```bash
RECOVERY=/home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-renderer-crash-20261106
sudo chgrp ian "$RECOVERY/protected-review-knxiq09v" &&
sudo chmod 750 "$RECOVERY/protected-review-knxiq09v" &&
python3 -I "$RECOVERY/bulk10000_reboot_recovery.py" --capture
```

The saved files created under the original review's umask already permit read
access; the directory was the inaccessible parent. Capture still checks all
hashes and identities before preparing the successor. Do not rerun `--review`
or remove its successful result. After capture starts, use the temperature,
module-preparation and bounded batch-resume commands above. The actual
root-to-user access repair remains an operator step until those commands run.

The version-1 source, pins and successful review copy remain preserved in
`permission-fix-v2/before/`. The original off-host archive is the earlier
investigation snapshot; version-2 deployment records are separate.

## Evidence

Investigation (`navigation_sessions/bulk10000-renderer-crash-20261106/INVESTIGATION.json`, local-only),
Linux tests (`navigation_sessions/bulk10000-renderer-crash-20261106/LINUX_TESTS.txt`, local-only),
recovery pins (`navigation_sessions/bulk10000-renderer-crash-20261106/RECOVERY_SHA256.json`, local-only),
archive receipt (`navigation_sessions/bulk10000-renderer-crash-20261106/ARCHIVE_RECEIPT.json`, local-only),
and off-host verification (`navigation_sessions/bulk10000-renderer-crash-20261106/OFFHOST_VERIFICATION.json`, local-only).

The off-host archive preserves the old native case, failed launch, renderer
proofs, independent audit and tested recovery helper. It is a snapshot of
incomplete closeout, not a protected teardown certificate. Images remain on
Halo; their hashes and scene-bound acceptance proofs were verified.
