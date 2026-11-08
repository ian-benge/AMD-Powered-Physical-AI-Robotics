# Begin the 200-image pilot collection

The November 6 pilot is complete at
`navigation_sessions/pilot200-20261106T144029Z/`: 200 images, zero automated
validation errors and clean protected closeout. See the
[cleanup fix and acceptance report](PILOT200_PROCESS_CLEANUP_FIX_20261106.md).
Visual review and four-class export are still pending. The commands below
remain available for future collections; no repeat is needed for this pilot.
For the separate 10,000-image collection, use the
[full collection guide](FULL10000_COLLECTION_GUIDE.md).

This starter generates the ten reserved scenes and collects 20 views per scene:
160 populated frames and 40 background frames. It uses the fixed Ogre capture
candidate from `capture-ogre-fix-20261106T140200Z/repeat-2`. Collection preserves
three-class source annotations and appearance metadata for later four-class
export review. It does not train a detector or approve images for training.

Run on Halo, leaving NPU, terrain and the manual rover simulation off.

Terminal 1, leave running:

```bash
~/bin/halo-rover capture
```

Terminal 2, leave visible and running:

```bash
~/bin/halo-rover temps
```

Wait for two capture health events and updating temperatures. In a Halo
graphical-desktop terminal, start collection:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/pilot200-collection-20261106/start_pilot200_collection.py
```

The starter prints `COLLECTION_DIR` and the progress-log path. Scenes and
outputs live in a fresh `navigation_sessions/pilot200-<UTC>/` directory.
Each scene uses its own Gazebo process and preserved attempt directories.
Native freshness, fan and temperature checks supervise the bounded job.
Ctrl+C requests supervised cancellation and waits for container cleanup; keep
the monitors running until the command returns and protected finish completes.
Each renderer launch must produce actual mapped Ogre/Mesa/Radeon provenance.
Runtime version 2 additionally binds proof to the current scene, process start
identity and unique Gazebo partition. Cleanup includes detached children after
the ROS launcher exits, and no attempt commits until cleanup reports zero
remaining owned processes. Resume checks the proof/cleanup content hashes.
Generation is limited to the predeclared seeds and bounded layout attempts;
failure retains the directory and logs. The process has a 90-minute ceiling,
and capture expiry can stop it earlier.

On success, expect `PILOT200_CAPTURE_COMPLETE_PENDING_REVIEW`. Results are
in `dataset/`, including `validation.json`, clean images, labels, metadata and
review previews; `COLLECTION_RESULT.json` summarizes the result. The pilot
profile checks depth while capturing but does not save depth image files.
Keep monitoring visible throughout collection.

After success, cancellation or failure, close the protected session:

```bash
~/bin/halo-rover finish
```

Keep capture and temperatures running through its full cooldown and final
journal capture. Review `MANUAL_SESSION_FINAL.json`, then Ctrl+C the monitors.
Review image/label quality and the metadata-based mudstone conversion before
training or larger collection.

Static checks, without generating scenes or launching a container:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/pilot200-collection-20261106/start_pilot200_collection.py --check
```

The starter checks the unchanged 200-frame profile, 786 candidate pins, all 80
prepared assets and its separate worker/cleanup-module manifest. Linux tests
cover exited launchers, detached children, escalation, unrelated processes,
stale/wrong-scene proof and changed resume evidence. The ten-image rendered
test passed previously; see the
[process-cleanup fix report](PILOT200_PROCESS_CLEANUP_FIX_20261106.md)
for the pilot result.

Resume requires fresh monitoring and an existing complete, hashed scene plan:

```bash
python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/pilot200-collection-20261106/start_pilot200_collection.py --resume /home/ian/capstone-training/rover-navigation/navigation_sessions/pilot200-REPLACE_WITH_YOUR_RUN
```

Use the actual printed `COLLECTION_DIR`. Resume verifies bundle and completed
capture hashes; incomplete generation requires a new output directory. It never
deletes failed attempts. The starter and worker have a separate SHA-256 manifest
and leave accepted simulation launchers and evidence unchanged.

When resuming a capture made before runtime version 2, the worker preserves its
entire `dataset/` as `dataset-before-process-cleanup-fix/`. It reuses the hashed
generated bundle and recaptures into a new `dataset/`. Legacy proof could match
the preceding scene, so those earlier images remain separate from the new
qualified capture. Current-version captures resume with verified file and
runtime hashes.
