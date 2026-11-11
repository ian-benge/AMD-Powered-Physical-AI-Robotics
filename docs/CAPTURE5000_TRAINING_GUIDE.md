# Training the 5,000-capture candidate

This separate Halo run uses 5,300 training images: 5,000 accepted captures and
300 images from the existing reviewed export. The original 100 validation and
100 test images retain their splits and label bytes. Source captures, the
unfinished batch 26 and the deployed model remain separate from this run.

Preparation verifies completed batch receipts, source hashes, geometry/depth-box
checks, all image pixels and label coordinates. Mudstone conversion uses the
existing appearance/asset identity rule in a new four-class export. RGB-visible
targets beyond usable navigation depth remain labeled. Missing identity or
conflicting classes stop preparation. Scene-group or identical-RGB overlap
between splits also stops preparation. Held-out rock assets remain excluded
from the new training labels.

The new geometry-validated captures have not undergone a complete human visual
review. Their review flags and conversion provenance remain in the manifest;
they are not marked human-reviewed. The owner explicitly requested training
this experimental candidate. Its metrics on the old held-out set do not establish
generalization to unseen scenes in the new environment. Candidate training and
model promotion are separate decisions.

## Recipe and environment

Fine-tune the existing four-class YOLOv8s checkpoint at 960 input size, batch 4,
seed 0, up to 100 epochs and patience 20. AdamW uses learning rate 0.00125 and
weight decay 0.0005. FP32, two loader workers, no compilation and no image cache
bound this first run. Validation chooses the checkpoint; the test set is not
used during training.

A separate environment shares the unchanged qualified Torch 2.12.0 / ROCm
7.14.1 packages and adds matching Torchvision 0.27.0 and Ultralytics 8.4.175.
No driver, firmware, power or fan setting is changed. The GPU worker sets and
checks process-local HIP blocking synchronization (flags 4). A bounded probe
checks a four-class forward/backward/optimizer step and GPU NMS before rollout.
During work, the native capture, fan, sensor and temperature checks continue.
Compute owners must match the recorded training PID/start time or its descendants,
with the same user and per-launch token. Other compute clients stop the worker.
Previously verified owners that have exited can remain in the latest native
capture sample until its next refresh. Their recorded identities remain valid
for that historical sample; live or reused PIDs still require full checks.
The original terrain-service identity check remains unchanged in its own kit.

## Start and monitor

The pinned deployment is under
`/home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108`.
Complete the preparation session's protected `halo-rover finish` first, keeping
capture and temperatures through cooldown and final journal capture. Then stop
those old monitors. Leave image collection, NPU, terrain and simulation off.

Run from the connected Halo desktop:

```bash
sudo /usr/bin/python3 -B /home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108/start_capture5000_training.py --run
```

The supervisor manages capture and temperatures. Leave it visible and keep the
desktop connected. Read progress from another terminal:

```bash
watch -n 10 /usr/bin/python3 -B /home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108/start_capture5000_training.py --status
```

Status includes the epoch, batches completed in that epoch, loss and the latest
completed checkpoint. The `TRAINING` event prints the exact `training.log` path
for `tail -f`. Final weights appear at
`runs/capture5000_yolov8s_960/weights/best.pt` under the deployment.

Each completed epoch preserves an optimizer-bearing checkpoint and its hash.
The worker pauses before another epoch would overrun its conservative monitoring
budget. Protected finish retains the existing 60-second cooldown and final
journal capture before monitors stop. Planned pauses explicitly close the data
loaders before reporting success. A fresh native window then resumes that
checkpoint. There are at most 50 windows and every renewal requires progress.
Native recording still lasts at most two hours. Real failures, NaNs, changed
inputs, a lost desktop, stalled cleanup or Ctrl+C stop the run for review.

Press Ctrl+C once for a deliberate stop and let the monitored closeout finish.
The last completed checkpoint is retained. Reuse the same `--run` command after
reviewing the stop; it validates checkpoint hashes, optimizer state, epoch and
dataset identity. It does not restart completed training or repeat collection.

After training, compare per-class precision/recall and false positives with the
current model, using the unchanged evaluation data. NPU export/calibration and
deployment require a separate qualification.

Sources: [AMD PyTorch installation](https://rocm.docs.amd.com/projects/ai-ecosystem/en/latest/frameworks/pytorch/install.html),
[Ultralytics training callbacks](https://docs.ultralytics.com/usage/callbacks/).
