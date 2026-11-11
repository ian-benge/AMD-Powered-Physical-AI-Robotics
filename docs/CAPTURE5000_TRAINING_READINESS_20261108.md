# Training from the stopped 5,000-image campaign

On November 8, the operator requested stopping further image collection and
using the existing captures for the next detector candidate. This supersedes
finishing the 10,000-image target for that candidate. The original reservation
and capture evidence remain unchanged; the 10,000-image campaign is incomplete.

## Verified state

The supervisor is stopped after KeyboardInterrupt. Both monitoring windows of
the last invocation finalized with no errors or monitoring gaps, and no Docker
workloads were running at inspection. No additional capture is needed to begin
preparing this training candidate.

| Captured data | Count |
| --- | ---: |
| Completed batches | 25 |
| Accepted images | 5,000 |
| Independent scene groups | 250 |
| Frames with empty labels | 1,000 |
| Original split | All training |

Batch 26 has no committed images and is excluded from the prepared training
snapshot. Preserve its partial generation and logs alongside earlier failures.

A metadata audit of the completed batches found these object counts after the
existing appearance-based mudstone conversion:

| Export class | Instances |
| --- | ---: |
| rock | 52,175 |
| pale_soil_patch | 6,897 |
| yellow_crystal_rock | 4,654 |
| mudstone_slab | 4,136 |

No rock-labeled frame was missing the appearance identity needed by that audit.
These counts establish metadata coverage, not completed visual review. The raw
capture retains three source class IDs. The prepared snapshot converts mudstone
only in its separate, versioned export.

## Prepared candidate data

The existing YOLOv8s training manifest still points to reviewed export
`f008f002e67aff76`, containing 300 training, 100 validation and 100 test images.
Its `review_export.json` SHA-256 is
`88ace1ac788ac4784679c484e3f29377c4465356764e4a3866c422462fbae068`.
The existing YOLOv8s checkpoint is available.

The November 8 preparation passed on Halo. The separate `dataset-v1` snapshot
contains **5,300 training, 100 validation and 100 test images**. The original
evaluation label bytes and every source split assignment are preserved. Scene
groups and identical RGB hashes do not overlap across splits. The new training
labels exclude the legacy held-out asset IDs `moon_rock_06` and `rock_09`.

All 5,500 images decoded successfully: 5,000 at 960 x 540 and 500 legacy images
at 1920 x 1080. Their original pixels are retained; the training input is 960.
Completed-batch receipts, source files, geometry checks and copied-file hashes
passed verification. The prepared manifest SHA-256 is
`9a6b7c82ac1dcb8bfc906306d0fc19c0c8ce3da893126f0e06b260e3a28ff691`.

| Training export class | Instances |
| --- | ---: |
| rock | 53,944 |
| pale_soil_patch | 7,075 |
| yellow_crystal_rock | 4,798 |
| mudstone_slab | 4,186 |

The owner explicitly requested starting this experimental candidate. New capture
review flags remain `human_review_complete: false`; no blanket visual approval
was added. Source captures, partial attempts and the deployed model are unchanged.

The old held-out set supports comparison with the current detector. It does not
provide a new blind test of the larger environment and corrected rendering;
report that limitation when interpreting results.

The isolated environment passed dependency checks without modifying the qualified
parent Torch/ROCm environment. The final Linux training suite passed all 20 tests,
including actual owned-process cleanup and compute-owner isolation cases.

The final GPU probe passed a four-class forward/backward/optimizer step and GPU
Torchvision NMS at batch 4 and 960 input size. It recorded Torch
`2.12.0+rocm7.14.1`, Torchvision `0.27.0+rocm7.14.1`, HIP `7.14.60850` and
requested/observed HIP flags 4 on Radeon 8060S. Eight guard samples from
20:45:20 through 20:45:34 UTC recorded a maximum Tctl of 61.125 C. This short
probe does not establish sustained training temperatures or endurance stability.
Cleanup left no owned process and the baseline checkpoint hash was unchanged.

Two failed probe attempts remain preserved. The first encountered the native
terrain-only compute-owner check. The training adapter now validates PID/start
time, user, launch token and ancestry while retaining native capture, fan,
sensor and temperature checks. The next exposed the inference checkpoint's
partial configuration and frozen gradients. The final probe restores default
loss settings and trainable parameters in memory, as the trainer does; it never
rewrites that checkpoint. A small CPU four-class backward/optimizer check also
passed before the final GPU probe.

Exact preparation identities:

| Artifact | SHA-256 |
| --- | --- |
| Original YOLOv8s checkpoint | `89bbcf4ff124996fb6a520c5bf751287c2b5ea47bbd1d7e2adaf7f530ee44ca3` |
| Final training release | `4d476136f189fb04ccdd786b368ba33d7d5e7ffdade802485ec9e666a17e6b45` |
| v5 source archive | `5f4cfba1737ce3fbc0451dae0e9e261da5cd430a92b13255fff051804e92fcea` |
| Successful probe result | `8b9bfe546fe3f71f7fb523d6eefbb7a76e82f88e28cdcb8a3d0d061f98dd8575` |
| Probe guard log | `73d035fe9b233129181c373e9dac3980e40d4d55c9021039f2ec4fd9db4dea88` |

The pinned runtime, source releases, Linux logs and `gpu-probe-ahg7mm42`
evidence are under
`/home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108`.

The first full startup reached one GPU batch with finite losses, then stopped
because the progress callback expected a tensor where Ultralytics 8.4.175 supplies
a named loss dictionary. The v4 correction preserves loss names, checks every
batch for finite values, and passed both unit tests and a check with real Torch
scalar tensors. No epoch or checkpoint completed in that failed startup.
Its `training-monitors-ss68er5v/window-001/WINDOW_RESULT.json` records a clean
protected finish at 20:49:10 UTC, no errors or monitoring gap, and monitors retained
through cooldown and journal capture. Its plots and configuration are preserved
under `failed-startups/startup-20261108T204757Z`; the source snapshot is unchanged.

The repaired run started on November 8 at 21:03 UTC under
`training-monitors-vra4n6rn`. Epoch 1 advanced past the repaired callback with
finite named losses. At the first live inspection, capture and temperature ages
were below four seconds, the guard reported Tctl 55.625 C, and the worker
recorded the qualified Torch/Torchvision/HIP versions with requested/observed
HIP flags 4. The original detector checkpoint hash remained unchanged.

The run subsequently completed 28 epochs. Its best validation mAP50-95 was
0.87077 at epoch 22. These remain validation results on the 100 legacy images;
they do not establish test-set performance or approve deployment.

Window 1 paused after epoch 14 and renewed successfully. Window 2 saved epoch 28
and wrote `PAUSED_FOR_MONITORING`, but the supervisor stopped when the last native
capture sample still named a data-loader process that had just exited. Both
windows retained monitoring through cooldown and final journal capture and
finalized cleanly, with no errors or monitoring gaps. The stop is recorded at
23:52:44 UTC on November 8; it was not early stopping or completed training.

The v5 correction retains the identities of previously verified exited owners
for those historical samples. Unknown owners, changed live identities and reused
PIDs remain rejected. Planned pauses explicitly close persistent data loaders
before reporting success. All 20 Linux tests passed, including a real process-exit
lifecycle test. A separate CPU check verified actual persistent-loader shutdown,
all snapshot hashes and the epoch-28 optimizer/early-stopping state. The checkpoint
is unchanged, with SHA-256
`2b2808e347c7f978593553b449b510f70ab17997a4f113a7f5107182065bc427`.
Its recorded best epoch is 22; restarting the same launcher resumes at epoch 29.
The corrected future live training handoff has not yet been observed. The operator
subsequently chose to keep training stopped and assess the validation-selected
epoch-22 checkpoint.

The [training guide](CAPTURE5000_TRAINING_GUIDE.md) records the recipe, protected
monitoring lifecycle, status commands and output paths. The stopped run did not
complete its planned 100 epochs. Matched GPU and NPU comparisons of the frozen
epoch-22 candidate against the unchanged detector are now recorded in the
[model assessment guide](CAPTURE5000_MODEL_ASSESSMENT.md). NPU test mAP50-95 was
78.90% versus 76.67% for the existing model; mudstone test AP decreased. The
candidate passed export/quantization parity, learned-compute NPU offload, matched
validation accuracy gates and a guarded HTTP inference check. Global model
promotion has not occurred. Full new-image visual review and new-environment
blind evaluation remain outstanding.

The captured collection is
`/home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-20261106T163426Z`.
The previous reviewed export is
`/home/ian/capstone-training/mars-pilot50-v2/dataset/coverage500_fourclass/review/exports/f008f002e67aff76`.
