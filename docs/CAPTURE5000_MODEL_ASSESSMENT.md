# Epoch-22 detector metrics and browser assessment

Training is intentionally stopped after 28 saved epochs. The assessed model is
the validation-selected best checkpoint from epoch 22, frozen before test use.
Its SHA-256 is `90fca4a3e4fa702f213c044e7ac09fb0271a4e70d6b71ca670d7e0913da1793f`.
The existing detector and accepted launchers remain available for rollback.

## Matched accuracy

Both models use the same 100 legacy validation and 100 legacy test images.
No test images were used for training or calibration. New synthetic labels have
automated geometry checks; full human review remains incomplete.

| Evaluation | Existing model val/test mAP50-95 | Epoch-22 model val/test mAP50-95 |
| --- | ---: | ---: |
| Ultralytics GPU, 960-square FP32 | 82.81% / 75.91% | 84.98% / 78.41% |
| Shared deployment evaluator, 576 x 960 NPU A16W8 | 84.20% / 76.67% | 85.82% / 78.90% |

The two evaluator rows use different preprocessing/matching conventions and
must not be compared as a quantization delta. In the shared deployment evaluator,
the candidate's unquantized FP32 ONNX reference scored 86.27% validation and
79.69% test mAP50-95. The new NPU artifact loses 0.45 and 0.79 percentage points
respectively. Training-time validation was 87.08%; it is retained as a separate
training result, not substituted for these matched reevaluations.

| Class | Existing NPU test mAP50-95 | New NPU test mAP50-95 | New recall at confidence 0.25 / IoU 0.5 |
| --- | ---: | ---: | ---: |
| rock | 73.48% | 78.61% | 92.49% |
| pale_soil_patch | 66.46% | 71.45% | 86.84% |
| yellow_crystal_rock | 93.06% | 95.59% | 95.45% |
| mudstone_slab | 73.67% | 69.95% | 80.95% (17/21) |

Mudstone test AP decreased while recall stayed at 17 of 21. Inspect slabs,
ordinary-rock confusion, partial burial, shadows and background false positives
during manual assessment. These legacy held-out results do not establish accuracy
on unseen 40 m scenes or a physical camera.

## NPU preparation

The 576 x 960 static ONNX export passed raw-output and decoded-box parity.
Quantization used only the original 300 reviewed training images, which are a
subset of the new model's training data. Their existing audit and split boundaries
are unchanged; no blanket review approval was added to the new captures.

CPU-only Torch 2.6 packages and the existing export dependencies were linked into
a separate calibration runtime. Original environments were not modified. Failed
attempts retain their outputs and logs, including compiler PATH setup, a guarded
stop after the original export runtime opened KFD, and the legacy CRLF shell
script failure. The successful NPU compile uses the unchanged native overlay,
working NPU environment and a new explicit cache root.

Per-channel A16W8, sparse FP32 DFL decoding, one ORT thread, spinning disabled and
X2 optimization level 0 match the established deployment format. Actual runtime
operator assignment and profiling verified learned convolutions on the NPU and
no learned CPU computation. Mean candidate execution was about 23.5 ms across
100 test images; p95 was about 24.2 ms. This is model execution timing, excluding
rendering, transport and browser drawing; it is not a live display-FPS result.

## Attended browser use

Keep a fresh native capture, updating visible temperatures, the native NPU
service and the 1600-cell candidate terrain service running. For this assessment,
use `start_capture5000_terrain.sh` in the training follow-up directory; it retains
the terrain implementation and adds the bounded desktop-inventory retry described
in the [thermal-stop report](CAPTURE5000_BROWSER_THERMAL_FIX.md). The model assessment
starter checks the scene, application, model, compilation and runtime identities.
It starts the new NPU service on 8778 and routes the perception client there.
The original service on 8775 remains the native reference/rollback. Every runtime
check verifies the new endpoint's source checkpoint, ONNX hash, compilation proof,
class schema and actual NPU execution; an old detector cannot satisfy that check.

From the Halo graphical desktop:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108/start_capture5000_assessment.sh 1
```

The run is bounded to 30 minutes. After `RENDERER_PROVENANCE_READY`, open
`http://127.0.0.1:8774` on Halo. Inspect the launch area, confirm clearance, wait
for readiness, and explicitly enable manual drive. Initial speed is 0.05 m/s;
the existing limits remain 0.20 m/s and 0.30 rad/s. Guard rejections remain stops
to investigate. Restart and Generate new scene retain the new detector endpoint
and the existing profile, camera, mapping and control settings.

The browser-specific resource profile disables RViz and limits container CPU
worker visibility. Its live RGB-D cadence is 15 Hz, with the existing 960 x 540
geometry and clipping; legacy launch and saved capture defaults remain 30 Hz.
Inspect the launch area in Gazebo, then minimize that separate
GUI while using the browser. The qualified Ogre sensor server keeps running.

For a Windows browser, keep a separate loopback tunnel running if one is not
already available:

```powershell
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -L 127.0.0.1:8874:127.0.0.1:8774 halo
```

Then open `http://127.0.0.1:8874`.

Press browser Stop when done, then Ctrl+C the assessment starter. Its cleanup
stops its candidate service and scene. Stop the candidate terrain terminal and
native NPU terminal, run `~/bin/halo-rover finish`, and retain capture/temperatures
through cooldown and final journal capture before closing them.

This is a separate manual-assessment candidate, not global model promotion.
The 40 m environment retains its experimental status and deferred live latency
and density qualification gates. No autonomous-navigation acceptance is implied.

Detailed evidence is under
`/home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108/`:
`evaluation-tzox37fr/` contains GPU reports, confusion matrices and plots;
`npu-candidate-epoch22/metrics/` contains all matched deployment evaluations;
`npu-candidate-epoch22/a16w8-sparse/` contains the NPU model and compile proof;
`assessment-session-*/` retains candidate readiness, HTTP smoke tests and cleanup.

## Observed starter verification

The Linux identity-rejection and inherited-settings tests passed. A guarded
candidate HTTP inference check passed and its candidate process stopped cleanly;
the baseline service and terrain remained available. Current-session classify,
depth and projection CPU/ROCm parity passed before graphics.

The first desktop launch stopped on a `loginctl show-session` inventory race:
a short-lived session disappeared between enumeration and inspection. Its scene
and candidate detector cleaned up without errors. The v2 wrapper reuses the
existing bounded whole-scan retry (three attempts, only that exact vanished-session
error). Tests verified that stale temperatures and changed graphical identities
still stop immediately. Native kit and application files were not changed.

The corrected attended launch on November 9 at 00:54 UTC (November 8 local time)
reached scene readiness with retained Ogre renderer provenance. Dashboard and
candidate-service metadata matched the new checkpoint and ONNX hashes, and more
than 1,200 actual camera frames were processed. A 960 x 540 JPEG was visually
inspected, with `X-Annotated: true` and current detector/localization/camera status.
The first inspection found the rover stopped and waiting for operator clearance.
The final recorded observation showed manual mode ready, healthy localization,
a zero command, 3,109 processed frames and camera age about 0.27 seconds. No
clearance or drive commands were sent by the agent. In-app browser automation could not connect;
verification used the dashboard HTTP routes and served image directly.

`ASSESSMENT_LIVE_VERIFICATION.json` retains that observation. The workspace copy
of `MODEL_METRICS_SUMMARY_V2.json` includes source-report hashes and compact
GPU/NPU results. The observed startup is not an attended driving acceptance or a
new 40 m latency/density qualification.

That initial session subsequently hit the retained 90 C terrain guard. Its
clean shutdown and the separately pinned browser-display repair are recorded
in the [thermal-stop report](CAPTURE5000_BROWSER_THERMAL_FIX.md). The initial
readiness observation above must not be interpreted as sustained thermal acceptance.
