# Browser navigation detector switched to NPU - October 21, 2026

The live browser navigation pipeline now runs the four-class **YOLOv8s A16W8
detector on the AMD NPU**. The GPU detector service has been stopped. Gazebo,
SLAM, the motion guard, map and detection-candidate state were not restarted.
The rover was stopped through the existing guard during handover and remains
ready for an explicit manual selection by the operator.

## Deployment

- Host source: `/home/ian/capstone-training/rover-navigation`.
- Host service: `http://127.0.0.1:8775`, PID **4010482** at verification.
- Browser: the existing `http://127.0.0.1:8774` page; existing Windows port
  forwarding is unchanged.
- Active scene/session: `restart-20261021T145720Z-e91aa8`, preserved.
- Source checkpoint:
  `/home/ian/capstone-training/mars-pilot50-v2/runs/detect/coverage500_fourclass_yolov8s_npu_960/weights/best.pt`.
- Checkpoint SHA-256:
  `89bbcf4ff124996fb6a520c5bf751287c2b5ea47bbd1d7e2adaf7f530ee44ca3`.
- NPU manifest:
  `/home/ian/capstone-training/mars-pilot50-v2/experiments/npu/resume_20261016T221548Z/candidate/manifest.json`.
- ONNX SHA-256:
  `2d47f7591563869cc7cbd6f118342d82db3427dbee8e8eff5c5b3b0589a8b03a`.
- Static input `[1,3,576,960]`, batch one, ONNX opset 17, A16W8, X2 compiler.
- Existing isolated runtime: `~/capstone-training/venvs/rover-npu`;
  Ryzen AI 1.8, ONNX Runtime VitisAI 1.27.0, XRT 2.25.37, kernel 7.0.0-38.

Learned feature/head computation runs on the NPU. CPU image preprocessing,
fixed sparse DFL decoding/NMS, drawing and JPEG encoding remain explicit. The
service process had `/dev/accel/accel0` open and no `/dev/dri` descriptor during
verification. GPU activity can remain visible because Gazebo and the desktop
still render on the GPU.

## Changes

The host HTTP detector service now supports explicit `torch` and `npu`
backends, importing only the selected runtime. NPU startup requires a matching
source-checkpoint hash, the reviewed four-class DFL artifact, warm-up, actual
NPU execution and matching compilation/runtime evidence. The existing artifact
and weights were reused; no training, quantization or firmware change was
needed.

The ROS HTTP client accepts a verified NPU deployment by its trained source
checkpoint while binding prediction requests to the ONNX artifact hash. It
preserves the same original-image coordinates, classes and timing fields.
Missing providers, CPU-only placement, invalid proof and inference errors do
not trigger GPU fallback. A backend inference failure latches the HTTP service
unready until explicitly restarted. Navigation's existing motion-guard policy
and command ownership remain unchanged.

`scripts/halo_navigation_detector.sh` now selects NPU by default and sources
the matched existing runtime. `--backend torch` remains an explicit rollback.
The browser detector label shows the actual backend/precision from status.

The new service was verified first on staging port 8778 while the GPU service
continued serving. Only the perception ROS node was then restarted with its
original parameters; the public detector endpoint stayed at 8775. The GPU
service and temporary staging service were stopped after handover. The
separate older port-8771 dashboard was not repurposed for navigation.

## Verification results

All **157 perception tests passed on Windows and in the Halo NPU environment**.
Coverage includes the NPU HTTP contract, source/artifact identity, missing
providers, compilation/execution proof, error latching, BGR/box preservation,
existing browser controls and deployment packaging. The two ROS Python
packages rebuilt successfully, and a real container-to-host camera prediction
verified the actual installed client, rather than only the source tree.

The subsequent 30-second observation used actual navigation camera frames
through the live browser pipeline, without a synthetic inference workload:

| Measurement | Result |
| --- | --- |
| Observation | 30.0001 seconds, seven samples |
| Host NPU service processed frames | 2,366 -> 3,275 |
| Navigation detector processed frames | 2,470 -> 3,379 |
| Processed rate, both counters | **30.30 FPS** |
| XRT submitted/completed work deltas for PID 4010482 | **909 / 909** |
| Backend in service and browser status | `npu` at every sample |
| Readiness / inference errors | Ready throughout / none |
| Browser image response | `X-Annotated: true` at every sample |
| Detector status age | Approximately 0.011-0.018 seconds |
| Session | Unchanged |
| Guard | Stopped, ready, zero command |

The node's absolute counter includes frames processed during the staged
handover; rates use counter differences within the measured interval. This is
a stationary pipeline observation, not a five-minute manual-drive acceptance,
display-FPS measurement, power measurement or a full requalification of the
older standalone browser launcher. No global `active_npu.json` acceptance
record was fabricated.

## Operator commands

The service is already running detached and survives SSH disconnection. Refresh
the existing browser page to load the backend label, inspect stopped/ready,
then explicitly choose **Enable manual drive** when ready to drive.

For a future service start, after confirming port 8775 is available:

```bash
cd /home/ian/capstone-training/rover-navigation
bash scripts/halo_navigation_detector.sh
```

For an explicit GPU rollback, stop the current detector service, then use
`bash scripts/halo_navigation_detector.sh --backend torch`. The navigation
client can reconnect to the explicit backend through the same endpoint.

To inspect the active service:

```bash
curl -fsS http://127.0.0.1:8775/status
```

Require model `backend: npu`, `executed_device: npu`, `model_family: YOLOv8`,
`precision: a16w8`, `proof_state: npu_executed`, verified compilation/offload,
and an increasing `processed_frames` counter. Browser `/status` also exposes
the effective detector metadata.

The locally patched amdtop NPU panel reads XRT work counters and will show
`ACTIVE`, PID 4010482 and completions/s while camera inference continues. Exit
and reopen amdtop if an older monitor process is still running. The percentage
gauge remains N/A because this loaded driver supplies no DRM busy-time counter.

## Evidence and rollback

Host logs and reports are retained in the Git-ignored directory:

`navigation_sessions/npu-detector-switch-20261021/`

It contains original source/status backups, staging tests, the Linux test log,
build and installed-client verification, handover records, service PID/log,
`pipeline_samples.json`, `pipeline_verification.json` and raw XRT reports.
The former GPU service command is retained in `handover_before.json`.
The original scene and all pre-existing session files remain available.
