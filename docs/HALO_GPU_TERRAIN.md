# AMD GPU terrain processing

Implemented on `navigation/mapping-and-exploration`, October 21, 2026. The host ROCm service accelerates depth back-projection, rigid transforms, elevation/count/roughness reductions, slope/height-span classification, and conservative terrain-memory projection. The ROS mapper retains terrain history and publishes the same grids. SLAM, Nav2, motion guarding and command expiry remain CPU components; YOLOv8 object detection remains on the NPU.

This is geometric terrain processing, not learned terrain segmentation or learned depth estimation. It consumes the existing measured/simulated depth. Thresholds remain 12 degrees slope and 0.03 metres height change. Unknown space remains unknown. Hazards still require distinct-frame confirmation for memory and cannot be cleared by later free readings.

## Halo launch

The current GPU service was started detached and is already listening on `127.0.0.1:8782`. After a reboot, start it in its own Halo terminal:

```bash
cd /home/ian/capstone-training/rover-navigation
bash scripts/halo_terrain_gpu.sh
```

The launcher uses the existing `/home/ian/miniforge3/envs/rover-yolo/bin/python`. No packages, drivers or firmware were installed or changed. `TERRAIN_PYTHON` can select another compatible ROCm Python. The service rejects missing HIP/GPU support and completes real GPU work before reporting ready.

Continue using the existing NPU detector launcher in its separate environment. The navigation launcher now defaults to `terrain_backend:=rocm_http`, checks GPU-service readiness before creating the container, and gives an explicit error if the service is unavailable. It does not silently switch to CPU.

For a new navigation session, use the normal scene command, for example:

```bash
cd /home/ian/capstone-training/rover-navigation
bash scripts/halo_navigation.sh \
  /home/ian/capstone-training/mars-pilot50-v2/dataset/live_eval_scenes \
  scene_050000 gpu_terrain_01 \
  preview:=true mission_controls:=true controls_port:=8777 \
  mapping_on_confirm:=true terrain_backend:=rocm_http \
  detector_backend:=remote_http detector_url:=http://127.0.0.1:8775 \
  detector_model:=/home/ian/capstone-training/mars-pilot50-v2/runs/detect/coverage500_fourclass_yolov8s_npu_960/weights/best.pt
```

Use a new session name and stop an existing navigation container through the normal scene workflow before manually launching another one. The browser restart supervisor carries `terrain_backend` and `terrain_url` into subsequent sessions. The current older supervisor process also gets GPU terrain through the new launch default.

After a mapper or scene restart, inspect the surroundings and click **Confirm launch area & start mapping**. Then select manual mode explicitly. Clearance requires explicit operator confirmation. The deployment restarted only the terrain node: its terrain history was reset, while the simulator, RTAB-Map, odometry, sample sites and NPU detector remained running. The rover was stopped for this handover. During the subsequent buffer correction, the user's just-given clearance was reapplied only after verifying the same session, zero motion command and unchanged rover pose.

## Status and failure behavior

```bash
curl -s http://127.0.0.1:8782/status
curl -s http://127.0.0.1:8774/status
```

The first response shows `backend: rocm`, the Radeon device name, HIP/PyTorch versions, successful operation count and last service timings. Browser status includes `mapping.compute`, `mapping.compute_timings`, `mapping.compute_error`, and total mapper `update_seconds`. Refreshing the normal browser page adds **Terrain: AMD GPU** to the mapping status.

RPC is loopback-only, numeric binary arrays with bounded headers and a 32 MiB maximum payload, no pickle, one GPU request at a time, and a 0.5-second client timeout. Processing is synchronous at the mapper's existing 5 Hz schedule; old pending depth is bounded and time-validated. Inference failure latches the GPU service unready. Transport or GPU errors prevent publication of a fresh safety grid, so the existing motion guard's depth-freshness checks expire. There is no automatic CPU fallback. Actual stop latency still requires a separate controlled live acceptance test.

## CPU rollback

Pass **`terrain_backend:=cpu`** to the normal navigation launch command. This uses the original CPU reference computations and needs no GPU service. A scene/mapper restart still requires operator clearance and manual rearming. Detection can remain on the NPU; terrain rollback does not change the detector backend.

## Measured results

Ten time-aligned RGB-D frames and their transforms were captured from the running scene, each with 52,142 valid sampled depth points. CPU and GPU produced **zero differences in terrain costs, hazard masks, unknown masks, and projected grids/reason flags** on these ten frames. Unit coverage additionally includes invalid and padded/endian depth, empty clouds, repeated cell indices, slopes and spans immediately below/above their thresholds, holes, unsupported points and rotated projection.

The following sequential comparison used these same frames, five warm-up iterations and 30 measured iterations per backend. Gazebo, the desktop and NPU detection remained running; this is a paired subsystem comparison, not an isolated peak-throughput claim.

| Depth/classification + map projection | Mean | p95 |
|---|---:|---:|
| CPU reference | 27.75 ms | 29.56 ms |
| Direct ROCm including input/output GPU transfers | 7.10 ms | 8.55 ms |
| ROCm through deployed loopback protocol | **17.75 ms** | **19.54 ms** |

The deployed path reduced the measured subsystem mean by about **36% (1.56× speedup)**. Direct ROCm was 3.91× faster, but that excludes the ROS-to-host RPC cost. These numbers exclude shared CPU terrain-memory fusion, coverage diagnostics, ROS message construction/publication, and intermittent merged-cloud diagnostics. They must not be presented as a 1.56× speedup for the complete mapper. Sustained live update timings are recorded separately after operator clearance; see the live verification below.

The implementation deliberately retains float64 geometry and conservative classifications. GPU reduction ordering can change a rounded nonhazard cost by one in general; tests require exact unknown/hazard masks and matching layer values within numerical tolerance. The recorded live frames had exact integer-cost agreement.

PyTorch's ROCm backend uses the `torch.cuda` API naming. A profiler trace recorded **208 GPU kernel events** during a real depth-frame calculation, establishing GPU execution rather than merely GPU availability. References: [PyTorch HIP semantics](https://docs.pytorch.org/docs/main/notes/hip.html), [scatter-reduction numerical behavior](https://docs.pytorch.org/docs/2.14/generated/torch.Tensor.scatter_reduce_.html).

## Live verification after operator clearance

The corrected deployment passed a **59.995-second stationary observation** with all 61 sampled GPU/NPU statuses ready, no mapper errors, and continuous confirmed mapping:

| Measurement | Result |
|---|---:|
| GPU depth/projection operations completed | 600 |
| Full terrain update, sampled mean / p95 / maximum | **32.95 / 35.05 / 37.13 ms** |
| NPU detector processed frames / rate | **1,818 / 30.30 FPS** |
| GPU whole-host activity, sampled mean / p95 | **20.02% / 21%** |
| Guard depth age, sampled mean / maximum | **110.11 / 113.49 ms** |
| Shared APU PPT power, sampled mean | 84.66 W |

The mapper retains its 5 Hz schedule. The total-update latency distribution uses latest-value snapshots, not every individual update. The rover remained stationary, and the scene had been restarted by the operator before confirmation; do not directly attribute differences from the earlier CPU-only whole-host baseline to this backend. Paired captured-frame timing above provides the controlled subsystem comparison. Power is shared package telemetry, not isolated GPU or NPU power, and demonstrates no energy-efficiency saving.

The first live integration attempt exposed a read-only RPC output buffer in the clearance-seeding path. Writable copies at the returned grid boundary corrected it; a new regression verifies both projected and classified grids can be modified. The failed attempt's samples remain in `live_samples_buffer_failure.json`. The final passing samples and summary are `live_samples.json` and `live_verification.json`.

## Reproduce and inspect evidence

Evidence is in `navigation_sessions/gpu-terrain-staging-20261021/` on the Halo:

- `frames/frame_*.npz`, `capture.json`, and `benchmark.json`: captured input and paired measurements.
- `gpu_parity_tests.log`: eleven passing protocol/parity tests on the actual AMD GPU.
- `ros_regressions_final.log`: 133 navigation tests, with four GPU-only cases skipped in the ROS container and run separately on the host GPU.
- `rocm_profile.json` and `rocm_execution_proof.json`: GPU kernel execution evidence.
- `source_before/`, `before_handover.json`, and service/node logs: backups and deployment state.
- `npu_perception_regressions.log`: 157 passing detector/browser-control/WebSocket regressions in the unchanged NPU environment.

Capture runs inside the ROS container; benchmark runs on the host with ROCm:

```bash
# Inside the navigation container, with ROS and the navigation overlay sourced:
python3 scripts/halo_terrain_gpu_check.py capture --output navigation_sessions/my_gpu_check

# On the Halo host, from the navigation project with the GPU service running:
PYTHONPATH=src/rover_navigation /home/ian/miniforge3/envs/rover-yolo/bin/python \
  scripts/halo_terrain_gpu_check.py benchmark --output navigation_sessions/my_gpu_check
```

This milestone does not establish peak GPU capacity, power savings, active-navigation deadlines, or five-minute driving acceptance. Future learned depth and segmentation must be profiled together with this service and the NPU detector.
