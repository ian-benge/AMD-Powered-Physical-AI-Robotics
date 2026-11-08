# Phase 3 mapping validation guide

See the [current acceptance record](PHASE3_MAPPING_ACCEPTANCE_20261105.md).
Phase 3 validates sensor-derived terrain and adaptive clearing. It preserves
the Phase 2 geometry, accepted renderer, deployed NPU model, isolated float64
ROCm environment and all motion limits.

## Controlled CPU validation

From the repository, with the navigation package and test dependencies on
`PYTHONPATH`, choose a new output directory:

```bash
python3 scripts/phase3_validate_mapping.py --output /new/phase3-controlled-replay
```

These are calibrated synthetic depth inputs, including a declared short-return
artifact. They are evaluation records and do not create a training dataset.

## Isolated ROS replay

The prepared candidate uses `scripts/phase3_ros_replay.py` with the saved
controlled inputs. It requires `ROS_DOMAIN_ID=219` and `PHASE3_MAPPING_BENCH=1`.
The guard output is remapped to `/phase3/bench_cmd_vel`, remains latched stopped
and must contain only zeros. No mode or motion requests are made. CPU replay
runs in a container without network/GPU devices; ROCm replay uses only the
retained loopback terrain service. This is separate from the rover's domain 87.

## Attended scene validation

Start a fresh capture and visible temperature terminal before the candidate
build/live checks. NPU, terrain and graphics remain off until the preceding
checks pass. Retain the original operator kit, power-saver and Quiet/Auto fans;
stop workloads at Tctl >=90 C or a capture/monitor/fan failure.

The separate Phase 3 desktop supervisor records exact-stamp RGB, depth,
calibration, local/global maps and cause grids for 120 wall seconds after
renderer provenance is ready. Confirm launch clearance promptly; the recorder
allows 40 seconds for initial mapped data. Use the live camera/map for manual
checks at 0.05 m/s and at most 0.30 rad/s. Do not bypass a guard rejection.
Stop the rover, wait for `PHASE3_SYNC_CAPTURE_COMPLETE`, then Ctrl+C the
supervisor. Avoid Restart during that bounded recording.

The separate source build, ten CPU ROS cases, ten ROCm ROS cases, three launch
setups and runtime preflight have passed. In a Halo graphical desktop terminal:

```bash
bash /home/ian/capstone-training/rover-navigation/navigation_sessions/mapping-phase3-20261105T162405Z/start-phase3-mapping.sh 3
```

Use the scene dashboard at `http://127.0.0.1:8774/`. The starter revalidates the
current capture, retained services, parity evidence and 1,860 candidate pins.
The accepted Phase 2 starter and baseline remain available.

At completion, retain capture and visible temperatures through
`~/bin/halo-rover finish`, its cooldown and final journal capture. Review
`MANUAL_SESSION_FINAL.json`, then stop the capture/temperature terminals.
