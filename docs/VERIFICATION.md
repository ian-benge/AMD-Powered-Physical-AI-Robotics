# Verification and recorded results

This publication separates reproducible CPU checks, ROS compilation, actual accelerator execution and bounded driving evidence. A passing unit suite cannot substitute for any of the other three.

## Run the source checks

```bash
python -m pip install -r requirements-test.txt
python scripts/run_checks.py --output-dir outputs/source-checks
```

Suites run in separate Python processes to isolate ROS test doubles. GitHub Actions runs the CPU suites on Linux and Windows. Real ROCm execution tests skip without compatible hardware; the ROS build workflow compiles the five integrated packages in Docker and does not drive a simulator.

The October 25 publication check ran **480 tests on Windows**, with four actual-ROCm cases skipped: 166 navigation, 157 perception, 129 environment, 24 training/geometry and four webcam-contract tests. The browser-control JavaScript harness also passed, and the offline procedural profile generated one scene with four planned views. See [the publication summary](../evidence/publication-20261025/checks.json) and its accompanying logs. These are fresh source checks, separate from the historical Halo measurements below.

## Recorded Halo measurements

| Area | Evidence | Meaning and limits |
| --- | --- | --- |
| NPU detector | [NPU report](NPU_TEST_REPORT.md), [navigation deployment](HALO_NAVIGATION_NPU_SWITCH_20261021.md) | YOLOv8s A16W8, static `[1,3,576,960]`, 38.46 mean isolated replay FPS, 0.76665 synthetic held-out mAP50–95 |
| GPU terrain | [GPU report](HALO_GPU_TERRAIN.md), [live summary](../evidence/gpu-terrain-staging-20261021/live_verification.json) | Paired subsystem timing 27.75 ms CPU vs 17.75 ms deployed ROCm/RPC; ten-frame exact mask/grid parity |
| Accelerator execution | [ROCm execution proof](../evidence/gpu-terrain-staging-20261021/rocm_execution_proof.json) | Actual profiled GPU work; NPU execution/assignment is separately documented in its report |
| Rock tracking | [Phase Three report](HALO_NAVIGATION_PHASE_THREE_REPORT_20261021.md), [three-trial summary](../evidence/phase3-validation-20261021/verified-trials-summary.json) | Three bounded translation/360°/fresh-revisit trials; zero false or duplicate confirmations across 1,330 replayed frames |
| Stops and ownership | [manual smoke report](HALO_NAVIGATION_SMOKE_REPORT_20261020.md) | Bounded manual motion, lease expiry, pause/resume and read-only planning; not full collision or autonomy acceptance |
| Earlier terrain failures | [Phase Two report](HALO_NAVIGATION_PHASE_TWO_REPORT_20261021.md) | Flat/edge localization disagreements and ramp initialization failures remain open |

NPU replay includes preprocessing, execution, decoding, drawing and JPEG encoding. It excludes camera delivery, disk reads, browser/network transport and Gazebo. Comparisons with the GPU baseline use different model architectures. Accuracy applies to the synthetic rover dataset. No physical-camera or energy-saving qualification is claimed.

The approximately 60 s integrated run was stationary: 1,818 NPU frames, 600 GPU operations, no mapper errors, and sampled total mapper p95 35.05 ms. These snapshots are not exhaustive scheduling measurements or moving-rover acceptance.

## Included evidence and omissions

[evidence/README.md](../evidence/README.md) identifies the curated files and their SHA-256 manifest. Reports, selected screenshots and test logs are checked in. Large RGB-D bundles, trained weights, vendor binaries, compiled caches and full session databases remain external. Historical documents explicitly identify local-only artifacts when they are omitted from this publication.

The October 23 recovery restored outage-damaged copies from the verified Windows archive and reran the original replay prefixes. Recovery was not a new driving trial. The recorded failures remain failures.

## Acceptance still open

- Physical wheel/obstacle contact coverage and complete fixed-obstacle detour/blockage tests.
- Reliable startup/localization across flat, edge, steep-ramp and other fixtures.
- Repeated autonomous travel to a reviewed soil site, stopped waiting and explicit return home.
- Broader scenes, long-duration mapping, physical OAK-D calibration and hardware motion.

The [roadmap](ROADMAP.md) describes the next milestone without presenting it as implemented.
