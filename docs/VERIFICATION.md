# Verification and recorded results

This publication separates reproducible CPU checks, ROS compilation, actual accelerator execution and bounded driving evidence. A passing unit suite cannot substitute for any of the other three.

## Run the source checks

```bash
python -m pip install -r requirements-test.txt
python scripts/run_checks.py --output-dir outputs/source-checks
```

Suites run in separate Python processes to isolate ROS test doubles. GitHub Actions runs the CPU suites on Linux and Windows plus six JavaScript browser harnesses. Real ROCm execution tests skip without compatible hardware; the ROS build workflow compiles six integrated packages in Docker, including the typed demo interface, and checks the real HTTP/ROS service and best-effort odometry without a simulator.

## November 11 AMD handoff

The [testing guide](AMD_TESTING_GUIDE.md) is the current reproduction entry point.
[Fresh source checks](../evidence/amd-testing-20261111/source-checks/checks.json)
and [selected recorded evidence](../evidence/amd-testing-20261111/README.md)
cover the newer training, detector assessment, mission and thermal work.
The Windows Python 3.12.10 run completed **805 tests with 16 hardware/OS skips**;
all six JavaScript harnesses passed. The publication itself does not start a new
hardware trial.

Capture stopped at 5,000 new images. A separate 5,300/100/100 train/validation/test
export was prepared, and the epoch-22 checkpoint was selected from 28 saved
training epochs. Same-set NPU test mAP50–95 improved from 76.67% to 78.90%, with
a mudstone regression and no completed new-scene manual diagnostic review.
[Metrics and scope](CAPTURE5000_MODEL_ASSESSMENT.md).

The autonomous rehearsal froze during its first return; no complete cycle was
accepted. The [incident review](AUTONOMOUS_DEMO_FREEZE_20261110.md) leaves the
initiating cause unresolved. A later [GUI-off stationary check](AUTONOMOUS_DEMO_BROWSER_PROFILE.md)
recorded peak 83.125°C, worst rolling publication p95 165.65 ms and median reported
RTF 0.996, followed by clean protected closeout. This does not close moving-load,
matched-baseline, below-80°C, localization-drift or endurance gates. Historical
results below remain scoped to their original configurations and dates.

The October 25 publication check ran **480 tests on Windows**, with four actual-ROCm cases skipped: 166 navigation, 157 perception, 129 environment, 24 training/geometry and four webcam-contract tests. The browser-control JavaScript harness also passed, and the offline procedural profile generated one scene with four planned views. See [the publication summary](../evidence/publication-20261025/checks.json) and its accompanying logs. These were fresh checks for the October publication; the November check summary above is separate from the historical Halo measurements.

The [October 27 camera/adaptive-terrain change](HALO_CAMERA_ADAPTIVE_REPORT_20261027.md)
ran **182 navigation tests** on both Windows and Halo Linux/ROS, with four
actual-ROCm cases skipped in each run. The package rebuilt on Halo, and the
camera-recovery and speed-control JavaScript harnesses passed. This scoped
verification does not replace the historical full-suite run or live terrain
acceptance.

The later [October 27 dashboard follow-up](HALO_DASHBOARD_UPDATE_20261027.md)
ran **189 navigation tests** on Windows and Halo Linux/ROS, with four actual-ROCm
cases skipped in each run. The package build and three JavaScript harnesses
passed. A live browser check verified retained idle manual selection, camera
expansion, generation of a new layout and fresh-map Restart using only zero
drive commands. It does not qualify motion or adaptive clearing.

## November development verification

The [November evidence index](../evidence/development-20261107/README.md) contains selected original summaries, archive receipts and source/published SHA-256 identities. Fresh publication checks cover six isolated CPU suites and four JavaScript browser harnesses. The collection suite runs without operator kits or hardware; native-only checks explicitly skip. See [the fresh check summary](../evidence/development-20261107/source-checks/checks.json) for exact counts and runtimes.

The [rendering](PHASE1_RENDERING_ACCEPTANCE_20261104.md), [reachable-terrain](PHASE2_TERRAIN_ACCEPTANCE_20261104.md) and [adaptive-mapping](PHASE3_MAPPING_ACCEPTANCE_20261105.md) phases have bounded acceptance records. Mapping replay compared 408 published local/global cost arrays exactly across ten actual CPU/ROCm ROS cases; its [scope note](../evidence/development-20261107/phase3-mapping-20261105/CPU_ROCM_REPLAY_SCOPE_NOTE.json) distinguishes cost-array comparison from cause-grid assertions.

The [40 m closeout](SIMULATION_ENVIRONMENT_V2_CLOSEOUT.md) records completed offline/CPU/ROCm checks and an experimental default decision. Sparse traversal and real-time factor passed, but **267.73 ms p95 local-publication latency exceeds the 200 ms target**. The live optimization, other density representatives and recorded staged-speed checks remain open. Monitoring gaps are retained; no endurance acceptance is claimed.

The [200-image pilot cleanup review](PILOT200_PROCESS_CLEANUP_FIX_20261106.md) records zero validation errors and zero surviving owned renderer processes. The [continuous-controller record](BULK_CONTINUOUS_ACCEPTANCE_20261107.md) records 61 Linux tests, two actual protected monitoring/cooldown cycles and live batch-10 startup with 1,800 prior images verified. These are dated measurements, not proof of full campaign completion, reviewed four-class export, detector training or unattended endurance.

Historical guides below retain the results and pending steps as recorded on their dates. Later reports supersede status decisions only within their stated scope.

## Recorded Halo measurements

The [November 4 rendering acceptance report](PHASE1_RENDERING_ACCEPTANCE_20261104.md)
records the separate baseline/candidate identities, image-level material and
range comparisons, published camera calibration, source/build checks and each
timed visibility trial. Use its current verdict and limitations; source checks
and baseline readiness do not establish long-duration full-stack stability.

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
- Full live 40 m latency/density/speed qualification, long-duration mapping, physical OAK-D calibration and hardware motion.
- Completion and review of the 10,000-image campaign, metadata-based four-class export and any subsequent detector update.

The [roadmap](ROADMAP.md) describes the next milestone without presenting it as implemented.
