# AMD evaluation handoff

This guide describes the source and recorded results available on **November 11,
2026**. The project is a simulated rover research prototype. AMD inference,
terrain geometry and manual operation have bounded evidence; the complete
autonomous exploration–sample–return mission has not passed acceptance.

## Start here

| Evaluation | Entry point | Requires |
|---|---|---|
| Algorithms, controls and recovery contracts | CPU checks below | Python 3.12; Node.js for browser harnesses |
| Installed ROS interfaces and QoS | Docker build and service probe below | Linux Docker; no AMD device or model |
| Procedural world and stopped simulator | [Getting started](GETTING_STARTED.md) | Docker and the documented display/rendering setup |
| Actual NPU inference | [NPU setup and evidence](../scripts/HALO_NPU_TESTING.md) | Separate vendor runtime, checkpoint/export and calibration artifacts |
| Actual GPU terrain geometry | [GPU terrain guide](HALO_GPU_TERRAIN.md) | Compatible ROCm environment and retained CPU-reference parity |
| Integrated browser assessment | [Detector assessment](CAPTURE5000_MODEL_ASSESSMENT.md), [GUI-off profile](AUTONOMOUS_DEMO_BROWSER_PROFILE.md) | Prepared Halo deployment, attended monitoring and checked model/renderer identities |
| Autonomous mission candidate | [Mission guide and gates](AUTONOMOUS_DEMO_GUIDE.md) | All platform gates plus fresh bounded movement qualification |

The clean checkout contains source, configuration, procedural/bundled assets,
tests and selected evidence. It does **not** contain the 5,000 captured images,
model weights, quantized ONNX, compiled NPU caches, proprietary SDKs, operator
monitoring kits or the pinned Halo application directories. Deployment-specific
scripts under `scripts/` preserve the exact host workflow and fail when those
prerequisites are absent. They are not a fresh-machine installer. Request the
corresponding artifacts from the project owner for a matched hardware replay.

## Reproduce the source checks

From a clean checkout:

```bash
git clone https://github.com/ian-benge/AMD-Powered-Physical-AI-Robotics.git
cd AMD-Powered-Physical-AI-Robotics
git rev-parse HEAD
python -m venv .venv
# Linux: source .venv/bin/activate
# Windows PowerShell: .\.venv\Scripts\Activate.ps1
python -m pip install -r requirements-test.txt
python scripts/run_checks.py --output-dir outputs/source-checks

node src/rover_perception/test/test_live_controls.cjs
node src/rover_navigation/test/test_manual_speed_controls.cjs
node src/rover_navigation/test/test_camera_recovery.cjs
node src/rover_navigation/test/test_control_lifecycle.cjs
node src/rover_navigation/test/test_demo_controls.cjs
node src/rover_navigation/test/test_demo_page_integration.cjs
```

Use Python 3.12. Each Python suite runs in a separate process because some ROS
adapter tests use transport doubles. The `collection` suite also covers training
and demo tooling. Platform-dependent process tests and actual ROCm tests report
skips where their prerequisites are absent. No model inference, rendering or
driving is started. [Fresh check results](../evidence/amd-testing-20261111/source-checks/checks.json)
and the GitHub Actions checks accompany this handoff.

## Build and check the installed ROS interface

On Linux with Docker access:

```bash
docker build -f docker/Dockerfile.jazzy -t rover-jazzy .
docker build -f docker/Dockerfile.navigation -t rover-navigation .
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/work" -w /work rover-navigation bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    colcon --log-base log_navigation build --base-paths src \
      --build-base build_navigation --install-base install_navigation \
      --packages-up-to rover_navigation
  '
docker run --rm --network none \
  -e ROS_DOMAIN_ID=236 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$PWD:/work" -w /work rover-navigation bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /work/install_navigation/setup.bash
    python3 scripts/probe_autonomous_demo.py
  '
```

The build includes six packages, including the new typed
`rover_navigation_interfaces/DemoCommand` service. The probe checks construction,
HTTP-to-ROS commands, reception of actual best-effort odometry and refusal to
start exploration without healthy mission inputs. It has no simulator or wheel
controller. This verifies middleware integration, not autonomous driving.

If Ubuntu's default archive hosts are unavailable, the Jazzy image accepts an
explicit HTTPS mirror override, for example
`--build-arg UBUNTU_MIRROR=https://mirrors.edge.kernel.org/ubuntu` on the first
`docker build` command. The navigation image inherits those sources. Ubuntu's
archive signing keys and apt verification stay enabled; an incomplete index
update fails the build. CI uses this override and a bounded mirror preflight
following the November 11 upstream DNS outage. This changes the image build's
package-download route, not the separately pinned Halo runtime images.

For a stopped procedural scene, continue with [Getting started, steps 3–4](GETTING_STARTED.md#3-generate-a-procedural-demonstration-world).
That portable CPU demonstration is separate from the qualified Halo Ogre path.
Do not use its rendering output as evidence that the Halo material/depth fixes
or accelerator stack have passed on another machine.

## Recorded AMD platform and configuration

These identify the measured development environment; they are not a claim of
current vendor support for every component combination.

| Component | Recorded configuration |
|---|---|
| Host | Ryzen AI Max / Radeon 8060S development system, called Halo |
| OS / kernel | Ubuntu 24.04.5; `6.17.0-1033-oem`; retained reports mark the OS/kernel combination experimental |
| Middleware | ROS 2 Jazzy; Gazebo Harmonic; RTAB-Map; Nav2 |
| GPU terrain/training | Torch `2.12.0+rocm7.14.1`, HIP `7.14.60850`; float64 terrain and requested/observed HIP scheduling flags 4 |
| NPU | Separate Ryzen AI 1.8 environment; A16W8 YOLOv8s; 576×960 static export; actual operator/profile checks reject learned CPU fallback |
| Browser candidate | 40×40 m sparse scene; 960×540 Ogre RGB-D at 15 Hz; 0.40 rad downward tilt; 20 m RGB / 8 m usable depth |
| Thermal isolation | Gazebo GUI and RViz disabled; bounded stationary exterior viewer; sensor rendering still uses the connected desktop |
| Guarded demo movement | Rehearsal at 0.05 m/s; maximum 0.10 m/s requires separate qualification; turn cap 0.30 rad/s |
| Temperature protection | 87°C qualification stop; unchanged native 90°C protection |

Host services, ROS and the two accelerator environments remain separate. Existing
power/fan policy and CPU/thread limits were retained during the GUI-off check.
There was no driver, firmware or fan-policy change presented as a freeze fix.
Exact source/model/renderer identities are in the [GUI-off report](AUTONOMOUS_DEMO_BROWSER_PROFILE.md#release-and-evidence-identities)
and [selected evidence manifest](../evidence/amd-testing-20261111/manifest.json).

## Results and open issues

| Area | Recorded result | What remains open |
|---|---|---|
| Data preparation | 5,000 new captures plus legacy data; 5,300 train / 100 val / 100 test; split/hash checks passed | Complete new-image manual review; original 10,000-image reservation is incomplete |
| Epoch-22 NPU detector | Same legacy test set: 78.90% mAP50–95 versus 76.67% baseline; 23.46 ms mean execution | Mudstone AP decreased; 100-image reserved unseen-scene diagnostic review and global promotion remain pending |
| GUI-off stationary mapping | Five steady minutes: 80.82°C mean, 83.125°C peak; worst reported rolling publication p95 165.65 ms; median reported RTF 0.996 | Sustained below-80°C target failed; no matched GUI-on comparison, moving-load or endurance acceptance |
| Earlier 40 m traversal | Sparse outer-area travel/ground review recorded | Earlier p95 267.73 ms exceeded 200 ms; stationary improvement does not close moving/density gates |
| Autonomous rehearsal | Real observation sweep and travel; 3.443 m total estimated travel before interruption | Host froze during first return; no accepted home return, simulated sample or complete cycle |
| Startup | Guard retained an `invalid_odometry` latch from uninitialized covariance; later inputs became healthy | Review startup transition before autonomous acceptance; never bypass the guard to obtain a pass |
| Contact | Isolated chassis-contact wiring test passed | Full wheel/contact coverage and complete missions remain unqualified |

The [freeze investigation](AUTONOMOUS_DEMO_FREEZE_20261110.md) records an unclean
shutdown with no logged causal GPU timeout/reset, kernel panic, OOM kill or
thermal shutdown near the failure. That absence does **not** rule out a hard
lockup or unflushed event. The initiating cause remains unknown. The later
GUI-off stationary check ended with a clean protected cooldown/journal capture;
it does not erase the earlier missing teardown.

## Suggested testing order

1. Run the clean-checkout CPU/browser checks and installed ROS service probe.
2. Confirm the artifact identities and supported runtime setup before accelerator
   tests. Repeat CPU/ROCm parity and actual NPU execution verification separately.
3. Establish fresh attended capture and visible temperature monitoring. Measure
   stationary rendering/mapping and retain off-host logs before motion.
4. Investigate startup/host stability and thermal margin, then run short guarded
   0.05 m/s trials. Stop on faults; do not relax freshness, unknown-ground,
   footprint, contact or temperature rules.
5. Resume the explicit acceptance matrix in the mission guide only after those
   platform checks pass. Use fresh launches/maps, and retain failed attempts.

## Report a result

Use the repository's **Testing report** issue template. Include the commit SHA,
test command, UTC interval, scene/seed, model and export hashes, runtime versions,
expected/observed behavior, test logs and skip reasons. For integrated tests add
temperature, rolling publication latency, sensor ages, reported RTF and the
protected shutdown outcome. Distinguish execution timing from end-to-end FPS,
and legacy held-out accuracy from unseen-scene accuracy.

Raw journals, core dumps and desktop captures may contain host/network details;
review them before attaching anything to a public issue. Large datasets, weights,
vendor caches and full private evidence archives remain outside GitHub. Selected
reports preserve their source hashes and disclose publication transformations.
