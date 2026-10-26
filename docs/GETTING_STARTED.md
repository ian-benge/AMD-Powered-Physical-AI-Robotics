# Getting started

Run commands from the repository root. The CPU checks are the shortest reproducible entry point. The AMD deployment needs separately prepared runtimes and model artifacts; it is not installed by the test requirements.

## 1. CPU checks on Windows or Linux

Use Python 3.12 and a fresh virtual environment:

```bash
python -m venv .venv
# Linux: source .venv/bin/activate
# Windows cmd: .venv\Scripts\activate.bat
# Windows PowerShell: .\.venv\Scripts\Activate.ps1
python -m pip install -r requirements-test.txt
python scripts/run_checks.py
```

If PowerShell blocks activation, use `.\.venv\Scripts\python.exe` in place of `python`; no execution-policy change is needed. Select individual suites with `--suite navigation` or `--suite perception`. ROCm execution tests skip when compatible hardware is absent. These checks do not run learned inference or prove driving acceptance.

## 2. Build the ROS simulator on Linux

Docker access is required. The image names below are used by the existing launchers.

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
```

## 3. Generate a procedural demonstration world

This profile uses generated geometry and the attributed bundled Fuel meshes. It downloads no Poly Haven assets and is not the scanned scene used in historical Halo measurements.

```bash
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/work" -w /work rover-navigation bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /work/install_navigation/setup.bash
    python3 -m mars_training_environment.generate \
      --config config/portfolio_demo.json --output dataset/portfolio-demo
  '
```

Generation writes portable scene metadata plus host-specific world paths. A fresh output folder is required after configuration changes. For scanned, textured scenes, follow [the environment guide](../src/mars_training_environment/README.md); that workflow fetches and prepares external assets separately.

## 4. Start a stopped CPU-only navigation scene

This launch explicitly chooses CPU terrain and disables the learned detector, so it does not require AMD runtime services or weights. Default Halo launchers otherwise expect the GPU terrain service.

```bash
bash scripts/halo_navigation.sh \
  "$PWD/dataset/portfolio-demo" scene_050000 portfolio-001 \
  terrain_backend:=cpu detector_backend:=none \
  preview:=true mission_controls:=true controls_port:=8774 \
  mapping_on_confirm:=true gui:=false rviz:=false
```

Open `http://127.0.0.1:8774` for controls and `http://127.0.0.1:8773/frame.png` for the raw camera. The map begins empty and motion remains stopped. On a headless host, forward these localhost ports through SSH. Use a new session name for every launch.

For Gazebo/RViz windows, use `gui:=true rviz:=true` from an active Ubuntu desktop. Inspect the launch area before confirming clearance or enabling manual control. Unknown terrain and unsafe ground remain blocked. Ctrl+C stops this foreground launch. Do not run another publisher of `/cmd_vel` in navigation domain 87.

The public source supports simulation, not a physical rover. Follow the [navigation guide](NAVIGATION.md) for interfaces and acceptance tools. The [manual mapping guide](HALO_MANUAL_MAPPING_DETECTIONS.md) describes the richer host-supervised deployment with scene Restart.

## 5. Enable AMD acceleration separately

The verified Halo used separate environments for ROS, NPU inference and ROCm terrain. Configure the [NPU detector](../scripts/HALO_NPU_TESTING.md) and [GPU terrain service](HALO_GPU_TERRAIN.md) only after the CPU build works. Historical Halo wrappers contain deployment-specific defaults; override paths/environment variables for your machine rather than creating the old developer directory structure.

Model weights, quantized ONNX, calibration images and compiled caches are not in Git. A provider list or successful compile is insufficient: the detector requires matching hashes, runtime identity, warm-up and actual NPU execution evidence. There is no silent CPU/GPU fallback.
