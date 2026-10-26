# Copilot instructions

## Project overview

Rover Vision is a local Python 3.10-3.13 application (Python 3.12 is the
tested target) for laptop webcam object detection. OpenCV supplies camera or
video frames, Ultralytics YOLO11 performs host-side inference, and OpenCV
renders the live window and annotations. The project is intentionally arranged
so a future Luxonis OAK-D capture adapter can replace the webcam adapter
without changing detector or application-loop code.

Run commands from the repository root. The Windows setup script creates the
project-local `.venv`, installs CPU PyTorch when needed, installs the pinned
application dependencies, and records local environment details:

```powershell
.\Setup-Windows.cmd
```

The equivalent Python setup command is:

```powershell
python scripts\setup_environment.py
```

Use the project interpreter explicitly after setup:

```powershell
.\.venv\Scripts\python.exe app.py
.\.venv\Scripts\python.exe scripts\check_install.py
.\.venv\Scripts\python.exe scripts\check_install.py --inference
```

The inference check runs real CPU inference on Ultralytics' bundled sample
image, or on a supplied local image:

```powershell
.\.venv\Scripts\python.exe scripts\check_install.py --inference --image "path\to\image.jpg"
```

Run the full automated test suite with:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

Run the supplied contract test module alone, or one test method:

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_detection_contract -v
.\.venv\Scripts\python.exe -m unittest tests.test_detection_contract.DetectionContractTests.test_original_frame_coordinates_and_centers -v
```

There is no separate build or lint command configured in this repository.
`scripts\check_install.py` and `pip check` are the intended dependency and
runtime diagnostics.

## Architecture

- `app.py` owns command-line overrides, the live processing loop, GUI
  lifecycle, overlays, FPS/inference reporting, and snapshot export.
- `vision/config.py` defines the `Settings` dataclass, loads `config.json`,
  resolves relative paths from the repository root, and validates all settings
  before hardware or model initialization.
- `vision/camera.py` contains the `OpenCVCamera` context manager. It opens a
  webcam index or local video path and returns BGR `uint8` frames. Video
  sources return `None` at end-of-file; webcam read failures raise.
- `vision/detector.py` contains the model-independent `Detection` contract,
  box decoding/clipping, and the `YoloDetector` wrapper. The model is loaded
  once and receives three-channel BGR `uint8` frames.
- `scripts\check_install.py` verifies imports and can exercise detector
  inference without opening a camera or GUI.
- `outputs\` holds generated snapshots and inference-check artifacts;
  `models\` holds downloaded or trained weights. Do not treat either as source
  code.

The frame pipeline is capture -> detector -> `Detection` objects -> drawing
and optional JSON/image export. Detection boxes and centers are always in
original-frame pixel coordinates, with origin at the upper-left and
`(x1, y1, x2, y2)` bounds. They are not depth, pose, grasp, or arm
coordinates. The current webcam implementation does not provide OAK-D depth;
an OAK-D adapter must preserve the camera context-manager/read contract and
return compatible BGR frames.

## Repository-specific conventions

- Keep camera access, GUI code, and model inference separated. Do not add
  webcam or display dependencies to `vision.detector`, and do not put model
  loading in the per-frame loop.
- Use `project_path()` for project-relative paths so commands work regardless
  of the caller's current directory. Configuration paths are relative to the
  repository root unless absolute.
- Validate configuration before opening hardware. Preserve the existing
  explicit `ValueError`/`RuntimeError` messages for invalid settings and
  startup failures rather than silently falling back.
- Preserve the `Detection` fields (`class_id`, `class_name`, `confidence`,
  `bbox_xyxy`, `center_xy`) and original-image coordinate semantics when
  changing inference or adding adapters.
- Keep the default device as `"cpu"` and use `"0"` only after a
  hardware-specific runtime has been verified. The setup helper is the laptop
  CPU baseline, not a universal AMD GPU installation recipe.
- Keep model weights and generated files out of code changes. A missing
  default `models\yolo11n.pt` may be downloaded by Ultralytics; custom model
  paths must already exist and should fail clearly when absent.
- Snapshots use a UTC timestamp stem and consist of raw image, annotated image,
  and JSON metadata. Preserve `coordinate_units: "original_image_pixels"` in
  exported detection metadata.
- Tests use the standard-library `unittest` runner and focus on pure
  configuration and detection-contract behavior, so new detector boundary
  behavior should be testable without a camera, GUI, or model download.
