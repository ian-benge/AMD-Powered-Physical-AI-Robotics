# Rover Vision

A small Python starter for Ian Benge's TAMU AMD rover perception subsystem.
It runs pretrained YOLO11n on a laptop webcam and keeps capture separate from
inference so the camera can later change to the original Luxonis OAK-D.

No Copilot subscription, hosted AI account, or API key is required. VS Code is
the editor; Python and the installed packages run the application on your computer.

## First run on Windows

1. Extract the entire ZIP into a writable directory. Open the extracted
   **rover-vision** folder, not its parent or the ZIP viewer.
2. Install **Python 3.12**. If needed, install the Python Install Manager from
   [python.org](https://www.python.org/downloads/windows/), then open a new
   PowerShell window and enter `pymanager install 3.12`.
3. Double-click **Setup-Windows.cmd** and wait for **Setup complete**. It creates
   a local `.venv`, installs CPU PyTorch if working PyTorch is absent, installs
   the application dependencies, checks imports, and records installed versions.
4. Open the folder in VS Code. Install the recommended **Python** and
   **Python Debugger** extensions published by Microsoft. These do not require Copilot.
5. Press **Ctrl+Shift+P**, choose **Python: Select Interpreter**, and select
   `.venv\Scripts\python.exe` for editing and the normal Run button.
   The supplied debugger configurations explicitly use the same environment.
6. Open **Run and Debug** with **Ctrl+Shift+D**, select
   **Rover: webcam detection**, and press **F5**.
7. The first run downloads `models/yolo11n.pt`. A separate camera window opens.
   Show it everyday objects such as a cup, bottle, phone, or chair.

If Python is already installed without the Windows launcher, open a terminal
in this folder and run `python scripts/setup_environment.py` using a compatible
interpreter. The setup helper supports Python 3.10-3.13; 3.12 is the tested target.

You can also run the application by double-clicking **Run-Windows.cmd**.
The batch launchers are conveniences for Windows; the application itself is Python.

### Equivalent terminal command

From the project folder in PowerShell, after setup:

```powershell
.\.venv\Scripts\python.exe app.py
```

The VS Code terminal's automatic environment activation is disabled to avoid
PowerShell activation-policy errors. Use the supplied Run and Debug profiles or
the explicit interpreter path above; plain `python` may select a different installation.

## Controls and results

| Control/output | Meaning |
|---|---|
| Q, Escape, or the window's X button | Stop and release the camera |
| Ctrl+C in the terminal | Stop with cleanup |
| S in the camera window | Save a raw JPG, annotated JPG, and JSON detections in `outputs/` |
| Box label | Predicted class and confidence score |
| Small center dot | Center of the bounding box, in image pixels |
| Loop FPS | Rolling throughput of the recent application loop |
| Inference ms | Model inference time reported by Ultralytics |

Loop FPS includes model prediction, drawing/display event handling, optional
snapshot writing, and reading the next frame. Model loading/download is excluded.
The first interval includes inference warm-up. It is **not** camera exposure FPS
or a measurement of physical capture-to-display latency; camera buffers are not timed.
Inference ms excludes capture, drawing and other application work.

Confidence is a model score, not a calibrated guarantee of correctness.
Saved JSON contains predicted labels, not human-verified training annotations.

## Configuration

Edit `config.json`, save it, and restart the program. Keep JSON double quotes
and avoid trailing commas.

| Key | Default | What to change |
|---|---|---|
| `camera_source` | `0` | Try `1` or `2` for another webcam; a quoted local video path also works |
| `camera_backend` | `"auto"` | On Windows only, try `"dshow"` or `"msmf"` if capture fails |
| `camera_width`, `camera_height` | `640`, `480` | Requested webcam dimensions; some cameras choose a different supported mode |
| `model_path` | `"models/yolo11n.pt"` | Later point to your trained `models/best.pt` |
| `device` | `"cpu"` | Use `"0"` only after the relevant GPU runtime works |
| `confidence` | `0.4` | Raise to hide lower-scoring predictions; lower to display more candidates |
| `image_size` | `640` | Try `320` for speed; smaller inputs can miss small objects |

The startup message prints the actual frame dimensions. Model input resizing is
handled by Ultralytics. Boxes are returned in the **original image's coordinates**.

Settings can also be overridden for one run:

```powershell
.\.venv\Scripts\python.exe app.py --source 1 --image-size 320 --confidence 0.5
.\.venv\Scripts\python.exe app.py --source "recordings/test.mp4"
```

Video files are decoded sequentially as fast as processing allows. Playback is
not paced to the video's recorded frame rate. A stopped video decoder ends the run.

## How the code works

| File | Responsibility |
|---|---|
| `app.py` | Application loop, overlays, keyboard/window controls, snapshot export |
| `vision/camera.py` | Open and read the webcam or local video, then release it |
| `vision/detector.py` | Load YOLO once; return structured predictions for BGR images |
| `vision/config.py` | Read, validate, and resolve project-relative settings |
| `scripts/setup_environment.py` | Install into the local environment and record versions |
| `scripts/check_install.py` | Diagnose imports and optionally run inference without a camera |
| `.vscode/launch.json` | Click-to-run debugger configurations |

For each frame, the capture component returns a three-channel BGR `uint8` image.
The detector returns `Detection` objects containing `class_id`, `class_name`,
`confidence`, `bbox_xyxy`, and `center_xy`, plus inference time in milliseconds.
The app draws those results and optionally saves the images and JSON.

The pixel origin is the upper-left corner: x increases rightward and y downward.
Box boundaries use `(x1, y1, x2, y2)` and can meet the image edge at width/height.
These are **not** depth measurements, object poses, grasp points, or arm coordinates.

## Checks and troubleshooting

Run **Rover: check imports** in VS Code to verify packages. Run
**Rover: check model without webcam** for real CPU inference on the bus image
bundled with Ultralytics. It writes `outputs/inference-check.jpg` and a JSON result.
Neither diagnostic proves webcam access or GUI behavior.

```powershell
.\.venv\Scripts\python.exe scripts/check_install.py --inference
.\.venv\Scripts\python.exe scripts/check_install.py --inference --image "my-photo.jpg"
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

| Symptom | Action |
|---|---|
| Setup cannot find Python | Install Python 3.12 through Python Install Manager; reopen the setup file |
| `ModuleNotFoundError` | Rerun setup and select the project's `.venv` interpreter |
| Invalid interpreter in VS Code | Extract first, run setup, and open the folder containing `app.py` |
| Webcam cannot open | Close Teams/Zoom/Camera apps; enable Windows Camera access, including desktop apps; check privacy shutter and source index |
| Black feed or frame-read error | Check lighting, camera shutter, connection, source index, and Windows capture backend |
| No GUI on a remote Linux session | Launch from a graphical desktop; the model-only diagnostic works without a display |
| Slow video | Try image_size 320; use good lighting; keep the nano model |
| Weights download fails | Allow access to GitHub model assets, or download official YOLO11n detection weights into `models/yolo11n.pt` |
| Wrong labels on custom parts | Expected for unseen classes; collect/label data and fine-tune later |
| GPU error | Set device back to cpu; verify the hardware-specific runtime separately |

The first package installation and pretrained weights download need internet.
After dependencies and weights are present, the detection loop can run offline.
Images are processed locally; this project does not upload webcam frames.

## OAK-D and Halo migration

The current adapter supports standard webcams. The selected original
[Luxonis OAK-D](https://shop.luxonis.com/products/oak-d) will need a **DepthAI**
capture adapter. It is not activated by simply changing camera_source to another
number. A future adapter should provide `__enter__`, `read`, and `__exit__`,
returning BGR `uint8` color frames. Host-side YOLO can then stay the same.

Depth support also needs synchronized/aligned depth and camera calibration.
The initial RGB detector does not automatically use the OAK-D's stereo depth.
Its factory calibration belongs to that camera; do not substitute webcam calibration.

For the Halo, transfer code, config, model weights, and any datasets/annotations
and representative test videos. Recreate the environment on the destination.
**Do not copy `.venv` across computers.** Install AMD's supported PyTorch/ROCm
build for the actual Halo processor and OS before installing application packages.
Then verify GPU inference and set device to `"0"`.

Use `requirements-local.txt` and `environment-local.json` as records of a working
machine, not as a universal GPU installation recipe. They are generated by setup.
The setup helper is intended for the laptop CPU baseline; a prepared Halo GPU
environment should be managed according to AMD's matching installation guide.

YOLO11n has learned common COCO categories. Your printed container, lid and sample
will need labeled training images. Start with varied webcam images, then evaluate
on the OAK-D and fine-tune with its images as needed. Split training and evaluation
by recording session so nearly identical video frames do not inflate accuracy.

When measuring AMD acceleration, use the same recorded video, weights, input size,
and settings on the Halo CPU and GPU. Report inference and application throughput
separately; do not mistake camera frame-rate limits for model-compute performance.

## References and third-party components

- [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11/)
- [Ultralytics prediction API](https://docs.ultralytics.com/modes/predict/)
- [PyTorch installation](https://pytorch.org/get-started/locally/)
- [VS Code Python debugging](https://code.visualstudio.com/docs/python/debugging)
- [DepthAI RGB example](https://docs.luxonis.com/software-v3/depthai/examples/rgb_preview/)
- [Ultralytics on AMD](https://docs.ultralytics.com/integrations/amd/)

Third-party packages and model weights retain their own licenses. Ultralytics
provides AGPL-3.0 and enterprise licensing options; no paid account is required to
run this local starter. Dependencies, model weights, and the third-party sample
image are downloaded separately and are not redistributed in this ZIP.
