# Validation before delivery

The project was checked in a Linux environment using Python 3.12.14. This is a
record of observed checks, not a claim that the user's laptop or camera was tested.

| Component | Version used for real inference |
|---|---|
| PyTorch | 2.14.0+cpu |
| Torchvision | 0.29.0+cpu |
| Ultralytics | 8.4.153 |
| OpenCV Python | 4.14.0.94 |
| NumPy | 2.5.2 |

Completed checks:

- Python files compiled and all supplied JSON configuration files parsed.
- Four automated boundary checks passed: original-image coordinate handling,
  clipped/invalid boxes, empty predictions, and invalid configuration values.
- Package imports and `pip check` passed.
- The Python setup helper completed against the prepared Linux test environment,
  including dependency checks and writing local version records. The Windows
  batch wrapper was not executed in this Linux environment.
- Official YOLO11n weights downloaded successfully into the configured model path.
- Real CPU inference on Ultralytics' bundled 810 x 1080 bus image returned five
  detections: one bus and four people. Approximate scores were 0.940, 0.888, 0.878,
  0.856, and 0.622. The annotated image was visually inspected.
- A short locally generated video was read as three BGR frames. End-of-file and
  capture release worked.
- The full application processed that video using real YOLO inference and real
  drawing, with GUI calls mocked because no desktop display was available.
- Windows wheel availability was checked for the pinned Ultralytics/OpenCV packages.

Still requires verification on the user's hardware:

- Windows batch launchers and the VS Code Run and Debug UI.
- Access to the built-in/USB webcam and the actual live display.
- Real Q/S/X window events and webcam behavior after unplugging or permissions changes.
- Laptop FPS and memory usage. Test-environment timings are not laptop benchmarks.
- OAK-D acquisition, depth/calibration, and Halo GPU inference, which are future stages.

To verify locally, run setup, launch the model-only check, and then start webcam
detection. Confirm a cup/person detection, save with S, and exit with Q. Reopen
the app and check that clicking the window's X also stops it and releases the camera.
