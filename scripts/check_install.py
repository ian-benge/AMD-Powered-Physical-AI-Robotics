"""Check imports, optionally exercising real inference without opening a camera."""

import argparse
from dataclasses import asdict
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inference", action="store_true", help="Run CPU inference on a real sample image")
    parser.add_argument("--image", help="Use your own image instead of Ultralytics' bundled bus.jpg")
    args = parser.parse_args()
    # This script intentionally imports the runtime stack without opening a
    # camera, making it useful for diagnosing installation problems remotely.
    print("Python:", sys.version.split()[0], "|", platform.platform())
    import cv2
    import torch
    import torchvision
    for package in ("torch", "torchvision", "ultralytics", "opencv-python", "numpy"):
        print(f"{package}: {version(package)}")
    from vision.config import load_settings, project_path
    from vision.detector import YoloDetector
    # Import package without creating a model or downloading weights.
    cache = ROOT / ".cache" / "ultralytics"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(cache))
    import ultralytics
    print("All required imports passed.")
    print("Accelerator available to PyTorch:", torch.cuda.is_available())
    print("ROCm/HIP version:", torch.version.hip)
    print("This does not prove that GPU inference works. The starter uses CPU.")
    if not args.inference:
        print("Model inference and webcam access were NOT tested. Use --inference or launch the app.")
        return 0
    # Use a known local sample by default; callers can provide a project-
    # relative or absolute image path when checking their own input.
    source = project_path(args.image) if args.image else Path(ultralytics.__file__).parent / "assets" / "bus.jpg"
    image = cv2.imread(str(source))
    if image is None:
        raise RuntimeError(f"Could not read {source}. Run again with --image followed by a local photo path.")
    settings = load_settings()
    settings.device = "cpu"
    detector = YoloDetector(settings)
    # A warm-up pass makes timing less dominated by first-call initialization.
    detector.predict(image)
    detections, inference_ms = detector.predict(image)
    if not args.image and settings.model_path.endswith("yolo11n.pt"):
        labels = {item.class_name for item in detections}
        if not {"bus", "person"}.intersection(labels):
            raise RuntimeError("Inference ran, but the bundled sample produced no expected bus/person detections.")
    from app import draw_detections
    annotated = draw_detections(cv2, image, detections)
    # Save both a visual artifact and structured metadata so the check can be
    # inspected without requiring a live GUI session.
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    if not cv2.imwrite(str(output / "inference-check.jpg"), annotated):
        raise RuntimeError("Could not save the inference-check image.")
    payload = {"source_image": str(source), "image_width": image.shape[1], "image_height": image.shape[0],
               "coordinate_units": "original_image_pixels", "device": "cpu",
               "inference_ms": inference_ms, "detections": [asdict(item) for item in detections]}
    (output / "inference-check.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Real CPU inference passed: {len(detections)} detections, {inference_ms:.1f} ms model inference.")
    print("Saved outputs/inference-check.jpg and outputs/inference-check.json.")
    print("Webcam capture and live GUI were NOT tested by this check.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Check failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
