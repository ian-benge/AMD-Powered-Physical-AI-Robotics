"""Run this file from VS Code. Q exits; S saves the current frame and results."""

import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
import sys
from time import perf_counter

from vision.config import ROOT, load_settings

WINDOW = "Rover Vision - Q: quit | S: save"


def parse_arguments():
    # Command-line values override config.json for one run and are not
    # written back to the persistent configuration file.
    parser = argparse.ArgumentParser(description="Local webcam object detection; CPU by default.")
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--source", help="Webcam index (0, 1, ...) or a local video filename")
    parser.add_argument("--device", help="cpu, or a GPU identifier after its runtime is configured")
    parser.add_argument("--model", help="Path to detection weights, relative to the project")
    parser.add_argument("--confidence", type=float)
    parser.add_argument("--image-size", type=int)
    return parser.parse_args()


def draw_detections(cv2, frame, detections):
    # Draw on a copy so the raw frame remains available for snapshot export.
    image = frame.copy()
    height, width = image.shape[:2]
    for detection in detections:
        # Detection coordinates are already in this frame's pixel space.
        x1, y1, x2, y2 = detection.bbox_xyxy
        left, top = min(round(x1), width - 1), min(round(y1), height - 1)
        right, bottom = min(round(x2), width - 1), min(round(y2), height - 1)
        color = (70, 220, 110)
        cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        center = tuple(round(v) for v in detection.center_xy)
        cv2.circle(image, center, 3, color, -1)
        label = f"{detection.class_name} {detection.confidence:.2f}"
        cv2.putText(image, label, (left, max(top - 8, 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return image


def save_snapshot(cv2, frame, annotated, detections):
    # Use one UTC timestamp for all three files so raw pixels, annotations,
    # and metadata can be matched later.
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    timestamp = datetime.now(timezone.utc)
    stem = timestamp.strftime("%Y%m%dT%H%M%S_%fZ")
    for suffix, image in (("raw", frame), ("annotated", annotated)):
        path = output / f"{stem}_{suffix}.jpg"
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"Could not save {path}.")
    metadata = {
        "saved_at_utc": timestamp.isoformat(),
        "image_width": frame.shape[1],
        "image_height": frame.shape[0],
        "coordinate_units": "original_image_pixels",
        "detections": [asdict(detection) for detection in detections],
    }
    (output / f"{stem}.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Saved raw image, annotated image, and detections to outputs/{stem}*", flush=True)


def run(settings):
    try:
        import cv2
        from vision.camera import OpenCVCamera
        from vision.detector import YoloDetector
    except (ImportError, OSError) as exc:
        raise RuntimeError(
            "A required package or native library could not load. Run Setup-Windows.cmd, "
            "then select this project's .venv interpreter in VS Code. "
            f"Original error: {exc}"
        ) from exc

    # Load/download once before capturing, so network delay is not counted as camera work.
    if sys.platform.startswith("linux") and not (os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY")):
        raise RuntimeError(
            "No graphical Linux session was detected. Run from a desktop session, "
            "or use scripts/check_install.py --inference for a model-only check."
        )
    # Load the model before opening the camera. A first-run weights download
    # should not look like a camera failure or distort loop timing.
    detector = YoloDetector(settings)
    intervals = deque(maxlen=30)
    last_inference_ms = 0.0
    frames = 0
    try:
        with OpenCVCamera(settings) as camera:
            # Check a frame before opening a GUI, so source errors stay clear.
            frame = camera.read()
            if frame is None:
                raise RuntimeError("The video contained no readable frames.")
            print(f"Source frame: {frame.shape[1]} x {frame.shape[0]} | device: {settings.device}", flush=True)
            print("Click the image and press Q to stop or S to save. Ctrl+C also stops the program.", flush=True)
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, min(frame.shape[1], 1280), min(frame.shape[0], 960))
            while frame is not None:
                # The interval covers the complete application cycle, unlike
                # inference_ms, which is only the model's reported duration.
                started = perf_counter()
                detections, last_inference_ms = detector.predict(frame)
                annotated = draw_detections(cv2, frame, detections)
                fps_text = f"{len(intervals) / sum(intervals):.1f}" if intervals else "warming up"
                cv2.putText(annotated, f"Loop FPS: {fps_text} | inference: {last_inference_ms:.1f} ms",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv2.imshow(WINDOW, annotated)
                key = cv2.waitKey(1) & 0xFF
                frames += 1
                if key in (ord("q"), ord("Q"), 27):
                    break
                try:
                    if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break  # Some backends remove the window immediately on X.
                if key in (ord("s"), ord("S")):
                    save_snapshot(cv2, frame, annotated, detections)
                frame = camera.read()
                # Includes inference, drawing, display/event handling and next-frame capture.
                intervals.append(max(perf_counter() - started, 1e-9))
            if frame is None:
                print("Reached the end of the video (or the decoder stopped).", flush=True)
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass  # Do not hide an earlier source/model error during cleanup.
    print(f"Stopped after {frames} frames. Camera released.", flush=True)
    return 0


def main():
    args = parse_arguments()
    try:
        # Load and validate first, then apply transient command-line
        # overrides, and validate again before touching hardware.
        settings = load_settings(args.config)
        if args.source is not None:
            settings.camera_source = int(args.source) if args.source.isdecimal() else args.source
        for argument, field in (("device", "device"), ("model", "model_path"),
                                ("confidence", "confidence"), ("image_size", "image_size")):
            value = getattr(args, argument)
            if value is not None:
                setattr(settings, field, value)
        settings.validate()
        return run(settings)
    except KeyboardInterrupt:
        print("\nStopped by user. Camera cleanup completed.")
        return 0
    except Exception as exc:
        print(f"\nRover Vision could not start or continue:\n{exc}", file=sys.stderr)
        print("See README.md for troubleshooting. Keep device set to cpu until GPU setup is verified.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
