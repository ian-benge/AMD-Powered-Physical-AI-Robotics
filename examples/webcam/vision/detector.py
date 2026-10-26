"""Host-side YOLO inference; no webcam or display code belongs here."""

from dataclasses import dataclass
import os
import math

from .config import ROOT, Settings, project_path


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    center_xy: tuple[float, float]


def decode_boxes(rows, names, frame_width: int, frame_height: int) -> list[Detection]:
    """YOLO has already mapped boxes back to the original image, not its input tensor."""
    detections = []
    for row in rows:
        # Ultralytics supplies x1, y1, x2, y2, confidence, and class id.
        # Convert values to floats so tensors and ordinary Python sequences
        # follow the same path.
        x1, y1, x2, y2, confidence, class_value = map(float, row[:6])
        if not all(math.isfinite(v) for v in (x1, y1, x2, y2, confidence, class_value)):
            continue
        # Keep coordinates inside the source image. The detector contract
        # uses source-frame pixels even though YOLO may resize its input.
        x1 = min(max(x1, 0.0), float(frame_width))
        x2 = min(max(x2, 0.0), float(frame_width))
        y1 = min(max(y1, 0.0), float(frame_height))
        y2 = min(max(y2, 0.0), float(frame_height))
        if x2 <= x1 or y2 <= y1:
            continue
        class_id = int(class_value)
        # Store a stable, serializable representation for drawing and JSON
        # output instead of leaking Ultralytics result objects to the app.
        detections.append(Detection(
            class_id=class_id,
            class_name=str(names[class_id]),
            confidence=confidence,
            bbox_xyxy=(x1, y1, x2, y2),
            center_xy=((x1 + x2) / 2.0, (y1 + y2) / 2.0),
        ))
    return detections


class YoloDetector:
    def __init__(self, settings: Settings):
        # Keep optional Ultralytics settings local to this project.
        cache = ROOT / ".cache" / "ultralytics"
        cache.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("YOLO_CONFIG_DIR", str(cache))
        from ultralytics import YOLO

        self.settings = settings
        weights = project_path(settings.model_path)
        weights.parent.mkdir(parents=True, exist_ok=True)
        if not weights.exists():
            if weights.name != "yolo11n.pt":
                raise RuntimeError(f"Model file not found: {weights}. Copy your trained weights there first.")
            print("Downloading pretrained YOLO11n weights. This first step requires internet access.", flush=True)
        try:
            # Model creation is intentionally done once at startup. Reusing
            # it for every frame avoids repeated loading and initialization.
            self.model = YOLO(str(weights))
        except Exception as exc:
            raise RuntimeError(
                f"Could not load {weights.name}. Check the model path and internet connection. "
                "For the first download, GitHub model assets must be reachable. "
                f"Original error: {exc}"
            ) from exc
        if self.model.task != "detect":
            raise RuntimeError("This starter expects an object-detection model, not a segmentation or pose model.")

    def predict(self, frame):
        # Fail early with a clear contract error instead of allowing a lower
        # level model/backend exception to obscure the input problem.
        if frame.ndim != 3 or frame.shape[2] != 3 or str(frame.dtype) != "uint8":
            raise ValueError("The detector expects a three-channel BGR uint8 image.")
        # Ultralytics handles preprocessing and maps returned boxes back to
        # the original frame dimensions.
        result = self.model.predict(
            source=frame,
            device=self.settings.device,
            imgsz=self.settings.image_size,
            conf=self.settings.confidence,
            verbose=False,
            save=False,
        )[0]
        height, width = frame.shape[:2]
        rows = [] if result.boxes is None else result.boxes.data.cpu().tolist()
        detections = decode_boxes(rows, result.names, width, height)
        # Ultralytics reports model-only inference time separately from the
        # camera, drawing, display, and file-writing work in app.py.
        return detections, float(result.speed.get("inference") or 0.0)
