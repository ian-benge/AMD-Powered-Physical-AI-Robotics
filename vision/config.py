"""Configuration is independent of cameras and machine-learning packages."""

from dataclasses import dataclass
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def project_path(value: str | Path) -> Path:
    # Resolve relative paths from the repository, not from the shell's
    # current directory, so the app behaves the same from VS Code or a
    # shortcut/batch file.
    path = Path(value).expanduser()
    return path if path.is_absolute() else ROOT / path


@dataclass
class Settings:
    camera_source: int | str = 0
    camera_backend: str = "auto"
    camera_width: int = 640
    camera_height: int = 480
    model_path: str = "models/yolo11n.pt"
    device: str = "cpu"
    confidence: float = 0.4
    image_size: int = 640

    def validate(self):
        # Validate all user-controlled settings before opening a camera or
        # loading a model. This keeps configuration errors actionable and
        # avoids partially initialized hardware resources.
        if isinstance(self.camera_source, bool) or not isinstance(self.camera_source, (int, str)):
            raise ValueError("camera_source must be a camera index or a local video path.")
        if isinstance(self.camera_source, int) and self.camera_source < 0:
            raise ValueError("camera_source must be nonnegative.")
        if isinstance(self.camera_source, str) and not self.camera_source.strip():
            raise ValueError("camera_source cannot be empty.")
        if self.camera_backend not in ("auto", "dshow", "msmf"):
            raise ValueError("camera_backend must be auto, dshow, or msmf.")
        for name in ("camera_width", "camera_height", "image_size"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer.")
        if self.image_size < 32 or self.image_size % 32:
            raise ValueError("image_size must be a multiple of 32, such as 320 or 640.")
        if isinstance(self.confidence, bool) or not isinstance(self.confidence, (float, int)):
            raise ValueError("confidence must be a number between 0 and 1.")
        if not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be between 0 and 1.")
        if not isinstance(self.model_path, str) or not self.model_path.strip():
            raise ValueError("model_path must be a nonempty path.")
        if not isinstance(self.device, str) or not self.device.strip():
            raise ValueError('device must be a string, such as "cpu" or "0".')
        return self


def load_settings(path: str | Path = "config.json") -> Settings:
    # Read UTF-8 with an optional BOM because configuration files may have
    # been created by Windows tools that emit a BOM.
    try:
        data = json.loads(project_path(path).read_text(encoding="utf-8-sig"))
    except FileNotFoundError as exc:
        raise ValueError(f"Configuration file was not found: {project_path(path)}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("The configuration must be a JSON object.")
    try:
        # Constructing Settings also catches unknown JSON keys through the
        # dataclass constructor; validate() then checks value ranges/types.
        return Settings(**data).validate()
    except TypeError as exc:
        raise ValueError(f"Unknown configuration option: {exc}") from exc
