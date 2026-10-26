"""Supply BGR images. A future DepthAI source can implement the same methods."""

import sys
import cv2

from .config import Settings, project_path


class OpenCVCamera:
    def __init__(self, settings: Settings):
        self.settings = settings
        self.capture = None
        self.is_video = isinstance(settings.camera_source, str)

    def __enter__(self):
        source = self.settings.camera_source
        if self.is_video:
            # Video paths are project-relative unless the user supplied an
            # absolute path, matching the rest of the configuration system.
            path = project_path(source)
            if not path.is_file():
                raise RuntimeError(f"Video file not found: {path}")
            source = str(path)
        backend = cv2.CAP_ANY
        if self.settings.camera_backend != "auto" and not self.is_video:
            # DirectShow and Media Foundation are Windows camera backends;
            # they are not valid choices for a non-Windows webcam session.
            if sys.platform != "win32":
                raise RuntimeError("dshow and msmf are Windows-only. Use camera_backend: auto.")
            backend = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}[self.settings.camera_backend]
        self.capture = cv2.VideoCapture(source, backend)
        if not self.capture.isOpened():
            self.close()
            raise RuntimeError(
                f"Could not open source {source!r}. For a webcam, close other camera apps, "
                "enable Windows camera access for desktop apps, or try camera_source 1. "
                "You can also try camera_backend dshow or msmf on Windows."
            )
        if not self.is_video:
            try:
                # These are requested dimensions. A camera may select a
                # different supported mode, so the caller uses actual frame
                # dimensions rather than assuming these values were applied.
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.settings.camera_width)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.settings.camera_height)
            except Exception:
                self.close()
                raise
        return self

    def read(self):
        """Return a BGR uint8 frame, or None at the end of a video file."""
        # OpenCV returns both a success flag and the frame object. Treat an
        # empty frame as a failed read even when a backend reports success.
        success, frame = self.capture.read()
        if success and frame is not None and frame.size:
            return frame
        if self.is_video:
            return None
        raise RuntimeError("The webcam stopped delivering frames. Check its connection and camera permissions.")

    def close(self):
        if self.capture is not None:
            # release() is required for webcams so other applications can
            # access the device after this process exits.
            self.capture.release()
            self.capture = None

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
