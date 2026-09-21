"""Check coordinate and configuration boundaries without downloading a model."""

import unittest

from vision.config import Settings
from vision.detector import decode_boxes


class DetectionContractTests(unittest.TestCase):
    def test_original_frame_coordinates_and_centers(self):
        # Coordinates belong to a 1280x720 camera, even if YOLO input was 640.
        # This protects the contract used by drawing and exported metadata.
        detections = decode_boxes([[100, 80, 900, 680, 0.9, 0]], {0: "sample"}, 1280, 720)
        self.assertEqual(detections[0].bbox_xyxy, (100, 80, 900, 680))
        self.assertEqual(detections[0].center_xy, (500, 380))

    def test_clips_edges_and_rejects_invalid_geometry(self):
        # Only the valid box should survive: the first is clipped, the
        # second has inverted geometry, and the third contains NaN.
        detections = decode_boxes([
            [-2, -1, 1300, 730, 0.8, 0],
            [30, 40, 10, 50, 0.8, 0],
            [0, 0, float("nan"), 20, 0.8, 0],
        ], {0: "sample"}, 1280, 720)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].center_xy, (640, 360))

    def test_empty_predictions(self):
        self.assertEqual(decode_boxes([], {}, 640, 480), [])

    def test_invalid_configuration_rejected_before_hardware(self):
        # These failures must happen during validation, before camera/model
        # initialization can have side effects.
        for kwargs in ({"confidence": float("nan")}, {"confidence": 1.1},
                       {"image_size": 0}, {"image_size": 321},
                       {"camera_source": -1}, {"device": 0}, {"camera_width": True}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    Settings(**kwargs).validate()


if __name__ == "__main__":
    unittest.main()
