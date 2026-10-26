"""Audit buffers must never pair a fresh camera image with an unrelated map."""
import importlib.util
from collections import deque
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


class AuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {name: MagicMock() for name in ('cv2', 'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
            'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf2_ros')}
        modules['rclpy.node'].Node = object
        path = Path(__file__).resolve().parents[3]/'scripts/halo_navigation_terrain_audit.py'
        spec = importlib.util.spec_from_file_location('phase_one_terrain_audit', path)
        cls.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(cls.module)

    def node(self):
        node = self.module.Audit.__new__(self.module.Audit)
        node.buffers = {name: {} for name in ('map', 'depth', 'rgb', 'info', 'local', 'reasons')}
        node.arrivals = {name: deque(maxlen=5000) for name in node.buffers}
        node.last_captured = -float('inf')
        node.tf = MagicMock()
        node.tf.can_transform.return_value = True
        return node

    def msg(self, stamp):
        return types.SimpleNamespace(header=types.SimpleNamespace(frame_id='camera_optical_frame',
            stamp=types.SimpleNamespace(sec=stamp, nanosec=0)), width=960, height=540)

    def test_buffers_are_bounded_during_absent_map_or_transform(self):
        node = self.node()
        for stamp in range(100):
            for name in node.buffers:
                node.cache(name, self.msg(stamp))
        self.assertEqual(len(node.buffers['rgb']), 40)
        self.assertEqual(len(node.buffers['map']), 10)

    def test_timing_keeps_wall_delivery_and_simulation_stamp_gaps_separate(self):
        result = self.module.timing_summary([(100., 1.), (100.2, 1.1), (100.6, 1.2)])
        self.assertAlmostEqual(result['max_gap_wall_seconds'], .4)
        self.assertAlmostEqual(result['max_stamp_gap_sim_seconds'], .1)
        self.assertAlmostEqual(result['rate_wall_hz'], 2/.6)

    def test_mismatched_stamps_and_missing_image_time_transform_wait(self):
        node = self.node()
        for name in node.buffers:
            node.cache(name, self.msg(2 if name == 'rgb' else 1))
        self.assertIsNone(node.frame())
        node.cache('rgb', self.msg(1))
        node.tf.can_transform.return_value = False
        self.assertIsNone(node.frame())
        node.tf.can_transform.return_value = True
        self.assertEqual(node.frame()[0].header.stamp.sec, 1)
        node.last_captured = 1.
        self.assertIsNone(node.frame())

    def test_dimension_mismatch_cannot_be_reported_as_synchronized(self):
        for field in ('width', 'optical_frame'):
            with self.subTest(field=field):
                node = self.node()
                for name in node.buffers:
                    node.cache(name, self.msg(1))
                if field == 'width':
                    node.buffers['rgb'][1.].width = 640
                else:
                    node.buffers['rgb'][1.].header.frame_id = 'unaligned_rgb'
                self.assertIsNone(node.frame())
