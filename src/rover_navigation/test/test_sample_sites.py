from collections import Counter, OrderedDict, deque
import importlib
import sys
from types import SimpleNamespace as NS
import unittest
import json
from pathlib import Path
import tempfile
import io
from unittest.mock import MagicMock, patch
import numpy as np
from rover_navigation.core import transform
from rover_navigation.sample_sites import SiteStore, depth_location
from test_site_tracking import observation, update, confirmed


class SampleGeometryTests(unittest.TestCase):
    def test_median_depth_rejects_outliers_and_respects_padding_endianness(self):
        rows = np.full((20, 24), 2000, dtype='>u2')
        rows[9, 9] = 8000
        p = depth_location(rows.tobytes(), '16UC1', 20, 20, 48, True,
                           [10., 0., 10., 0., 10., 10., 0., 0., 1.], [4., 4., 16., 16.])
        np.testing.assert_allclose(p, [0., 0., 2.])

    def test_missing_depth_and_unaligned_boxes_are_rejected(self):
        rows = np.zeros((20, 20), dtype='<f4')
        for box in ([4., 4., 16., 16.], [-1., 0., 5., 5.]):
            with self.assertRaises(ValueError):
                depth_location(rows.tobytes(), '32FC1', 20, 20, 80, False,
                               [10., 0., 10., 0., 10., 10., 0., 0., 1.], box)

    def test_merge_counts_classes_and_localization_projection(self):
        store = SiteStore()
        update(store, 1., 0., [observation(point=(1., 2., 0.), confidence=.5)])
        update(store, 2., .2, [observation('mudstone_slab', point=(1.1, 2., 0.), confidence=.8)])
        update(store, 2., .2, [observation(point=(1.1, 2., 0.), confidence=.9)])
        self.assertEqual(len(store.objects), 1)
        self.assertEqual(store.objects[0]['detection_count'], 2)
        self.assertEqual(store.objects[0]['max_confidence'], .8)
        before = np.array(store.objects[0]['odom_position'])
        after = transform([before], [3., 0., 0.], [0., 0., 0., 1.])[0]
        np.testing.assert_allclose(after-before, [3., 0., 0.])
        np.testing.assert_allclose(store.objects[0]['odom_position'], before)


class SampleAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg', 'tf2_ros', 'vision_msgs', 'vision_msgs.msg',
                 'visualization_msgs', 'visualization_msgs.msg')
        modules = {name: MagicMock() for name in names}
        modules['rclpy.node'].Node = object
        modules['tf2_ros'].TransformException = RuntimeError
        with patch.dict(sys.modules, modules):
            cls.module = importlib.import_module('rover_navigation.sample_sites_node')

    def node(self):
        node = self.module.SampleSites.__new__(self.module.SampleSites)
        node.guard, node.guard_wall = dict(ready=True, sim_time=3.000000002), self.module.time.monotonic()
        node.detector, node.detector_wall = dict(ready=True), self.module.time.monotonic()
        node.fault, node.counts = '', Counter()
        node.pending, node.seen = deque(), deque()
        node.depths, node.infos, node.images = OrderedDict(), OrderedDict(), OrderedDict()
        node.odometry = deque()
        node.suspension, node.last_processed, node.processing_ms, node.localization_fault = '', -float('inf'), 0., ''
        node.event_stream = io.StringIO()
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 3000000000
        node.store, node.tf = SiteStore(), MagicMock()
        return node

    def test_missing_exact_transform_waits_without_using_latest(self):
        node = self.node()
        stamp = NS(sec=3, nanosec=2)
        header = NS(stamp=stamp, frame_id='camera_optical_frame')
        msg = NS(header=header, detections=[])
        node.pending.append((msg, self.module.time.monotonic()))
        node.depths[(3, 2)] = NS(header=header, width=20, height=20)
        node.infos[(3, 2)] = NS(header=header, width=20, height=20)
        node.images[(3, 2)] = NS(header=header, width=20, height=20)
        node.tf.lookup_transform.side_effect = RuntimeError('Exact transform missing')
        with patch.object(self.module.Time, 'from_msg', return_value='image-stamp'):
            node.process()
        node.tf.lookup_transform.assert_called_once_with('odom', 'camera_optical_frame', 'image-stamp')
        self.assertEqual(len(node.pending), 1)
        self.assertFalse(node.store.objects)

    def test_marker_reprojection_and_persisted_odom_position(self):
        node = self.node()
        node.store = confirmed()
        node.store.objects[0]['odom_position'] = [1., 2., 0.]
        node.pub, node.marker_pub, node.tentative_pub, node.get_clock = MagicMock(), MagicMock(), MagicMock(), MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 3000000000
        node.archived, node.odometry_resets, node.last_save = [], 1, 0.
        node.tf.lookup_transform.return_value = NS(header=NS(stamp=NS(sec=3, nanosec=0)), transform=NS(
            translation=NS(x=3., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
        with tempfile.TemporaryDirectory() as folder:
            node.output = Path(folder)/'objects.json'
            node.publish()
            sent = json.loads(node.pub.publish.call_args.args[0].data)
            self.assertEqual(sent['objects'][0]['map_position'], [4., 2., 0.])
            stored = json.loads(node.output.read_text())
            self.assertEqual(stored['tracks'][0]['odom_position'], [1., 2., 0.])
            self.assertEqual(sent['schema_version'], 2)
            node.tf.lookup_transform.return_value.transform.translation.x = 5.
            node.publish()
            sent = json.loads(node.pub.publish.call_args.args[0].data)
            self.assertEqual(sent['objects'][0]['map_position'], [6., 2., 0.])
            self.assertEqual(node.store.objects[0]['odom_position'], [1., 2., 0.])

    def test_tentative_is_never_published_in_confirmed_array(self):
        node = self.node()
        update(node.store, 1., 0.)
        node.pub, node.marker_pub, node.tentative_pub, node.get_clock = [MagicMock() for _ in range(4)]
        node.get_clock.return_value.now.return_value.nanoseconds = 3000000000
        node.archived, node.odometry_resets, node.last_save = [], 1, 0.
        node.tf.lookup_transform.return_value = NS(header=NS(stamp=NS(sec=3, nanosec=0)), transform=NS(
            translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
        with tempfile.TemporaryDirectory() as folder:
            node.output = Path(folder)/'objects.json'
            node.publish()
            sent = json.loads(node.pub.publish.call_args.args[0].data)
            self.assertEqual(sent['objects'], [])
            self.assertEqual(len(sent['tentative_objects']), 1)

    def test_detector_failure_suspends_without_negative_evidence(self):
        node = self.node()
        node.store = confirmed()
        node.detector['ready'] = False
        node.pending.append((NS(), self.module.time.monotonic()))
        node.process()
        self.assertEqual(node.store.objects[0]['state'], 'confirmed')
        self.assertEqual(node.store.objects[0]['contradiction_count'], 0)

    def test_odometry_reset_archives_tracks_and_starts_distinct_epoch(self):
        node = self.node()
        node.store, node.archived, node.odometry_resets = confirmed(), [], 0
        node.guard_received(NS(data=json.dumps(dict(ready=True, odometry_resets=1, sim_time=3.))))
        self.assertFalse(node.store.objects)
        self.assertEqual(node.store.epoch, 1)
        self.assertEqual(node.archived[0]['objects'][0]['state'], 'confirmed')

    def test_covariance_requires_image_time_bracket(self):
        node = self.node()
        with self.assertRaises(ValueError):
            node.covariance_at(3.)
        msg = NS(pose=NS(covariance=np.zeros(36), pose=NS(position=NS(x=0., y=0., z=0.))))
        node.odometry.extend([(2.98, msg), (3.02, msg)])
        np.testing.assert_allclose(node.covariance_at(3.), np.zeros((6, 6)))

    def test_current_projection_uses_simulation_clock_not_delayed_guard_status(self):
        node = self.node()
        node.store, node.last_processed = confirmed(), 3.
        node.guard['sim_time'] = 2.8
        node.pub, node.marker_pub, node.tentative_pub = MagicMock(), MagicMock(), MagicMock()
        node.archived, node.odometry_resets, node.last_save = [], 1, 0.
        node.tf.lookup_transform.return_value = NS(header=NS(stamp=NS(sec=3, nanosec=0)), transform=NS(
            translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
        with tempfile.TemporaryDirectory() as folder:
            node.output = Path(folder)/'objects.json'
            node.publish()
        sent = json.loads(node.pub.publish.call_args.args[0].data)
        self.assertTrue(sent['ready'])
        self.assertEqual(len(sent['objects']), 1)
        self.assertEqual(sent['map_transform_age_seconds'], 0.)
        node.tf.lookup_transform.assert_called_with('map', 'odom', node.get_clock().now())
