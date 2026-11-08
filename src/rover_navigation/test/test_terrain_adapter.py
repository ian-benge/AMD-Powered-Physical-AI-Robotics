"""Check time-aligned depth processing and persistent operator clearance."""
from collections import deque
import importlib
import hashlib
import io
import json
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np


def tf(x=0., y=0., z=0.):
    return types.SimpleNamespace(transform=types.SimpleNamespace(
        translation=types.SimpleNamespace(x=x, y=y, z=z),
        rotation=types.SimpleNamespace(x=0., y=0., z=0., w=1.)))


class TerrainAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ('message_filters', 'rclpy', 'rclpy.duration', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
                 'grid_map_msgs', 'grid_map_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'sensor_msgs',
                 'sensor_msgs.msg', 'sensor_msgs_py', 'sensor_msgs_py.point_cloud2',
                 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'tf2_ros')
        modules = {name: MagicMock() for name in names}
        modules['rclpy.node'].Node = object
        modules['tf2_ros'].TransformException = RuntimeError
        with patch.dict(sys.modules, modules):
            cls.module = importlib.import_module('rover_navigation.terrain_node')

    def node(self):
        node = self.module.Terrain.__new__(self.module.Terrain)
        node.compute = self.module.CpuTerrainCompute()
        node.compute_error = ''
        node.tf = MagicMock()
        node.start = node.start_map = None
        node.latest = None, None, 10.
        node.pending = deque(maxlen=20)
        node.last_stamp, node.last_global = None, -float('inf')
        node.global_cloud = node.cached_cloud = node.memory = None
        node.local_reasons_pub, node.map_reasons_pub = MagicMock(), MagicMock()
        node.get_logger = MagicMock()
        node.get_parameter = lambda name: types.SimpleNamespace(value={
            'clear_start_radius_m': 1.1, 'max_slope_deg': 12., 'max_step_m': .03, 'map_size_m': 20.,
            'hazard_confirmations': 2}[name])
        return node

    def sample(self, stamp, received=10.):
        sec = int(stamp)
        stamp = types.SimpleNamespace(sec=sec, nanosec=round((stamp-sec)*1e9))
        header = types.SimpleNamespace(stamp=stamp, frame_id='camera_optical_frame')
        depth = types.SimpleNamespace(header=header, width=48, height=48, step=192, encoding='32FC1',
                                      is_bigendian=False, data=np.ones((48, 48), dtype='<f4').tobytes())
        info = types.SimpleNamespace(header=header, width=48, height=48,
                                     k=[40., 0., 24., 0., 40., 24., 0., 0., 1.])
        return depth, info, received

    def test_newest_untransformable_frame_does_not_starve_older_ready_depth(self):
        node = self.node()
        older, newer = self.sample(2.), self.sample(2.1)
        node.pending.extend([older, newer])
        node.latest = newer
        node.tf.can_transform.return_value = False

        def lookup(target, source, when):
            if when.nanosec or source != 'camera_optical_frame' and source != 'base_link':
                raise RuntimeError('Extrapolation into the future')
            return tf()

        node.tf.lookup_transform.side_effect = lookup
        node.local_pub = MagicMock()
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 2_200_000_000
        with patch.object(self.module.time, 'monotonic', return_value=10.), \
                patch.object(self.module.Time, 'from_msg', side_effect=lambda stamp: stamp):
            node.update()
        self.assertEqual(node.last_stamp, 2.)
        self.assertIs(node.local_pub.publish.call_args.args[0].header.stamp, older[0].header.stamp)
        self.assertEqual(list(node.pending), [newer])
        # A waiting lookup would block the executor that must receive the TF.
        self.assertTrue(all(not call.kwargs for call in node.tf.lookup_transform.call_args_list))

    def test_missing_transform_is_retried_without_new_camera_message(self):
        node = self.node()
        node.pending.append(self.sample(2.))
        node.tf.lookup_transform.side_effect = RuntimeError('Transform not received yet')
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            self.assertIsNone(node.ready_depth(2.2))
        self.assertEqual(len(node.pending), 1)
        node.tf.lookup_transform.side_effect = None
        node.tf.lookup_transform.return_value = tf()
        with patch.object(self.module.time, 'monotonic', return_value=10.1):
            self.assertIsNotNone(node.ready_depth(2.3))

    def test_expired_or_already_processed_depth_cannot_be_republished_as_fresh(self):
        for now_sim, wall, processed in ((2.7, 10., None), (2.2, 11.1, None), (2.2, 10., 2.)):
            with self.subTest(now_sim=now_sim, wall=wall, processed=processed):
                node = self.node()
                node.pending.append(self.sample(2.))
                node.last_stamp = processed
                with patch.object(self.module.time, 'monotonic', return_value=wall):
                    self.assertIsNone(node.ready_depth(now_sim))
                node.tf.lookup_transform.assert_not_called()
                self.assertFalse(node.pending)

    def test_depth_without_matching_base_pose_is_not_ready(self):
        node = self.node()
        node.pending.append(self.sample(2.))
        node.tf.lookup_transform.side_effect = [tf(), RuntimeError('Base pose not received')]
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            self.assertIsNone(node.ready_depth(2.2))

    def test_newest_ready_frame_is_selected_but_future_clock_frame_waits(self):
        node = self.node()
        older, newest, future = self.sample(2.), self.sample(2.1), self.sample(2.3)
        node.pending.extend([older, newest, future])
        node.tf.lookup_transform.return_value = tf()
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            self.assertIs(node.ready_depth(2.2)[0], newest[0])
        self.assertEqual(node.tf.lookup_transform.call_count, 2)

    def test_clock_rewind_discards_pre_reset_queue_map_and_clearance(self):
        node = self.node()
        node.last_stamp = 2.
        node.latest = self.sample(1.)
        node.pending.extend([self.sample(1.9), node.latest])
        node.start = node.start_map = np.array([0., 0., 0.])
        node.memory = object()
        node.global_cloud = np.array([[0., 0., 0.]])
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
        node.tf.lookup_transform.side_effect = RuntimeError('Waiting for reset transforms')
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.update()
        self.assertEqual(list(node.pending), [node.latest])
        self.assertIsNone(node.memory)
        self.assertIsNone(node.start)
        self.assertIsNone(node.start_map)
        self.assertIsNone(node.global_cloud)
        self.assertIsNone(node.last_stamp)

    def test_camera_queue_is_bounded(self):
        node = self.node()
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            for i in range(25):
                depth, info, _ = self.sample(2.+i*.01)
                node.camera(depth, info)
        self.assertEqual(len(node.pending), 20)
        self.assertIs(node.pending[-1], node.latest)

    def test_adaptive_updates_pause_without_localization_but_keep_fresh_hazards(self):
        node = self.node()
        node.adaptive_enabled = True
        node.localization = MagicMock()
        node.localization.check.return_value = 'localization invalid, uncertain or stale'
        node.latest = self.sample(2.)
        node.pending.append(node.latest)
        node.local_pub, node.map_pub = MagicMock(), MagicMock()
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 2_000_000_000
        transform = tf()
        transform.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=2, nanosec=0))
        node.tf.lookup_transform.return_value = transform
        node.tf.can_transform.return_value = True
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.update()
        self.assertEqual(node.evidence.status['suspended'], 'localization invalid, uncertain or stale')
        self.assertEqual(node.last_stamp, 2.)
        node.local_pub.publish.assert_called_once()
        # A map correction must itself be fresh before any cells can be cleared.
        node.localization.check.return_value = ''
        transform.header.stamp.sec = 0
        node.latest = self.sample(2.2)
        node.pending.append(node.latest)
        node.get_clock.return_value.now.return_value.nanoseconds = 2_200_000_000
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.update()
        self.assertEqual(node.evidence.status['suspended'], 'map localization unavailable or stale')

    def test_adaptive_checks_current_map_correction_despite_future_dated_latest_tf(self):
        node = self.node()
        node.adaptive_enabled = True
        node.localization = MagicMock()
        node.localization.check.return_value = ''
        node.latest = self.sample(2.)
        node.pending.append(node.latest)
        node.local_pub, node.map_pub = MagicMock(), MagicMock()
        node.get_clock = MagicMock()
        current_time = node.get_clock.return_value.now.return_value
        current_time.nanoseconds = 2_000_000_000

        def lookup(target, source, when):
            transform = tf()
            transform.header = types.SimpleNamespace(stamp=types.SimpleNamespace(
                sec=2, nanosec=0 if when is current_time else 100_000_000))
            if source == 'odom' and when is current_time and unavailable[0]:
                raise RuntimeError('Current transform is not bracketed')
            return transform

        unavailable = [False]
        node.tf.lookup_transform.side_effect = lookup
        node.tf.can_transform.return_value = True
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.update()
        self.assertEqual(node.evidence.status['suspended'], '')
        node.tf.lookup_transform.assert_any_call('map', 'odom', current_time)
        # A failed current-time lookup must suspend clearing but retain fresh
        # local safety input, rather than trusting the latest future stamp.
        unavailable[0] = True
        node.latest = self.sample(2.2)
        node.pending.append(node.latest)
        current_time.nanoseconds = 2_200_000_000
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.update()
        self.assertEqual(node.evidence.status['suspended'], 'map localization unavailable or stale')
        self.assertEqual(node.local_pub.publish.call_count, 2)

    def test_gpu_transport_failure_does_not_publish_fresh_safety_grid(self):
        node=self.node()
        node.pending.append(self.sample(2.));node.latest=node.pending[-1]
        node.tf.lookup_transform.return_value=tf()
        node.local_pub=MagicMock();node.compute=MagicMock()
        node.event_stream=io.StringIO()
        node.compute.depth.side_effect=OSError('GPU service unavailable')
        node.get_clock=MagicMock();node.get_clock.return_value.now.return_value.nanoseconds=2_000_000_000
        with patch.object(self.module.time,'monotonic',return_value=10.):node.update()
        node.local_pub.publish.assert_not_called()
        self.assertIn('GPU service unavailable',node.compute_error)
        self.assertEqual(node.event_stream.getvalue(), '')

    def test_frame_record_pins_encoded_input_transform_and_both_published_maps(self):
        node=self.node()
        node.event_stream=io.StringIO()
        node.evidence=None
        depth,info,_=self.sample(2.)
        grid=self.module.Grid(np.array([[0,100],[-1,20]],np.int8),.025,1.,2.)
        global_grid=self.module.Grid(np.array([[100]],np.int8),.05,-1.,-1.)
        node.record_frame(depth,info,tf(z=.762),tf(x=.3),grid,
                          {'observation_count':np.ones((2,2))},grid,global_grid)
        row=json.loads(node.event_stream.getvalue())
        self.assertEqual(row['input_sha256'],hashlib.sha256(depth.data).hexdigest())
        self.assertEqual(row['local']['sha256'],hashlib.sha256(grid.data.tobytes()).hexdigest())
        self.assertEqual(row['global_map']['sha256'],hashlib.sha256(global_grid.data.tobytes()).hexdigest())
        self.assertEqual(row['projection_to_odom']['translation'],[.3,0.,0.])
        self.assertEqual(row['thresholds'],{'slope_deg':12.,'height_change_m':.03})

    def test_fresh_restart_does_not_accumulate_terrain_before_confirmation(self):
        node = self.node()
        node.mapping_on_confirm = True
        node.latest = self.sample(2.)
        node.pending.append(node.latest)
        node.local_pub = MagicMock()
        node.update()
        self.assertIsNone(node.memory)
        node.local_pub.publish.assert_not_called()
        node.tf.lookup_transform.assert_not_called()

    def test_failed_confirmation_does_not_prevent_retry(self):
        node = self.node()
        node.tf.lookup_transform.side_effect = [tf(1., 1.), RuntimeError('Map not ready')]
        response = types.SimpleNamespace(success=False, message='')
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.confirm(None, response)
        self.assertFalse(response.success)
        self.assertIsNone(node.start)
        node.tf.lookup_transform.side_effect = [tf(1., 1.), tf(4., 2.)]
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.confirm(None, response)
        self.assertTrue(response.success)
        np.testing.assert_allclose(node.start_map, [5., 3., 0.])

    def test_loop_correction_preserves_map_anchor_and_reprojects_local_clearance(self):
        node = self.node()
        node.start, node.start_map = np.array([[1., 1., 0.]]), np.array([5., 3., 0.])
        stamp = types.SimpleNamespace(sec=2, nanosec=0)
        header = types.SimpleNamespace(stamp=stamp, frame_id='camera_optical_frame')
        depth = types.SimpleNamespace(header=header, width=48, height=48, step=192, encoding='32FC1',
                                      is_bigendian=False, data=np.ones((48, 48), dtype='<f4').tobytes())
        info = types.SimpleNamespace(k=[40., 0., 24., 0., 40., 24., 0., 0., 1.])
        node.latest = depth, info, 10.
        node.pending.append(node.latest)
        node.last_stamp, node.last_global = None, -float('inf')
        node.global_cloud = np.array([[5., 3., 0.]])
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 2_000_000_000
        node.tf.lookup_transform.side_effect = lambda target, child, *args, **kwargs: {
            ('odom', 'camera_optical_frame'): tf(), ('odom', 'base_link'): tf(1., 1.),
            ('map', 'odom'): tf(3., 2.), ('odom', 'map'): tf(-3., -2.)}[(target, child)]
        node.local_pub, node.map_pub, node.publish_layers = MagicMock(), MagicMock(), MagicMock()
        positions = []
        seed = node.seeded

        def capture(grid, position):
            positions.append(np.array(position))
            return seed(grid, position)

        node.seeded = capture
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            node.update()
        self.assertEqual(len(positions), 2)
        np.testing.assert_allclose(positions[0], [2., 1., 0.])
        np.testing.assert_allclose(positions[1], [5., 3., 0.])
