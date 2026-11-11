"""Exercise ROS callback policy with message doubles, including stale stamps."""
import importlib
import json
import math
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from rover_navigation.core import Grid
from rover_navigation.safety import SafetyGate


class NodeDouble:
    def __init__(self, name):
        self.parameters={}
        for method in ('create_publisher', 'create_client', 'create_subscription',
                       'create_service', 'create_timer'):
            setattr(self, method, MagicMock())

    def declare_parameter(self,name,value):
        self.parameters[name]=value

    def get_parameter(self,name):
        return types.SimpleNamespace(value=self.parameters[name])


class GuardAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ('rclpy', 'rclpy.clock', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
                 'action_msgs', 'action_msgs.srv', 'geometry_msgs', 'geometry_msgs.msg',
                 'nav_msgs', 'nav_msgs.msg', 'rosgraph_msgs', 'rosgraph_msgs.msg',
                 'ros_gz_interfaces', 'ros_gz_interfaces.msg',
                 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'tf2_ros')
        cls.modules = {name: MagicMock() for name in names}
        cls.modules['rclpy.node'].Node = NodeDouble
        # Limit exports to the Jazzy API: an unrestricted MagicMock also accepts
        # nonexistent imports and previously hid the qos_profile_clock crash.
        qos = types.ModuleType('rclpy.qos')
        qos.QoSProfile = types.SimpleNamespace
        qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT='best_effort')
        qos.qos_profile_sensor_data = object()
        cls.modules['rclpy.qos'] = qos
        cls.modules['tf2_ros'].TransformException = RuntimeError
        with patch.dict(sys.modules, cls.modules):
            cls.module = importlib.import_module('rover_navigation.guard_node')

    def setUp(self):
        self.node = self.module.Guard.__new__(self.module.Guard)
        self.node.gate = SafetyGate()
        self.node.gate.clock(2., 100.)
        self.node.last_odom = self.node.last_grid = -math.inf
        self.node.pose = self.node.grid = None
        self.node.stop_generation = 0
        self.node.in_contact = False
        self.node.reason_grids = {}
        self.node.status, self.node.published, self.node.status_sequence = MagicMock(), None, 0
        self.node.reset_odometry, self.node.armed_once = MagicMock(), False
        self.node.lost_since, self.node.last_reset, self.node.odometry_resets = None, -math.inf, 0
        self.node.get_logger = MagicMock()

    def test_startup_accepts_jazzy_qos_api_and_keeps_only_latest_clock_sample(self):
        node = self.module.Guard()
        clock = next(call for call in node.create_subscription.call_args_list
                     if call.args[1] == '/clock')
        self.assertEqual(clock.args[3].depth, 1)
        self.assertEqual(clock.args[3].reliability,
                         self.modules['rclpy.qos'].ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(node.gate.mode, 'stopped')
        self.assertEqual(node.create_timer.call_args.args[0], .05)

    def message(self, stamp=2.):
        return types.SimpleNamespace(header=types.SimpleNamespace(frame_id='odom', stamp=types.SimpleNamespace(
            sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))), child_frame_id='base_link',
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0., y=0., z=0.),
                orientation=types.SimpleNamespace(x=0., y=0., z=0., w=1.)), covariance=[0.]*36))

    def test_duplicate_and_stale_odometry_do_not_refresh_watchdog(self):
        with patch.object(self.module.time, 'monotonic', return_value=100.):
            self.node.odometry(self.message())
        with patch.object(self.module.time, 'monotonic', return_value=101.):
            self.node.odometry(self.message())
            self.node.odometry(self.message(1.))
        self.assertEqual(self.node.gate.sensors['pose'], 100.)

    def test_invalid_orientation_and_tracking_loss_covariance_stop_immediately(self):
        for mode in ('quaternion', 'covariance'):
            msg = self.message()
            if mode == 'quaternion':
                msg.pose.pose.orientation.w = 0.
            else:
                msg.pose.covariance[0] = 9999.
            self.node.gate.mode = 'autonomous'
            self.node.odometry(msg)
            self.assertEqual(self.node.gate.reason, 'invalid_odometry')
            self.assertIn('odometry', self.node.gate.fault['detail'])

    def test_nonfinite_covariance_diagnostic_stays_valid_json(self):
        msg = self.message()
        msg.pose.covariance[0] = math.nan
        self.node.odometry(msg)
        self.assertIsNone(self.node.gate.fault['detail']['odometry']['covariance_diagonal'][0])
        json.dumps(self.node.gate.fault, allow_nan=False)

    def test_lost_odometry_is_reset_only_before_any_motion(self):
        lost = self.message()
        lost.pose.covariance[0] = 9999.
        self.node.reset_odometry.service_is_ready.return_value = True
        for wall in (100., 101., 102.1, 103., 106.):
            with patch.object(self.module.time, 'monotonic', return_value=wall):
                self.node.odometry(lost)
        self.assertEqual(self.node.reset_odometry.call_async.call_count, 1)  # after 2 s, then rate-limited
        with patch.object(self.module.time, 'monotonic', return_value=107.2):
            self.node.odometry(lost)
        self.assertEqual(self.node.odometry_resets, 2)
        self.assertEqual(self.node.gate.reason, 'invalid_odometry')  # a reset never rearms motion
        self.node.armed_once = True
        with patch.object(self.module.time, 'monotonic', return_value=130.):
            self.node.odometry(lost)
        self.assertEqual(self.node.odometry_resets, 2)

    def test_clock_rewind_discards_cached_geometry(self):
        self.node.pose = (1., 2., 3.)
        self.node.grid = object()
        self.node.clock_received(types.SimpleNamespace(clock=types.SimpleNamespace(sec=0, nanosec=0)))
        self.assertIsNone(self.node.pose)
        self.assertIsNone(self.node.grid)
        self.assertEqual(self.node.gate.reason, 'simulation_clock_reset')

    def test_chassis_contact_stops_immediately_and_does_not_auto_resume(self):
        self.node.pub, self.node.cancel = MagicMock(), MagicMock()
        self.node.gate.mode = 'autonomous'
        self.node.contact(types.SimpleNamespace(contacts=[object()]))
        self.assertEqual(self.node.gate.reason, 'chassis_contact')
        self.assertEqual(json.loads(self.node.status.publish.call_args.args[0].data)['fault']['detail']['contacts'], 1)
        self.node.pub.publish.assert_called_once()
        self.node.contact(types.SimpleNamespace(contacts=[object()]))
        self.node.pub.publish.assert_called_once()  # sustained contact does not flood cancellation requests
        self.node.contact(types.SimpleNamespace(contacts=[]))
        self.assertEqual(self.node.gate.mode, 'stopped')

    def armed(self, transform_stamp=2.):
        node = self.node
        node.tf, node.pub, node.status, node.cancel = MagicMock(), MagicMock(), MagicMock(), MagicMock()
        node.count, node.published, node.status_sequence = 1, None, 0
        node.tf.lookup_transform.return_value = types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=int(transform_stamp), nanosec=0)))
        node.gate.health('depth', 100.)
        node.gate.health('pose', 100.)
        with patch.object(self.module.time, 'monotonic', return_value=100.):
            self.assertTrue(node.select('autonomous', types.SimpleNamespace(success=False, message='')).success)
        node.cancel.reset_mock()
        return node

    def published(self):
        return json.loads(self.node.status.publish.call_args.args[0].data)

    def test_safety_fault_is_published_in_its_tick_and_survives_later_stop(self):
        node = self.armed(transform_stamp=1.)
        with patch.object(self.module.time, 'monotonic', return_value=100.05):
            node.tick()
        status = self.published()
        self.assertEqual((status['mode'], status['reason']), ('stopped', 'map_localization_stale'))
        self.assertAlmostEqual(status['fault']['detail']['map_transform_age_seconds'], 1.)
        node.cancel.call_async.assert_called_once()
        with patch.object(self.module.time, 'monotonic', return_value=100.06):
            node.stop(None, types.SimpleNamespace(success=False, message=''))
        status = self.published()
        self.assertEqual(status['reason'], 'operator_stop')
        self.assertEqual(status['fault']['reason'], 'map_localization_stale')

    def test_unsafe_ground_fault_records_rejected_command_and_sample(self):
        node = self.armed()
        node.grid = Grid(np.zeros((100, 100), dtype=np.int8), .05, -2.5, -2.5)
        node.grid.data[:, 52:] = -1
        node.pose = (0., 0., 0.)
        node.gate.command('autonomous', .2, 0., 100.)
        with patch.object(self.module.time, 'monotonic', return_value=100.05):
            node.tick()
        fault = self.published()['fault']
        self.assertEqual(fault['reason'], 'unsafe_or_unobserved_ground')
        self.assertEqual(fault['detail']['command'], [.2, 0.])
        self.assertEqual(fault['detail']['cell'], -1)

    def test_enable_response_fences_statuses_published_before_arming(self):
        node = self.armed()
        reply = json.loads(node.status.publish.call_args.args[0].data)
        with patch.object(self.module.time, 'monotonic', return_value=100.):
            response = node.select('autonomous', types.SimpleNamespace(success=False, message=''))
        self.assertEqual(json.loads(response.message)['status_sequence'], self.published()['sequence'])
        self.assertGreater(self.published()['sequence'], reply['sequence'])

    def test_reason_evidence_does_not_authorize_motion_or_refresh_watchdog(self):
        node = self.armed()
        node.grid = Grid(np.full((100, 100), 100, np.int8), .05, -2.5, -2.5)
        node.last_grid = 2.
        node.pose = (0., 0., 0.)
        # Even incorrect optional "clear" evidence cannot bypass the cost grid.
        node.reason_grids[2.] = Grid(np.zeros((100, 100), np.int8), .05, -2.5, -2.5)
        node.gate.command('autonomous', .05, 0., 100.)
        with patch.object(self.module.time, 'monotonic', return_value=100.05):
            node.tick()
        fault = self.published()['fault']['detail']
        self.assertEqual(node.gate.mode, 'stopped')
        self.assertEqual(fault['causes'], ['hazard_evidence_unavailable'])
        self.assertEqual(len(fault['sweep']), 11)
        self.assertIn('cell_center', fault)
        sensors = dict(node.gate.sensors)
        msg = self.message()
        with patch.object(self.module, 'from_grid', return_value=Grid(np.full((100, 100), 6, np.int8), .05, -2.5, -2.5)):
            node.ground_reasons(msg)
        self.assertEqual(node.gate.sensors, sensors)
        self.assertEqual(node.gate.mode, 'stopped')
        self.assertEqual(node.gate.fault['detail']['causes'], ['excessive_slope', 'excessive_height_change'])

    def test_stale_reason_grid_cannot_explain_new_hazard(self):
        node = self.node
        node.grid = Grid(np.full((100, 100), 100, np.int8), .05, -2.5, -2.5)
        node.pose, node.last_grid = (0., 0., 0.), 2.
        node.reason_grids[1.] = Grid(np.full((100, 100), 2, np.int8), .05, -2.5, -2.5)
        self.assertEqual(node.rejection(.05, 0.)['causes'], ['hazard_evidence_unavailable'])


if __name__ == '__main__':
    unittest.main()
