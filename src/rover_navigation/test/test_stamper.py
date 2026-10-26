"""Verify the independent downstream lease while ROS time is frozen."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.)
        self.angular = types.SimpleNamespace(z=0.)


class Stamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.twist = Twist()


class StamperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {name: MagicMock() for name in ('rclpy', 'rclpy.node', 'rclpy.clock', 'geometry_msgs', 'geometry_msgs.msg')}
        modules['rclpy.node'].Node = object
        modules['geometry_msgs.msg'].Twist, modules['geometry_msgs.msg'].TwistStamped = Twist, Stamped
        source = Path(__file__).resolve().parents[3]/'src/four_wheel_sim/four_wheel_sim/twist_stamper.py'
        spec = importlib.util.spec_from_file_location('tested_stamper', source)
        cls.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(cls.module)

    def node(self, timeout=.35):
        node = self.module.TwistStamper.__new__(self.module.TwistStamper)
        node.timeout, node.frame, node.received = timeout, 'base_link', 1.
        node.pub = MagicMock()
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = 'frozen ROS clock'
        return node

    def test_lost_guard_produces_zero_despite_frozen_ros_clock(self):
        node = self.node()
        command = Twist()
        command.linear.x = .2
        with patch.object(self.module.time, 'monotonic', return_value=1.):
            node.cb(command)
        with patch.object(self.module.time, 'monotonic', return_value=1.4):
            node.expire()
        output = node.pub.publish.call_args.args[0]
        self.assertEqual(output.header.stamp, 'frozen ROS clock')
        self.assertEqual(output.twist.linear.x, 0.)
        self.assertEqual(output.twist.angular.z, 0.)

    def test_legacy_default_has_no_new_expiry_behavior(self):
        node = self.node(timeout=0.)
        with patch.object(self.module.time, 'monotonic', return_value=99.):
            node.expire()
        node.pub.publish.assert_not_called()

    def test_fresh_command_is_not_overridden_by_watchdog(self):
        node = self.node()
        with patch.object(self.module.time, 'monotonic', return_value=1.1):
            node.expire()
        node.pub.publish.assert_not_called()
