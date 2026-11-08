import importlib
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


class NodeDouble:
    # rclpy.Node.clients is read-only; preserve that API in the startup test.
    clients = property(lambda self: iter(()))
    parameter_overrides = {}

    def __init__(self, name):
        self.create_client = MagicMock()
        self.create_subscription = MagicMock()
        self.create_publisher = MagicMock()
        parameters = {}
        self.declare_parameter = MagicMock(side_effect=lambda name,value: parameters.setdefault(
            name,self.parameter_overrides.get(name,value)))
        self.get_parameter = lambda name: SimpleNamespace(value=parameters[name])


class ControlsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {name: MagicMock() for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.time',
            'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
            'rosgraph_msgs', 'rosgraph_msgs.msg',
            'tf2_ros', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv')}
        modules['rclpy.node'].Node = NodeDouble
        with patch.dict(sys.modules, modules):
            cls.module = importlib.import_module('rover_navigation.mission_controls')

    def test_controls_construct_using_real_node_property_contract(self):
        node = self.module.Controls()
        self.assertEqual(set(node.service_clients), {'start', 'stop', 'manual', 'confirm'})

    def test_declared_qualification_limits_reject_a_cached_higher_speed(self):
        with patch.dict(NodeDouble.parameter_overrides,dict(max_linear_speed=.05,max_angular_speed=.3)):
            node=self.module.Controls()
        self.assertEqual(node.manual_input.limits['linear']['max'],.05)
        node.latest['status']=dict(data=dict(mode='manual',ready=True),received=self.module.time.monotonic())
        node.manual_pub=MagicMock();token=node.manual_input.arm()
        self.assertTrue(node.drive(dict(session=token,sequence=1,linear=.05,angular=0.))['success'])
        with self.assertRaises(ValueError):node.drive(dict(session=token,sequence=2,linear=.2,angular=0.))

    def test_image_detail_failure_cannot_crash_controls_or_refresh_guard_health(self):
        node = self.module.Controls()
        message = MagicMock()
        with patch.object(self.module, 'image_to_bgr', side_effect=ValueError('unsupported camera input')):
            node.raw_image(message)
        self.assertIs(node.camera[0], message)
        self.assertFalse(node.latest['camera_health']['data']['available'])
        self.assertNotIn('status', node.latest)

    def test_http_stop_routes_to_service_and_cross_origin_start_is_rejected(self):
        node = MagicMock()
        node.snapshot.return_value = dict(status=dict(mode='stopped'))
        node.call.return_value = dict(success=True, message='Stopped')
        server = ThreadingHTTPServer(('127.0.0.1', 0), self.module.handler(node))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            connection = HTTPConnection('127.0.0.1', server.server_port)
            connection.request('POST', '/start', headers={'Host': '127.0.0.1:8774', 'Origin': 'https://elsewhere.invalid'})
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read()
            node.call.assert_not_called()
            connection.request('POST', '/stop', headers={'Host': '127.0.0.1:8774'})
            response = connection.getresponse()
            self.assertTrue(json.loads(response.read())['success'])
            node.call.assert_called_once_with('stop')
            node.map_image = b'png-test-bytes'
            connection.request('GET', '/map.png?t=123', headers={'Host': '127.0.0.1:8874'})
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(response.read(), b'png-test-bytes')
            connection.close()
        finally:
            server.shutdown()
            server.server_close()

    def test_motion_rejection_names_unknown_ground_instead_of_generic_not_ready(self):
        node = self.module.Controls()
        node.latest['status'] = dict(data=dict(mode='stopped', ready=True,
                                             reason='unsafe_or_unobserved_ground'), received=self.module.time.monotonic())
        with self.assertRaisesRegex(ValueError, 'unknown or unsafe ground'):
            node.drive({})
        node.create_publisher.assert_not_called()

    def test_drive_does_not_compute_visualization_or_call_supervisor(self):
        node = self.module.Controls()
        node.latest['status'] = dict(data=dict(mode='manual', ready=True), received=self.module.time.monotonic())
        node.manual_pub = MagicMock()
        token = node.manual_input.arm()
        node.snapshot = MagicMock(side_effect=AssertionError('No visualization in command path'))
        self.assertTrue(node.drive(dict(session=token, sequence=1, linear=0., angular=0.))['success'])
        node.snapshot.assert_not_called()
        node.manual_pub.publish.assert_called_once()
