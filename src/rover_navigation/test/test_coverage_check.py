"""Independent pose scoring must handle frame alignment without concealing drift."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch


class CoverageCheckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {name: MagicMock() for name in ('rclpy', 'rclpy.node', 'rclpy.qos',
            'nav_msgs', 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg')}
        modules['rclpy.node'].Node = object
        path = Path(__file__).resolve().parents[3]/'scripts/halo_navigation_coverage_check.py'
        spec = importlib.util.spec_from_file_location('coverage_acceptance_tested', path)
        cls.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(cls.module)

    def pose(self, x=0., y=0., yaw=0.):
        return dict(x=x, y=y, yaw=yaw, wall=100.)

    def test_origin_and_initial_heading_do_not_count_as_relative_drift(self):
        start = dict(truth=self.pose(4., 2., math.pi/2), estimate=self.pose())
        end = dict(truth=self.pose(4., 3., math.pi/2), estimate=self.pose(1.))
        score = self.module.compare(start, end)
        self.assertAlmostEqual(score['truth_displacement_m'], 1.)
        self.assertAlmostEqual(score['relative_endpoint_error_m'], 0.)
        self.assertAlmostEqual(score['relative_yaw_error_deg'], 0.)

    def test_lateral_drift_and_final_yaw_disagreement_remain_visible(self):
        start = dict(truth=self.pose(), estimate=self.pose())
        end = dict(truth=self.pose(1.), estimate=self.pose(1., .3, math.radians(8.)))
        score = self.module.compare(start, end)
        self.assertAlmostEqual(score['relative_endpoint_error_m'], .3)
        self.assertAlmostEqual(score['relative_yaw_error_deg'], 8.)
