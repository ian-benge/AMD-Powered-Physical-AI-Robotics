import math
import unittest
import numpy as np

from rover_navigation.core import Grid, TerrainMemory, terrain_reasons, footprint_polygon, transform
from rover_navigation.diagnostics import localization_health, map_overlay, reason_colors


class DiagnosticsTests(unittest.TestCase):
    def test_localization_age_uses_simulation_clock_and_rejects_absent_or_old_clock(self):
        self.assertTrue(localization_health(24.95, 24.9)['healthy'])
        self.assertFalse(localization_health(None, 24.9)['healthy'])
        self.assertFalse(localization_health(24.95, 23.)['healthy'])
        self.assertFalse(localization_health(24.95, 25.1)['healthy'])

    def test_causes_match_actual_costs_and_preserve_unknown(self):
        grid = Grid(np.array([[-1, 0, 100, 100, 100]], np.int8), .05, 0., 0.)
        layers = dict(slope=np.array([[90., 90., 13., 0., 20.]]),
                      height_span=np.array([[1., 1., .01, .031, .04]]))
        reasons = terrain_reasons(grid, layers)
        np.testing.assert_array_equal(reasons.data, [[-1, 0, 2, 4, 6]])
        np.testing.assert_array_equal(grid.data, [[-1, 0, 100, 100, 100]])

    def test_retained_hazard_keeps_both_causes_after_free_readings(self):
        memory = TerrainMemory(size=1., resolution=.05)
        frame = Grid(np.full((20, 20), -1, np.int8), .05, -.5, -.5)
        reason = Grid(frame.data.copy(), .05, -.5, -.5)
        frame.data[10, 10], reason.data[10, 10] = 100, 2
        memory.remember(frame, reason)
        self.assertEqual(memory.grid.data[10, 10], -1)
        reason.data[10, 10] = 4
        memory.remember(frame, reason)
        frame.data[10, 10], reason.data[10, 10] = 0, 0
        memory.remember(frame, reason)
        self.assertEqual(memory.grid.data[10, 10], 100)
        self.assertEqual(memory.reasons().data[10, 10], 6)
        project = lambda p: transform(p, (.1, -.1, 0.), (0., 0., math.sin(.2), math.cos(.2)))
        cost, cause = memory.project(project, 1., .05, reasons=True)
        np.testing.assert_array_equal(cost.data, memory.project(project, 1., .05).data)
        np.testing.assert_array_equal(cost.data >= 90, cause.data > 0)
        np.testing.assert_array_equal(cost.data < 0, cause.data < 0)
        self.assertTrue((cause.data == 6).any())

    def test_frozen_rejection_and_current_footprint_share_map_correction(self):
        poly = footprint_polygon((0., 0., 0.))
        status = dict(geometry=dict(frame='odom', stamp=1., footprint=poly, sweep=[poly]),
                      fault=dict(reason='unsafe_or_unobserved_ground', detail=dict(frame='odom',
                         grid_stamp=1., x=1., y=0., cell_center=[1., 0.], resolution=.05,
                         causes=['unknown_terrain'], sweep=[poly])))
        overlay = map_overlay(status, lambda p: transform(p, (2., 3., 0.), (0., 0., math.sqrt(.5), math.sqrt(.5))))
        np.testing.assert_allclose(overlay['rejection']['point'], [2., 4.])
        np.testing.assert_allclose(overlay['footprint'], overlay['rejection']['sweep'][0])
        self.assertEqual(len(overlay['rejection']['cell']), 4)
        self.assertEqual(overlay['rejection']['causes'], ['unknown_terrain'])

    def test_unknown_diagnostic_color_does_not_look_like_a_hazard(self):
        colors = reason_colors(np.array([[-1, 0, 2, 4, 6, 8]], np.int16))
        self.assertEqual(len({tuple(c) for c in colors[0]}), 6)
