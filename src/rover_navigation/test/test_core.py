import json
import math
import unittest

import numpy as np

from rover_navigation.core import (DRIVE_CENTER_X, FOOTPRINT, Grid, TerrainMemory, depth_points, drive_pose,
                                    footprint_clear, footprint_conflict, frontiers, outline, reachable,
                                    return_required, swept_clear, swept_conflict, swept_poses, transform,
                                    traversability, turn_radius)
from rover_navigation.safety import SafetyGate


class GeometryTests(unittest.TestCase):
    def test_depth_millimetres_endianness_padding_and_intrinsics(self):
        rows = np.array([[1000, 0, 999], [2000, 65535, 999]], dtype='>u2')
        points = depth_points(rows.tobytes(), '16UC1', 2, 2, 6, True,
                              [2., 0., 0., 0., 2., 0., 0., 0., 1.], stride=1)
        np.testing.assert_allclose(points, [[0, 0, 1], [0, 1, 2]])

    def test_depth_invalid_nan_infinity_and_clipping_are_not_ground(self):
        image = np.array([[np.nan, np.inf, 0., .2, 9., 1.]], dtype='<f4')
        points = depth_points(image.tobytes(), '32FC1', 6, 1, 24, False,
                              [1., 0., 0., 0., 1., 0., 0., 0., 1.], 1)
        self.assertEqual(len(points), 1)
        with self.assertRaises(ValueError):
            depth_points(b'', 'rgb8', 1, 1, 3, False, [1.]*9)
        with self.assertRaises(ValueError):
            depth_points(b'', '32FC1', 1, 1, 4, False, [1.]*9)

    def test_pose_transform_retains_pitch_and_height(self):
        p = transform([[1., 0., 0.]], [2., 3., 4.], [0., math.sin(math.pi/4), 0., math.cos(math.pi/4)])
        np.testing.assert_allclose(p, [[2., 3., 3.]], atol=1e-9)
        with self.assertRaises(ValueError):
            transform([[0., 0., 0.]], [0.]*3, [0.]*4)


class TerrainTests(unittest.TestCase):
    def surface(self, slope=0.):
        x, y = np.meshgrid(np.arange(-.98, 1., .025), np.arange(-.98, 1., .025))
        return np.column_stack([x.ravel(), y.ravel(), (x*math.tan(math.radians(slope))).ravel()])

    def test_flat_observed_ground_is_traversable(self):
        grid, layers = traversability(self.surface(), size=2.)
        self.assertEqual(grid.at(0., 0.), 0)
        self.assertEqual(layers['slope'][20, 20], 0.)

    def test_slope_is_measured_from_depth_geometry(self):
        grid, _ = traversability(self.surface(20.), size=2.)
        self.assertEqual(grid.at(0., 0.), 100)

    def test_vertical_rock_column_blocks_even_without_neighbouring_ground(self):
        grid, _ = traversability([[.01, .01, 0.], [.01, .01, .15]], size=2.)
        self.assertEqual(grid.at(.01, .01), 100)

    def test_repeated_samples_retain_height_hazards_and_per_cell_statistics(self):
        z = np.tile([0., .01, .02, .15], 1000)
        points = np.column_stack([np.full(len(z), .01), np.full(len(z), .01), z])
        grid, layers = traversability(points, size=2.)
        cell = grid.cell(.01, .01)
        self.assertEqual(grid.at(.01, .01), 100)
        self.assertEqual(layers['observation_count'][cell], len(z))
        self.assertEqual(layers['elevation'][cell], 0.)
        self.assertAlmostEqual(layers['height_span'][cell], .15)
        self.assertAlmostEqual(layers['roughness'][cell], np.std(z))
        self.assertEqual(grid.at(-.1, -.1), -1)

    def test_no_points_and_unsupported_single_point_remain_unknown(self):
        for cloud in ([], [[.01, .01, 0.]]):
            grid, _ = traversability(cloud, size=2.)
            self.assertEqual(grid.at(.01, .01), -1)

    def test_measured_edge_of_a_single_view_is_classified(self):
        cloud = self.surface()
        near = cloud[cloud[:, 0] >= .5]  # a frame's near edge at x = 0.5
        grid, _ = traversability(near, size=2.)
        self.assertEqual(grid.at(.51, 0.), 0)
        self.assertEqual(grid.at(.45, 0.), -1)
        steep = self.surface(20.)
        grid, _ = traversability(steep[steep[:, 0] >= .5], size=2.)
        self.assertEqual(grid.at(.51, 0.), 100)  # one-sided slope still measures the hazard

    def test_unobserved_hole_is_not_interpolated_free(self):
        cloud = self.surface()
        cloud = cloud[np.hypot(cloud[:, 0], cloud[:, 1]) > .3]
        grid, _ = traversability(cloud, size=2.)
        self.assertEqual(grid.at(0., 0.), -1)


class TerrainMemoryTests(unittest.TestCase):
    def frame(self, z=0., x0=-.98):
        x, y = np.meshgrid(np.arange(x0, x0+1.96, .0125), np.arange(-.98, .98, .0125))
        return np.column_stack([x.ravel(), y.ravel(), np.full(x.size, z)])

    def test_flat_frames_with_inconsistent_height_do_not_create_steps(self):
        lower, higher = self.frame(0.), self.frame(.05, x0=-.5)
        merged, _ = traversability(np.vstack([lower, higher]), .025, 4.)
        self.assertGreater(int((merged.data >= 90).sum()), 100)  # the cross-frame failure
        memory = TerrainMemory(4., .025)
        for points in (lower, higher, lower, higher):
            memory.remember(traversability(points, .025, 4.)[0])
        self.assertEqual(int((memory.grid.data >= 90).sum()), 0)
        self.assertEqual(memory.grid.at(0., 0.), 0)

    def test_hazard_needs_two_frames_and_free_readings_never_clear_it(self):
        rock = np.vstack([self.frame(), [[.51, .01, .15]]])
        memory = TerrainMemory(4., .025)
        memory.remember(traversability(rock, .025, 4.)[0])
        self.assertEqual(memory.grid.at(.51, .01), -1)  # one frame: not retained
        memory.remember(traversability(rock, .025, 4.)[0])
        self.assertEqual(memory.grid.at(.51, .01), 100)
        memory.remember(traversability(self.frame(), .025, 4.)[0])
        self.assertEqual(memory.grid.at(.51, .01), 100)
        self.assertEqual(memory.grid.at(-.5, 0.), 0)

    def test_projection_moves_memory_and_never_promotes_partial_cells(self):
        memory = TerrainMemory(4., .025, confirmations=1)
        memory.grid.data[memory.grid.cell(.0, .0)] = 0
        memory.grid.data[memory.grid.cell(1.01, .01)] = 100
        moved = memory.project(lambda p: p+[.5, 0., 0.], 4., .05)  # map = odom - 0.5 m in x
        self.assertEqual(moved.at(.51, .01), 100)
        self.assertEqual(moved.at(1.01, .01), -1)
        self.assertEqual(moved.at(-.49, .01), -1)  # one of four sub-cells known: still unknown


class RoutingTests(unittest.TestCase):
    def grid(self):
        return Grid(np.zeros((100, 100), dtype=np.int8), .05, -2.5, -2.5)

    def test_body_interior_and_corners_are_checked(self):
        grid = self.grid()
        self.assertTrue(footprint_clear(grid, 0., 0., 0.))
        grid.data[grid.cell(.15, 0.)] = 100  # ahead of the front edge, 0.095 m past the drive center
        self.assertTrue(footprint_clear(grid, 0., 0., 0.))
        grid.data[grid.cell(-.3, 0.)] = 100  # rear overhang behind the drive center
        self.assertFalse(footprint_clear(grid, 0., 0., 0.))

    def test_drive_center_is_ahead_of_base_link(self):
        x, y, yaw = drive_pose(1., 2., math.pi/2)
        self.assertAlmostEqual(x, 1.)
        self.assertAlmostEqual(y, 2.+DRIVE_CENTER_X)
        self.assertEqual(FOOTPRINT, outline())
        self.assertAlmostEqual(turn_radius(), math.hypot(.33, .17)+.025)

    def test_turn_swings_rear_about_drive_center_not_base_link(self):
        grid = self.grid()
        grid.data[grid.cell(-.175, -.375)] = 100  # outside the current footprint
        self.assertTrue(footprint_clear(grid, 0., 0., 0.))
        self.assertFalse(swept_clear(grid, (0., 0., 0.), 0., .6))
        # The same body pose, rotated about base_link as before, never reaches it.
        self.assertTrue(all(footprint_clear(grid, *p, footprint=outline(0.))
                            for p in swept_poses((-DRIVE_CENTER_X, 0., 0.), 0., .6)))

    def test_swept_path_stops_before_unknown(self):
        grid = self.grid()
        grid.data[:, 52:] = -1
        self.assertTrue(footprint_clear(grid, 0., 0., 0.))
        self.assertFalse(swept_clear(grid, (0., 0., 0.), .2, 0.))

    def test_conflict_evidence_agrees_with_clear_checks(self):
        grid = self.grid()
        self.assertIsNone(footprint_conflict(grid, 0., 0., 0.))
        self.assertIsNone(swept_conflict(grid, (0., 0., 0.), .2, 0.))
        grid.data[grid.cell(-.3, 0.)] = 100
        hazard = footprint_conflict(grid, 0., 0., 0.)
        self.assertEqual(hazard['cell'], 100)
        self.assertEqual(grid.at(hazard['x'], hazard['y']), 100)
        grid = self.grid()
        grid.data[:, 52:] = -1
        unknown = swept_conflict(grid, (0., 0., 0.), .2, 0.)
        self.assertEqual(unknown['cell'], -1)
        self.assertGreater(unknown['step'], 0)  # current footprint is clear; the projected one is not
        self.assertEqual(json.loads(json.dumps(unknown)), unknown)
        self.assertIsNone(footprint_conflict(grid, 10., 0., 0.)['cell'])

    def test_unknown_wall_and_insufficient_width_are_not_routes(self):
        grid = self.grid()
        grid.data[:, 60] = -1
        distances, _ = reachable(grid, (0., 0.))
        self.assertFalse(np.isfinite(distances[50, 70]))
        self.assertTrue(np.isfinite(distances[50, 50]))

    def test_frontiers_are_set_back_on_known_ground_and_unreachable_ones_ignored(self):
        grid = self.grid()
        grid.data[:, 75:] = -1
        choices = frontiers(grid, (0., 0.))
        self.assertTrue(choices)
        for x, y, yaw, _ in choices:
            self.assertTrue(footprint_clear(grid, x, y, 0.))
            self.assertGreater(grid.x+75*.05-x, .29)
            self.assertAlmostEqual(yaw, 0.)  # look towards unknown, not the previous route

    def test_fully_observed_area_has_no_artificial_boundary_frontiers(self):
        self.assertEqual(frontiers(self.grid(), (0., 0.)), [])

    def test_return_budget_contains_contingency(self):
        self.assertFalse(return_required(0., 100., 10.))
        self.assertTrue(return_required(20., 100., 10.))
        self.assertTrue(return_required(0., 100., math.inf))


class SafetyTests(unittest.TestCase):
    def ready(self):
        gate = SafetyGate()
        gate.clock(1., 1.)
        gate.health('pose', 1.)
        gate.health('depth', 1.)
        return gate

    def test_startup_and_missing_sensor_refuse_motion(self):
        gate = SafetyGate()
        self.assertFalse(gate.select('autonomous', 1.))
        self.assertEqual(gate.output(1., lambda v, w: True), (0., 0.))

    def test_takeover_clears_both_command_leases(self):
        gate = self.ready()
        gate.select('autonomous', 1.)
        gate.command('autonomous', .2, 0., 1.)
        self.assertEqual(gate.output(1.1, lambda v, w: True), (.2, 0.))
        gate.select('manual', 1.1)
        self.assertEqual(gate.output(1.15, lambda v, w: True), (0., 0.))
        gate.command('autonomous', .2, 0., 1.15)
        self.assertEqual(gate.output(1.2, lambda v, w: True), (0., 0.))

    def test_sensor_fault_latches_and_cannot_resume_on_fresh_command(self):
        gate = self.ready()
        gate.select('autonomous', 1.)
        gate.command('autonomous', .2, 0., 1.)
        self.assertEqual(gate.output(2., lambda v, w: True), (0., 0.))
        gate.clock(2., 2.)
        gate.health('depth', 2.)
        gate.health('pose', 2.)
        gate.command('autonomous', .2, 0., 2.)
        self.assertEqual(gate.output(2., lambda v, w: True), (0., 0.))

    def test_paused_simulation_stops_despite_fresh_sensor_callbacks(self):
        gate = self.ready()
        gate.select('manual', 1.)
        gate.clock(1., 1.6)
        gate.health('depth', 1.6)
        gate.health('pose', 1.6)
        gate.command('manual', .2, 0., 1.6)
        self.assertEqual(gate.output(1.6, lambda v, w: True), (0., 0.))

    def test_nan_reverse_and_excess_speed(self):
        gate = self.ready()
        gate.select('manual', 1.)
        gate.command('manual', -1., 9., 1.)
        self.assertEqual(gate.output(1., lambda v, w: True), (0., .6))
        gate.command('manual', 9., -9., 1.)
        self.assertEqual(gate.output(1., lambda v, w: True), (.5, -.6))
        gate.command('manual', math.nan, 0., 1.)
        self.assertEqual(gate.mode, 'stopped')

    def test_hazard_and_clock_reset_latch(self):
        gate = self.ready()
        gate.select('autonomous', 1.)
        gate.command('autonomous', .2, 0., 1.)
        gate.output(1., lambda v, w: False)
        self.assertEqual(gate.reason, 'unsafe_or_unobserved_ground')
        gate.clock(0., 1.1)
        self.assertEqual(gate.reason, 'simulation_clock_reset')
        self.assertFalse(gate.ready(1.1))
        gate.clock(1., 2.)
        gate.health('depth', 2.)
        gate.health('pose', 2.)
        self.assertFalse(gate.select('manual', 2.))

    def test_first_fault_survives_operator_stop_until_reselection(self):
        gate = self.ready()
        gate.select('autonomous', 1.)
        self.assertIsNone(gate.fault)
        gate.stop('map_localization_stale')
        gate.stop('command_timeout')
        gate.stop('operator_stop')
        self.assertEqual(gate.reason, 'operator_stop')
        self.assertEqual(gate.fault, dict(reason='map_localization_stale', sim_time=1.))
        gate.select('manual', 1.)
        gate.stop('operator_stop')
        self.assertIsNone(gate.fault)

    def test_expired_moving_command_latches_until_mode_selected_again(self):
        gate = self.ready()
        gate.select('autonomous', 1.)
        gate.command('autonomous', .2, 0., 1.)
        self.assertEqual(gate.output(1.4, lambda v, w: True), (0., 0.))
        self.assertEqual(gate.reason, 'command_timeout')
        gate.command('autonomous', .2, 0., 1.41)
        self.assertEqual(gate.output(1.42, lambda v, w: True), (0., 0.))


if __name__ == '__main__':
    unittest.main()
