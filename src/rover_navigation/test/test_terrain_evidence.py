"""Hazards can only be reconsidered from independent positive ground evidence."""
import math
import unittest

import numpy as np

from rover_navigation.core import Grid, TerrainMemory
from rover_navigation.terrain_evidence import ClearanceLocalization, TerrainEvidence, supported_ground, ClearingPolicy
from rover_navigation.safety import SafetyGate


class AdaptiveTerrainTests(unittest.TestCase):
    def setUp(self):
        self.memory = TerrainMemory(4., .025)
        self.evidence = TerrainEvidence(self.memory)
        self.row, self.col = self.memory.grid.cell(.01, .01)

    def frame(self, hazard=False):
        costs = np.zeros((160, 160), np.int8)
        layers = dict(elevation=np.zeros(costs.shape), slope=np.zeros(costs.shape),
                      height_span=np.zeros(costs.shape), observation_count=np.full(costs.shape, 8))
        if hazard:
            costs[self.row, self.col] = 100
            layers['height_span'][self.row, self.col] = .05
        return Grid(costs, .025, -2., -2.), layers

    def observe(self, stamp, camera=(-1.5, 0.), hazard=False, suspension='', modify=None):
        frame, layers = self.frame(hazard)
        if modify:
            modify(frame, layers)
        self.memory.remember(frame)
        self.evidence.observe(self.memory, frame, layers, camera, stamp, suspension)

    def establish(self):
        self.observe(1., hazard=True)
        self.observe(1.2, hazard=True)
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def clear_views(self, start=2., **kw):
        for stamp, camera in [(start, (-1., 0.)), (start+1., (-.75, 0.)), (start+2., (-.5, 0.))]:
            self.observe(stamp, camera, **kw)

    def test_three_closer_views_clear_and_reset_retained_causes_and_hits(self):
        self.establish()
        self.clear_views()
        self.assertEqual(self.memory.grid.at(.01, .01), 0)
        self.assertEqual(self.memory.hits[self.row, self.col], 0)
        self.assertEqual(self.memory.reasons().at(.01, .01), 0)
        self.assertEqual(self.evidence.total_cleared, 1)
        cleared = next(event for event in self.evidence.events if event['kind'] == 'hazards_cleared')
        self.assertEqual(cleared['cells'], [[self.row, self.col]])
        self.assertEqual(len(cleared['view_positions'][0]), 3)
        # Another real obstacle is again an immediate frame hazard, retained after two frames.
        self.observe(5., hazard=True)
        self.observe(5.2, hazard=True)
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_hundreds_of_frames_and_pure_rotation_cannot_clear(self):
        self.establish()
        for stamp in np.linspace(2., 20., 300):
            self.observe(stamp, (-.8, 0.))
        self.assertEqual(self.evidence.clear_views[self.row, self.col], 1)
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_fast_views_followed_by_stationary_frames_cannot_manufacture_span(self):
        self.establish()
        for stamp, camera in [(2., (-1., 0.)), (2.1, (-.75, 0.)), (2.2, (-.5, 0.))]:
            self.observe(stamp, camera)
        self.assertEqual(self.evidence.clear_views[self.row, self.col], 2)
        self.assertEqual(self.memory.grid.at(.01, .01), 100)
        # A fresh third viewpoint at a qualified time can complete the evidence.
        self.observe(4., (-.5, 0.))
        self.assertEqual(self.memory.grid.at(.01, .01), 0)

    def test_views_must_be_closer_and_old_unknown_evidence_is_protected(self):
        self.establish()
        for stamp, camera in [(2., (-1.5, .3)), (3., (-1.5, .6)), (4., (-1.5, .9))]:
            self.observe(stamp, camera)
        self.assertEqual(self.evidence.clear_views[self.row, self.col], 0)
        protected = TerrainEvidence(self.memory)
        self.evidence = protected
        self.clear_views(5.)
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_three_independent_hazard_views_are_protected(self):
        self.establish()
        self.observe(1.4, (-1.5, .3), hazard=True)
        self.observe(1.6, (-1.5, .6), hazard=True)
        self.clear_views()
        self.assertTrue(self.evidence.protected[self.row, self.col])
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_diagnostics_distinguish_weak_pending_and_protected_evidence_during_suspension(self):
        self.establish()
        self.assertEqual(self.evidence.status['weak_hazard_cells'], 1)
        self.assertEqual(self.evidence.status['protected_cells'], 0)
        self.observe(2., (-1., 0.))
        self.assertEqual(self.evidence.status['max_supporting_views'], 1)
        self.assertEqual(self.evidence.status['pending_cells'], 1)
        self.observe(3., (-.75, 0.), hazard=True, suspension='localization uncertain')
        self.assertEqual(self.evidence.status['pending_cells'], 0)
        self.assertEqual(self.evidence.status['weak_hazard_cells'], 0)
        self.assertEqual(self.evidence.status['protected_cells'], 1)
        self.assertEqual(self.evidence.status['unknown_cells'], 0)  # this fixture measures the complete grid
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_no_clearing_for_sparse_missing_rough_steep_or_raised_surfaces(self):
        changes = [lambda f, a: a['observation_count'].fill(1),
                   lambda f, a: a['slope'].fill(9),
                   lambda f, a: a['height_span'].fill(.02),
                   lambda f, a: a['elevation'].fill(.06),
                   lambda f, a: a['elevation'].fill(np.nan),
                   lambda f, a: f.data.fill(-1),
                   lambda f, a: f.data.__setitem__((self.row, self.col+1), -1),
                   lambda f, a: f.data.__setitem__((self.row, self.col+1), 100)]
        for change in changes:
            with self.subTest(change=changes.index(change)):
                self.setUp()
                self.establish()
                self.clear_views(modify=change)
                self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_a_new_hazard_or_localization_loss_resets_pending_clearance(self):
        self.establish()
        self.observe(2., (-1., 0.))
        self.observe(3., (-.75, 0.))
        self.observe(3.5, (-1.5, 0.), hazard=True)
        self.assertEqual(self.evidence.clear_views[self.row, self.col], 0)
        self.observe(4., (-.5, 0.))
        self.observe(5., (-.25, 0.), suspension='localization uncertain')
        self.assertEqual(self.evidence.clear_views[self.row, self.col], 0)
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_duplicate_frames_and_expired_partial_evidence_cannot_complete(self):
        self.establish()
        self.observe(2., (-1., 0.))
        self.observe(2., (-.75, 0.))
        self.observe(1.9, (-.5, 0.))
        self.assertEqual(self.evidence.clear_views[self.row, self.col], 1)
        self.observe(35., (-.5, 0.))
        self.assertEqual(self.evidence.clear_views[self.row, self.col], 1)
        self.assertEqual(self.memory.grid.at(.01, .01), 100)

    def test_missing_layers_edges_and_different_window_alignment_stay_conservative(self):
        frame, layers = self.frame()
        mask = supported_ground(frame, layers, ClearingPolicy())
        self.assertTrue(mask[self.row, self.col])
        self.assertFalse(mask[0, 0])
        self.assertFalse(supported_ground(frame, {}, ClearingPolicy()).any())
        self.establish()
        # A smaller moving depth window uses the same odometry cell boundaries.
        frame = Grid(np.zeros((80, 80), np.int8), .025, -1., -1.)
        layers = {name: value[40:120, 40:120] for name, value in layers.items()}
        for stamp, camera in [(2., (-1., 0.)), (3., (-.75, 0.)), (4., (-.5, 0.))]:
            self.memory.remember(frame)
            self.evidence.observe(self.memory, frame, layers, camera, stamp)
        self.assertEqual(self.memory.grid.at(.01, .01), 0)
        projected = self.memory.project(lambda points: points+[.5, 0., 0.], 4., .05, reasons=True)
        self.assertLess(projected[0].at(-.49, .01), 90)

    def test_clearing_one_cell_does_not_clear_other_obstacles_or_rearm_guard(self):
        self.establish()
        guard = SafetyGate()
        guard.clock(2., 10.)
        guard.health('pose', 10.)
        guard.health('depth', 10.)
        self.assertTrue(guard.select('manual', 10.))
        guard.command('manual', .05, 0., 10.)
        self.assertEqual(guard.output(10., lambda v, w: self.memory.grid.at(.01, .01) < 90), (0., 0.))
        self.assertEqual(guard.reason, 'unsafe_or_unobserved_ground')

        def other_rock(frame, layers):
            r, c = self.memory.grid.cell(.41, .01)
            frame.data[r, c] = 100
            layers['height_span'][r, c] = .10

        self.clear_views(modify=other_rock)
        self.assertEqual(self.memory.grid.at(.01, .01), 0)
        self.assertEqual(self.memory.grid.at(.41, .01), 100)
        self.assertEqual(guard.output(10., lambda v, w: True), (0., 0.))
        self.assertEqual(guard.mode, 'stopped')
        self.assertEqual(guard.reason, 'unsafe_or_unobserved_ground')


class ClearingLocalizationTests(unittest.TestCase):
    def add(self, buffer, stamp, now=10., x=0., covariance=None):
        c = np.eye(6)*.00001 if covariance is None else covariance
        buffer.add(stamp, [x, 0., 0.], [0., 0., 0., 1.], c.ravel(), now)

    def test_missing_stale_invalid_or_uncertain_localization_cannot_clear(self):
        buffer = ClearanceLocalization()
        self.assertTrue(buffer.check(1., 10.))
        self.add(buffer, 1.)
        self.assertEqual(buffer.check(1., 10.1), '')
        self.assertTrue(buffer.check(1., 10.6))
        self.assertTrue(buffer.check(1.2, 10.1))
        for covariance in (np.eye(6)*9999, np.eye(6)*math.radians(6)**2, np.eye(6)*np.nan):
            self.add(buffer, 1.1, covariance=covariance)
            self.assertTrue(buffer.check(1.1, 10.1))

    def test_duplicates_do_not_refresh_and_unreported_pose_jumps_latch_clearing_suspension(self):
        buffer = ClearanceLocalization()
        self.add(buffer, 1.)
        self.add(buffer, 1., now=12.)
        self.assertTrue(buffer.check(1., 12.))
        self.add(buffer, 1.1, now=12., x=1.)
        self.assertIn('discontinuity', buffer.check(1.1, 12.))
        self.add(buffer, 1.2, now=12.)
        self.assertIn('discontinuity', buffer.check(1.2, 12.))


if __name__ == '__main__':
    unittest.main()
