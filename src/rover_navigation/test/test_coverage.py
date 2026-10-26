import math
import unittest
import numpy as np

from rover_navigation.core import Grid, transform
from rover_navigation.coverage import coverage_profile, ground_view, image_detail
from rover_navigation.diagnostics import map_overlay


class CoverageTests(unittest.TestCase):
    def test_uniform_camera_is_flagged_without_treating_detail_as_odometry_confidence(self):
        self.assertTrue(image_detail(np.full((64, 64, 3), 180, np.uint8))['low_detail'])
        image = np.zeros((64, 64, 3), np.uint8)
        image[32:] = 255
        self.assertFalse(image_detail(image)['low_detail'])

    def camera(self, pitch):
        return dict(tilt_rad=pitch, optical_center_height_m=.762, near_m=.3, far_m=8.,
                    horizontal_fov_rad=1.22, width=960, height=540, mount_xyz=[.2, 0., .682])

    def test_pitch_improves_near_ground_but_reduces_far_view(self):
        views = [ground_view(self.camera(p)) for p in (.4, .6, .8)]
        self.assertGreater(views[0]['near_base_m'], views[1]['near_base_m'])
        self.assertGreater(views[1]['near_base_m'], views[2]['near_base_m'])
        self.assertGreater(views[0]['far_base_m'], views[2]['far_base_m'])
        self.assertTrue(.65 < views[1]['near_base_m'] < .8)
        for pitch in (float('nan'), float('inf'), -.1, math.pi/2):
            with self.subTest(pitch=pitch), self.assertRaises(ValueError):
                ground_view(self.camera(pitch))

    def test_predicted_ground_rays_obey_optical_z_clip(self):
        camera = self.camera(0.)
        view = ground_view(camera)
        self.assertAlmostEqual(view['far_base_m']-.2, 8.)
        self.assertGreater(view['near_base_m'], view['camera_height_m'])

    def grid(self):
        return Grid(np.zeros((320, 320), np.int8), .025, -4., -4.)

    def test_current_frame_unknown_gap_is_visible_without_seed_or_memory(self):
        grid = self.grid()
        grid.data[:, :grid.cell(.8, 0.)[1]] = -1
        before = grid.data.copy()
        report = coverage_profile(grid, (0., 0., 0.))
        self.assertTrue(.55 <= report['first_observed_strip_ahead_of_front_m'] <= .65)
        self.assertEqual(report['bands'][0]['kind'], 'unknown')
        self.assertEqual(report['source'], 'single_depth_frame_before_memory_and_seed')
        self.assertLess(report['observed_sample_fraction'], 1.)
        np.testing.assert_array_equal(grid.data, before)

    def test_hazards_and_tiny_unobserved_holes_cannot_disappear_in_a_clear_strip(self):
        grid = self.grid()
        grid.data[grid.cell(.31, .01)] = -1
        grid.data[grid.cell(.71, .01)] = 100
        report = coverage_profile(grid, (0., 0., 0.))
        kinds = [b['kind'] for b in report['bands']]
        self.assertIn('unknown', kinds)
        self.assertIn('hazard', kinds)
        self.assertGreater(report['hazard_sample_fraction'], 0.)

    def test_all_unknown_is_unavailable_and_rotated_corridor_stays_in_its_frame(self):
        grid = self.grid()
        grid.data[:] = -1
        report = coverage_profile(grid, (1., 1., math.pi/2))
        self.assertIsNone(report['first_observed_strip_ahead_of_front_m'])
        self.assertEqual(report['observed_sample_fraction'], 0.)
        self.assertEqual(len(report['bands']), 1)
        report['stamp'] = 3.
        overlay = map_overlay({}, lambda p: transform(p, (2., -1., 0.), (0., 0., 0., 1.)), report)
        np.testing.assert_allclose(overlay['coverage'][0]['polygon'][0], [2.805, 0.225])
        self.assertEqual(overlay['coverage_stamp'], 3.)
