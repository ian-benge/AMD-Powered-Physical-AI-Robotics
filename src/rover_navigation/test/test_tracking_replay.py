import importlib.util
from pathlib import Path
import unittest

from rover_navigation.sample_sites import DEFAULTS
from test_site_tracking import observation

spec = importlib.util.spec_from_file_location('phase3_replay', Path(__file__).resolve().parents[3]/'scripts/halo_navigation_tracking_replay.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayTests(unittest.TestCase):
    def events(self, false_supported=False):
        yield dict(kind='configuration', defaults=DEFAULTS)
        for i in range(3):
            yield dict(kind='frame', epoch=0, stamp=1.+i, observer=[i*.2, 0., 0., 0., 0., 0., 1.],
                       camera=[i*.2, 0., 1., 0., 0., 0., 1.], eligible_visible_misses=[],
                       observations=[observation(), observation(point=(3., 0., 0.), supported=false_supported)])

    def test_positive_and_negative_labeled_replay(self):
        trial = dict(start=dict(truth=dict(x=0., y=0., yaw=0.), estimate=dict(x=0., y=0., yaw=0.)))
        scene = dict(objects=[dict(name='a', xyz=[1., 0., 0.], radius_m=.1)])
        result = module.replay(self.events(), trial, scene)
        self.assertTrue(result['passed'])
        self.assertEqual(result['negative_observations_without_raised_support'], 3)
        self.assertEqual(len(result['confirmed']), 1)
        result = module.replay(self.events(True), trial, scene)
        self.assertFalse(result['passed'])
        self.assertEqual(len(result['reference_evaluation']['false_confirmed']), 1)

    def test_replay_rejects_different_thresholds(self):
        with self.assertRaises(ValueError):
            module.replay([dict(kind='configuration', defaults=dict(DEFAULTS, min_views=2))], {}, {})
