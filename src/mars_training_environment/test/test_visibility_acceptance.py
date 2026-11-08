"""Acceptance must reject RGB loss even when depth geometry remains correct."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts'))
from phase1_evaluate_visibility import metric_failures


class VisibilityAcceptanceTests(unittest.TestCase):
    def healthy(self):
        return dict(p95_error_m=.00004,fraction_over_3cm=0,
                    expected_finite_missing_fraction=0,unexpected_usable_fraction=0,
                    visibility=[dict(name='buried_rock',predicted_pixels=6000,rgb_chroma_support_fraction=1)],
                    near_occluded_pixels=3000,near_background_support_fraction=1,
                    transparent_panel=dict(sky_pixels=400,intermediate_alpha_fraction=1))

    def test_rgb_disappearance_fails_with_perfect_depth(self):
        metrics=self.healthy()
        self.assertEqual(metric_failures(metrics),[])
        metrics['visibility'][0]['rgb_chroma_support_fraction']=0
        self.assertIn('rgb_visibility:buried_rock',metric_failures(metrics))

    def test_missing_and_unexpected_depth_are_independent_failures(self):
        metrics=self.healthy(); metrics['expected_finite_missing_fraction']=.1
        self.assertIn('expected_finite_missing_fraction',metric_failures(metrics))
        metrics=self.healthy(); metrics['unexpected_usable_fraction']=.1
        self.assertIn('unexpected_usable_fraction',metric_failures(metrics))

    def test_opacity_and_near_clipping_are_not_hidden_by_good_geometry(self):
        metrics=self.healthy(); metrics['transparent_panel']['intermediate_alpha_fraction']=0
        self.assertIn('transparent_panel_blending',metric_failures(metrics))
        metrics=self.healthy(); metrics['near_background_support_fraction']=0
        self.assertIn('near_clipping',metric_failures(metrics))


if __name__=='__main__': unittest.main()
