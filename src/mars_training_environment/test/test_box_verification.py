import copy
import json
from pathlib import Path
import tempfile
import unittest
from contextlib import contextmanager

import numpy as np

from mars_training_environment.annotation import instance_depth_support
from mars_training_environment.box_verification import (
    SENSOR_DEPTH, GAZEBO_BOXES, capture_box_contract, sensor_depth_box_check,
    persisted_box_check_errors)
from mars_training_environment.capture_attempts import capture_scene
from mars_training_environment.common import read_config
from mars_training_environment.environment_v2 import configuration


class BoxVerificationTests(unittest.TestCase):
    def setUp(self):
        self.config = configuration(read_config())
        self.policy = self.config['labels']
        self.mask = np.zeros((30, 40), np.int32)
        self.mask[5:15, 10:25] = 1
        self.expected = np.full(self.mask.shape, 2.)
        self.objects = [dict(name='target')]
        self.labels = [dict(name='target', xmin=10, ymin=5, xmax=25, ymax=15)]

    def check(self, measured, labels=None):
        return sensor_depth_box_check(self.labels if labels is None else labels, self.objects,
            self.mask, self.expected, measured, self.policy, .3, 8.)

    def quality(self, measured):
        return dict(visibility_check=dict(objects=instance_depth_support(self.mask,
            self.expected, measured, self.objects, self.labels, self.policy, .3, 8.,
            rgb_only_geometry=True)))

    def test_supported_sensor_keeps_strict_depth_and_visibility_contract(self):
        contract = capture_box_contract(self.config)
        self.assertEqual(contract['backend'], SENSOR_DEPTH)
        self.assertFalse(contract['renderer_instance_camera'])
        for key in ('require_depth_check', 'require_visibility_check'):
            broken = copy.deepcopy(self.config); broken['capture'][key] = False
            with self.assertRaises(ValueError): capture_box_contract(broken)
        legacy = read_config()
        self.assertEqual(capture_box_contract(legacy)['backend'], GAZEBO_BOXES)

    def test_measured_depth_and_box_extent_both_required(self):
        good = self.check(self.expected)
        self.assertTrue(good['agrees'])
        self.assertFalse(persisted_box_check_errors(good, self.labels,
                         self.quality(self.expected), self.config))
        wrong = copy.deepcopy(self.labels); wrong[0]['xmin'] = 26; wrong[0]['xmax'] = 39
        self.assertFalse(self.check(self.expected, wrong)['agrees'])
        corrupt = self.expected.copy(); corrupt[self.mask == 1] += .5
        self.assertFalse(self.check(corrupt)['agrees'])
        self.assertFalse(self.check(None)['agrees'])
        self.assertFalse(self.check(np.full_like(self.expected, np.inf))['agrees'])

    def test_row_major_decoder_output_matches_2d_and_truncation_fails(self):
        self.assertEqual(self.check(self.expected.ravel().tolist()), self.check(self.expected))
        self.assertFalse(self.check(self.expected.ravel()[:-1])['agrees'])
        self.assertFalse(self.check(self.expected.T)['agrees'])

    def test_partial_depth_range_preserves_full_rgb_box(self):
        self.expected[5:10, 10:25] = 12.
        measured = self.expected.copy(); measured[measured > 8] = np.inf
        result = self.check(measured)
        self.assertTrue(result['agrees'])
        self.assertEqual(result['objects'][0]['usable_depth_expected_pixels'], 75)
        measured[10:15, 10:25] = np.inf
        self.assertFalse(self.check(measured)['agrees'])

    def test_rgb_only_target_has_explicit_geometry_scope_and_is_retained(self):
        self.expected[:] = 12.
        measured = np.full_like(self.expected, np.inf)
        result = self.check(measured)
        self.assertTrue(result['agrees'])
        self.assertEqual(result['matched'], 1)
        self.assertIsNone(result['objects'][0]['sensor_box_iou'])
        self.assertFalse(persisted_box_check_errors(result, self.labels,
                         self.quality(measured), self.config))
        json.dumps(result, allow_nan=False)
        self.assertFalse(self.check(None)['agrees'])

    def test_background_still_requires_received_usable_depth(self):
        self.objects = []; self.labels = []; self.mask[:] = 0
        self.assertTrue(self.check(self.expected)['agrees'])
        self.assertFalse(self.check(None)['agrees'])
        self.assertFalse(self.check(np.full_like(self.expected, np.inf))['agrees'])

    def test_saved_audit_rejects_missing_wrong_or_incomplete_verification(self):
        result = self.check(self.expected); quality = self.quality(self.expected)
        for check in (None, dict(backend=GAZEBO_BOXES),
                      dict(result, objects=[]), dict(result, agrees=False)):
            self.assertTrue(persisted_box_check_errors(check, self.labels, quality, self.config))
        result['objects'][0]['support_fraction'] = 0.
        self.assertTrue(persisted_box_check_errors(result, self.labels, quality, self.config))

    def test_legacy_check_without_backend_remains_readable(self):
        self.assertFalse(persisted_box_check_errors(dict(agrees=False), [], {}, read_config()))

    def test_fatal_renderer_launch_zero_is_not_retried(self):
        @contextmanager
        def crashing(cmd, log, qualified_desktop=False):
            log.write(b'BoundingBox camera not supported by: ogre\nSegmentation fault\n')
            log.flush()
            class Process:
                def wait(self, timeout): return 0
            yield Process()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); entry = dict(id='scene_1', views=8)
            with self.assertRaisesRegex(RuntimeError, 'Fatal renderer failure; all evidence retained'):
                capture_scene(root, root, entry, dict(config=self.config), False, crashing)
            attempts = list((root/'attempts/scene_1').iterdir())
            self.assertEqual(len(attempts), 1)
            self.assertEqual(json.loads((attempts[0]/'attempt-result.json').read_text())['result'], 'FAILED_PRESERVED')


if __name__ == '__main__':
    unittest.main()
