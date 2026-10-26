"""Behavioral tests: false terrain, independent evidence, revisits and retirement."""
import math
import unittest
import numpy as np

from rover_navigation.sample_sites import SiteStore
from rover_navigation.site_geometry import measure, pose_covariance, visible_region, color_histogram


def observation(name='rock', point=(1., 0., .1), supported=True, confidence=.8, box=None):
    return dict(class_name=name, odom_position=list(point), confidence=confidence,
                covariance=(np.eye(3)*.05**2).tolist(), extent=[.1, .1, .1], appearance=None,
                box=box, geometry=dict(supported=supported, contradiction=not supported,
                                      reason='raised geometry supported' if supported else 'insufficient raised geometry'))


def update(store, stamp, x, rows=None, **kwargs):
    return store.update([observation()] if rows is None else rows, stamp,
                        [x, 0., 0., 0., 0., 0., 1.], [x, 0., 1.], **kwargs)


def confirmed():
    store = SiteStore()
    for stamp, x in ((1., 0.), (2., .2), (3., .4)):
        update(store, stamp, x)
    return store


class TrackTests(unittest.TestCase):
    def test_single_detection_is_tentative(self):
        store = SiteStore()
        update(store, 1., 0.)
        self.assertEqual(store.objects[0]['state'], 'tentative')

    def test_hundreds_of_adjacent_frames_do_not_confirm_or_shrink_covariance(self):
        store = SiteStore()
        update(store, 1., 0.)
        first = np.array(store.objects[0]['covariance'])
        for i in range(1, 300):
            update(store, 1+i/30, 0.)
        self.assertEqual(len(store.objects), 1)
        self.assertEqual(store.objects[0]['independent_view_count'], 1)
        self.assertEqual(store.objects[0]['state'], 'tentative')
        np.testing.assert_allclose(store.objects[0]['covariance'], first)

    def test_three_translated_views_confirm(self):
        item = confirmed().objects[0]
        self.assertEqual(item['state'], 'confirmed')
        self.assertEqual(item['confirmation_view_count'], 3)
        self.assertAlmostEqual(item['evidence_score'], 16/17)

    def test_full_rotation_camera_arc_cannot_confirm(self):
        store = SiteStore()
        for i in range(20):
            angle = i*math.pi/10
            store.update([observation()], float(i+1), [0., 0., 0., 0., 0., math.sin(angle/2), math.cos(angle/2)],
                         [.25*math.cos(angle), .25*math.sin(angle), 1.])
        self.assertEqual(store.objects[0]['independent_view_count'], 1)

    def test_revisit_preserves_id_and_out_of_view_does_not_remove(self):
        store = confirmed()
        for i in range(4, 80):
            update(store, float(i), 0., [])
        decisions = update(store, 80., 0.)
        self.assertEqual(decisions[0]['id'], '0:1')
        self.assertEqual(len(store.objects), 1)
        self.assertEqual(store.objects[0]['state'], 'confirmed')

    def test_label_change_updates_same_object(self):
        store = SiteStore()
        update(store, 1., 0.)
        update(store, 2., .2, [observation('yellow_crystal_rock')])
        self.assertEqual(len(store.objects), 1)
        self.assertEqual(store.objects[0]['state'], 'tentative')
        self.assertAlmostEqual(store.objects[0]['class_support'], .5)

    def test_soil_and_slab_never_automatically_confirm(self):
        for name in ('pale_soil_patch', 'mudstone_slab'):
            store = SiteStore()
            for i in range(4):
                update(store, i+1., i*.2, [observation(name)])
            self.assertEqual(store.objects[0]['state'], 'tentative')

    def test_low_confidence_and_insufficient_time_cannot_confirm(self):
        for confidence, step in ((.49, 1.), (.8, .1)):
            store = SiteStore()
            for i in range(3):
                update(store, 1+i*step, i*.2, [observation(confidence=confidence)])
            self.assertEqual(store.objects[0]['state'], 'tentative')

    def test_dirt_mislabeled_rock_cannot_confirm_across_views(self):
        store = SiteStore()
        for i in range(3):
            update(store, float(i+1), i*.2, [observation(supported=False, confidence=.99)])
        self.assertFalse(any(item['state'] == 'confirmed' for item in store.objects))

    def test_clear_misses_demote_then_retire_and_retain_audit(self):
        store = confirmed()
        for i in range(3):
            update(store, float(4+i), .6+i*.2, [], visible_misses=['0:1'])
        self.assertEqual(store.objects[0]['state'], 'unverified')
        self.assertEqual(store.objects[0]['confirmation_view_count'], 0)
        update(store, 7., 1.2, [], visible_misses=['0:1'])
        self.assertEqual(store.objects[0]['state'], 'retired')
        self.assertTrue(any(e['kind'] == 'transition' and e.get('state') == 'retired' for e in store.events))

    def test_reconfirmation_requires_three_new_views(self):
        store = confirmed()
        for i in range(3):
            update(store, 4.+i, .6+i*.2, [], visible_misses=['0:1'])
        for i in range(2):
            update(store, 7.+i, 1.4+i*.2)
            self.assertEqual(store.objects[0]['state'], 'unverified')
        update(store, 9., 1.8)
        self.assertEqual(store.objects[0]['state'], 'confirmed')

    def test_nearby_rocks_stay_separate_and_ambiguous_detection_does_not_penalize(self):
        store = SiteStore()
        rows = [observation(point=(1., -.1, .1)), observation(point=(1., .1, .1))]
        update(store, 1., 0., rows)
        result = update(store, 2., .2, visible_misses=['0:1', '0:2'])
        self.assertEqual(result[0]['outcome'], 'ambiguous')
        self.assertEqual(len(store.objects), 2)
        self.assertEqual(store.ambiguous[0]['state'], 'unverified')
        self.assertTrue(all(x['contradiction_count'] == 0 for x in store.objects))
        update(store, 3., .4, rows)
        self.assertEqual(len(store.objects), 2)

    def test_duplicate_class_boxes_in_one_frame_do_not_create_two_tracks(self):
        store = SiteStore()
        update(store, 1., 0., [observation(box=[10, 10, 30, 30]),
                              observation('yellow_crystal_rock', box=[10, 10, 30, 30])])
        self.assertEqual(len(store.objects), 1)

    def test_stationary_misses_cannot_accumulate(self):
        store = confirmed()
        for i in range(50):
            update(store, 4.+i, .6, [], visible_misses=['0:1'])
        self.assertEqual(store.objects[0]['contradiction_count'], 1)
        self.assertEqual(store.objects[0]['state'], 'confirmed')

    def test_expiry_affects_only_tentative_tracks(self):
        store = SiteStore()
        update(store, 1., 0.)
        update(store, 32., 0., [])
        self.assertEqual(store.objects[0]['state'], 'retired')
        store = confirmed()
        update(store, 100., 0., [])
        self.assertEqual(store.objects[0]['state'], 'confirmed')

    def test_stale_frames_and_suspension_cannot_update_evidence(self):
        store = SiteStore()
        update(store, 2., 0.)
        update(store, 1., .2)
        update(store, 3., .4, suspension='localization uncertain')
        self.assertEqual(store.objects[0]['detection_count'], 1)
        self.assertEqual(store.objects[0]['independent_view_count'], 1)

    def test_appearance_cannot_authorize_far_match(self):
        store = SiteStore()
        row = observation()
        row['appearance'] = [1., 0.]
        update(store, 1., 0., [row])
        other = dict(row, odom_position=[2., 0., .1])
        update(store, 2., .2, [other])
        self.assertEqual(len(store.objects), 2)
        self.assertTrue(all(item['state'] == 'tentative' for item in store.objects))

    def test_epoch_ids_are_distinct_and_exports_exclude_private_filter_state(self):
        a, b = SiteStore(), SiteStore(epoch=1)
        update(a, 1., 0.)
        update(b, 1., 0.)
        self.assertNotEqual(a.objects[0]['track_id'], b.objects[0]['track_id'])
        self.assertFalse(any(k.startswith('_') for k in a.export(a.objects[0])))


class GeometryTests(unittest.TestCase):
    k = [100., 0., 60., 0., 100., 60., 0., 0., 1.]

    def test_supporting_plane_rejects_flat_dirt_and_accepts_raised_object(self):
        depth = np.ones((120, 120))
        flat = measure(depth, self.k, [45, 45, 75, 75], [0., 0., 1.], [1., 0., 0., 0.], np.zeros((6, 6)))
        self.assertFalse(flat['geometry']['supported'])
        self.assertTrue(flat['geometry']['contradiction'])
        depth[45:75, 45:75] = .9
        rock = measure(depth, self.k, [45, 45, 75, 75], [0., 0., 1.], [1., 0., 0., 0.], np.zeros((6, 6)))
        self.assertTrue(rock['geometry']['supported'])
        self.assertAlmostEqual(rock['odom_position'][2], .1)
        self.assertGreaterEqual(np.linalg.eigvalsh(rock['covariance']).min(), .05**2-1e-10)

    def test_missing_ring_is_ambiguous_not_a_contradiction(self):
        depth = np.full((120, 120), np.nan)
        depth[45:75, 45:75] = .9
        result = measure(depth, self.k, [45, 45, 75, 75], [0., 0., 1.], [1., 0., 0., 0.], np.zeros((6, 6)))
        self.assertFalse(result['geometry']['supported'])
        self.assertFalse(result['geometry']['contradiction'])

    def test_invalid_pose_uncertainty_is_rejected(self):
        for index, value in ((0, .011), (35, math.radians(6)**2), (0, -1.), (0, math.nan)):
            covariance = np.zeros(36)
            covariance[index] = value
            with self.assertRaises(ValueError):
                pose_covariance(covariance)

    def test_visibility_excludes_occlusion_missing_depth_and_border(self):
        item = observation(point=(0., 0., 2.))
        depth = np.full((120, 120), 2.)
        self.assertTrue(visible_region(item, depth, self.k, [0., 0., 0.], [0., 0., 0., 1.])[0])
        for value in (1., np.nan):
            self.assertFalse(visible_region(item, np.full_like(depth, value), self.k,
                                            [0., 0., 0.], [0., 0., 0., 1.])[0])
        item['odom_position'] = [1.1, 0., 2.]
        self.assertFalse(visible_region(item, depth, self.k, [0., 0., 0.], [0., 0., 0., 1.])[0])

    def test_color_descriptor_is_normalized_and_exposure_tolerant(self):
        a = np.full((20, 20, 3), [30, 60, 90], np.uint8)
        first = color_histogram(a, [0, 0, 20, 20])
        self.assertAlmostEqual(sum(first), 1.)
        np.testing.assert_allclose(first, color_histogram(a*2, [0, 0, 20, 20]))
