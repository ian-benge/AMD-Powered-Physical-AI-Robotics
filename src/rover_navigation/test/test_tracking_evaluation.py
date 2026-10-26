import unittest
from rover_navigation.tracking_evaluation import reference_matches, revisit_evidence


class ReferenceTests(unittest.TestCase):
    def test_persistence_alone_is_not_a_successful_revisit(self):
        before = dict(objects=[dict(track_id='0:1', last_stamp=1.)])
        after = dict(accepted_stamp=25., objects=[dict(track_id='0:1', last_stamp=1.)])
        self.assertFalse(revisit_evidence(before, after)['passed'])
        after['objects'][0]['last_stamp'] = 24.8
        self.assertTrue(revisit_evidence(before, after)['passed'])
        after['objects'][0]['track_id'] = '0:2'
        self.assertFalse(revisit_evidence(before, after)['passed'])

    def test_reference_scoring_accounts_for_coordinate_origins_and_yaw(self):
        start = dict(truth=dict(x=3., y=4., yaw=1.5707963267948966), estimate=dict(x=0., y=0., yaw=0.))
        scene = dict(objects=[dict(name='rock_a', xyz=[3., 5., 0.], radius_m=.1)])
        row = dict(track_id='0:1', class_name='rock', odom_position=[1., 0., 0.])
        result = reference_matches([row], start, scene)
        self.assertEqual(result['matches'][0]['reference'], 'rock_a')
        self.assertFalse(result['false_confirmed'])

    def test_false_duplicate_and_unconfirmed_references_are_reported(self):
        start = dict(truth=dict(x=0., y=0., yaw=0.), estimate=dict(x=0., y=0., yaw=0.))
        scene = dict(objects=[dict(name='a', xyz=[1., 0., 0.], radius_m=.1),
                              dict(name='b', xyz=[2., 0., 0.], radius_m=.1)])
        rows = [dict(track_id=f'0:{i}', class_name='rock', odom_position=[x, 0., 0.])
                for i, x in enumerate((1., 1.02, 4.))]
        result = reference_matches(rows, start, scene)
        self.assertEqual(result['duplicate_references'], ['a'])
        self.assertEqual(result['false_confirmed'][0]['track_id'], '0:2')
        self.assertEqual(result['unconfirmed_scene_references'], ['b'])
