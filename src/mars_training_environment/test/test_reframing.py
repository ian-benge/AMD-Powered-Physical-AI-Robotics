import copy
import math
import unittest

import numpy as np

from mars_training_environment.common import read_config
from mars_training_environment.reframing import (closer_target_view, clearer_target_view, framing_headroom,
                                               MAX_REFRAMES, SIZE_FAILURE, MARGIN_FAILURE, VIEW_ANGLES)


class ReframingTests(unittest.TestCase):
    def setUp(self):
        self.scene = {'config':read_config(), 'terrain':{'size_m':20, 'base_heights':np.zeros((17,17)).tolist()},
                      'objects':[{'name':'target','xyz':[0,0,0],'radius_m':.03}], 'regions':[]}
        self.view = {'id':'view_004','target_name':'target','difficult':True,
                     'xyz':[-2.4,0,.09],'rpy':[0,0,.35]}
        self.checks = {'errors':[SIZE_FAILURE], 'target_framing':{'side_at_training_px':11.}}

    def test_size_failure_moves_closer_with_audited_pose_and_preserved_plan(self):
        original = copy.deepcopy(self.view)
        result = closer_target_view(self.scene,self.view,self.checks,[-2.4,0,.08])
        self.assertIsNotNone(result)
        self.assertGreater(result['xyz'][0],self.view['xyz'][0])
        self.assertEqual(result['rpy'][2],self.view['rpy'][2])
        self.assertEqual(result['target_name'],'target')
        self.assertTrue(result['difficult'])
        self.assertEqual(result['framing_adjustments'][0]['from_measured_xyz'],[-2.4,0,.08])
        self.assertEqual(self.view,original)
        camera_xy = np.array(result['xyz'][:2])+.2*np.array([math.cos(.35),math.sin(.35)])
        self.assertGreaterEqual(np.linalg.norm(camera_xy),1.8)

    def test_missing_target_or_any_visibility_failure_cannot_trigger_a_move(self):
        for errors in (['target_not_visible'], [SIZE_FAILURE,'rock: insufficient_sensor_depth_support'],
                       ['target_too_occluded'], []):
            self.assertIsNone(closer_target_view(self.scene,self.view,dict(self.checks,errors=errors),self.view['xyz']))

    def test_retries_are_bounded_and_range_floor_is_respected(self):
        exhausted = dict(self.view,framing_adjustments=[{}]*MAX_REFRAMES)
        self.assertIsNone(closer_target_view(self.scene,exhausted,self.checks,self.view['xyz']))
        self.assertIsNone(closer_target_view(self.scene,self.view,self.checks,[-2,0,.08]))

    def test_blocked_endpoint_is_rejected(self):
        self.scene['objects'].append({'name':'obstacle','xyz':[-2.05,0,0],'radius_m':.15})
        self.assertIsNone(closer_target_view(self.scene,self.view,self.checks,self.view['xyz']))

    def test_steep_terrain_is_rejected(self):
        row = np.linspace(-10,10,17)*.5
        self.scene['terrain']['base_heights'] = np.tile(row,(17,1)).tolist()
        self.assertIsNone(closer_target_view(self.scene,self.view,self.checks,self.view['xyz']))

    def test_invalid_size_and_unknown_target_are_rejected(self):
        for side in (0,float('nan'),float('inf')):
            checks = dict(self.checks,target_framing={'side_at_training_px':side})
            self.assertIsNone(closer_target_view(self.scene,self.view,checks,self.view['xyz']))
        self.assertIsNone(closer_target_view(self.scene,dict(self.view,target_name='missing'),self.checks,self.view['xyz']))

    def test_small_top_margin_failure_gets_audited_closer_pose_without_relaxing_margin(self):
        self.scene['config']['camera'].update(render_engine='ogre',width=960,height=540)
        self.scene['config']['terrain'].update(profile='mixed_reachable',profile_version=2)
        view=dict(self.view,difficult=False)
        checks=dict(errors=[MARGIN_FAILURE],target_framing=dict(side_at_training_px=73.,bbox_xyxy=[355,53,616,126]))
        result=closer_target_view(self.scene,view,checks,view['xyz'])
        self.assertIsNotNone(result)
        self.assertEqual(result['framing_adjustments'][0]['reason'],MARGIN_FAILURE)
        self.assertEqual(result['target_name'],view['target_name'])
        self.assertEqual(self.scene['config']['capture']['target_margin_fraction'],.1)
        self.assertGreater(result['xyz'][0],view['xyz'][0])

    def test_margin_recovery_does_not_hide_other_failures_or_large_clipping(self):
        self.scene['config']['camera'].update(render_engine='ogre',width=960,height=540)
        self.scene['config']['terrain'].update(profile='mixed_reachable',profile_version=2)
        view=dict(self.view,difficult=False)
        checks=dict(errors=[MARGIN_FAILURE],target_framing=dict(side_at_training_px=73.,bbox_xyxy=[355,53,616,126]))
        for box in [[355,45,616,126],[95,53,616,126],[355,53,616,501]]:
            bad=dict(checks,target_framing=dict(side_at_training_px=73.,bbox_xyxy=box))
            self.assertIsNone(closer_target_view(self.scene,view,bad,view['xyz']))
        bad=dict(checks,errors=[MARGIN_FAILURE,'rock: insufficient_sensor_depth_support'])
        self.assertIsNone(closer_target_view(self.scene,view,bad,view['xyz']))


class ClearerViewTests(unittest.TestCase):
    def setUp(self):
        config=read_config()
        config['camera'].update(render_engine='ogre',width=960,height=540)
        config['terrain'].update(profile='mixed_reachable',profile_version=2)
        self.scene=dict(config=config,terrain=dict(size_m=40,base_heights=np.zeros((17,17)).tolist()),
                        objects=[dict(name='target',xyz=[0,0,0],radius_m=.04)],regions=[])
        self.view=dict(id='view_011',xyz=[-2.6,0,.09],rpy=[0,0,.05],target_name='target',
                       requested_target_class_id=0,requested_export_class_id=3,
                       target_export_class_id=3,difficult=False,
                       planned_framing=dict(passed=True,visible_fraction=.7,side_at_training_px=39,
                                            bbox_xyxy=[400,100,550,150]))
        self.checks=dict(errors=['target_too_occluded'])
        self.good=dict(passed=True,visible_fraction=.85,side_at_training_px=40,
                       bbox_xyxy=[400,100,550,150],target_name='target')

    def propose(self,predict=None,**kwargs):
        args=dict(scene=self.scene,view=self.view,checks=self.checks,measured_xyz=[-2.6,0,.08],
                  predict=predict or (lambda candidate:dict(self.good)),diagnostics='diagnostics/view_011')
        args.update(kwargs)
        return clearer_target_view(**args)

    def test_occluded_target_gets_new_audited_pose_and_unchanged_export_schedule(self):
        scene,view=copy.deepcopy(self.scene),copy.deepcopy(self.view)
        result=self.propose()
        self.assertIsNotNone(result)
        self.assertNotEqual(result['xyz'],view['xyz'])
        for field in ('id','target_name','requested_target_class_id','requested_export_class_id',
                      'target_export_class_id','difficult'):
            self.assertEqual(result[field],view[field])
        self.assertEqual(scene,self.scene);self.assertEqual(view,self.view)
        history=result['framing_adjustments'][-1]
        self.assertEqual(history['rejected_errors'],['target_too_occluded'])
        self.assertEqual(history['rejected_diagnostics'],'diagnostics/view_011')
        self.assertTrue(history['fresh_sensor_acceptance_required'])
        self.assertTrue(history['acceptance_policy_unchanged'])
        self.assertEqual(history['settling_probes'],3)

    def test_nominal_only_improvement_is_rejected_when_settling_loses_headroom(self):
        calls=[]
        def predict(candidate):
            calls.append(candidate)
            if abs(candidate['rpy'][1])>.001:
                return dict(self.good,visible_fraction=.699)
            return dict(self.good)
        self.assertIsNone(self.propose(predict=predict))
        self.assertGreater(len(calls),3)
        self.assertLessEqual(len(calls),len(VIEW_ANGLES)*3*3)

    def test_search_is_finite_and_actual_failures_cannot_be_accepted_as_predictions(self):
        calls=[]
        def predict(candidate):
            calls.append(candidate)
            return dict(self.good,passed=False,visible_fraction=.699)
        self.assertIsNone(self.propose(predict=predict))
        self.assertLessEqual(len(calls),len(VIEW_ANGLES)*3)
        exhausted=dict(self.view,framing_adjustments=[{}]*MAX_REFRAMES)
        calls.clear()
        self.assertIsNone(self.propose(view=exhausted,predict=predict))
        self.assertEqual(calls,[])

    def test_missing_depth_class_global_or_box_failures_do_not_trigger_target_search(self):
        for error in ('visibility_check_missing_sensor_depth','aggregate_mesh_depth_mismatch',
                      'target_class_does_not_match_requested_quota','rock: insufficient_sensor_depth_support',
                      'rock: insufficient_sensor_depth_box_support'):
            calls=[]
            result=self.propose(checks=dict(errors=['target_too_occluded',error]),
                                predict=lambda c:calls.append(c))
            self.assertIsNone(result);self.assertEqual(calls,[])

    def test_preflight_only_changes_marginal_saved_views_and_preserves_raw_plan(self):
        original=copy.deepcopy(self.view)
        result=self.propose(checks=None)
        self.assertIsNotNone(result)
        self.assertEqual(result['framing_adjustments'][-1]['reason'],'target_framing_preflight')
        self.assertEqual(self.view,original)
        self.assertIsNone(self.propose(checks=None,view=dict(self.view,planned_framing=self.good)))
        self.assertIsNone(self.propose(checks=None,view={k:v for k,v in self.view.items() if k!='planned_framing'}))

    def test_legacy_profiles_and_backgrounds_keep_existing_behavior(self):
        for version,engine in ((1,'ogre'),(2,'ogre2')):
            scene=copy.deepcopy(self.scene)
            scene['config']['terrain']['profile_version']=version
            scene['config']['camera']['render_engine']=engine
            self.assertIsNone(self.propose(scene=scene))
        self.assertIsNone(self.propose(view=dict(self.view,target_name=None)))

    def test_collision_range_slope_and_all_boundaries_are_enforced_before_projection(self):
        scene=copy.deepcopy(self.scene)
        scene['objects'].append(dict(name='obstacle',xyz=[-2.3,0,0],radius_m=1.5))
        self.assertIsNone(self.propose(scene=scene))
        scene=copy.deepcopy(self.scene)
        scene['terrain']['base_heights']=np.tile(np.linspace(-20,20,17),(17,1)).tolist()
        self.assertIsNone(self.propose(scene=scene))
        self.assertIsNone(self.propose(measured_xyz=[-9,0,.08]))
        for xy in ((25,0),(-25,0),(0,25),(0,-25)):
            scene=copy.deepcopy(self.scene)
            scene['objects'][0]['xyz']=[xy[0],xy[1],0]
            self.assertIsNone(self.propose(scene=scene,measured_xyz=[xy[0]-2.6,xy[1],.08]))

    def test_difficult_azimuth_is_retained_and_existing_view_poses_are_not_duplicated(self):
        view=dict(self.view,difficult=True,rpy=[0,0,.35])
        result=self.propose(view=view)
        offset=result['framing_adjustments'][-1]['orbit_angle_rad']
        self.assertAlmostEqual(result['rpy'][2]-.35,offset)
        scene=dict(self.scene,views=[dict(result,id='other')])
        alternative=self.propose(view=view,scene=scene)
        self.assertFalse(np.allclose(result['xyz']+result['rpy'],alternative['xyz']+alternative['rpy']))

    def test_planning_headroom_does_not_modify_the_actual_acceptance_policy(self):
        from mars_training_environment.framing import target_framing
        policy=self.scene['config']['capture']
        camera=self.scene['config']['camera']
        label=dict(name='target',xmin=400,ymin=100,xmax=550,ymax=150,
                   visible_fraction=.7,truncated=False)
        accepted=target_framing([label],'target',camera,policy)
        self.assertTrue(accepted['passed'])
        self.assertFalse(framing_headroom(accepted,policy,camera))
        self.assertEqual(policy['target_min_visible_fraction'],.7)
        for fraction in (float('nan'),float('inf')):
            self.assertFalse(framing_headroom(dict(self.good,visible_fraction=fraction),policy,camera))


if __name__ == '__main__':
    unittest.main()
