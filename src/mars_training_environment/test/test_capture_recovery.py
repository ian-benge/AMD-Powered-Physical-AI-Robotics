import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

from mars_training_environment.common import read_config
from mars_training_environment.capture_recovery import (incidental_depth_view, YAW_OFFSETS,
    EDGE_YAW_OFFSETS, terrain_visibility_context,localized_depth_context)


class CaptureRecoveryTests(unittest.TestCase):
    def setUp(self):
        config=read_config()
        config['camera']['render_engine']='ogre'
        config['terrain'].update(profile='mixed_reachable',profile_version=2)
        self.scene=dict(config=config,terrain=dict(size_m=20,base_heights=np.zeros((17,17)).tolist()),
            objects=[dict(name='target',xyz=[0,0,0],radius_m=.03)],regions=[])
        self.view=dict(id='view_006',target_name='target',requested_export_class_id=1,
            difficult=False,xyz=[-2.4,0,.09],rpy=[0,0,0])
        self.checks=dict(errors=['incidental: insufficient_sensor_depth_support',
                                'incidental: insufficient_sensor_depth_box_support'],
                         target_framing=dict(passed=True))
        self.quality=dict(visibility_check=dict(checked=True,objects=[
            dict(name='target',passed=True),dict(name='incidental',passed=False,
                usable_depth_expected_pixels=20,sensor_valid_depth_pixels=20,
                support_fraction=.8,median_error_m=.0001)]))

    def propose(self, **kwargs):
        values=dict(scene=self.scene,view=self.view,checks=self.checks,quality=self.quality,
                    measured_xyz=[-2.4,0,.08],diagnostics='diagnostics/scene/view_006')
        values.update(kwargs)
        return incidental_depth_view(**values)

    def test_proposes_audited_view_without_mutating_scene_policy_or_target(self):
        scene,view=copy.deepcopy(self.scene),copy.deepcopy(self.view)
        result=self.propose()
        self.assertIsNotNone(result)
        self.assertEqual(result['target_name'],view['target_name'])
        self.assertEqual(result['requested_export_class_id'],1)
        self.assertEqual(result['xyz'],view['xyz'])
        self.assertEqual(result['rpy'][2],YAW_OFFSETS[0])
        self.assertEqual(self.scene,scene)
        self.assertEqual(self.view,view)
        self.assertEqual(result['view_recoveries'][0]['rejected_errors'],self.checks['errors'])
        self.assertTrue(result['view_recoveries'][0]['acceptance_policy_unchanged'])

    def test_recovery_is_bounded_and_offsets_are_relative_to_original_view(self):
        view=self.view
        for offset in YAW_OFFSETS:
            view=self.propose(view=view)
            self.assertIsNotNone(view)
            self.assertAlmostEqual(view['rpy'][2],offset)
        self.assertIsNone(self.propose(view=view))

    def test_sensor_loss_global_error_and_target_failures_stop(self):
        for errors in [[],['aggregate_mesh_depth_mismatch'],['sensor_depth_missing_or_shape_mismatch'],
                       ['target_too_small_at_training_resolution'],['target: insufficient_sensor_depth_support']]:
            self.assertIsNone(self.propose(checks=dict(self.checks,errors=errors)))
        self.assertIsNone(self.propose(quality={}))
        self.assertIsNone(self.propose(checks=dict(self.checks,target_framing=dict(passed=False))))
        self.assertIsNone(self.propose(view=dict(self.view,target_name=None)))

    def test_legacy_camera_and_terrain_profiles_keep_existing_behavior(self):
        legacy=copy.deepcopy(self.scene)
        legacy['config']['camera']['render_engine']='ogre2'
        self.assertIsNone(self.propose(scene=legacy))
        legacy=copy.deepcopy(self.scene)
        legacy['config']['terrain']['profile_version']=1
        self.assertIsNone(self.propose(scene=legacy))

    def test_widespread_or_large_depth_disagreement_stops(self):
        for field,value in [('support_fraction',.5),('support_fraction',float('nan')),
                            ('median_error_m',.04),('median_error_m',float('inf')),
                            ('sensor_valid_depth_pixels',0)]:
            quality=copy.deepcopy(self.quality)
            quality['visibility_check']['objects'][1][field]=value
            self.assertIsNone(self.propose(quality=quality))
        checks=dict(self.checks,errors=self.checks['errors']+['other: insufficient_sensor_depth_support'])
        self.assertIsNone(self.propose(checks=checks))

    def test_collision_range_boundaries_and_slopes_are_not_bypassed(self):
        self.scene['objects'].append(dict(name='blocked',xyz=[-2.3,0,0],radius_m=.2))
        self.assertIsNone(self.propose())
        self.scene['objects'].pop()
        self.assertIsNone(self.propose(view=dict(self.view,xyz=[-4,0,.09])))
        self.assertIsNone(self.propose(view=dict(self.view,xyz=[-9.5,0,.09])))
        row=np.linspace(-10,10,17)*math.tan(math.radians(30))
        self.scene['terrain']['base_heights']=np.tile(row,(17,1)).tolist()
        self.assertIsNone(self.propose())

    def test_dense_frame_uses_a_fractional_proposal_limit_and_still_rejects_widespread_errors(self):
        rows=[dict(name='target',passed=True)]+[dict(name=f'rock_{i}',passed=i>=4,
            usable_depth_expected_pixels=40,sensor_valid_depth_pixels=40,
            support_fraction=.8 if i<4 else 1.,median_error_m=.0001) for i in range(40)]
        quality=dict(visibility_check=dict(checked=True,objects=rows))
        def checks(count):
            return dict(self.checks,errors=[f'rock_{i}: insufficient_sensor_depth_support' for i in range(count)])
        self.assertIsNotNone(self.propose(quality=quality,checks=checks(4)))
        for row in rows[1:12]:row.update(passed=False,support_fraction=.8)
        self.assertIsNone(self.propose(quality=quality,checks=checks(11)))

    def edge_fixture(self):
        scene=copy.deepcopy(self.scene)
        scene['config']['camera'].update(width=8,height=6,near_m=.3,far_m=8.,rgb_far_m=20.)
        expected=np.full((6,8),7.6)
        instances=np.zeros((6,8),int);instances[2:4,:4]=2
        observed=expected.copy();observed[2,:3]+=.04
        quality=copy.deepcopy(self.quality)
        quality['visibility_check']['objects'][1].update(usable_depth_expected_pixels=8,
            sensor_valid_depth_pixels=8,support_fraction=5/8,median_error_m=.0001)
        rendered=dict(quality=quality,labels=[dict(name='incidental',bbox_xyxy=[0,2,4,4],truncated=True)],
            objects=[dict(name='target'),dict(name='incidental')],instances=instances,
            expected=expected,observed=observed,pose=([-2.4,0,.08],[0,0,0,1]),
            extrinsics=(np.eye(3),np.zeros(3)),intrinsics=dict(width=8,height=6,fx=5.,fy=5.,cx=4.,cy=3.))
        return scene,rendered

    def edge_context(self,scene,rendered,view=None,ground=None):
        ground=rendered['observed'] if ground is None else ground
        with patch('mars_training_environment.annotation.rasterize',return_value=ground):
            return terrain_visibility_context(scene,view or self.view,self.checks,rendered,
                                               (np.zeros((3,3)),np.array([[0,1,2]])))

    def test_far_clipped_surface_requires_measured_terrain_agreement(self):
        scene,rendered=self.edge_fixture()
        context=self.edge_context(scene,rendered)
        self.assertEqual(context['objects']['incidental']['terrain_agreement_fraction'],1.)
        self.assertEqual(context['objects']['incidental']['mismatched_pixels'],3)
        self.assertTrue(context['full_frame_acceptance_required'])
        # The original rejection is preserved; context only permits another pose.
        self.assertEqual(rendered['quality']['visibility_check']['objects'][1]['support_fraction'],5/8)
        self.assertFalse(rendered['quality']['visibility_check']['objects'][1]['passed'])
        self.assertIsNone(self.edge_context(scene,rendered,ground=rendered['expected']))

    def test_low_support_cannot_recover_without_independent_context(self):
        scene,rendered=self.edge_fixture()
        self.assertIsNone(self.propose(scene=scene,quality=rendered['quality']))
        context=self.edge_context(scene,rendered)
        result=self.propose(scene=scene,quality=rendered['quality'],recovery_context=context)
        self.assertIsNotNone(result)
        self.assertEqual(result['view_recoveries'][0]['reason'],'clipped_incidental_terrain_visibility')
        self.assertEqual(result['rpy'][2],EDGE_YAW_OFFSETS[0])
        self.assertEqual(result['target_name'],self.view['target_name'])
        self.assertEqual(result['xyz'],self.view['xyz'])
        self.assertEqual(result['requested_export_class_id'],self.view['requested_export_class_id'])

    def test_edge_recovery_turns_toward_each_edge_and_is_bounded(self):
        for side in ['left','right']:
            scene,rendered=self.edge_fixture()
            if side=='right':rendered['labels'][0]['bbox_xyxy']=[4,2,8,4]
            context=self.edge_context(scene,rendered)
            view=self.view
            for offset in EDGE_YAW_OFFSETS:
                view=self.propose(scene=scene,quality=rendered['quality'],view=view,recovery_context=context)
                self.assertAlmostEqual(view['rpy'][2],offset if side=='left' else -offset)
            self.assertIsNone(self.propose(scene=scene,quality=rendered['quality'],view=view,
                                          recovery_context=context))

    def test_new_unclipped_failure_cannot_use_edge_exception(self):
        scene,rendered=self.edge_fixture();rendered['labels'][0].update(truncated=False,bbox_xyxy=[1,2,5,4])
        self.assertIsNone(self.edge_context(scene,rendered))

    def test_original_edge_object_can_continue_inside_frame_with_fresh_evidence(self):
        scene,rendered=self.edge_fixture();context=self.edge_context(scene,rendered)
        view=self.propose(scene=scene,quality=rendered['quality'],recovery_context=context)
        rendered['labels'][0].update(truncated=False,bbox_xyxy=[1,2,5,4])
        fresh=self.edge_context(scene,rendered,view=view)
        self.assertIsNotNone(fresh)
        self.assertIsNone(self.edge_context(scene,rendered,view=view,ground=rendered['expected']))
        self.assertIsNone(self.propose(scene=scene,quality=rendered['quality'],view=view))

    def test_near_objects_missing_depth_and_opposite_surface_error_are_not_eligible(self):
        for change in ['near','missing','front','invalid','large','not_clipped']:
            scene,rendered=self.edge_fixture()
            if change=='near':rendered['expected'][2,0]=7.0
            elif change=='missing':rendered['observed']=None
            elif change=='front':rendered['observed'][2,:3]=rendered['expected'][2,:3]-.04
            elif change=='invalid':rendered['observed'][2,:3]=np.nan
            elif change=='large':rendered['quality']['visibility_check']['objects'][1]['support_fraction']=.4
            else:rendered['labels'][0]['truncated']=False
            with self.subTest(change=change):self.assertIsNone(self.edge_context(scene,rendered))


class LocalizedPixelEvidenceTests(unittest.TestCase):
    setUp=CaptureRecoveryTests.setUp
    propose=CaptureRecoveryTests.propose
    def pixel_fixture(self):
        scene=copy.deepcopy(self.scene)
        scene['config']['camera'].update(width=128,height=128,near_m=.3,far_m=8.)
        expected=np.full((128,128),5.)
        observed=expected.copy();instances=np.zeros((128,128),int)
        instances[50:60,50:60]=1
        rows=[dict(name='target',passed=True)]
        objects=[dict(name='target')];errors=[]
        for i in range(3):
            name=f'incidental_{i}'
            mask=np.zeros((128,128),bool);mask[i*2:i*2+2,:25]=True
            instances[mask]=i+2
            values=np.full(50,5.001);values[:15]=5.04;observed[mask]=values
            objects.append(dict(name=name))
            rows.append(dict(name=name,passed=False,usable_depth_expected_pixels=50,
                sensor_valid_depth_pixels=50,support_fraction=.7,median_error_m=.001))
            errors.extend([name+': insufficient_sensor_depth_support',name+': insufficient_sensor_depth_box_support'])
        quality=dict(visibility_check=dict(checked=True,objects=rows))
        checks=dict(errors=errors,target_framing=dict(passed=True))
        rendered=dict(expected=expected,observed=observed,instances=instances,objects=objects,quality=quality)
        return scene,checks,rendered

    def test_multiple_small_incidental_failures_use_measured_pixel_locality(self):
        scene,checks,rendered=self.pixel_fixture()
        self.assertIsNone(self.propose(scene=scene,checks=checks,quality=rendered['quality']))
        context=localized_depth_context(scene,self.view,checks,rendered)
        self.assertIsNotNone(context)
        self.assertLess(context['affected_pixel_fraction'],.02)
        self.assertGreater(context['global_support_fraction'],.98)
        self.assertIsNotNone(self.propose(scene=scene,checks=checks,quality=rendered['quality'],
                                         recovery_context=context))
        self.assertEqual(checks['errors'][0],'incidental_0: insufficient_sensor_depth_support')

    def test_missing_sensor_global_offset_or_invalid_coverage_prevents_proposals(self):
        for change in ['missing','offset','dropout','pixel_area','severity','primary']:
            scene,checks,rendered=self.pixel_fixture()
            if change=='missing':rendered['observed']=None
            elif change=='offset':rendered['observed']+=.005
            elif change=='dropout':rendered['observed'][:20]=np.nan
            elif change=='pixel_area':rendered['instances'][20:50]=2
            elif change=='severity':rendered['quality']['visibility_check']['objects'][1]['median_error_m']=float('inf')
            else:
                checks['errors'].append('target: insufficient_sensor_depth_support')
                rendered['quality']['visibility_check']['objects'][0]['passed']=False
            with self.subTest(change=change):
                self.assertIsNone(localized_depth_context(scene,self.view,checks,rendered))

    def test_pixel_evidence_does_not_hide_non_depth_errors_or_change_thresholds(self):
        scene,checks,rendered=self.pixel_fixture()
        original=copy.deepcopy(scene['config']['labels'])
        checks['errors'].append('target_outside_frame_margin')
        self.assertIsNone(localized_depth_context(scene,self.view,checks,rendered))
        self.assertEqual(scene['config']['labels'],original)

    def test_localized_evidence_gets_two_turns_then_safe_orbits_with_finite_limit(self):
        scene,checks,rendered=self.pixel_fixture()
        context=localized_depth_context(scene,self.view,checks,rendered)
        view=self.view
        for index in range(8):
            view=self.propose(scene=scene,view=view,checks=checks,quality=rendered['quality'],recovery_context=context)
            self.assertIsNotNone(view)
            self.assertEqual(view['view_recoveries'][-1]['candidate_index'],index)
            self.assertEqual(view['view_recoveries'][-1]['orbit_about_target'],index>=2)
            self.assertEqual(view['target_name'],'target')
            distance=np.linalg.norm(np.asarray(view['xyz'][:2]))
            self.assertAlmostEqual(distance,np.linalg.norm(self.view['xyz'][:2]))
        self.assertIsNone(self.propose(scene=scene,view=view,checks=checks,quality=rendered['quality'],recovery_context=context))

    def test_severe_local_object_rejection_can_only_authorize_another_fresh_view(self):
        scene,checks,rendered=self.pixel_fixture()
        row=rendered['quality']['visibility_check']['objects'][1]
        row.update(support_fraction=.2,median_error_m=.2)
        mask=rendered['instances']==2
        values=np.full(int(mask.sum()),5.2);values[:10]=5.001
        rendered['observed'][mask]=values
        context=localized_depth_context(scene,self.view,checks,rendered)
        self.assertIsNotNone(context)
        self.assertGreater(context['global_support_fraction'],.98)
        result=self.propose(scene=scene,checks=checks,quality=rendered['quality'],recovery_context=context)
        self.assertIsNotNone(result)
        self.assertTrue(result['view_recoveries'][-1]['acceptance_policy_unchanged'])
        self.assertEqual(scene['config']['labels']['depth_support_fraction'],.9)
        self.assertFalse(row['passed'])

    def test_rejected_alternative_with_bad_framing_continues_to_next_candidate_without_export(self):
        scene,checks,rendered=self.pixel_fixture()
        context=localized_depth_context(scene,self.view,checks,rendered)
        first=self.propose(scene=scene,checks=checks,quality=rendered['quality'],recovery_context=context)
        mixed=dict(checks,errors=checks['errors']+['target_too_occluded'],target_framing=dict(passed=False))
        self.assertIsNone(localized_depth_context(scene,self.view,mixed,rendered))
        fresh=localized_depth_context(scene,first,mixed,rendered)
        self.assertIsNotNone(fresh)
        second=self.propose(scene=scene,view=first,checks=mixed,quality=rendered['quality'],recovery_context=fresh)
        self.assertIsNotNone(second)
        self.assertEqual(second['view_recoveries'][-1]['candidate_index'],1)
        self.assertIn('target_too_occluded',second['view_recoveries'][-1]['rejected_errors'])
        for error in ['target_not_visible','aggregate_mesh_depth_mismatch','target_class_does_not_match_requested_quota']:
            bad=dict(mixed,errors=mixed['errors']+[error])
            self.assertIsNone(localized_depth_context(scene,first,bad,rendered))


class DepthBoundaryRecoveryTests(unittest.TestCase):
    setUp=CaptureRecoveryTests.setUp
    propose=CaptureRecoveryTests.propose

    def fixture(self):
        scene=copy.deepcopy(self.scene)
        scene['config']['camera'].update(width=128,height=128,near_m=.3,far_m=8.,
                                         rgb_far_m=20.,far_clip_mode='axial')
        expected=np.full((128,128),5.)
        instances=np.zeros((128,128),int);instances[50:60,50:60]=1;instances[0,:48]=2
        expected[0,:9]=np.linspace(7.987,7.996,9);expected[0,9:48]=8.05
        observed=expected.copy();observed[0,3:48]=np.inf
        rows=[dict(name='target',passed=True),dict(name='incidental',source='mesh',passed=False,
            usable_depth_expected_pixels=9,sensor_valid_depth_pixels=3,support_fraction=1/3,
            median_error_m=0.,visible_pixels=48,rgb_only_pixels=39)]
        checks=dict(errors=['incidental: '+reason for reason in [
            'insufficient_sensor_depth_support','insufficient_sensor_depth_box_support',
            'sensor_depth_box_extent_mismatch']],target_framing=dict(passed=True))
        rendered=dict(expected=expected,observed=observed,instances=instances,
            objects=[dict(name='target'),dict(name='incidental')],
            quality=dict(visibility_check=dict(checked=True,objects=rows)))
        return scene,checks,rendered

    def test_measured_cutoff_sliver_permits_a_fresh_view_but_never_accepts_the_frame(self):
        scene,checks,rendered=self.fixture();original=copy.deepcopy(scene)
        self.assertIsNone(self.propose(scene=scene,checks=checks,quality=rendered['quality']))
        context=localized_depth_context(scene,self.view,checks,rendered)
        self.assertIsNotNone(context)
        evidence=context['objects']['incidental']['depth_boundary']
        self.assertEqual((evidence['usable_pixels'],evidence['valid_pixels'],evidence['clipped_pixels']),(9,3,6))
        self.assertEqual(evidence['geometry_pixels_beyond_depth'],39)
        result=self.propose(scene=scene,checks=checks,quality=rendered['quality'],recovery_context=context)
        self.assertIsNotNone(result)
        self.assertEqual(result['target_name'],self.view['target_name'])
        self.assertEqual(result['requested_export_class_id'],self.view['requested_export_class_id'])
        self.assertFalse(rendered['quality']['visibility_check']['objects'][1]['passed'])
        self.assertEqual(scene,original)
        self.assertEqual(scene['config']['labels']['depth_support_fraction'],.9)

    def test_bad_sensor_data_non_boundary_disagreement_and_primary_failure_still_stop(self):
        for change in ['missing','nan','zero','near','negative_inf','too_close','no_far_geometry',
                       'no_supported_pixel','supported_error','global_error','primary','class','radial','ground']:
            scene,checks,rendered=self.fixture()
            if change=='missing':rendered['observed']=None
            elif change=='nan':rendered['observed'][0,3]=np.nan
            elif change=='zero':rendered['observed'][0,3]=0.
            elif change=='near':rendered['observed'][0,3]=1.
            elif change=='negative_inf':rendered['observed'][0,3]=-np.inf
            elif change=='too_close':rendered['expected'][0,3]=7.9
            elif change=='no_far_geometry':rendered['expected'][0,9:48]=7.999
            elif change=='no_supported_pixel':rendered['observed'][0,:9]=np.inf
            elif change=='supported_error':rendered['observed'][0,0]-=.01
            elif change=='global_error':rendered['observed'][10:]+= .005
            elif change=='primary':checks['errors'].append('target: insufficient_sensor_depth_support')
            elif change=='class':checks['errors'].append('target_class_does_not_match_requested_quota')
            elif change=='radial':scene['config']['camera']['far_clip_mode']='radial'
            else:rendered['quality']['visibility_check']['objects'][1]['source']='ground_region'
            with self.subTest(change=change):
                self.assertIsNone(localized_depth_context(scene,self.view,checks,rendered))

    def test_low_coverage_and_box_extent_mismatch_require_the_complete_boundary_proof(self):
        scene,checks,rendered=self.fixture()
        context=localized_depth_context(scene,self.view,checks,rendered)
        for field,value in [('limited_to_far_clip',False),('clipped_pixels',5),('far_m',20.),
                            ('expected_usable_min_m',7.9),('band_m',.2),('valid_pixels',0)]:
            invalid=copy.deepcopy(context);invalid['objects']['incidental']['depth_boundary'][field]=value
            with self.subTest(field=field):
                self.assertIsNone(self.propose(scene=scene,checks=checks,quality=rendered['quality'],recovery_context=invalid))

    def test_boundary_recovery_uses_the_existing_finite_pose_search(self):
        scene,checks,rendered=self.fixture()
        context=localized_depth_context(scene,self.view,checks,rendered);view=self.view
        for index in range(8):
            view=self.propose(scene=scene,view=view,checks=checks,quality=rendered['quality'],recovery_context=context)
            self.assertEqual(view['view_recoveries'][-1]['candidate_index'],index)
        self.assertIsNone(self.propose(scene=scene,view=view,checks=checks,
                                      quality=rendered['quality'],recovery_context=context))


if __name__=='__main__':
    unittest.main()
