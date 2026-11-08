import math
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from mars_training_environment.robot import camera_clips
from mars_training_environment.common import read_config
from mars_training_environment.annotation import Annotator, rasterize, instance_depth_support
from mars_training_environment.generate import write_world


class ClipContractTests(unittest.TestCase):
    def test_candidate_profile_selects_ogre_for_future_capture_worlds(self):
        root=Path(__file__).resolve().parents[3]
        profile=read_config(root/'config/rendering/phase1_ogre_candidate.json')
        self.assertEqual(camera_clips(profile['camera']),(.3,20,8))
        self.assertEqual(profile['camera']['near_clip_mode'],'occlude')
        with tempfile.TemporaryDirectory() as directory:
            world=Path(directory)/'world.sdf'
            scene=dict(id='evaluator_only',config=profile,objects=[],ground=dict(layers=[]),
                       lighting=dict(azimuth=0,elevation=1,intensity=1,ambient=.4,sky_rgb=[.5]*3))
            write_world(directory,scene,[],output_path=world)
            self.assertEqual(ET.parse(world).findtext('.//plugin/render_engine'),'ogre')
            bad=Path(directory)/'bad.json'
            bad.write_text(json.dumps(dict(camera=dict(render_engine='ogre'))))
            with self.assertRaisesRegex(ValueError,'matching qualified clipping'): read_config(bad)

    def test_ogre_near_occluder_cannot_invent_a_farther_rgb_label(self):
        k=dict(width=60,height=40,fx=40.,fy=40.,cx=29.5,cy=19.5)
        faces=np.array([[0,1,2],[0,2,3]])
        square=lambda z,s: np.array([[-s,-s,z],[s,-s,z],[s,s,z],[-s,s,z]])
        annotator=Annotator.__new__(Annotator)
        annotator.scene=dict(config=read_config())
        camera=annotator.scene['config']['camera']
        camera.update(near_clip_mode='occlude',far_clip_mode='axial')
        annotator.terrain=(square(4,10),faces)
        annotator.meshes=[(dict(name='near',asset_id='near'),square(.2,.04),faces),
                          (dict(name='behind',asset_id='behind'),square(2,.5),faces)]
        annotator.regions=[]; annotator.paint_mask=None
        with patch('mars_training_environment.annotation.camera_rigid_from_base',return_value=(np.eye(3),np.zeros(3))):
            _,_,quality,instances=annotator.annotate([0,0,0],[0,0,0,1],{},k)
            self.assertEqual(instances[19,29],0)
            self.assertTrue(np.isinf(annotator.last_expected_depth[19,29]))
            self.assertGreater(quality['near_occluded_pixels'],0)
            camera['near_clip_mode']='clip'
            _,_,_,historical=annotator.annotate([0,0,0],[0,0,0,1],{},k)
            self.assertEqual(historical[19,29],2)

    def test_old_profiles_keep_their_depth_contract(self):
        self.assertEqual(camera_clips(dict(near_m=.3, far_m=8)), (.3,8,8))

    def test_rgb_range_cannot_extend_usable_depth(self):
        self.assertEqual(camera_clips(dict(near_m=.3,far_m=8,rgb_far_m=20)),(.3,20,8))
        for value in (0,7,math.nan,math.inf):
            with self.assertRaises(ValueError):
                camera_clips(dict(near_m=.3,far_m=8,rgb_far_m=value))

    def test_config_accepts_explicit_range_without_changing_historical_profile(self):
        original=read_config()
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'profile.json'
            path.write_text(json.dumps({'camera':{'rgb_far_m':20,'far_clip_mode':'axial'}}))
            extended=read_config(path)
        self.assertEqual(camera_clips(extended['camera']),(.3,20,8))
        self.assertNotIn('rgb_far_m',original['camera'])

    def test_qualified_axial_rgb_and_historical_radial_clip_differ_off_axis(self):
        k=dict(width=80,height=60,fx=50.,fy=50.,cx=40.,cy=30.)
        vertices=[[3,-.4,7.8],[4,-.4,7.8],[4,.4,7.8],[3,.4,7.8]]
        faces=[[0,1,2],[0,2,3]]
        radial=rasterize(vertices,faces,k,.3,8)
        axial=rasterize(vertices,faces,k,.3,8,far_clip_mode='axial')
        self.assertFalse(np.isfinite(radial).any())
        self.assertTrue(np.isfinite(axial).any())
        self.assertTrue(np.allclose(axial[np.isfinite(axial)],7.8))

    def test_distant_rgb_label_cannot_pass_existing_depth_acceptance(self):
        instances=np.ones((10,10),np.int32)
        objects=[dict(name='far_rock')]
        for measured in (np.full((10,10),np.inf),np.full((10,10),12.)):
            checks=instance_depth_support(instances,np.full((10,10),12.),measured,
                                           objects,objects,read_config()['labels'],.3,8)
            self.assertFalse(checks[0]['passed'])
            self.assertEqual(checks[0]['sensor_valid_depth_pixels'],0)


if __name__=='__main__':
    unittest.main()
