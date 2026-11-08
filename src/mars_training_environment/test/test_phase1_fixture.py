"""Independent geometry, capture decoding and provenance failure regressions."""
import importlib.util
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from phase1_analyze_fixture import BOX_FACES, box_depth, expected, rgb_support, near_background_metrics
from phase1_capture_fixture import camera_intrinsics, decode, trajectory
from phase1_run_fixture import load_guard
from phase1_run_fixture import BASELINE
from phase1_supervise_live import verify_configuration
from phase1_pose_controller import validate_pose
from phase1_pose_sync import measured_pose_bracket
from phase1_visibility_fixture import generate
from mars_training_environment.annotation import rasterize


class VisibilityFixtureTests(unittest.TestCase):
    def test_pose_near_in_time_is_insufficient_when_an_update_crosses_the_frame(self):
        before=[10_000_000,[0,0,.762],[0,.2,0,.98]]
        after=[40_000_000,[0,0,.762],[0,.2,0,.98]]
        self.assertTrue(measured_pose_bracket([before,after],20_000_000)['stable'])
        after[2]=[0,.2,.003,.98]
        bracket=measured_pose_bracket([before,after],20_000_000)
        self.assertLess(bracket['gap_s'],.05)
        self.assertFalse(bracket['stable'])
        self.assertIsNone(measured_pose_bracket([before],20_000_000))
        after[2]=before[2];after[0]=100_000_000
        self.assertFalse(measured_pose_bracket([before,after],20_000_000)['stable'])

    def test_near_check_excludes_silhouette_but_still_detects_interior_rgb_loss(self):
        mask=np.zeros((30,30),bool);mask[5:25,5:25]=True
        rgb=np.zeros((30,30,3),np.uint8);rgb[7:23,7:23]=255
        metrics=near_background_metrics(rgb,mask,[1,1,1])
        self.assertLess(metrics['near_raw_background_support_fraction'],.9)
        self.assertEqual(metrics['near_comparison_pixels'],256)
        self.assertEqual(metrics['near_background_support_fraction'],1)
        rgb[7:23,7:23]=0
        self.assertEqual(near_background_metrics(rgb,mask,[1,1,1])['near_background_support_fraction'],0)

    def test_pose_client_is_restricted_to_the_fixture_trajectory(self):
        self.assertEqual(validate_pose(1.8,.35),(1.8,.35))
        for x,yaw in [(-.1,0),(1.81,0),(0,.36),(float('nan'),0),(0,float('inf'))]:
            with self.assertRaises(ValueError): validate_pose(x,yaw)

    def test_measured_camera_principal_point_is_preserved(self):
        info=dict(width=960,height=540,intrinsics=dict(k=[686.7,0,480.,0,686.7,270.,0,0,1]))
        k=camera_intrinsics(info)
        self.assertEqual((k['cx'],k['cy']),(480.,270.))
        info['intrinsics']['k'][0]=float('nan')
        with self.assertRaises(ValueError): camera_intrinsics(info)

    def test_live_candidate_requires_separate_image_and_exact_source_pins(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); source=root/'source.py'; source.write_text('pinned')
            scene=root/'scene.json'; scene.write_text('{}')
            configuration=dict(project=str(root),baseline_image=BASELINE,image='sha256:'+'1'*64,
                files={'source.py':hashlib.sha256(source.read_bytes()).hexdigest()},
                scene_file=str(scene),scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                camera=dict(width=960,height=540,near_m=.3,far_m=8.,rgb_far_m=20.,tilt_rad=.4,
                            far_clip_mode='axial',near_clip_mode='occlude',render_engine='ogre'))
            path=root/'configuration.json'; path.write_text(json.dumps(configuration))
            self.assertEqual(verify_configuration(path)[1],root.resolve())
            source.write_text('changed')
            with self.assertRaisesRegex(ValueError,'source/install changed'): verify_configuration(path)
            source.write_text('pinned'); configuration['image']=BASELINE; path.write_text(json.dumps(configuration))
            with self.assertRaisesRegex(ValueError,'Separate candidate'): verify_configuration(path)

    def test_rgb_check_distinguishes_buried_target_from_foreground_ground(self):
        self.assertFalse(rgb_support([[59,19,6]],[.8,.8,.1])[0])
        for texture in ([210,120,70],[230,145,80]):
            sample=.8*np.array(texture)*[.8,.8,.1]
            self.assertTrue(rgb_support([sample],[.8,.8,.1])[0])

    def test_range_changes_only_camera_limits_and_keeps_assets_deterministic(self):
        with tempfile.TemporaryDirectory() as directory:
            a=generate(Path(directory)/'a',8,asset_root=Path('/evidence/assets'))
            b=generate(Path(directory)/'b',20,asset_root=Path('/evidence/assets'))
            self.assertEqual(a['objects'],b['objects'])
            self.assertEqual(a['asset_sha256'],b['asset_sha256'])
            self.assertEqual(a['camera']['far_m'],8)
            self.assertEqual(b['camera']['far_m'],8)
            self.assertEqual(b['camera']['near_m'],.3)
            self.assertIn('/evidence/assets/rock.dae',(Path(directory)/'a/world.sdf').read_text())
            pose=[0,a['camera']['xyz'],[0,np.sin(.2),0,np.cos(.2)]]
            _,owners,layers=expected(a,pose,8,False)
            self.assertFalse(np.isfinite(layers['near_clipped']).any())
            self.assertTrue(np.isfinite(layers['buried_rock']).any())
            self.assertGreater((owners==next(i for i,o in enumerate(a['objects'],1) if o['name']=='overlap_front')).sum(),0)
        with tempfile.TemporaryDirectory() as directory:
            for value in (7,21,np.nan,np.inf):
                with self.assertRaises(ValueError): generate(Path(directory)/'bad',value)
                self.assertFalse((Path(directory)/'bad').exists())

    def test_analytic_box_depth_agrees_with_annotation_without_using_its_rasterizer(self):
        item=dict(xyz=[.5,0,4],size=[2,2,1])
        k=dict(width=60,height=40,fx=40.,fy=40.,cx=29.5,cy=19.5)
        vertices=np.array([[x,y,z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]])*item['size']+item['xyz']
        for radial in (False,True):
            analytic=box_depth(item,np.zeros(3),np.eye(3),k,.3,4,radial)
            annotation=rasterize(vertices,BOX_FACES,k,.3,4,far_clip_mode='radial' if radial else 'axial')
            np.testing.assert_array_equal(np.isfinite(analytic),np.isfinite(annotation))
            np.testing.assert_allclose(analytic,annotation)

    def test_decoding_respects_strides_and_rejects_unknown_pixel_formats(self):
        rgb=SimpleNamespace(width=2,height=1,step=8,pixel_format_type=3,data=bytes([1,2,3,4,5,6,0,0]))
        depth=SimpleNamespace(width=2,height=1,step=12,pixel_format_type=13,data=np.array([1.,7.,99.],dtype='<f4').tobytes())
        pixels,values=decode(rgb,depth)
        np.testing.assert_array_equal(pixels,[[[1,2,3],[4,5,6]]])
        np.testing.assert_array_equal(values,[[1.,7.]])
        depth.pixel_format_type=6
        with self.assertRaises(ValueError): decode(rgb,depth)

    def test_evaluator_replay_returns_to_identical_pose_and_has_bounded_motion(self):
        for mode,period in [('stationary',120),('approach_retreat',120),('rotation',60)]:
            np.testing.assert_allclose(trajectory(mode,0),trajectory(mode,period),atol=1e-12)
            samples=np.array([trajectory(mode,t) for t in np.arange(0,120,.1)])
            self.assertLess(np.abs(np.diff(samples[:,0])).max()/.1,.05)
            self.assertLess(np.abs(np.diff(samples[:,1])).max()/.1,.30)

    def test_changed_operator_kit_is_rejected_before_import_or_workload(self):
        with tempfile.TemporaryDirectory() as directory:
            kit=Path(directory)
            (kit/'trial.py').write_text("raise RuntimeError('must not import')")
            (kit/'KIT_MANIFEST.json').write_text(json.dumps(dict(files={'trial.py':'0'*64})))
            with self.assertRaisesRegex(RuntimeError,'operator kit changed'): load_guard(kit)

    def test_renderer_patch_refuses_different_or_already_patched_source(self):
        spec=importlib.util.spec_from_file_location('phase1_patch',ROOT/'docker/rendering/patch_renderer.py')
        module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        with self.assertRaisesRegex(ValueError,'Unexpected OgreMaterial'): module.patch(b'changed source')


if __name__=='__main__': unittest.main()
