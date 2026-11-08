import json
from pathlib import Path
import tempfile
import unittest
from contextlib import contextmanager
import numpy as np

from mars_training_environment.common import read_config
from mars_training_environment.environment_v2 import (configuration,apron_mesh,operating_map_size,
    export_class_id,target_names,stream,qualification_recipe,QUALIFICATION_SEEDS)
from mars_training_environment.planning import class_view_schedule,targets_for_class,class_coverage
from mars_training_environment.reachability import validate_profile,validate_layout,mixed_reachable_config
from mars_training_environment.terrain import Terrain
from mars_training_environment.capture_attempts import evidence_hashes,verify_completed,commit_attempt


class EnvironmentV2Tests(unittest.TestCase):
    def test_profile_and_legacy_are_distinct(self):
        old=mixed_reachable_config(read_config());new=configuration(read_config())
        validate_profile(old['terrain']);validate_profile(new['terrain'])
        self.assertEqual(old['terrain']['size_m'],20.)
        self.assertEqual(new['terrain']['composite_px'],8192)
        self.assertEqual(mixed_reachable_config(new),new)
        self.assertEqual(operating_map_size({'config':new}),40.)
        with self.assertRaises(ValueError):operating_map_size({'config':new},20)
        new['terrain']['grid_size']=129
        with self.assertRaises(ValueError):validate_profile(new['terrain'])

    def test_apron_has_exact_boundary_and_no_operating_faces(self):
        x,y=np.meshgrid(np.linspace(-20,20,257),np.linspace(-20,20,257))
        field=Terrain(.1*np.sin(x/5)+.2*np.cos(y/7),40)
        v,f=apron_mesh(field)
        self.assertEqual(float(np.abs(v[:,:2]).max()),40.)
        boundary=np.max(abs(v[:,:2]),axis=1)==20.
        self.assertTrue(np.allclose(v[boundary,2],field.height(v[boundary,0],v[boundary,1]),atol=1e-10))
        centers=v[f].mean(axis=1)
        self.assertTrue((np.max(abs(centers[:,:2]),axis=1)>20).all())
        self.assertTrue(np.isfinite(v).all())

    def test_windowed_material_recipe_is_reproducible_and_keeps_mask_scale(self):
        from mars_training_environment.candidates import paint_ground_regions
        regions=[dict(name='soil',class_name='silica_soil',mask_id=1,dust_cover=.3,
                      polygon_xy=[[-.4,-.3],[.4,-.3],[.4,.3],[-.4,.3]])]
        def paint(windowed):
            return paint_ground_regions(np.full((512,512,3),100,np.float32),
                np.tile([0.,0.,1.],(512,512,1)).astype(np.float32),np.full((512,512),100,np.float32),
                regions,4.,np.random.RandomState(77),region_windows=windowed)
        a,b,old=paint(True),paint(True),paint(False)
        self.assertTrue(all(np.array_equal(x,y) for x,y in zip(a,b)))
        self.assertTrue(np.array_equal(a[3],old[3]))
        self.assertTrue(np.isfinite(a[0]).all())
        self.assertTrue(np.allclose(np.linalg.norm(a[1],axis=2),1.))

    def test_four_class_targeting_does_not_relabel_source(self):
        cap=configuration(read_config())['capture']
        schedule=class_view_schedule(cap,stream(1,'views'))
        self.assertEqual([schedule.count(i) for i in range(4)],[5]*4)
        slab=dict(name='slab',class_name='cheyava_rock',asset_id='cand_slab')
        rock=dict(name='rock',class_name='rock')
        self.assertEqual(targets_for_class([slab,rock],3,cap),[slab])
        self.assertEqual(export_class_id(slab),3)
        from rover_training.geometry import class_id_for,CLASS_NAMES
        self.assertEqual(class_id_for('cheyava_rock'),0)
        self.assertEqual(len(CLASS_NAMES),3)

    def test_four_class_audit_reports_mudstone_and_rejects_duplicate_viewpoints(self):
        from rover_training.geometry import CLASS_NAMES
        cap=configuration(read_config())['capture']
        objects=[dict(name='rock',class_name='rock'),dict(name='soil',class_name='silica_soil'),
                 dict(name='crystal',class_name='sulfur_rock'),dict(name='slab',class_name='cheyava_rock')]
        from rover_training.geometry import class_id_for
        views=[]
        for export,obj in enumerate(objects):
            source=class_id_for(obj['class_name'])
            for i in range(5):
                views.append(dict(id=str(len(views)),target_name=obj['name'],target_class_id=source,
                    requested_target_class_id=source,target_export_class_id=export,requested_export_class_id=export,
                    xyz=[float(len(views)),0.,0.],rpy=[0.,0.,0.]))
        scene=dict(id='fixture',split='train',seed=1,empty=False,class_names=list(CLASS_NAMES),
                   config=dict(capture=cap,dataset={}),objects=objects,views=views)
        report=class_coverage([scene])
        self.assertFalse(report['errors'])
        self.assertEqual(report['expected_target_counts']['train']['mudstone_slab'],5)
        views[1]['xyz']=views[0]['xyz'][:]
        self.assertTrue(class_coverage([scene])['errors'])

    def test_background_certifies_outer_connectivity_without_targets(self):
        r=validate_layout(Terrain(np.zeros((257,257)),40),[],[],version=2,background=True)
        self.assertEqual(r['routes'],[])
        self.assertFalse(r['truth_to_navigation'])

    def test_predeclared_seeds_and_random_stream_independence(self):
        self.assertEqual(len(set(QUALIFICATION_SEEDS)),50)
        self.assertEqual({qualification_recipe(i)['density'] for i in range(50)},{'sparse','standard','cluttered'})
        self.assertTrue(np.array_equal(stream(4,'lighting').rand(8),stream(4,'lighting').rand(8)))
        self.assertFalse(np.array_equal(stream(4,'lighting').rand(8),stream(4,'views').rand(8)))

    def test_resume_checks_content_and_commit_is_idempotent(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);attempt=root/'attempt';output=root/'output'
            for p in ('metadata/train','images/train','labels/train'):(attempt/p).mkdir(parents=True)
            (attempt/'images/train/a.png').write_bytes(b'fixture')
            (attempt/'labels/train/a.txt').write_text('')
            meta=dict(scene_sha256='scene',image='images/train/a.png',label='labels/train/a.txt')
            (attempt/'metadata/train/a.json').write_text(json.dumps(meta))
            entry=dict(id='scene',content_sha256='scene',views=1)
            report=dict(scene_sha256='scene',frames=1,records=['metadata/train/a.json'])
            report['evidence_sha256']=evidence_hashes(attempt,report,entry)
            commit_attempt(attempt,output,report,entry);commit_attempt(attempt,output,report,entry)
            verify_completed(output,report,entry)
            (output/'images/train/a.png').write_bytes(b'tampered')
            with self.assertRaises(ValueError):verify_completed(output,report,entry)

    def test_failed_attempts_keep_their_individual_logs(self):
        from mars_training_environment.capture_attempts import capture_scene
        @contextmanager
        def failing(cmd,log,qualified_desktop=False):
            self.assertTrue(qualified_desktop);log.write(b'preserve this attempt\n');log.flush()
            class Process:
                def wait(self,timeout):return 1
            yield Process()
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);entry=dict(id='scene_1',views=20,content_sha256='scene')
            scene=dict(config=configuration(read_config()))
            with self.assertRaisesRegex(RuntimeError,'all evidence retained'):
                capture_scene(root,root,entry,scene,False,failing)
            attempts=sorted((root/'attempts/scene_1').iterdir())
            self.assertEqual(len(attempts),3)
            self.assertTrue(all((d/'launch.log').read_bytes()==b'preserve this attempt\n' for d in attempts))
            self.assertTrue(all(json.loads((d/'attempt-result.json').read_text())['result']=='FAILED_PRESERVED' for d in attempts))
