import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import uuid

import capture_training_dataset as data
import capture_training_worker as worker
import start_capture5000_training as starter


def box(**changes):
    result=dict(name='item',class_id=0,class_name='rock',appearance_name='rock',
        xmin=10,ymin=20,xmax=30,ymax=40)
    result.update(changes);return result


class TrainingLabels(unittest.TestCase):
    def test_only_metadata_identified_slabs_change_class_and_boxes_are_preserved(self):
        original=[box(),box(name='slab',appearance_name='cheyava_rock'),
                  box(name='other_slab',appearance_name='veined_lookalike')]
        before=copy.deepcopy(original)
        converted,changes=data.four_class_labels(original)
        self.assertEqual(original,before)
        self.assertEqual([b['class_id'] for b in converted],[0,3,3])
        self.assertEqual(changes,['slab','other_slab'])
        for a,b in zip(original,converted):
            self.assertEqual([a[k] for k in ['xmin','ymin','xmax','ymax']],
                             [b[k] for k in ['xmin','ymin','xmax','ymax']])

    def test_candidate_asset_identity_is_an_explicit_slab_rule(self):
        labels,_=data.four_class_labels([box(asset_id='cand_slab',appearance_name='ordinary')])
        self.assertEqual(labels[0]['class_id'],3)

    def test_missing_identity_or_conflicting_class_is_not_silently_converted(self):
        for value in [box(appearance_name=None),box(class_id=1,class_name='pale_soil_patch',appearance_name='cheyava_rock'),
                      box(class_id=3,class_name='mudstone_slab'),box(class_id=True)]:
            with self.subTest(value=value),self.assertRaises(ValueError):data.four_class_labels([value])

    def test_rgb_only_targets_are_retained_without_depth(self):
        value=box(visibility_check=dict(depth_check_scope='geometry_only_beyond_usable_depth',valid_depth_pixels=0))
        labels,_=data.four_class_labels([value])
        self.assertEqual(labels,[value])

    def test_empty_frames_and_source_schema_have_exact_yolo_semantics(self):
        self.assertEqual(data.label_text([],960,540),'')
        self.assertEqual(data.label_text([box(xmin=0,ymin=0,xmax=960,ymax=540)],960,540),
                         '0 0.500000 0.500000 1.000000 1.000000\n')
        with self.assertRaises(ValueError):data.label_text([box(class_id=3)],960,540,classes=3)

    def test_non_finite_out_of_bounds_and_reversed_boxes_stop_export(self):
        for value in [box(xmin=float('nan')),box(xmax=float('inf')),box(xmin=-1),box(xmin=31),box(ymax=541)]:
            with self.subTest(value=value),self.assertRaises(ValueError):data.label_text([value],960,540)


class DatasetIsolation(unittest.TestCase):
    def rows(self):
        return [dict(id='a',split='train',scene_group='first',image_sha256='one'),
                dict(id='b',split='val',scene_group='second',image_sha256='two')]

    def test_related_views_or_identical_rgb_cannot_cross_splits(self):
        for key in ['scene_group','image_sha256']:
            rows=self.rows();rows[1][key]=rows[0][key]
            with self.subTest(key=key),self.assertRaises(ValueError):data.check_isolation(rows)

    def test_duplicate_identity_and_unknown_split_are_rejected(self):
        for key,value in [('id','a'),('split','other')]:
            rows=self.rows();rows[1][key]=value
            with self.assertRaises(ValueError):data.check_isolation(rows)

    def test_shared_training_scene_stays_together(self):
        rows=self.rows();rows[1].update(split='train',scene_group='first')
        data.check_isolation(rows)

    def test_path_escape_and_changed_copy_are_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name)
            with self.assertRaises(ValueError):data.inside(root,'../escape')
            source=root/'source';source.write_bytes(b'first');expected=data.sha(source);source.write_bytes(b'changed')
            with self.assertRaises(ValueError):data.copy_checked(source,root/'copy',expected)
            self.assertFalse((root/'copy').exists())

    def test_export_rejects_mutation_and_unpinned_extra_training_image(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);files={}
            for relative in ['images/train/a.png','labels/train/a.txt','metadata/train/a.json']:
                p=root/relative;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b'checked');files[relative]=data.sha(p)
            row=dict(self.rows()[0],image='images/train/a.png',label='labels/train/a.txt',metadata='metadata/train/a.json',
                image_sha256=files['images/train/a.png'],label_sha256=files['labels/train/a.txt'],metadata_sha256=files['metadata/train/a.json'])
            data.save(root/'DATASET_MANIFEST.json',dict(schema=1,class_names=list(data.NAMES),frames=[row],files=files,split_counts=dict(train=1)))
            data.verify_export(root)
            (root/'images/train/extra.png').write_bytes(b'unpinned')
            with self.assertRaisesRegex(ValueError,'Unpinned'):data.verify_export(root)
            (root/'images/train/extra.png').unlink();(root/'labels/train/a.txt').write_bytes(b'edited')
            with self.assertRaisesRegex(ValueError,'hash changed'):data.verify_export(root)


class MonitoringWindows(unittest.TestCase):
    def test_only_previously_verified_dead_sampled_owners_are_accepted(self):
        parent=dict(pid=10,start_ticks=100,uid=1001,ppid=1,token='owned')
        child=dict(pid=11,start_ticks=101,uid=1001,ppid=10,token='owned')
        processes={10:dict(parent),11:dict(child)}
        def inspect(pid):
            if pid not in processes:raise FileNotFoundError(pid)
            return processes[pid]
        starter.validate_compute_owners({10,11},parent,inspect)
        processes[11]=dict(child,state='Z',token=None)
        starter.validate_compute_owners({11},parent,inspect)
        self.assertEqual(parent['_retired_sampled_pids'],[11])
        processes[11]=dict(child,state='Z',start_ticks=999,token=None)
        with self.assertRaises(RuntimeError):starter.validate_compute_owners({11},parent,inspect)
        processes[11]=dict(child,token='unrelated')
        with self.assertRaises(RuntimeError):starter.validate_compute_owners({11},parent,inspect)
        del processes[11];starter.validate_compute_owners({11},parent,inspect)
        with self.assertRaises(RuntimeError):starter.validate_compute_owners({99},parent,inspect)
        processes[99]=dict(pid=99,start_ticks=300,uid=1001,ppid=10,state='Z',token=None)
        with self.assertRaises(RuntimeError):starter.validate_compute_owners({99},parent,inspect)
        processes[10]=dict(parent,state='Z',token=None)
        starter.validate_compute_owners({10,11},parent,inspect)
        del processes[10];starter.validate_compute_owners({10,11},parent,inspect)

    @unittest.skipUnless(sys.platform=='linux','Linux process identity lifecycle')
    def test_real_sampled_worker_identity_survives_its_clean_exit(self):
        token=str(uuid.uuid4())
        process=subprocess.Popen([sys.executable,'-c','import time;time.sleep(.5)'],
            env=dict(os.environ,ROVER_TRAINING_TOKEN=token),start_new_session=True)
        try:
            identity=starter.process_identity(process.pid)
            starter.validate_compute_owners({process.pid},identity)
            process.wait(timeout=5)
            starter.validate_compute_owners({process.pid},identity)
            self.assertEqual(identity['_retired_sampled_pids'],[process.pid])
        finally:
            if process.poll() is None:process.terminate()
            process.wait(timeout=5)

    def test_named_losses_preserve_components_and_reject_non_finite_or_missing_values(self):
        losses=dict(box_loss=.5,cls_loss=.25,dfl_loss=.75)
        self.assertEqual(worker.finite_losses(losses),losses)
        for invalid in [None,[],{},dict(box_loss=float('nan')),dict(cls_loss=float('inf'))]:
            with self.assertRaises(RuntimeError):worker.finite_losses(invalid)

    def test_compute_owners_require_live_worker_identity_and_descendant_token(self):
        parent=dict(pid=10,start_ticks=100,uid=1001,ppid=1,token='owned')
        child=dict(pid=11,start_ticks=101,uid=1001,ppid=10,token='owned')
        other=dict(pid=12,start_ticks=102,uid=1001,ppid=1,token=None)
        processes={10:parent,11:child,12:other}
        starter.validate_compute_owners({10,11},parent,processes.__getitem__)
        starter.validate_compute_owners(set(),None,processes.__getitem__)
        with self.assertRaises(RuntimeError):starter.validate_compute_owners({10},None,processes.__getitem__)
        with self.assertRaises(RuntimeError):starter.validate_compute_owners({12},parent,processes.__getitem__)
        for changed in [dict(token='other'),dict(uid=0),dict(ppid=1)]:
            processes[11]=dict(child,**changed)
            with self.assertRaises(RuntimeError):starter.validate_compute_owners({11},parent,processes.__getitem__)
        processes[11]=child;processes[10]=dict(parent,start_ticks=999)
        with self.assertRaises(RuntimeError):starter.validate_compute_owners({10},parent,processes.__getitem__)

    def test_pause_is_before_another_epoch_and_never_overrides_completion(self):
        self.assertTrue(worker.pause_due(900,1000,100,False))
        self.assertFalse(worker.pause_due(100,1000,100,False))
        self.assertFalse(worker.pause_due(1000,1000,100,True))

    def test_resume_requires_hash_optimizer_epoch_and_same_dataset(self):
        with tempfile.TemporaryDirectory() as name:
            p=Path(name)/'epoch.pt';p.write_bytes(b'checkpoint')
            checkpoint=dict(path=str(p),sha256=data.sha(p),completed_epochs=3)
            config=dict(recipe=dict(epochs=100),dataset='/dataset')
            valid=dict(epoch=2,optimizer={'state':'present'},train_args=dict(data='/dataset/data.yaml'))
            fake=Mock();fake.load.return_value=valid
            self.assertEqual(worker.check_resume(checkpoint,config,fake),p)
            for change in [dict(optimizer=None),dict(epoch=1),dict(train_args=dict(data='/other/data.yaml'))]:
                fake.load.return_value=dict(valid,**change)
                with self.assertRaises(RuntimeError):worker.check_resume(checkpoint,config,fake)
            p.write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError,'changed'):worker.check_resume(checkpoint,config,fake)

    def test_only_clean_checkpoint_progress_renews_and_closes_before_reopening(self):
        engine=Mock();engine.open_window.side_effect=['first','second']
        engine.train_window.side_effect=[dict(stage='PAUSED_FOR_MONITORING',completed_epochs=3),
            dict(stage='TRAINING_COMPLETE_PENDING_EVALUATION',completed_epochs=5)]
        engine.close_window.return_value=True
        with tempfile.TemporaryDirectory() as name,patch.object(starter,'HERE',Path(name)):
            result=starter.monitored_training(engine,{})
        self.assertEqual(result['completed_epochs'],5)
        self.assertEqual([call[0] for call in engine.mock_calls],
            ['open_window','train_window','close_window','open_window','train_window','close_window'])

    def test_training_failure_no_progress_or_unclean_finish_never_retries(self):
        for failure,epochs,closed in [(RuntimeError('failed'),3,True),(None,0,True),(None,3,False)]:
            engine=Mock();engine.train_window.side_effect=failure
            engine.train_window.return_value=dict(stage='PAUSED_FOR_MONITORING',completed_epochs=epochs)
            engine.close_window.return_value=closed
            with tempfile.TemporaryDirectory() as name,patch.object(starter,'HERE',Path(name)):
                with self.assertRaises(RuntimeError):starter.monitored_training(engine,{})
            engine.open_window.assert_called_once();engine.close_window.assert_called_once()

    @unittest.skipUnless(sys.platform=='linux' and hasattr(os,'pidfd_open'),'Linux pidfd ownership check')
    def test_real_cleanup_stops_only_the_token_owned_process(self):
        token=str(uuid.uuid4())
        own=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],
            env=dict(os.environ,ROVER_TRAINING_TOKEN=token),start_new_session=True)
        other=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True)
        try:
            result=starter.cleanup_owned(token,os.getuid())
            self.assertEqual(result['remaining'],[]);own.wait(timeout=5);self.assertIsNone(other.poll())
        finally:
            for process in [own,other]:
                if process.poll() is None:process.terminate()
                process.wait(timeout=5)


if __name__=='__main__':unittest.main()
