import json
import datetime as dt
from pathlib import Path
import sys
import tempfile
from types import ModuleType
import unittest
from unittest.mock import patch,Mock

import bulk10000_plan as plan
import start_bulk10000_collection as starter
import bulk_guarded_step as guarded


class Reservations(unittest.TestCase):
    def test_fixed_split_counts_backgrounds_and_unique_scenes(self):
        value = plan.reservation('profile')
        self.assertEqual(value, plan.reservation('profile'))
        self.assertEqual(value['split_images'], dict(train=8000, val=1000, test=1000))
        scenes = [e for b in value['batches'] for e in b['scenes']]
        self.assertEqual(sum(e['empty'] for e in scenes), 100)
        self.assertEqual(len({e['appearance_family'] for e in scenes}), 500)
        for split in plan.SPLITS:
            chosen = [e for e in scenes if e['split'] == split]
            for key, levels in [('density', plan.DENSITIES), ('sunlight', plan.SUNLIGHT)]:
                counts = [sum(e[key] == level for e in chosen) for level in levels]
                # Sunlight occurs in blocks of three; density is interleaved.
                self.assertLessEqual(max(counts)-min(counts), 3)

    def test_existing_split_hash_contract(self):
        self.assertEqual([plan.split_for_seed(s) for s in [4312000,4312001,4312013,4312016]], ['train']*4)

    def test_changed_scene_identity_is_rejected(self):
        for key, value in [('seed', 1), ('views', 19), ('split', 'test'), ('appearance_family', 'shared')]:
            candidate = plan.reservation('profile')
            candidate['batches'][0]['scenes'][0][key] = value
            with self.assertRaises(ValueError):
                plan.validate_reservation(candidate)

    def test_configuration_retains_camera_and_annotation_policy(self):
        profile = Path(__file__).with_name('pilot200_environment_v2.json')
        if not profile.exists():
            profile = Path(__file__).parents[1]/'src/mars_training_environment/config/pilot200_environment_v2.json'
        source = json.loads(profile.read_text())
        original = plan.digest(source)
        b = plan.reservation('profile')['batches'][0]
        for entry in b['scenes']:
            config = plan.scene_configuration(source, b, entry)
            self.assertEqual(config['camera'], source['camera'])
            self.assertEqual(config['capture'], source['capture'])
            self.assertEqual(config['labels'], source['labels'])
            self.assertEqual(config['rocks']['held_out_ids'], source['rocks']['held_out_ids'])
            self.assertEqual(config['rocks']['count_range'], plan.DENSITIES[entry['density']])
            self.assertEqual(config['dataset']['seeds'], [e['seed'] for e in b['scenes']])
            self.assertEqual(config['dataset']['empty_seeds'], [e['seed'] for e in b['scenes'] if e['empty']])
        self.assertEqual(plan.digest(source), original)

    def test_changed_or_escaped_evidence_cannot_resume(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            (root/'proof').write_text('original')
            files = {'proof':plan.sha(root/'proof')}
            plan.verify_files(root, files)
            (root/'proof').write_text('changed')
            with self.assertRaises(ValueError):
                plan.verify_files(root, files)
            with self.assertRaises(ValueError):
                plan.verify_files(root, {'../foreign':'hash'})

    def test_missing_receipt_does_not_count_a_batch(self):
        batch = plan.reservation('profile')['batches'][0]
        with tempfile.TemporaryDirectory() as name:
            self.assertFalse(starter.verify_prior(Path(name),batch,'reservation',{}))

    def test_guard_failure_cannot_be_promoted_as_completed(self):
        batch = plan.reservation('profile')['batches'][0]
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            folder = root/'batches'/batch['id'];folder.mkdir(parents=True)
            (folder/'BATCH_RESULT.json').write_text('{}')
            guard = root/'guard.json'
            guard.write_text(json.dumps(dict(failure='monitor expired',exit_code=1)))
            receipts = root/'batch-receipts';receipts.mkdir()
            (receipts/(batch['id']+'.json')).write_text(json.dumps(dict(batch_id=batch['id'],
                split='train',frames=200,backgrounds=40,
                result_sha256=plan.sha(folder/'BATCH_RESULT.json'),
                guard_result='guard.json',guard_sha256=plan.sha(guard))))
            with patch.object(plan,'verify_batch') as verify:
                with self.assertRaisesRegex(ValueError,'guard did not pass'):
                    starter.verify_prior(root,batch,'reservation',{})
                verify.assert_not_called()

    def test_manifests_cannot_replace_completeness_checks(self):
        batch = plan.reservation('profile')['batches'][0]
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);(root/'dataset').mkdir()
            validation=dict(errors=[],frames=200,empty_frames=40,split_counts={'train':200},
                sensor_depth_bbox_check=dict(backend='sensor_depth_v1',frames_checked=200,frames_agreeing=200))
            (root/'dataset/validation.json').write_text(json.dumps(validation))
            (root/'BATCH_PINS.json').write_text(json.dumps(dict(files={
                'dataset/validation.json':plan.sha(root/'dataset/validation.json')})))
            (root/'BATCH_RESULT.json').write_text(json.dumps(dict(status='BULK_BATCH_COMPLETE_PENDING_REVIEW',
                batch_id=batch['id'],reservation_sha256='reservation',frames=200,background_frames=40,
                split='train',scene_ids=[e['id'] for e in batch['scenes']],pins_sha256=plan.sha(root/'BATCH_PINS.json'))))
            with self.assertRaises(FileNotFoundError):
                plan.verify_batch(root,batch,'reservation')


class SessionImports(unittest.TestCase):
    def test_kit_module_becomes_importable_only_after_verified_loader(self):
        with tempfile.TemporaryDirectory() as name:
            kit=Path(name)
            (kit/'remote_session.py').write_text('kit_marker = "verified operator kit"\n')
            trial=object();live_guard=object()
            fixture=ModuleType('phase1_run_fixture')
            fixture.live_guard=live_guard

            def load_guard(path):
                self.assertEqual(path,kit)
                self.assertNotIn(str(kit),sys.path)
                sys.path.insert(0,str(path))
                return trial

            fixture.load_guard=load_guard
            with patch.dict(sys.modules,{'phase1_run_fixture':fixture}), patch.object(sys,'path',sys.path[:]):
                sys.modules.pop('remote_session',None)
                actual, guard, remote=starter.load_session_modules(kit)
                self.assertIs(actual,trial)
                self.assertIs(guard,live_guard)
                self.assertEqual(remote.kit_marker,'verified operator kit')
                self.assertEqual(Path(remote.__file__).parent,kit)

    def test_rejected_kit_does_not_import_remote_session(self):
        fixture=ModuleType('phase1_run_fixture')
        fixture.live_guard=object()
        def reject(_):
            raise RuntimeError('Retained operator kit changed')
        fixture.load_guard=reject
        with patch.dict(sys.modules,{'phase1_run_fixture':fixture}), patch.object(sys,'path',sys.path[:]):
            sys.modules.pop('remote_session',None)
            with self.assertRaisesRegex(RuntimeError,'operator kit changed'):
                starter.load_session_modules(Path('/changed-kit'))
            self.assertNotIn('remote_session',sys.modules)


class CaptureRuntime(unittest.TestCase):
    def test_capture_extension_cannot_change_geometry_or_acceptance_code(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);control=root/'control';control.mkdir()
            app=root/'bulk10000-capture-recovery-20261106/application-v1';app.mkdir(parents=True)
            protected='source/src/mars_training_environment/mars_training_environment/annotation.py'
            file=app/protected;file.parent.mkdir(parents=True);file.write_text('unchanged')
            pins=dict(source_archive_sha256='base',image='image',files={protected:plan.sha(file)})
            runtime=dict(version=1,application=str(app),base_source_archive_sha256='base',
                         image='image',files=dict(pins['files']))
            manifest=control/'CAPTURE_RUNTIME.json';manifest.write_text(json.dumps(runtime))
            with patch.object(starter,'CONTROL',control),patch.object(starter.pilot,'BASE',root):
                self.assertEqual(starter.capture_application(pins),app)
                file.write_text('changed geometry')
                runtime['files'][protected]=plan.sha(file)
                manifest.write_text(json.dumps(runtime))
                with self.assertRaisesRegex(ValueError,'protected application file'):
                    starter.capture_application(pins)


class SessionRenewalAndProgress(unittest.TestCase):
    def write_capture(self,case,**override):
        event=dict(event='capture_started',utc='2026-11-07T16:00:00+00:00',duration_seconds=7200)
        event.update(override)
        (case/'capture-events.jsonl').write_text(json.dumps(event)+'\n')

    def test_native_duration_and_elapsed_time_are_used_without_changing_evidence(self):
        with tempfile.TemporaryDirectory() as name:
            case=Path(name);self.write_capture(case)
            path=case/'capture-events.jsonl';before=path.read_bytes()
            clock=dt.datetime(2026,11,7,17,30,tzinfo=dt.timezone.utc)
            self.assertEqual(starter.capture_time_remaining(case,clock),1800)
            self.assertEqual(path.read_bytes(),before)
            self.assertEqual(starter.capture_time_remaining(case,clock+dt.timedelta(hours=1)),0)

    def test_invalid_duration_or_future_capture_cannot_authorize_another_batch(self):
        with tempfile.TemporaryDirectory() as name:
            case=Path(name);clock=dt.datetime(2026,11,7,15,tzinfo=dt.timezone.utc)
            for duration in (0,7201,True,7200.):
                self.write_capture(case,duration_seconds=duration)
                with self.assertRaises(ValueError):starter.capture_time_remaining(case,clock)
            self.write_capture(case)
            with self.assertRaisesRegex(ValueError,'future'):starter.capture_time_remaining(case,clock)

    def test_renewal_is_requested_before_new_batch_without_extending_monitoring(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);self.write_capture(root)
            (root/'batch-receipts').mkdir()
            fresh=starter.session_budget(root,root,dt.datetime(2026,11,7,16,tzinfo=dt.timezone.utc))
            short=starter.session_budget(root,root,dt.datetime(2026,11,7,17,40,tzinfo=dt.timezone.utc))
            self.assertTrue(fresh['start_batch']);self.assertFalse(short['start_batch'])
            self.assertTrue(short['native_duration_limit_unchanged'])
            self.assertGreater(short['required_seconds'],short['remaining_seconds'])

    def test_recent_batch_duration_is_budgeted_with_headroom(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);self.write_capture(root)
            receipts=root/'batch-receipts';receipts.mkdir()
            result=root/'invocations/20261107T160000000000Z/guarded-step/result.json'
            result.parent.mkdir(parents=True)
            result.write_text(json.dumps(dict(utc='2026-11-07T16:40:00+00:00')))
            (receipts/'batch-001.json').write_text(json.dumps(dict(guard_result=result.relative_to(root).as_posix())))
            budget=starter.session_budget(root,root,dt.datetime(2026,11,7,17,10,tzinfo=dt.timezone.utc))
            self.assertFalse(budget['start_batch'])
            self.assertGreater(budget['required_seconds'],2400)

    def test_status_reports_partial_batch_without_counting_it_as_completed(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);reservation=plan.reservation('profile')
            (root/'SCENE_RESERVATIONS.json').write_text(json.dumps(reservation))
            (root/'PROGRESS.json').write_text(json.dumps(dict(stage='RUNNING',batch='batch-007')))
            receipts=root/'batch-receipts';receipts.mkdir()
            (receipts/'batch-001.json').write_text(json.dumps(dict(split='train',frames=200)))
            dataset=root/'batches/batch-007/dataset'
            meta=dataset/'metadata/train';meta.mkdir(parents=True)
            for n in range(3):(meta/f'frame{n}.json').write_text('{}')
            reports=dataset/'reports';reports.mkdir();(reports/'scene_1.json').write_text('{}')
            attempt=dataset/'attempts/scene_2/attempt-0001';attempt.mkdir(parents=True)
            (attempt/'launch.log').write_text('active')
            active_meta=attempt/'metadata/train';active_meta.mkdir(parents=True)
            (active_meta/'view_000.json').write_text('{}')
            result=starter.status(root)
            self.assertEqual(result['recorded_images'],200)
            self.assertEqual(result['completed_batches'],1)
            self.assertEqual(result['current_batch']['committed_images'],3)
            self.assertEqual(result['current_batch']['attempt_saved_views'],1)
            self.assertEqual(result['current_batch']['scene'],'scene_2')


class InventoryRace(unittest.TestCase):
    missing="['loginctl', 'show-session', '296', '-p', 'Name']: Failed to get path for session '296': No session '296' known\n"

    def test_disappeared_session_repeats_the_complete_guard_and_records_retry(self):
        check=Mock(side_effect=[AssertionError(self.missing),{'full_guard':'passed'}]);notice=Mock()
        with patch.object(guarded.time,'sleep'):
            result=guarded.retry_session_scan(check,notice)
        self.assertEqual(result,{'full_guard':'passed'})
        self.assertEqual(check.call_count,2);notice.assert_called_once()

    def test_graphical_session_loss_after_rescan_still_stops(self):
        check=Mock(side_effect=[AssertionError(self.missing),AssertionError('Exactly one active graphical desktop is required')])
        with patch.object(guarded.time,'sleep'):
            with self.assertRaisesRegex(AssertionError,'graphical desktop'):
                guarded.retry_session_scan(check)
        self.assertEqual(check.call_count,2)

    def test_repeated_inventory_race_has_a_finite_limit(self):
        check=Mock(side_effect=AssertionError(self.missing))
        with patch.object(guarded.time,'sleep'):
            with self.assertRaises(AssertionError):guarded.retry_session_scan(check)
        self.assertEqual(check.call_count,3)

    def test_other_failures_and_nonmatching_session_ids_are_never_retried(self):
        messages=['Visible temperature monitor is stale','Missing fan RPM','Graphical session changed during this case',
                  'Connect the RDP viewer before starting workloads',
                  self.missing.replace("No session '296'","No session '297'"),
                  self.missing.replace('No session \'296\' known','Access denied')]
        for message in messages:
            check=Mock(side_effect=AssertionError(message))
            with self.assertRaises(AssertionError):guarded.retry_session_scan(check)
            self.assertEqual(check.call_count,1)
        check=Mock(side_effect=RuntimeError(self.missing))
        with self.assertRaises(RuntimeError):guarded.retry_session_scan(check)
        self.assertEqual(check.call_count,1)

    def test_driver_cleanup_refuses_foreign_image_or_mount(self):
        app=Path('/candidate').resolve()
        for image,mount in [('foreign',str(app)),('image',str(app/'other'))]:
            inspected=Mock(returncode=0,stdout=json.dumps([dict(Image=image,
                Mounts=[dict(Destination='/work',Source=mount)],Id='id')]))
            with patch.object(guarded.subprocess,'run',return_value=inspected) as run:
                with self.assertRaisesRegex(RuntimeError,'different container identity'):
                    guarded.stop_owned_container(dict(container='own',image='image'),app)
                self.assertEqual(run.call_count,1)

    def test_driver_startup_failure_keeps_a_result_and_starts_no_workload(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);config=root/'config.json'
            config.write_text(json.dumps(dict(application=str(root/'app'),output=str(root/'out'),
                kit=str(root/'kit'),case=str(root/'case'),container='own',image='image',
                timeout_seconds=30,command=['workload'])))
            fixture=ModuleType('phase1_run_fixture');trial=Mock()
            trial.guard.side_effect=AssertionError('Wrong boot');trial.utc.return_value='utc'
            fixture.load_guard=Mock(return_value=trial);fixture.live_guard=Mock()
            with patch.dict(sys.modules,{'phase1_run_fixture':fixture}),patch.object(sys,'argv',['guard',str(config)]),\
                 patch.object(guarded.signal,'signal'),patch.object(guarded.subprocess,'Popen') as launch:
                self.assertEqual(guarded.main(),1)
                launch.assert_not_called()
            result=json.loads((root/'out/result.json').read_text())
            self.assertEqual(result['failure'],'Wrong boot');self.assertIsNone(result['exit_code'])


class CachedSessionBudget(unittest.TestCase):
    def test_verified_partial_batch_uses_remaining_work_and_changed_cache_stops(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);reservation=plan.reservation('profile');batch=reservation['batches'][0]
            (root/'SCENE_RESERVATIONS.json').write_text(json.dumps(reservation))
            SessionRenewalAndProgress().write_capture(root)
            (root/'batch-receipts').mkdir()
            folder=root/'batches'/batch['id'];bundle=folder/'bundle';bundle.mkdir(parents=True)
            entries=[dict(id=e['id'],content_sha256='scene-'+e['id'],views=20) for e in batch['scenes']]
            (bundle/'plan.json').write_text(json.dumps(dict(scenes=entries)))
            (folder/'BUNDLE_PINS.json').write_text(json.dumps(dict(batch=batch,
                reservation_sha256=plan.digest(reservation),files={'plan.json':plan.sha(bundle/'plan.json')})))
            dataset=folder/'dataset';reports=dataset/'reports';reports.mkdir(parents=True)
            artifact=dataset/'checked-metadata';artifact.write_text('verified files')
            for entry in entries[:7]:
                (reports/(entry['id']+'.json')).write_text(json.dumps(dict(scene_sha256=entry['content_sha256'],
                    frames=20,records=[f'frame-{i}' for i in range(20)],
                    evidence_sha256={'checked-metadata':plan.sha(artifact)})))
            clock=dt.datetime(2026,11,7,17,40,tzinfo=dt.timezone.utc)
            full=starter.session_budget(root,root,clock)
            cached=starter.session_budget(root,root,clock,batch=batch)
            self.assertFalse(full['start_batch']);self.assertTrue(cached['start_batch'])
            self.assertEqual(cached['remaining_scenes'],3);self.assertTrue(cached['generation_reused'])
            artifact.write_text('changed')
            with self.assertRaises(ValueError):starter.session_budget(root,root,clock,batch=batch)


if __name__ == '__main__':
    unittest.main()
