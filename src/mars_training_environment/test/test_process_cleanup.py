import copy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from mars_training_environment.process_cleanup import (CaptureProcessScope, verify_scene_renderer,
                                                      prepare_dataset_output, verify_runtime_qualification)


class RendererScopeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.log = Path(self.tmp.name)/'ogre.log'; self.log.write_text('current')
        self.scope = Mock(partition='mars_capture_owned')
        self.scope.processes.return_value = [dict(pid=123, start_ticks=77, uid=1001, state='S')]
        self.world = PurePosixPath('/collection/bundle/scenes/scene_1/world.sdf')
        self.value = dict(processes=[dict(pid=123, command='gz sim -r -s '+str(self.world), libraries=[
            dict(path='/lib/libgz-rendering8-ogre.so', sha256='f6e8842247198a2b34b25cfdb40a8f4bc4f156c2e90a54135a68698dc239a3e6'),
            dict(path='/lib/libgallium.so', sha256='driver')])],
            renderer_identities=['GL_RENDERER = Radeon 8060S Graphics (radeonsi)'])

    def verify(self, value=None, launch=0):
        return verify_scene_renderer(self.value if value is None else value, self.scope,
                                     self.world, launch, self.log)

    def test_current_scene_partition_pid_and_fresh_log_pass(self):
        result = self.verify(); self.assertEqual(result['renderer']['start_ticks'], 77)
        self.assertEqual(result['version'], 2)

    def test_old_scene_cannot_pass_even_with_qualified_plugin_and_gpu(self):
        value = copy.deepcopy(self.value); value['processes'][0]['command'] = 'gz sim /old/world.sdf'
        with self.assertRaisesRegex(RuntimeError, 'different scene'): self.verify(value)

    def test_foreign_partition_and_multiple_renderers_rejected(self):
        self.scope.processes.return_value = []
        with self.assertRaisesRegex(RuntimeError, 'outside'): self.verify()
        value = copy.deepcopy(self.value); value['processes'] *= 2
        with self.assertRaisesRegex(RuntimeError, 'exactly one'): self.verify(value)

    def test_old_log_and_changed_plugin_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'predates'): self.verify(launch=time.time()+5)
        value = copy.deepcopy(self.value); value['processes'][0]['libraries'][0]['sha256'] = 'wrong'
        with self.assertRaisesRegex(RuntimeError, 'Ogre mapping'): self.verify(value)


class EvidenceCompatibilityTests(unittest.TestCase):
    def test_qualified_runtime_hashes_and_cleanup_are_required_for_resume(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); output = root/'dataset'; proof = root/'renderer-provenance/run/launch-001'
            proof.mkdir(parents=True); attempt = output/'attempts/scene_1/attempt-0001'; attempt.mkdir(parents=True)
            observation = proof/'attempt-001.json'; observation.write_text('{}')
            sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
            world = str(root/'bundle/scenes/scene_1/world.sdf')
            passed = dict(version=2, world=world, partition='owned', attempt=1, observation_sha256=sha(observation))
            (proof/'PASS.json').write_text(json.dumps(passed))
            (proof/'CLEANUP.json').write_text(json.dumps(dict(remaining=[], partition='owned')))
            value = dict(version=2, world=world, scene_sha256='scene', proof=proof.relative_to(root).as_posix(),
                proof_sha256=sha(proof/'PASS.json'), cleanup_sha256=sha(proof/'CLEANUP.json'))
            (attempt/'RUNTIME_QUALIFICATION.json').write_text(json.dumps(value))
            report = dict(scene_id='scene_1', scene_sha256='scene', attempt=attempt.relative_to(output).as_posix())
            self.assertEqual(verify_runtime_qualification(root, output, report)['version'], 2)
            wrong = dict(report, scene_sha256='other')
            with self.assertRaisesRegex(ValueError, 'different scene'):
                verify_runtime_qualification(root, output, wrong)
            observation.write_text('{"tampered":true}')
            with self.assertRaisesRegex(ValueError, 'observation changed'):
                verify_runtime_qualification(root, output, report)

    def test_legacy_capture_is_preserved_and_not_reused_as_qualified(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); old = root/'dataset/metadata/train'; old.mkdir(parents=True)
            (old/'old.json').write_text('original')
            output = prepare_dataset_output(root)
            self.assertFalse((output/'metadata').exists())
            self.assertEqual((root/'dataset-before-process-cleanup-fix/metadata/train/old.json').read_text(), 'original')
            self.assertEqual(json.loads((root/'PROCESS_FIX_MIGRATION.json').read_text())['preserved_metadata_frames'], 1)
            self.assertEqual(prepare_dataset_output(root), output)

    def test_preserved_capture_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); (root/'dataset').mkdir(); (root/'dataset-before-process-cleanup-fix').mkdir()
            with self.assertRaisesRegex(ValueError, 'refusing to replace'): prepare_dataset_output(root)

    def test_missing_runtime_evidence_is_not_reusable(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaises(FileNotFoundError):
                verify_runtime_qualification(root, root/'dataset', dict(attempt='attempts/scene_1/a'))

    def test_runtime_evidence_cannot_escape_dataset(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaisesRegex(ValueError, 'escapes'):
                verify_runtime_qualification(root, root/'dataset', dict(attempt='../foreign'))


class StartupOwnershipTests(unittest.TestCase):
    def identity(self):
        return dict(pid=123, uid=1001, start_ticks=77, state='S')

    def test_transient_empty_environment_waits_for_same_process_identity(self):
        with patch('mars_training_environment.process_cleanup.process_identity', return_value=self.identity()), \
             patch('mars_training_environment.process_cleanup.partition_for', side_effect=[None, 'mars_capture_owned']), \
             patch('mars_training_environment.process_cleanup.os.getuid', return_value=1001, create=True), \
             patch('mars_training_environment.process_cleanup.time.sleep') as sleep:
            process = Mock(pid=123); process.poll.return_value = None
            self.assertEqual(CaptureProcessScope(process).partition, 'mars_capture_owned')
            sleep.assert_called_once_with(.05)

    def test_missing_ownership_still_fails_after_bounded_wait(self):
        with patch('mars_training_environment.process_cleanup.process_identity', return_value=self.identity()), \
             patch('mars_training_environment.process_cleanup.partition_for', return_value=None), \
             patch('mars_training_environment.process_cleanup.time.sleep') as sleep:
            process = Mock(pid=123); process.poll.return_value = None
            with self.assertRaisesRegex(RuntimeError, 'no unique owned'): CaptureProcessScope(process)
            self.assertEqual(sleep.call_count, 20)


@unittest.skipUnless(sys.platform == 'linux' and hasattr(os, 'pidfd_open'), 'Linux process ownership checks')
class LinuxCleanupTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name); self.children = []

    def tearDown(self):
        for child in self.children:
            if child.poll() is None: child.kill()
            child.wait(timeout=5)

    def spawn(self, code, partition):
        process = subprocess.Popen([sys.executable, '-c', code], start_new_session=True,
            env=dict(os.environ, GZ_PARTITION=partition), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(process); return process

    def detached(self, stubborn=False):
        marker = self.root/'child-ready'
        child = ('import signal,time,pathlib; '+
                 ('signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); ' if stubborn else '')+
                 f'pathlib.Path({str(marker)!r}).write_text("ready"); time.sleep(30)')
        parent = 'import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",'+repr(child)+'],start_new_session=True); time.sleep(.4)'
        process = self.spawn(parent, 'mars_capture_test_'+str(time.monotonic_ns()))
        scope = CaptureProcessScope(process)
        self.addCleanup(scope.cleanup, .3)
        deadline = time.monotonic()+5
        while not marker.exists():
            if time.monotonic()>deadline: self.fail('Detached child did not start')
            time.sleep(.01)
        return process, scope

    def test_detached_child_removed_after_leader_already_exited(self):
        parent, scope = self.detached(); parent.wait(timeout=5)
        self.assertTrue(scope.processes())
        result = scope.cleanup(stage_seconds=.3)
        self.assertFalse(result['remaining']); self.assertFalse(scope.processes())

    def test_stubborn_child_escalates_without_signalling_unrelated_process(self):
        other = self.spawn('import time; time.sleep(30)', 'mars_capture_unrelated_'+str(time.monotonic_ns()))
        parent, scope = self.detached(stubborn=True); parent.wait(timeout=5)
        result = scope.cleanup(stage_seconds=.15)
        self.assertTrue(any(a['signal']==signal.SIGKILL for a in result['actions']))
        self.assertIsNone(other.poll()); self.assertFalse(scope.processes())

    def test_survivors_make_cleanup_fail(self):
        parent, scope = self.detached(); parent.wait(timeout=5)
        try:
            with patch.object(scope, 'signal'):
                with self.assertRaisesRegex(RuntimeError, 'survived'): scope.cleanup(stage_seconds=0)
        finally:
            scope.cleanup(stage_seconds=.3)

    def test_pid_start_time_change_refuses_signal(self):
        parent, scope = self.detached(); parent.wait(timeout=5)
        identity = scope.processes()[0]; changed = dict(identity, start_ticks=identity['start_ticks']+1)
        try:
            with self.assertRaisesRegex(RuntimeError, 'identity changed'): scope.signal(changed, signal.SIGKILL)
        finally:
            scope.cleanup(stage_seconds=.3)


if __name__ == '__main__':
    unittest.main()
