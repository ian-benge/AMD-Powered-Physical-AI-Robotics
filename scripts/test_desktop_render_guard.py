from contextlib import contextmanager
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import desktop_render_guard as guard


class RandRProbe(unittest.TestCase):
    def libraries(self, count=1, version=(1, 6), connected=True):
        x11, randr = Mock(), Mock()
        x11.XOpenDisplay.return_value = 123 if connected else None
        x11.XDefaultRootWindow.return_value = 456

        def query(_, major, minor):
            major._obj.value, minor._obj.value = version
            return 1

        def monitors(_, root, active, number):
            self.assertEqual((root, active), (456, 1))
            number._obj.value = count
            return 789 if count else None

        randr.XRRQueryVersion.side_effect = query
        randr.XRRGetMonitors.side_effect = monitors
        return x11, randr

    def test_active_query_frees_result_and_closes_connection(self):
        x11, randr = self.libraries()
        with patch.dict(guard.os.environ, {'DISPLAY': ':0'}), patch.object(guard.ctypes, 'CDLL', side_effect=[x11, randr]):
            self.assertEqual(guard.query_active_output()['active_monitors'], 1)
        randr.XRRFreeMonitors.assert_called_once_with(789)
        x11.XCloseDisplay.assert_called_once_with(123)

    def test_reachable_x11_with_zero_monitors_fails_closed(self):
        x11, randr = self.libraries(count=0)
        with patch.dict(guard.os.environ, {'DISPLAY': ':0'}), patch.object(guard.ctypes, 'CDLL', side_effect=[x11, randr]):
            with self.assertRaisesRegex(guard.DesktopUnavailable, 'no active desktop output'):
                guard.query_active_output()
        x11.XCloseDisplay.assert_called_once()

    def test_old_randr_or_absent_connection_is_rejected(self):
        for options in [dict(version=(1, 4)), dict(connected=False)]:
            x11, randr = self.libraries(**options)
            with self.subTest(options=options), patch.dict(guard.os.environ, {'DISPLAY': ':0'}), \
                    patch.object(guard.ctypes, 'CDLL', side_effect=[x11, randr]):
                with self.assertRaises(guard.DesktopUnavailable):
                    guard.query_active_output()
                randr.XRRGetMonitors.assert_not_called()


class BoundedProbe(unittest.TestCase):
    def test_success_is_bound_to_the_current_display(self):
        value = dict(version=1, display=':0', active_monitors=1, randr=[1, 6])
        with patch.dict(guard.os.environ, {'DISPLAY': ':0'}), \
                patch.object(guard.subprocess, 'run', return_value=Mock(returncode=0, stdout=json.dumps(value))) as run:
            self.assertEqual(guard.active_output(), value)
        self.assertEqual(run.call_args.kwargs['timeout'], 5)
        self.assertEqual(run.call_args.args[0][-1], '--probe')

    def test_failure_timeout_crash_and_incomplete_evidence_stop(self):
        good = dict(version=1, display=':0', active_monitors=1, randr=[1, 6])
        failures = [Mock(returncode=1, stdout='{"error":"no active desktop output"}'),
            Mock(returncode=-11, stdout=''), Mock(returncode=0, stdout='[]'),
            Mock(returncode=0, stdout=json.dumps(dict(good, display=':1'))),
            Mock(returncode=0, stdout=json.dumps(dict(good, active_monitors=0))),
            Mock(returncode=0, stdout=json.dumps(dict(good, active_monitors=True))),
            Mock(returncode=0, stdout=json.dumps(dict(good, randr=[])))]
        for result in failures:
            with self.subTest(result=result), patch.dict(guard.os.environ, {'DISPLAY': ':0'}), \
                    patch.object(guard.subprocess, 'run', return_value=result):
                with self.assertRaises(guard.DesktopUnavailable): guard.active_output()
        for error in [FileNotFoundError(), subprocess.TimeoutExpired('query', 5)]:
            with patch.object(guard.subprocess, 'run', side_effect=error):
                with self.assertRaises(guard.DesktopUnavailable): guard.active_output()

    def test_stop_evidence_is_immutable_and_scoped_to_invocation(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); invocation = root/'invocation.json'
            invocation.write_text('{"utc":"first"}')
            first = guard.stop_path(root, invocation)
            with patch.object(guard, 'active_output', side_effect=guard.DesktopUnavailable('zero outputs')):
                with self.assertRaises(guard.DesktopUnavailable): guard.require_desktop(first)
                saved = first.read_bytes()
                with self.assertRaises(guard.DesktopUnavailable): guard.require_desktop(first)
            self.assertEqual(saved, first.read_bytes())
            invocation.write_text('{"utc":"second"}')
            self.assertNotEqual(first, guard.stop_path(root, invocation))
            self.assertEqual(json.loads(saved)['stage'], 'DESKTOP_RECONNECT_REQUIRED')


class DesktopLifetime(unittest.TestCase):
    def test_output_loss_interrupts_only_the_held_launch_and_propagates(self):
        signaled = threading.Event()
        process = Mock(); process.poll.return_value = None
        process.send_signal.side_effect = lambda _: signaled.set()
        with patch.object(guard, 'require_desktop', side_effect=guard.DesktopUnavailable('lost output')):
            with self.assertRaisesRegex(guard.DesktopUnavailable, 'lost output'):
                with guard.watch_desktop(process, Path('/unused'), interval=.001):
                    self.assertTrue(signaled.wait(2))
        process.send_signal.assert_called_once_with(signal.SIGINT)
        process.kill.assert_not_called()

    def test_finished_launch_has_no_spurious_output_failure(self):
        process = Mock(); process.poll.return_value = 0
        with patch.object(guard, 'require_desktop') as probe:
            with guard.watch_desktop(process, None, interval=.001):
                threading.Event().wait(.01)
        probe.assert_not_called(); process.send_signal.assert_not_called()

    def test_prelaunch_loss_preserves_one_attempt_and_starts_no_renderer(self):
        try:
            import process_cleanup
        except ModuleNotFoundError:
            from mars_training_environment import process_cleanup
        # Deployed workers receive a pinned standalone copy of this same module.
        previous = sys.modules.get('process_cleanup')
        sys.modules['process_cleanup'] = process_cleanup
        try:
            import pilot200_capture_worker as worker
        finally:
            if previous is None: sys.modules.pop('process_cleanup', None)
            else: sys.modules['process_cleanup'] = previous
        from mars_training_environment.capture_attempts import capture_scene
        with tempfile.TemporaryDirectory() as name:
            root = Path(name); invocation = root/'invocation.json'
            invocation.write_text('{"utc":"2026-11-08T17:00:00+00:00"}')
            original = Mock()
            process = worker.qualified_process(original, root, invocation)
            with patch.object(worker, 'require_desktop', side_effect=guard.DesktopUnavailable('lost output')):
                with self.assertRaisesRegex(RuntimeError, 'Fatal renderer failure'):
                    capture_scene(root/'bundle', root/'dataset', dict(id='scene_1', views=20),
                        dict(config=dict(capture=dict(timeout_wall_sec=1))), True, process)
            original.assert_not_called()
            attempts = list((root/'dataset/attempts/scene_1').iterdir())
            self.assertEqual(len(attempts), 1)
            self.assertEqual(json.loads((attempts[0]/'attempt-result.json').read_text())['result'], 'FAILED_PRESERVED')


if __name__ == '__main__':
    unittest.main()
