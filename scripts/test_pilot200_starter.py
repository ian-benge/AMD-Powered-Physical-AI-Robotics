import json
from pathlib import Path
import signal
import subprocess
import unittest
from unittest.mock import Mock, patch

import start_pilot200_collection as starter


class CancellationTests(unittest.TestCase):
    def test_cancel_allows_guard_cleanup_before_return(self):
        child = Mock(); child.wait.side_effect = [KeyboardInterrupt, 0]; child.poll.return_value = 0
        with patch.object(starter.subprocess, 'Popen', return_value=child), patch.object(starter, 'stop_owned_container') as stop:
            self.assertEqual(starter.run_guarded(['guard'], 'own', 'image', Path('/candidate')), 130)
        child.send_signal.assert_called_once_with(signal.SIGINT)
        child.kill.assert_not_called()
        stop.assert_called_once_with('own', 'image', Path('/candidate'))

    def test_guard_timeout_stops_owned_container_and_reaps_guard(self):
        child = Mock(); child.wait.side_effect = [KeyboardInterrupt, subprocess.TimeoutExpired('guard', 45), 0]
        child.poll.return_value = None
        with patch.object(starter.subprocess, 'Popen', return_value=child), patch.object(starter, 'stop_owned_container') as stop:
            self.assertEqual(starter.run_guarded(['guard'], 'own', 'image', Path('/candidate')), 130)
        stop.assert_called_once(); child.kill.assert_called_once()

    def test_cleanup_refuses_different_image_or_mount(self):
        for image, source in [('other', '/candidate'), ('image', '/different')]:
            value = dict(Image=image, Mounts=[dict(Destination='/work', Source=source)], State=dict(Running=True), Id='id')
            inspected = Mock(returncode=0, stdout=json.dumps([value]))
            with patch.object(starter.subprocess, 'run', return_value=inspected) as run:
                with self.assertRaises(RuntimeError):
                    starter.stop_owned_container('own', 'image', Path('/candidate'))
                self.assertEqual(run.call_count, 1)

    def test_success_still_checks_for_leftover_owned_container(self):
        child = Mock(); child.wait.return_value = 0; child.poll.return_value = 0
        with patch.object(starter.subprocess, 'Popen', return_value=child), patch.object(starter, 'stop_owned_container') as stop:
            self.assertEqual(starter.run_guarded(['guard'], 'own', 'image', Path('/candidate')), 0)
        stop.assert_called_once(); child.kill.assert_not_called()


class DesktopAuthorityTests(unittest.TestCase):
    def setUp(self):
        guard = patch.object(starter, 'require_desktop')
        self.guard = guard.start()
        self.addCleanup(guard.stop)

    def process(self, display=':0'):
        value=Mock()
        value.read_bytes.return_value=('/usr/bin/Xwayland\0'+display+'\0-auth\0/run/user/1001/authority\0').encode()
        return value

    def test_on_demand_display_is_queried_before_process_discovery(self):
        probe=Mock(returncode=0)
        with patch.dict(starter.os.environ,{'DISPLAY':':0'},clear=True), \
                patch.object(starter.subprocess,'run',return_value=probe) as run, \
                patch.object(starter.subprocess,'check_output',return_value=b'0100abcd\n'), \
                patch.object(starter.Path,'glob') as discover:
            def after_query(_):
                self.assertTrue(run.called,'Xwayland discovery preceded the first X11 client')
                self.assertEqual(run.call_args.args[0],['xdpyinfo','-display',':0'])
                return [self.process()]
            discover.side_effect=after_query
            display,authority=starter.desktop_authority()
            try:
                self.assertEqual(display,':0')
                self.assertTrue(authority.is_file())
                self.assertEqual(run.call_count,2)
                self.assertEqual(run.call_args.args[0][-2:],['nmerge','-'])
            finally:authority.unlink()

    def test_unreachable_display_stops_before_authority_discovery(self):
        with patch.dict(starter.os.environ,{'DISPLAY':':0'},clear=True), \
                patch.object(starter.subprocess,'run',return_value=Mock(returncode=1)), \
                patch.object(starter.Path,'glob') as discover:
            with self.assertRaisesRegex(ValueError,'display is unavailable'):starter.desktop_authority()
            discover.assert_not_called()

    def test_bounded_query_failure_is_not_a_software_fallback(self):
        for error in [FileNotFoundError('xdpyinfo'),subprocess.TimeoutExpired('xdpyinfo',15)]:
            with self.subTest(error=error),patch.dict(starter.os.environ,{'DISPLAY':':0'},clear=True), \
                    patch.object(starter.subprocess,'run',side_effect=error), \
                    patch.object(starter.Path,'glob') as discover:
                with self.assertRaisesRegex(ValueError,'X11 query failed'):starter.desktop_authority()
                discover.assert_not_called()

    def test_missing_display_stops_before_query(self):
        with patch.dict(starter.os.environ,{},clear=True),patch.object(starter.subprocess,'run') as run:
            with self.assertRaisesRegex(ValueError,'graphical-desktop terminal'):starter.desktop_authority()
            run.assert_not_called()

    def test_ambiguous_or_wrong_display_still_stops(self):
        for processes in [[],[self.process(),self.process()],[self.process(':1')]]:
            with self.subTest(count=len(processes)),patch.dict(starter.os.environ,{'DISPLAY':':0'},clear=True), \
                    patch.object(starter.subprocess,'run',return_value=Mock(returncode=0)), \
                    patch.object(starter.Path,'glob',return_value=processes), \
                    patch.object(starter.subprocess,'check_output') as authority:
                with self.assertRaisesRegex(ValueError,'ambiguous'):starter.desktop_authority()
                authority.assert_not_called()

    def test_missing_cookie_still_stops(self):
        with patch.dict(starter.os.environ,{'DISPLAY':':0'},clear=True), \
                patch.object(starter.subprocess,'run',return_value=Mock(returncode=0)), \
                patch.object(starter.Path,'glob',return_value=[self.process()]), \
                patch.object(starter.subprocess,'check_output',return_value=b''):
            with self.assertRaisesRegex(ValueError,'authority is unavailable'):starter.desktop_authority()

    def test_reachable_x11_without_active_output_stops_before_cookie_discovery(self):
        from desktop_render_guard import DesktopUnavailable
        self.guard.side_effect = DesktopUnavailable('no active desktop output')
        with patch.dict(starter.os.environ, {'DISPLAY': ':0'}, clear=True), \
                patch.object(starter.subprocess, 'run', return_value=Mock(returncode=0)), \
                patch.object(starter.Path, 'glob') as discover:
            with self.assertRaisesRegex(DesktopUnavailable, 'no active desktop output'):
                starter.desktop_authority()
            discover.assert_not_called()


if __name__ == '__main__':
    unittest.main()
