import copy
import datetime as dt
import json
from pathlib import Path
import signal
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import start_bulk10000_continuous as continuous


def progress(batches, stage='SESSION_RENEWAL_REQUIRED'):
    return dict(collection='/campaign', recorded_images=batches*200, planned_images=10000,
                completed_batches=batches, planned_batches=50, progress=dict(stage=stage))


class FakeEngine:
    def __init__(self, statuses, codes=None, closed=True, failure=None):
        self.statuses = iter(statuses)
        self.codes = iter(codes or [0]*50)
        self.closed, self.failure = closed, failure
        self.events = []

    def progress(self):
        return next(self.statuses)

    def open_window(self, index):
        self.events.append(('open', index))
        return index

    def collect(self, window):
        self.events.append(('collect', window))
        if self.failure:
            raise self.failure
        return next(self.codes)

    def close_window(self, window, reason):
        self.events.append(('close', window, reason))
        return self.closed


class CampaignRotation(unittest.TestCase):
    def test_renewal_finishes_old_window_before_starting_another(self):
        engine = FakeEngine([progress(48), progress(49), progress(50, 'FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW')])
        self.assertEqual(continuous.run_campaign(engine), 'complete')
        self.assertEqual(engine.events, [('open', 1), ('collect', 1), ('close', 1, 'renew'),
                                         ('open', 2), ('collect', 2), ('close', 2, 'complete')])

    def test_nonzero_collector_exit_never_retries(self):
        engine = FakeEngine([progress(9), progress(9, 'STOPPED_REVIEW_LOG')], [1])
        with self.assertRaisesRegex(RuntimeError, 'no automatic retry'):
            continuous.run_campaign(engine)
        self.assertEqual(engine.events[-1], ('close', 1, 'failure'))
        self.assertEqual(len(engine.events), 3)

    def test_display_loss_closes_monitoring_and_requires_reconnection_without_retry(self):
        engine = FakeEngine([progress(21), progress(21, 'DESKTOP_RECONNECT_REQUIRED')], [1])
        with self.assertRaisesRegex(RuntimeError, 'DESKTOP_RECONNECT_REQUIRED.*no automatic retry'):
            continuous.run_campaign(engine)
        self.assertEqual(engine.events, [('open', 1), ('collect', 1), ('close', 1, 'failure')])

    def test_keyboard_interrupt_keeps_monitors_until_close_and_never_renews(self):
        engine = FakeEngine([progress(9)], failure=KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            continuous.run_campaign(engine)
        self.assertEqual(engine.events[-1], ('close', 1, 'failure'))

    def test_missing_or_unclean_teardown_prevents_renewal(self):
        engine = FakeEngine([progress(9), progress(10)], closed=False)
        with self.assertRaisesRegex(RuntimeError, 'teardown failed'):
            continuous.run_campaign(engine)
        self.assertEqual(len(engine.events), 3)

    def test_no_progress_cannot_cause_an_endless_renewal_loop(self):
        engine = FakeEngine([progress(9), progress(9)])
        with self.assertRaisesRegex(RuntimeError, 'forward progress'):
            continuous.run_campaign(engine)
        self.assertEqual(engine.events[-1], ('close', 1, 'failure'))

    def test_unexpected_clean_exit_stops_for_review(self):
        engine = FakeEngine([progress(9), progress(10, 'BATCH_COMPLETE')])
        with self.assertRaisesRegex(RuntimeError, 'review required'):
            continuous.run_campaign(engine)

    def test_window_limit_is_finite_and_leaves_closed_monitors(self):
        engine = FakeEngine([progress(9), progress(10)])
        self.assertEqual(continuous.run_campaign(engine, 1), 'window_limit')
        self.assertEqual(engine.events[-1], ('close', 1, 'renew'))

    def test_already_complete_starts_no_monitors(self):
        engine = FakeEngine([progress(50, 'FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW')])
        self.assertEqual(continuous.run_campaign(engine), 'complete')
        self.assertEqual(engine.events, [])

    def test_full_count_without_campaign_completion_is_rejected(self):
        with self.assertRaises(RuntimeError):
            continuous.run_campaign(FakeEngine([progress(50)]))

    def test_count_identity_regression_or_inconsistent_receipts_are_rejected(self):
        variants = [progress(8), dict(progress(10), collection='/different'),
                    dict(progress(10), recorded_images=1999), dict(progress(10), recorded_images=True)]
        for after in variants:
            with self.subTest(after=after), self.assertRaises(RuntimeError):
                continuous.classify_completion(progress(9), after, 0)


class MonitorFreshness(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.case = Path(self.temporary.name)
        self.now = dt.datetime.now(dt.timezone.utc)
        self.state = dict(case=str(self.case), boot_id='boot', state='running', event='health',
            health=[dict(utc=(self.now-dt.timedelta(seconds=offset)).isoformat(), event='health', boot_id='boot')
                    for offset in [6, 1]])
        self.write_state()

    def tearDown(self):
        self.temporary.cleanup()

    def write_state(self):
        (self.case/'LATEST_CAPTURE_STATE.json').write_text(json.dumps(self.state))

    def test_two_fresh_same_boot_samples_are_required(self):
        self.assertEqual(continuous.healthy_state(self.case, 'boot', self.now), self.state)
        self.state['health'] = self.state['health'][:1]
        self.write_state()
        with self.assertRaises(RuntimeError):
            continuous.healthy_state(self.case, 'boot', self.now)

    def test_stale_future_wrong_boot_and_stopped_capture_fail_closed(self):
        for mutate in [lambda s: s.update(boot_id='other'), lambda s: s.update(state='stopped'),
                       lambda s: s['health'][-1].update(utc=(self.now-dt.timedelta(seconds=30)).isoformat()),
                       lambda s: s['health'][-1].update(utc=(self.now+dt.timedelta(seconds=1)).isoformat())]:
            old = copy.deepcopy(self.state)
            mutate(self.state)
            self.write_state()
            with self.assertRaises(RuntimeError):
                continuous.healthy_state(self.case, 'boot', self.now)
            self.state = old

    def test_native_incident_markers_are_never_automatically_cleared(self):
        for name in continuous.STOP_MARKERS:
            path = self.case/name
            path.write_text('incident')
            with self.assertRaisesRegex(RuntimeError, 'incident/stop'):
                continuous.healthy_state(self.case, 'boot', self.now)
            self.assertEqual(path.read_text(), 'incident')
            path.unlink()

    def test_cooldown_also_checks_temperature_and_health_freshness(self):
        engine = continuous.NativeEngine.__new__(continuous.NativeEngine)
        engine.boot = 'boot'
        window = dict(case=self.case, monitors=[Mock(poll=lambda: None)])
        row = dict(utc=(self.now-dt.timedelta(seconds=20)).isoformat(), boot_id='boot', stop_required=True)
        with patch.object(continuous, 'latest_temperature', return_value=row):
            with self.assertRaisesRegex(RuntimeError, 'Temperature monitor'):
                engine.checkpoint(window, allow_incident=True)

    def test_live_incident_does_not_prevent_logging_through_cooldown(self):
        engine = continuous.NativeEngine.__new__(continuous.NativeEngine)
        engine.boot = 'boot'
        window = dict(case=self.case, monitors=[Mock(poll=lambda: None)])
        row = dict(utc=self.now.isoformat(), boot_id='boot', stop_required=True)
        with patch.object(continuous, 'latest_temperature', return_value=row):
            engine.checkpoint(window, allow_incident=True)
            with self.assertRaises(RuntimeError):
                engine.checkpoint(window)


class ProtectedFinish(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.folder = Path(self.temporary.name)
        self.case = self.folder/'case'
        self.case.mkdir()
        self.events = []
        self.engine = continuous.NativeEngine.__new__(continuous.NativeEngine)
        self.engine.event = Mock()
        self.engine.pause = lambda seconds, *a, **k: self.events.append(('cooldown', seconds))
        self.engine.checkpoint = Mock()
        self.engine.active = True
        self.clean = True
        self.fail_journal = False

        def command(argv, protected=False):
            self.events.append(('command', argv[-1] if not protected else 'journal', protected))
            if protected and self.fail_journal:
                raise RuntimeError('journal failed')
            if protected:
                (self.case/'POST_TEARDOWN.json').write_text(json.dumps(dict(post_teardown_clean=self.clean)))
            if argv[-1] == 'finish':
                (self.case/'MANUAL_SESSION_FINAL.json').write_text(json.dumps(dict(post_teardown_clean=self.clean)))

        self.engine.command = command
        self.processes = []
        for name in ['capture', 'temperatures']:
            process = Mock()
            process.poll.return_value = None
            def stop(sig, name=name, process=process):
                self.events.append(('stop', name, sig))
                process.poll.return_value = 0
            process.send_signal.side_effect = stop
            self.processes.append(process)
        self.logs = [Mock(), Mock()]
        self.window = dict(index=1, label='label', kit=self.folder/'kit', folder=self.folder,
                           case=self.case, monitors=self.processes, logs=self.logs, monitor_gap=False)

    def tearDown(self):
        self.temporary.cleanup()

    def test_full_native_finish_precedes_stopping_either_monitor(self):
        self.assertTrue(self.engine.close_window(self.window, 'renew'))
        self.assertEqual(self.events, [('command', 'idle', False), ('cooldown', 60),
            ('command', 'journal', True), ('command', 'finish', False),
            ('stop', 'temperatures', signal.SIGINT), ('stop', 'capture', signal.SIGINT)])
        result = continuous.read(self.folder/'WINDOW_RESULT.json')
        self.assertTrue(result['clean_for_automatic_renewal'])
        self.assertTrue(result['monitors_kept_through_cooldown_and_journal'])
        self.assertIsNone(self.engine.active)

    def test_protected_incident_review_prevents_automatic_renewal(self):
        self.clean = False
        self.assertFalse(self.engine.close_window(self.window, 'failure'))
        self.assertFalse(continuous.read(self.folder/'WINDOW_RESULT.json')['clean_for_automatic_renewal'])
        self.assertEqual(self.events[-2][0], 'stop')

    def test_missing_final_journal_leaves_monitors_open_for_review(self):
        self.fail_journal = True
        self.assertFalse(self.engine.close_window(self.window, 'failure'))
        self.assertFalse(any(event[0] == 'stop' for event in self.events))
        self.assertTrue(continuous.read(self.folder/'WINDOW_RESULT.json')['monitoring_left_open'])
        for log in self.logs:
            log.close.assert_not_called()

    def test_lost_monitor_records_gap_and_refuses_promotion(self):
        self.processes[0].poll.return_value = 1
        self.engine.pause = Mock(side_effect=RuntimeError('monitor stopped'))
        self.assertFalse(self.engine.close_window(self.window, 'failure'))
        result = continuous.read(self.folder/'WINDOW_RESULT.json')
        self.assertTrue(result['monitoring_gap'])
        self.assertFalse(result['clean_for_automatic_renewal'])

    def test_alive_but_stale_monitor_also_records_a_gap(self):
        self.engine.pause = Mock(side_effect=RuntimeError('stale monitor'))
        self.engine.checkpoint = Mock(side_effect=RuntimeError('stale monitor'))
        self.assertFalse(self.engine.close_window(self.window, 'failure'))
        self.assertTrue(continuous.read(self.folder/'WINDOW_RESULT.json')['monitoring_gap'])


class QualifiedInputsAndPrivileges(unittest.TestCase):
    @unittest.skipUnless(sys.platform.startswith('linux') and (continuous.STAGE/'manage.py').is_file(),
                         'Qualified Halo system tools; generic CI has no operator kit')
    def test_native_tools_resolve_in_the_sanitized_user_environment(self):
        for command in ['modinfo', 'powerprofilesctl', 'fuser', 'docker', 'loginctl']:
            with self.subTest(command=command):
                self.assertIsNotNone(shutil.which(command, path=continuous.USER_PATH), command)

    def test_collector_uses_user_credentials_and_the_bound_campaign(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            engine = continuous.NativeEngine.__new__(continuous.NativeEngine)
            engine.interrupted = False
            engine.collection = folder/'campaign'
            engine.event = Mock()
            engine.as_user = Mock(return_value=dict(user=1001, group=1001, extra_groups=[1001, 992], env={}))
            child = Mock(returncode=0)
            child.poll.return_value = 0
            with patch.object(continuous.subprocess, 'Popen', return_value=child) as spawn:
                self.assertEqual(engine.collect(dict(index=1, case=folder/'case', folder=folder)), 0)
            self.assertEqual(spawn.call_args.kwargs['user'], 1001)
            self.assertEqual(spawn.call_args.kwargs['group'], 1001)
            self.assertEqual(spawn.call_args.args[0][-2:], ['--collection', str(engine.collection)])

    def test_cancel_before_collection_starts_no_subprocess(self):
        engine = continuous.NativeEngine.__new__(continuous.NativeEngine)
        engine.interrupted = True
        with patch.object(continuous.subprocess, 'Popen') as spawn, self.assertRaises(KeyboardInterrupt):
            engine.collect({})
        spawn.assert_not_called()

    def test_changed_controller_or_native_helper_blocks_release(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            native = root/'manage.py'
            native.write_text('original')
            (root/'CONTINUOUS_PINS.json').write_text(json.dumps(dict(version=1, native_window_seconds=7200,
                files={str(native.resolve()): continuous.sha(native)})))
            with patch.object(continuous, 'HERE', root):
                continuous.verify_release()
                native.write_text('changed')
                with self.assertRaisesRegex(RuntimeError, 'input changed'):
                    continuous.verify_release()

    def test_user_subprocesses_drop_uid_gid_groups_and_environment(self):
        engine = continuous.NativeEngine.__new__(continuous.NativeEngine)
        engine.account = SimpleNamespace(pw_uid=1001, pw_gid=1001, pw_name='ian', pw_dir='/home/ian')
        engine.environment = dict(HOME='/home/ian', PATH='/usr/bin:/bin')
        with patch.object(continuous.os, 'getgrouplist', return_value=[1001, 992], create=True):
            values = engine.as_user()
        self.assertEqual(values['user'], 1001)
        self.assertEqual(values['group'], 1001)
        self.assertEqual(values['extra_groups'], [1001, 992])
        self.assertNotIn('SUDO_UID', values['env'])

    def test_missing_desktop_does_not_select_a_fallback_renderer(self):
        engine = continuous.NativeEngine.__new__(continuous.NativeEngine)
        engine.account = SimpleNamespace(pw_uid=-99)
        with patch.object(continuous.Path, 'glob', return_value=[]):
            with self.assertRaisesRegex(RuntimeError, 'connected Halo graphical'):
                engine.desktop_environment()

    def test_window_check_runs_in_a_fresh_process_with_standard_native_guards(self):
        module = Mock()
        module.session.return_value = (Path('/kit'), Path('/case'), Mock(), Mock())
        with patch.dict('sys.modules', {'start_bulk10000_collection': module}), \
                patch.object(continuous, 'healthy_state') as healthy, \
                patch.object(continuous.Path, 'read_text', return_value='boot'):
            value = continuous.user_window_check()
        module.session.assert_called_once_with()
        healthy.assert_called_once_with(Path('/case'), 'boot')
        self.assertEqual(value['guard'], 'PASS')

    def test_two_cycle_qualification_never_calls_the_collector(self):
        engine = FakeEngine([])
        engine.pause = Mock()
        engine.event = Mock()
        continuous.run_qualification(engine)
        self.assertEqual(engine.events, [('open', 1), ('close', 1, 'monitor_rotation_qualification'),
                                         ('open', 2), ('close', 2, 'monitor_rotation_qualification')])
        engine.event.assert_called_once_with('MONITOR_ROTATION_QUALIFIED', windows=2, rendered_images=0)


if __name__ == '__main__':
    unittest.main()
