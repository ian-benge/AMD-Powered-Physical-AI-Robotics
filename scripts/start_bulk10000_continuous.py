#!/usr/bin/env python3
"""Rotate native monitored sessions until the reserved image campaign completes.

The supervisor requires one local sudo launch. Capture/journal finalization and
module checks use their existing protected helpers. Collection, preparation and
temperature monitoring run as the desktop user. No sudo rule, service, hardware
limit, renderer, dataset gate or native kit is changed.
"""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import tempfile
import time

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')
STAGE = Path('/home/ian/capstone-training/rover-sim')
TOOLS = Path('/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103')
CONTROL = BASE/'bulk10000-collection-20261106'
HERE = Path(__file__).resolve().parent
PYTHON = '/usr/bin/python3'
USER_PATH = '/usr/sbin:/usr/bin:/sbin:/bin'
STOP_MARKERS = ('INCIDENT_DETECTED.txt', 'CONTROL_STOP_REQUIRED.txt')


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    path = Path(path)
    if path.is_symlink() or not stat.S_ISREG(path.stat().st_mode):
        raise RuntimeError('Expected regular pinned file: '+str(path))
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def atomic(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def verify_release():
    manifest = read(HERE/'CONTINUOUS_PINS.json')
    if manifest.get('version') != 1 or manifest.get('native_window_seconds') != 7200:
        raise RuntimeError('Continuous release contract changed')
    for name, expected in manifest['files'].items():
        path = Path(name)
        if not path.is_absolute():
            path = HERE/path
        if sha(path) != expected:
            raise RuntimeError('Continuous release input changed: '+name)
    return manifest


def checked_progress(value):
    frames, batches = value.get('recorded_images'), value.get('completed_batches')
    if (type(frames) is not int or type(batches) is not int or
            not 0 <= frames <= 10000 or frames != batches*200 or
            value.get('planned_images') != 10000 or value.get('planned_batches') != 50):
        raise RuntimeError('Campaign progress is inconsistent')
    return frames


def classify_completion(before, after, code):
    """Only clean monitoring-budget pauses may automatically start a new window."""
    previous, current = checked_progress(before), checked_progress(after)
    if after.get('collection') != before.get('collection') or current < previous:
        raise RuntimeError('Campaign identity/progress changed')
    if code != 0:
        if after.get('progress', {}).get('stage') == 'DESKTOP_RECONNECT_REQUIRED':
            raise RuntimeError('DESKTOP_RECONNECT_REQUIRED: reconnect the Halo graphical desktop, then resume; '
                               'outputs retained; no automatic retry')
        raise RuntimeError('Collector failed or was interrupted; outputs retained; no automatic retry')
    stage = after.get('progress', {}).get('stage')
    if stage == 'FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW' and current == 10000:
        return 'complete'
    if stage == 'SESSION_RENEWAL_REQUIRED' and current > previous:
        return 'renew'
    raise RuntimeError('Collector stopped without clean completion or forward progress; review required')


def run_campaign(engine, max_windows=50):
    before = engine.progress()
    if checked_progress(before) == 10000:
        if before.get('progress', {}).get('stage') != 'FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW':
            raise RuntimeError('Full count lacks campaign completion evidence')
        return 'complete'
    for index in range(1, max_windows+1):
        window = engine.open_window(index)
        decision = None
        error = None
        try:
            code = engine.collect(window)
            after = engine.progress()
            decision = classify_completion(before, after, code)
        except BaseException as failure:
            error = failure
        # Finish/cooldown/final journal precede stopping monitors, even on failure.
        closed = engine.close_window(window, reason=decision or 'failure')
        if error is not None:
            raise error
        if not closed:
            raise RuntimeError('Monitoring or protected teardown failed; automatic renewal refused')
        if decision == 'complete':
            return 'complete'
        before = after
    return 'window_limit'


def latest_temperature(case):
    path = case/'temperature-live.jsonl'
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size-65536))
        lines = stream.read().splitlines()
    if not lines:
        raise RuntimeError('Temperature monitor has no sample')
    return json.loads(lines[-1])


def healthy_state(case, boot, now=None):
    if any((case/name).exists() for name in STOP_MARKERS):
        raise RuntimeError('Native capture recorded an incident/stop marker')
    value = read(case/'LATEST_CAPTURE_STATE.json')
    rows = value.get('health', [])
    if (value.get('case') != str(case) or value.get('boot_id') != boot or
            value.get('state') != 'running' or value.get('event') != 'health' or len(rows) < 2):
        raise RuntimeError('Two fresh native capture health samples are required')
    clock = now or dt.datetime.now(dt.timezone.utc)
    stamps = [dt.datetime.fromisoformat(row['utc']) for row in rows[-2:]]
    if (any(row.get('boot_id') != boot or row.get('event') != 'health' for row in rows[-2:]) or
            any(stamp.tzinfo is None for stamp in stamps) or stamps[0] >= stamps[1] or
            not 0 <= (clock-stamps[-1]).total_seconds() < 20):
        raise RuntimeError('Native capture health is stale or belongs to another boot')
    return value


def verify_completed_campaign(collection):
    sys.path.insert(0, str(CONTROL))
    import start_bulk10000_collection as starter
    reservation = read(collection/'SCENE_RESERVATIONS.json')
    identity = read(collection/'COLLECTION_IDENTITY.json')
    if starter.policy.digest(reservation) != identity['reservation_sha256']:
        raise RuntimeError('Completed campaign reservation changed')
    starter.policy.validate_reservation(reservation)
    seen = {}
    for batch in reservation['batches']:
        if not starter.verify_prior(collection, batch, identity['reservation_sha256'], seen):
            raise RuntimeError('Completed campaign has a missing batch')
    print('ALL_10000_COMPLETED_OUTPUTS_VERIFIED')


def native_action(action):
    """Use the same narrow login-session rescan around native setup checks."""
    import importlib.util
    sys.path.insert(0, str(CONTROL))
    from bulk_guarded_step import retry_session_scan
    spec = importlib.util.spec_from_file_location('continuous_native_manage', STAGE/'manage.py')
    manager = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(manager)
    selected, kit, case = manager.selected()
    if action == 'modules':
        if os.geteuid() != 0:
            raise RuntimeError('Native module setup requires the protected supervisor')
        retry_session_scan(lambda: manager.modules(selected, kit, case))
    else:
        retry_session_scan(lambda: manager.acknowledge(kit, case))


def user_window_check():
    """Fresh process per kit avoids carrying old session modules into a renewal."""
    sys.path.insert(0, str(CONTROL))
    import start_bulk10000_collection as starter
    kit, case, trial, live_guard = starter.session()
    healthy_state(case, Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    return dict(kit=str(kit), case=str(case), guard='PASS')


class NativeEngine:
    def __init__(self, manifest, collection, output, account):
        self.manifest, self.collection, self.output = manifest, collection, output
        self.account = account
        self.boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.interrupted = False
        self.active = None
        self.environment = self.desktop_environment()
        self.identity = {name: sha(collection/name)
                         for name in ['COLLECTION_IDENTITY.json', 'SCENE_RESERVATIONS.json']}

    def event(self, stage, **values):
        value = dict(utc=utc(), stage=stage, collection=str(self.collection), run=str(self.output), **values)
        with (self.output/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(value)+'\n')
            stream.flush()
            os.fsync(stream.fileno())
        atomic(self.output/'STATUS.json', value)
        atomic(HERE/'CONTINUOUS_STATUS.json', value)
        print(stage, json.dumps(values), flush=True)

    def desktop_environment(self):
        environments = []
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                if proc.stat().st_uid != self.account.pw_uid:
                    continue
                argv = (proc/'cmdline').read_bytes().split(b'\0')
                if not argv or Path(os.fsdecode(argv[0])).name != 'gnome-terminal-server':
                    continue
                data = dict(row.split(b'=', 1) for row in (proc/'environ').read_bytes().split(b'\0') if b'=' in row)
                environments.append({os.fsdecode(k): os.fsdecode(v) for k, v in data.items()
                    if k in [b'DISPLAY', b'XAUTHORITY', b'XDG_RUNTIME_DIR', b'WAYLAND_DISPLAY']})
            except (OSError, ValueError):
                continue
        if len(environments) != 1 or not environments[0].get('DISPLAY'):
            raise RuntimeError('Use the single connected Halo graphical desktop terminal')
        environment = dict(PATH=USER_PATH, HOME=self.account.pw_dir,
                           USER=self.account.pw_name, LOGNAME=self.account.pw_name,
                           LANG='C.UTF-8', PYTHONDONTWRITEBYTECODE='1', **environments[0])
        authority = Path(environment['XAUTHORITY'])
        if (authority.is_symlink() or authority.stat().st_uid != self.account.pw_uid or
                authority.parent != Path('/run/user')/str(self.account.pw_uid)):
            raise RuntimeError('Desktop Xauthority identity is ambiguous')
        return environment

    def as_user(self):
        return dict(user=self.account.pw_uid, group=self.account.pw_gid,
                    extra_groups=os.getgrouplist(self.account.pw_name, self.account.pw_gid),
                    env=self.environment, cwd=self.account.pw_dir)

    def command(self, arguments, protected=False, timeout=180):
        verify_release()
        if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != self.boot:
            raise RuntimeError('Boot changed; automatic continuation refused')
        result = subprocess.run(arguments, capture_output=True, text=True, timeout=timeout,
                                **({} if protected else self.as_user()))
        with (self.output/'commands.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(utc=utc(), command=arguments, protected=protected,
                returncode=result.returncode, stdout=result.stdout, stderr=result.stderr))+'\n')
            stream.flush()
            os.fsync(stream.fileno())
        if result.returncode:
            raise RuntimeError('Native command failed: '+repr(arguments)+'\n'+result.stdout+result.stderr)
        return result.stdout

    def progress(self):
        for name, expected in self.identity.items():
            if sha(self.collection/name) != expected:
                raise RuntimeError('Campaign reservation/identity changed')
        return json.loads(self.command([PYTHON, '-B', str(CONTROL/'start_bulk10000_collection.py'),
                                       '--status', '--collection', str(self.collection)]))

    def checkpoint(self, window, allow_incident=False):
        if any(process.poll() is not None for process in window['monitors']):
            raise RuntimeError('A monitoring process stopped before protected finish')
        if allow_incident:
            state = read(window['case']/'LATEST_CAPTURE_STATE.json')
            age = (dt.datetime.now(dt.timezone.utc)-dt.datetime.fromisoformat(state['health'][-1]['utc'])).total_seconds()
            if state.get('boot_id') != self.boot or not 0 <= age < 20:
                raise RuntimeError('Evidence monitoring became stale during cooldown')
        else:
            healthy_state(window['case'], self.boot)
        temperature = latest_temperature(window['case'])
        age = (dt.datetime.now(dt.timezone.utc)-dt.datetime.fromisoformat(temperature['utc'])).total_seconds()
        if (temperature.get('boot_id') != self.boot or not 0 <= age < 10 or
                (not allow_incident and temperature.get('stop_required'))):
            raise RuntimeError('Temperature monitor is stale or requires a stop')

    def pause(self, seconds, window=None, allow_incident=False):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if window:
                self.checkpoint(window, allow_incident)
                self.display_temperatures(window)
            time.sleep(min(1., max(0., deadline-time.monotonic())))

    def display_temperatures(self, window):
        if time.monotonic()-window.get('last_display', 0.) < 2:
            return
        row = latest_temperature(window['case'])
        values = {name: f"{value['value']:.1f} {value['unit']}" for name, value in row['sensors'].items()
                  if 'value' in value and value.get('unit') in ('C', 'RPM')}
        print('TEMPERATURES', row['utc'], json.dumps(values), flush=True)
        window['last_display'] = time.monotonic()

    def open_window(self, index):
        if self.interrupted:
            raise KeyboardInterrupt
        self.event('STARTING_MONITORING', window=index)
        self.command([PYTHON, '-I', str(STAGE/'manage.py'), 'prepare'])
        selected = read(STAGE/'CURRENT.json')
        kit = Path(selected['kit'])
        if (kit.parent != STAGE/'runs' or kit.name != selected['label'] or kit.is_symlink() or
                selected['boot_id'] != self.boot):
            raise RuntimeError('New native kit identity is invalid')
        for name, expected in read(kit/'KIT_MANIFEST.json')['files'].items():
            path = kit/name
            if not path.resolve().is_relative_to(kit) or sha(path) != expected:
                raise RuntimeError('New native kit pin changed: '+name)
        folder = self.output/f'window-{index:03d}'
        folder.mkdir()
        window = dict(index=index, kit=kit, label=selected['label'], folder=folder,
                      monitors=[], logs=[], monitor_gap=False)
        self.active = window
        try:
            capture_log = (folder/'capture.log').open('xb')
            window['logs'].append(capture_log)
            capture = subprocess.Popen([PYTHON, '-u', '-I', str(kit/'capture_rocm_retest.py'),
                '--label', selected['label'], '--seconds', '7200'], stdout=capture_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            window['monitors'].append(capture)
            pointer = TOOLS/('capture-'+selected['label']+'.path')
            deadline = time.monotonic()+180
            while True:
                if self.interrupted:
                    raise KeyboardInterrupt
                if capture.poll() is not None:
                    raise RuntimeError('Native capture preflight failed; see '+str(folder/'capture.log'))
                if pointer.exists():
                    case = Path(pointer.read_text().strip())
                    if (case.parent != BASE or case.is_symlink() or
                            not case.name.startswith('oem617-7141-'+selected['label']+'-')):
                        raise RuntimeError('New capture case path is invalid')
                    window['case'] = case
                    if (case/'LATEST_CAPTURE_STATE.json').exists():
                        state = read(case/'LATEST_CAPTURE_STATE.json')
                        if any((case/name).exists() for name in STOP_MARKERS) or state.get('state') == 'stopped':
                            raise RuntimeError('Native capture stopped or recorded an incident during startup')
                        if len(state.get('health', [])) >= 2:
                            healthy_state(case, self.boot)
                            break
                if time.monotonic() >= deadline:
                    raise RuntimeError('Native capture did not produce two health events in time')
                time.sleep(.5)
            self.command([PYTHON, '-B', str(HERE/'start_bulk10000_continuous.py'), '--native-action', 'acknowledge'])
            # Reuse the native reviewed module loader, including its fan checks.
            self.command([PYTHON, '-B', str(HERE/'start_bulk10000_continuous.py'), '--native-action', 'modules'], protected=True)
            temperature_log = (folder/'temperature-display.log').open('xb')
            window['logs'].append(temperature_log)
            temperatures = subprocess.Popen([PYTHON, '-u', '-I', str(kit/'temperature_live.py'), selected['label']],
                stdout=temperature_log, stderr=subprocess.STDOUT, start_new_session=True, **self.as_user())
            window['monitors'].append(temperatures)
            deadline = time.monotonic()+20
            while not (case/'temperature-live.jsonl').exists() or not (case/'temperature-live.jsonl').stat().st_size:
                if temperatures.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('Native temperature monitor did not start')
                time.sleep(.2)
            self.checkpoint(window)
            checked = json.loads(self.command([PYTHON, '-B', str(HERE/'start_bulk10000_continuous.py'), '--check-window']))
            if checked['kit'] != str(kit) or checked['case'] != str(case) or checked['guard'] != 'PASS':
                raise RuntimeError('New window failed the unchanged native live guard')
            save(folder/'WINDOW_IDENTITY.json', dict(utc=utc(), case=str(case), kit=str(kit),
                boot_id=self.boot, native_duration_seconds=7200,
                kit_manifest_sha256=sha(kit/'KIT_MANIFEST.json'), fresh_native_guard='PASS',
                operator_confirmation='Operator launched automatic monitored collection locally with sudo',
                capture_pid=capture.pid, temperature_pid=temperatures.pid))
            self.event('MONITORING_READY', window=index, case=str(case))
            self.display_temperatures(window)
            return window
        except BaseException:
            # Attempt protected final capture even for partial startup, then stop.
            self.close_window(window, reason='startup_failure')
            raise

    def collect(self, window):
        if self.interrupted:
            raise KeyboardInterrupt
        self.event('COLLECTING', window=window['index'], case=str(window['case']))
        arguments = [PYTHON, '-B', str(CONTROL/'start_bulk10000_collection.py'),
                     '--run', '--collection', str(self.collection)]
        log = (window['folder']/'collector.log').open('xb')
        child = None
        deadline = time.monotonic()+7200
        next_progress = 0.
        error = None
        try:
            child = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True, **self.as_user())
            while child.poll() is None:
                if self.interrupted:
                    raise KeyboardInterrupt
                if time.monotonic() >= deadline:
                    raise RuntimeError('Collector exceeded the native monitoring window')
                self.checkpoint(window)
                self.display_temperatures(window)
                if time.monotonic() >= next_progress:
                    current = self.progress()
                    self.event('COLLECTING', window=window['index'], case=str(window['case']),
                        recorded_images=current['recorded_images'], completed_batches=current['completed_batches'],
                        current_batch=current.get('current_batch', {}))
                    next_progress = time.monotonic()+30
                time.sleep(.5)
            return child.returncode
        except BaseException as failure:
            error = failure
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=90)
                except subprocess.TimeoutExpired:
                    # Preserve evidence monitors; do not claim successful cleanup.
                    self.event('CLEANUP_REQUIRES_REVIEW', collector_pid=child.pid)
                    raise RuntimeError('Collector cleanup stalled; monitoring remains open for local review') from failure
            raise
        finally:
            log.close()
            save(window['folder']/'COLLECTOR_RESULT.json', dict(utc=utc(), command=arguments,
                exit_code=child.poll() if child is not None else None,
                error=type(error).__name__ if error else None))

    def close_window(self, window, reason):
        self.event('FINISHING_MONITORING', window=window['index'], reason=reason,
                   case=str(window.get('case', 'not-created')))
        errors = []
        finalized = False
        try:
            self.command([PYTHON, '-I', str(STAGE/'manage.py'), 'idle'])
            if 'case' not in window:
                raise RuntimeError('Capture never created its case; preserve preflight output')
            # Exactly the native finish order: idle, 60 seconds, protected journal,
            # user finalization, and only then terminate this window's monitors.
            self.pause(60, window, allow_incident=True)
            self.command([PYTHON, '-I', str(window['kit']/'finish_rocm_retest.py'), window['label']], protected=True)
            self.command([PYTHON, '-I', str(STAGE/'manage.py'), 'finish'])
            finalized = True
            post = read(window['case']/'POST_TEARDOWN.json')
            final = read(window['case']/'MANUAL_SESSION_FINAL.json')
            if not post.get('post_teardown_clean') or not final.get('post_teardown_clean'):
                errors.append('Protected teardown requires incident review')
            self.checkpoint(window)
        except BaseException as error:
            errors.append(type(error).__name__+': '+str(error))
            window['monitor_gap'] = any(p.poll() is not None for p in window['monitors'])
            try:
                self.checkpoint(window, allow_incident=True)
            except BaseException:
                window['monitor_gap'] = True
        if finalized:
            for process in reversed(window['monitors']):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        errors.append('A monitor did not stop after protected finish')
            for stream in window['logs']:
                stream.close()
            self.active = None
        # On missing final journal or unknown surviving workload, leave capture
        # and temperatures running for the operator instead of losing evidence.
        save(window['folder']/'WINDOW_RESULT.json', dict(utc=utc(), reason=reason,
            native_finalized=finalized, errors=errors, monitoring_gap=window['monitor_gap'],
            monitors_kept_through_cooldown_and_journal=finalized and not window['monitor_gap'],
            clean_for_automatic_renewal=finalized and not errors,
            monitoring_left_open=not finalized,
            post_teardown=str(window['case']/'POST_TEARDOWN.json') if 'case' in window else None))
        return finalized and not errors


def run_qualification(engine):
    """Two real monitor/cooldown/journal cycles; no renderer or image capture."""
    for index in (1, 2):
        window = engine.open_window(index)
        try:
            engine.pause(2, window)
        finally:
            closed = engine.close_window(window, reason='monitor_rotation_qualification')
        if not closed:
            raise RuntimeError('Native monitor rotation qualification failed')
    engine.event('MONITOR_ROTATION_QUALIFIED', windows=2, rendered_images=0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--run', action='store_true')
    group.add_argument('--qualify', action='store_true', help='Two native monitoring cycles; no GPU workload')
    group.add_argument('--check', action='store_true', help='Verify fixed inputs; start nothing')
    group.add_argument('--status', action='store_true')
    group.add_argument('--check-window', action='store_true', help=argparse.SUPPRESS)
    group.add_argument('--native-action', choices=['acknowledge', 'modules'], help=argparse.SUPPRESS)
    group.add_argument('--verify-complete', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--max-windows', type=int, default=50)
    args = parser.parse_args(argv)
    if not 1 <= args.max_windows <= 50:
        parser.error('--max-windows must be from 1 to 50')
    if args.status:
        value = read(HERE/'CONTINUOUS_STATUS.json')
        result = subprocess.run([PYTHON, '-B', str(CONTROL/'start_bulk10000_collection.py'),
                                 '--status', '--collection', value['collection']], capture_output=True, text=True, timeout=30)
        if result.returncode:
            value['campaign_status_error'] = result.stderr
        else:
            value['campaign'] = json.loads(result.stdout)
        print(json.dumps(value, indent=2))
        return 0
    manifest = verify_release()
    if args.check:
        print('CONTINUOUS_STATIC_CHECK_PASS: native two-hour windows and protected finish unchanged')
        return 0
    if args.check_window:
        print(json.dumps(user_window_check()))
        return 0
    if args.native_action:
        native_action(args.native_action)
        return 0
    if args.verify_complete:
        if args.verify_complete.parent != BASE or args.verify_complete.is_symlink():
            raise RuntimeError('Completed collection path is invalid')
        verify_completed_campaign(args.verify_complete)
        return 0
    if os.geteuid() != 0:
        parser.error('Run this supervisor with sudo from the Halo graphical desktop terminal')
    import fcntl
    import pwd
    account = pwd.getpwnam('ian')
    if os.environ.get('SUDO_UID') != str(account.pw_uid):
        raise RuntimeError('Launch with local sudo as the qualified desktop user ian')
    os.umask(0o022)
    with (HERE/'continuous.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        collection = Path((CONTROL/'LATEST_COLLECTION.path').read_text().strip())
        if collection.parent != BASE or not collection.name.startswith('bulk10000-') or collection.is_symlink():
            raise RuntimeError('Existing campaign path is invalid')
        output = Path(tempfile.mkdtemp(prefix='bulk10000-continuous-'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-', dir=BASE))
        output.chmod(0o755)
        engine = NativeEngine(manifest, collection, output, account)
        save(output/'RUN_IDENTITY.json', dict(utc=utc(), collection=str(collection), boot_id=engine.boot,
            desktop_user=account.pw_name, child_uid=account.pw_uid, native_window_seconds=7200,
            max_windows=args.max_windows, release_manifest_sha256=sha(HERE/'CONTINUOUS_PINS.json'),
            privileged_scope='Native capture, reviewed modules, protected final journal only',
            qualification_only=args.qualify, training_approved=False))
        def interrupt(*_):
            engine.interrupted = True
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, interrupt)
        try:
            if args.qualify:
                run_qualification(engine)
                result = 'qualified'
            else:
                engine.command([PYTHON, '-B', str(CONTROL/'start_bulk10000_collection.py'), '--check'])
                result = run_campaign(engine, args.max_windows)
                if result == 'complete':
                    engine.command([PYTHON, '-B', str(HERE/'start_bulk10000_continuous.py'),
                                    '--verify-complete', str(collection)], timeout=3600)
            engine.event('COMPLETE_PENDING_IMAGE_REVIEW' if result == 'complete' else result.upper(),
                         training_approved=False)
            return 0
        except BaseException as error:
            engine.event('STOPPED_REVIEW_REQUIRED', error=type(error).__name__+': '+str(error),
                         monitoring_left_open=engine.active is not None)
            print('Outputs preserved. No automatic retry after this stop. Evidence: '+str(output), flush=True)
            return 130 if isinstance(error, KeyboardInterrupt) else 1


if __name__ == '__main__':
    raise SystemExit(main())
