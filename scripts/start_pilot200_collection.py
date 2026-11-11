#!/usr/bin/env python3
"""Attended Halo starter for the separate 200-frame pilot; no training."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
from desktop_render_guard import require_desktop

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')
FIX = BASE/'capture-ogre-fix-20261106T140200Z'
CANDIDATE = FIX/'repeat-2'
APP = CANDIDATE/'application'
ASSETS = CANDIDATE/'bundles/populated/assets'
MANIFEST_SHA = 'ab0173e15f028a0a2ddebb86926f3262b51f525f684fb5b7f207299bfeae5ae9'
PINS_SHA = '684ecf192735b32ed9edef2fefbf5dc52bd415e3f03e1c82e5e49063c3fe7048'
GUARD = BASE/'environment-v2-20261105T225758Z/run_guarded_step.py'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2); stream.write('\n')


def stop_owned_container(name, image, application):
    inspected = subprocess.run(['docker', 'inspect', name], capture_output=True, text=True, timeout=15)
    if inspected.returncode:
        return
    value = json.loads(inspected.stdout)[0]
    if value['Image'] != image or not any(m['Destination'] == '/work' and
            Path(m['Source']).resolve() == application.resolve() for m in value['Mounts']):
        raise RuntimeError('Refusing cleanup of a container with a different identity')
    if value['State']['Running']:
        subprocess.run(['docker', 'stop', '--timeout', '10', value['Id']],
                       capture_output=True, check=True, timeout=25)


def run_guarded(command, name, image, application):
    # subprocess.run kills its child on KeyboardInterrupt before the guard can
    # finish Docker cleanup. Keep the guard alive while it handles cancellation.
    signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, 'SIGHUP'):
        signals.append(signal.SIGHUP)
    previous = {sig: signal.getsignal(sig) for sig in signals}

    def cancel(*_):
        raise KeyboardInterrupt

    child = None
    try:
        for sig in signals:
            signal.signal(sig, cancel)
        child = subprocess.Popen(command, start_new_session=True)
        try:
            return child.wait()
        except KeyboardInterrupt:
            for sig in signals:
                signal.signal(sig, signal.SIG_IGN)
            print('Stopping collection; waiting for container cleanup. Keep monitors running.', flush=True)
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=45)
            except subprocess.TimeoutExpired:
                pass
            return 130
    finally:
        try:
            stop_owned_container(name, image, application)
        finally:
            if child is not None and child.poll() is None:
                child.kill(); child.wait(timeout=10)
            for sig, handler in previous.items():
                signal.signal(sig, handler)


def check_candidate():
    if sha(FIX/'FIX_EVIDENCE_MANIFEST.json') != MANIFEST_SHA or sha(CANDIDATE/'CANDIDATE_PINS.json') != PINS_SHA:
        raise ValueError('Qualified capture candidate manifest changed')
    pins = json.loads((CANDIDATE/'CANDIDATE_PINS.json').read_text())
    for name, expected in pins['files'].items():
        if sha(APP/name) != expected:
            raise ValueError('Candidate pin changed: '+name)
    manifest = json.loads((FIX/'FIX_EVIDENCE_MANIFEST.json').read_text())
    prefix = 'repeat-2/bundles/populated/assets/'
    assets = {name[len(prefix):]: value['sha256'] for name, value in manifest['files'].items()
              if name.startswith(prefix)}
    if not assets or set(assets) != {p.relative_to(ASSETS).as_posix() for p in ASSETS.rglob('*') if p.is_file()}:
        raise ValueError('Prepared asset inventory changed')
    for name, expected in assets.items():
        if sha(ASSETS/name) != expected:
            raise ValueError('Prepared asset changed: '+name)
    own = Path(__file__).resolve().parent
    tools = json.loads((own/'STARTER_SHA256.json').read_text())
    for name, expected in tools.items():
        if sha(own/name) != expected:
            raise ValueError('Collection starter changed: '+name)
    print(f'PILOT200_STATIC_CHECK_PASS: {len(pins["files"])} candidate pins; {len(assets)} assets', flush=True)
    return pins


def desktop_authority():
    if not os.environ.get('DISPLAY'):
        raise ValueError('Run from a Halo graphical-desktop terminal')
    # Mutter can own the X sockets before starting Xwayland on demand. A plain
    # X11 query establishes that desktop connection without creating a GL
    # context; scanning /proc first would reject an otherwise valid new boot.
    try:
        probe = subprocess.run(['xdpyinfo', '-display', os.environ['DISPLAY']],
                               capture_output=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ValueError('Desktop X11 query failed; keep the Halo desktop connected') from error
    if probe.returncode:
        raise ValueError('Desktop X11 display is unavailable; keep the Halo desktop connected')
    require_desktop()
    found = []
    for proc in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = proc.read_bytes().decode().split('\0')
            if args and Path(args[0]).name == 'Xwayland' and '-auth' in args:
                found.append((next(a for a in args if a.startswith(':')), args[args.index('-auth')+1]))
        except (FileNotFoundError, PermissionError, ProcessLookupError, StopIteration):
            pass
    if len(found) != 1 or found[0][0] != os.environ['DISPLAY']:
        raise ValueError('Qualified desktop display/Xwayland identity is ambiguous')
    display, source = found[0]
    secret = subprocess.check_output(['xauth', '-f', source, 'nlist', display])
    if not secret.strip():
        raise ValueError('Desktop authority is unavailable')
    fd, name = tempfile.mkstemp(prefix='pilot200-xauth-'); os.close(fd)
    path = Path(name); path.chmod(0o600)
    try:
        records = b'\n'.join(b'ffff'+row[4:] for row in secret.splitlines())+b'\n'
        subprocess.run(['xauth', '-f', str(path), 'nmerge', '-'], input=records,
                       capture_output=True, check=True)
    except BaseException:
        path.unlink(); raise
    return display, path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Static checks only; no container or capture')
    parser.add_argument('--resume', type=Path, help='Prior collection with a completed pinned scene plan')
    args = parser.parse_args()
    pins = check_candidate()
    if args.check:
        return 0
    context = dict(line.split('=', 1) for line in subprocess.check_output(
        ['python3', '-I', '/home/ian/capstone-training/rover-sim/manage.py', 'context'], text=True).splitlines() if '=' in line)
    kit = Path(context['SIM_KIT'])
    pointer = Path('/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103')/('capture-'+context['SIM_LABEL']+'.path')
    case = Path(pointer.read_text().strip())
    if (case/'MANUAL_SESSION_FINAL.json').exists():
        raise ValueError('Start fresh capture and temperature terminals first')
    sys.path.insert(0, str(APP/'management'))
    from phase1_run_fixture import load_guard, live_guard
    trial = load_guard(kit)
    import remote_session
    inventory = remote_session.inventory(require_view=True)
    acknowledgement = case/'local-capture-confirmed.json'
    if not acknowledgement.exists():
        save(acknowledgement, dict(utc=trial.utc(), case=str(case),
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            delivery='local-files-only', offhost_delivery_verified=False,
            operator_confirmation='Operator invoked attended collection starter from the Halo desktop', **inventory))
    trial.guard(case); live_guard(trial, case)
    if subprocess.check_output(['docker', 'ps', '-q'], text=True).strip():
        raise ValueError('Stop other workloads before collection; leave NPU and terrain off')
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    if args.resume:
        root = args.resume.resolve()
        if root.parent != BASE or not root.name.startswith('pilot200-'):
            raise ValueError('Resume requires an existing pilot200 collection directory')
        identity = json.loads((root/'COLLECTION_IDENTITY.json').read_text())
        if identity['source_archive_sha256'] != pins['source_archive_sha256']:
            raise ValueError('Resume candidate differs from original collection')
    else:
        root = BASE/('pilot200-'+stamp); root.mkdir()
        save(root/'COLLECTION_IDENTITY.json', dict(source_archive_sha256=pins['source_archive_sha256'],
            candidate=str(CANDIDATE), planned_images=200, scenes=10, training_approved=False))
    print('COLLECTION_DIR='+str(root), flush=True)
    run = root/('invocation-'+stamp); run.mkdir()
    worker = Path(__file__).resolve().with_name('pilot200_capture_worker.py')
    shutil.copy2(worker, run/worker.name)
    cleanup = Path(__file__).resolve().with_name('process_cleanup.py')
    shutil.copy2(cleanup, run/cleanup.name)
    desktop_guard = Path(__file__).resolve().with_name('desktop_render_guard.py')
    shutil.copy2(desktop_guard, run/desktop_guard.name)
    save(run/'INVOCATION.json', dict(case=str(case), kit=str(kit), resume=bool(args.resume),
                                   worker_sha256=sha(worker), cleanup_sha256=sha(cleanup),
                                   desktop_guard_sha256=sha(desktop_guard), runtime_version=2, utc=trial.utc()))
    display, authority = desktop_authority()
    name = 'rover-pilot200-'+stamp.lower()
    command = ['docker', 'run', '--rm', '--init', '--name', name, '--network', 'host',
        '--shm-size=1g', '--user', f'{os.getuid()}:{os.getgid()}', '--device', '/dev/dri',
        '--group-add', '992']
    for value in ['HOME=/tmp', 'ROS_LOG_DIR=/collection/roslogs', 'DISPLAY='+display,
                  'XAUTHORITY=/tmp/pilot200.xauth', 'LIBGL_ALWAYS_SOFTWARE=0', 'ROS_DOMAIN_ID=231',
                  'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
                  'PYTHONPATH=/work/source/scripts:/work/management:/work/testdeps',
                  'PYTHONDONTWRITEBYTECODE=1', 'PYTHONUNBUFFERED=1',
                  'OMP_NUM_THREADS=2', 'OPENBLAS_NUM_THREADS=2']:
        command += ['-e', value]
    for value in ['/tmp/.X11-unix:/tmp/.X11-unix:ro', str(authority)+':/tmp/pilot200.xauth:ro',
                  str(APP)+':/work:ro', str(ASSETS)+':/prepared-assets:ro', str(root)+':/collection',
                  str(run)+':/invocation:ro']:
        command += ['-v', value]
    command += ['-w', '/work/source', pins['image'], 'bash', '-c',
        'source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && '
        'exec python3 /invocation/pilot200_capture_worker.py'+(' --resume' if args.resume else '')]
    config = run/'guard-configuration.json'
    save(config, dict(application=str(APP), output=str(run/'guarded-step'), case=str(case),
        kit=str(kit), image=pins['image'], container=name, timeout_seconds=5400, command=command))
    print('Planning ten scenes, then capturing 200 frames. Keep temperatures visible.', flush=True)
    print('Progress log: '+str(run/'guarded-step/log.txt'), flush=True)
    try:
        code = run_guarded(['python3', str(GUARD), str(config)], name, pins['image'], APP)
    finally:
        authority.unlink(missing_ok=True)
    if code:
        print('Collection stopped; outputs/logs preserved. Resume requires a completed pinned scene plan.', flush=True)
        return code
    print('PILOT200_COLLECTION_COMPLETE_PENDING_REVIEW', flush=True)
    print('Run ~/bin/halo-rover finish; keep monitors running through cooldown and final capture.', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
