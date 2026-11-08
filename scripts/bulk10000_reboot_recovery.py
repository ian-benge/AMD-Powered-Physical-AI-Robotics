"""One reviewed post-reboot recovery; preserves the unfinished native session.

No renderer, native kit, safety gate or normal launcher is changed. --review is
a protected local-terminal action. --capture prepares a fresh, unchanged native
kit only after that review, then runs its usual sudo capture helper.
"""
import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time


BASE = Path('/home/ian/capstone-training')
STAGE = BASE/'rover-sim'
COLLECTION = BASE/'rover-navigation/navigation_sessions/bulk10000-20261106T163426Z'
CASE = COLLECTION.parent/'oem617-7141-rocm-manual-20261106t183917z-20a4-20261106T183918Z-6m9lfouq'
OLD_BOOT = '5e78f358-9aba-46f7-abdf-0ce9ae3265f3'
SELECTION_SHA = 'bd9b45ffd82283a3e41a94094d5d2f82390a7e4e4c19d8525ecbce3599182f99'
NATIVE_PINS = {
    'manage.py': '3d24fd08fbe8d94acc86f94980382310bd8e0b80ae3ef53a02c7da1afb1394b1',
    'rover-sim.sh': '30c3f4fdfb5702d9e993ec4b90aa8fb8b1b13ecd1cb71e5611c08f90bfff90c2',
}
INCIDENT = re.compile(r'ring .*timeout|amdgpu_job_timedout|GPU reset begin|GPU reset succeeded|'
    r'\bMES\b.*(?:fail|error)|VRAM is lost|GPU fault|kernel panic|soft lockup|'
    r'hard LOCKUP|out of memory|oom-kill|machine check|hardware error', re.I)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    path = Path(path)
    require(not path.is_symlink() and path.is_file(), 'Not a regular evidence file: '+str(path))
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2); stream.write('\n')
        stream.flush(); os.fsync(stream.fileno())


def boot():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def journal_review(text, expected_boot):
    rows = [json.loads(line) for line in text.splitlines() if line.strip()]
    require(rows, 'No protected kernel journal was available; stop for manual investigation')
    require(all(row.get('_BOOT_ID') == expected_boot.replace('-', '') and
                row.get('_TRANSPORT') == 'kernel' for row in rows), 'Kernel journal boot/transport mismatch')
    return [row for row in rows if INCIDENT.search(str(row.get('MESSAGE', '')))]


def cooled(rows):
    if len(rows) < 7:
        return False
    rows = rows[-7:]
    stamps = [dt.datetime.fromisoformat(row['utc']) for row in rows]
    return ((stamps[-1]-stamps[0]).total_seconds() >= 30 and
            all(0 < (b-a).total_seconds() <= 10 for a,b in zip(stamps,stamps[1:])) and
            all(-20 <= row['tctl_c'] <= 70 for row in rows))


def unchanged_selection(current_boot):
    require(current_boot != OLD_BOOT, 'This command is only for the recorded reboot')
    require(sha(STAGE/'CURRENT.json') == SELECTION_SHA, 'Native selection changed; do not replace another session')
    selection = read(STAGE/'CURRENT.json')
    require(selection['boot_id'] == OLD_BOOT, 'Wrong predecessor boot')
    require((CASE/'boot-id.txt').read_text().strip() == OLD_BOOT, 'Wrong predecessor case')
    require(read(CASE/'LATEST_CAPTURE_STATE.json')['state'] == 'stopped', 'Predecessor capture was not recorded stopped')
    require(not (CASE/'POST_TEARDOWN.json').exists() and not (CASE/'MANUAL_SESSION_FINAL.json').exists(),
            'Predecessor now has final evidence; use its normal review')
    for name, expected in NATIVE_PINS.items():
        require(sha(STAGE/name) == expected, 'Native launcher changed: '+name)
    return selection


def command(argv):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=60)
    return dict(argv=argv, returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)


def idle():
    commands = [command(['docker', 'ps', '-q'])]
    commands += [command(['ss', '-ltnH', 'sport', '=', ':'+str(port)]) for port in (8775,8777,8782)]
    require(all(c['returncode'] == 0 and not c['stdout'].strip() for c in commands),
            'Workloads remain or idle inspection failed')
    return commands


def tctl():
    readings = []
    for device in Path('/sys/class/hwmon').glob('hwmon*'):
        if (device/'name').read_text().strip() != 'k10temp':
            continue
        for label in device.glob('temp*_label'):
            if label.read_text().strip() == 'Tctl':
                readings.append(float(label.with_name(label.name.replace('_label','_input')).read_text())/1000)
    require(len(readings) == 1 and -20 <= readings[0] < 90, 'Missing, ambiguous or unsafe Tctl')
    return readings[0]


def publish_review_evidence(evidence, paths, operator_group):
    """Keep root ownership while permitting the operator to verify saved hashes.

    mkdtemp creates a private 0700 directory. Publish it only after every file
    is complete, and grant the original operator's group read/traverse access.
    """
    paths = list(paths)
    require(paths and not evidence.is_symlink(), 'Invalid review evidence directory')
    require(all(path.parent == evidence and not path.is_symlink() and path.is_file()
                for path in paths), 'Review evidence must contain regular direct children')
    for path in paths:
        os.chown(path, -1, operator_group)
        path.chmod(0o640)
    os.chown(evidence, -1, operator_group)
    evidence.chmod(0o750)


def review(folder, acknowledge):
    require(os.geteuid() == 0 and os.isatty(0), 'Run --review with sudo in the Halo terminal')
    require(acknowledge, 'Acknowledge the missing original protected teardown explicitly')
    current_boot = boot(); unchanged_selection(current_boot)
    require(not (folder/'PROTECTED_REVIEW.json').exists(), 'Review already exists; it will never be overwritten')
    evidence = Path(tempfile.mkdtemp(prefix='protected-review-', dir=folder))
    report = dict(version=1, utc=utc(), case=str(CASE), old_boot_id=OLD_BOOT,
        recovery_boot_id=current_boot, original_selection_sha256=SELECTION_SHA,
        original_teardown_completed=False, monitoring_gap=True, post_teardown_clean=False,
        operator_acknowledged_missing_teardown=True, commands={}, files={},
        status='BLOCKED', limitations=[
            'Original capture ended before protected finish; no clean original teardown claim.',
            'Continuous monitoring through cooldown and reboot is unavailable.',
            'A fresh native capture, fan qualification and all collection gates are still required.',
            'The Ogre startup segfault is recorded; its underlying cause is unresolved.'])
    try:
        report['idle_before'] = idle()
        # Preserve the old session and failed launch evidence before new session preparation.
        sources = [STAGE/'CURRENT.json', CASE/'capture-events.jsonl', CASE/'temperature-live.jsonl',
            CASE/'LATEST_CAPTURE_STATE.json', CASE/'CAPTURE_PREFLIGHT.json', CASE/'boot-id.txt',
            COLLECTION/'batches/batch-003/dataset/attempts/scene_4320056/attempt-0001/launch.log']
        for index, source in enumerate(sources):
            dest = evidence/(str(index)+'-'+source.name)
            with source.open('rb') as src, dest.open('xb') as out:
                import shutil
                shutil.copyfileobj(src, out)
            require(sha(dest) == sha(source), 'Evidence changed during preservation')
            report['files'][dest.relative_to(folder).as_posix()] = sha(dest)
        for name, expected in [('old_kernel', OLD_BOOT), ('current_kernel', current_boot)]:
            result = command(['journalctl', '_BOOT_ID='+expected.replace('-',''), '_TRANSPORT=kernel',
                              '--no-pager', '-o', 'json'])
            report['commands'][name] = {k:v for k,v in result.items() if k != 'stdout'}
            dest = evidence/(name+'.jsonl'); dest.write_text(result['stdout'])
            report['files'][dest.relative_to(folder).as_posix()] = sha(dest)
            require(result['returncode'] == 0, 'Protected kernel journal inspection failed')
            matches = journal_review(result['stdout'], expected)
            report[name+'_incident_matches'] = matches
            require(not matches, name+': kernel incident needs investigation; collection remains blocked')
        require(not any((CASE/name).exists() for name in ['INCIDENT_DETECTED.txt','CONTROL_STOP_REQUIRED.txt']),
                'Predecessor has an incident/stop marker; investigate it first')
        samples = []
        print('Recording 60 seconds of current-boot idle cooldown; original monitoring gap remains recorded.', flush=True)
        start = time.monotonic()
        while True:
            sample = dict(utc=utc(), tctl_c=tctl()); samples.append(sample)
            print(json.dumps(sample), flush=True)
            if time.monotonic()-start >= 60:
                break
            time.sleep(5)
        report['cooldown_samples'] = samples
        require(cooled(samples), 'Current boot did not meet the retained cooldown requirement')
        report['idle_after'] = idle()
        kfd = command(['fuser', '-v', '/dev/kfd']); report['commands']['kfd'] = kfd
        require(kfd['returncode'] == 1 and not kfd['stdout'].strip() and not kfd['stderr'].strip(),
                'KFD has owners or inspection failed')
        # Include any kernel incident generated during the current cooldown.
        final = command(['journalctl', '_BOOT_ID='+current_boot.replace('-',''), '_TRANSPORT=kernel',
                         '--no-pager', '-o', 'json'])
        dest = evidence/'final_current_kernel.jsonl'; dest.write_text(final['stdout'])
        report['files'][dest.relative_to(folder).as_posix()] = sha(dest)
        require(final['returncode'] == 0 and not journal_review(final['stdout'],current_boot),
                'Current kernel incident during recovery review')
        unchanged_selection(current_boot)
        publish_review_evidence(evidence, (folder/name for name in report['files']), CASE.stat().st_gid)
        report['status'] = 'REVIEWED_REBOOT_WITH_MISSING_TEARDOWN'
        report['fresh_session_preparation_permitted'] = True
    except Exception as error:
        report['failure'] = str(error)
        raise
    finally:
        report['finished_utc'] = utc(); save(folder/'PROTECTED_REVIEW.json', report)
        print(json.dumps({k:report.get(k) for k in ['status','failure','post_teardown_clean',
                         'monitoring_gap','fresh_session_preparation_permitted']}), flush=True)


def review_identity(report, current_boot):
    require(report.get('status') == 'REVIEWED_REBOOT_WITH_MISSING_TEARDOWN' and
            report.get('fresh_session_preparation_permitted') is True and
            report.get('old_boot_id') == OLD_BOOT and report.get('recovery_boot_id') == current_boot and
            report.get('case') == str(CASE) and report.get('original_selection_sha256') == SELECTION_SHA and
            report.get('post_teardown_clean') is False and report.get('monitoring_gap') is True,
            'Protected review identity/status mismatch')


def verify_review(folder, current_boot):
    path = folder/'PROTECTED_REVIEW.json'
    info = path.stat()
    require(not path.is_symlink() and info.st_uid == 0 and not info.st_mode & 0o022,
            'A root-owned protected review is required')
    report = read(path)
    review_identity(report,current_boot)
    require(bool(report['files']), 'Protected review has no evidence hashes')
    for name, expected in report['files'].items():
        dest = folder/name
        require(not Path(name).is_absolute() and '..' not in Path(name).parts and
                dest.resolve().is_relative_to(folder.resolve()), 'Review evidence path escapes its folder')
        require(sha(dest) == expected, 'Protected review evidence changed: '+name)
    return report


def capture(folder):
    require(os.geteuid() == CASE.stat().st_uid != 0, 'Run --capture as the original Halo user, without sudo')
    current_boot = boot(); unchanged_selection(current_boot); verify_review(folder,current_boot)
    spec = importlib.util.spec_from_file_location('native_recovery_manage', STAGE/'manage.py')
    native = importlib.util.module_from_spec(spec); spec.loader.exec_module(native)
    # Same native preparation requirements; only the documented predecessor-close
    # step differs, because clean teardown cannot be reconstructed after a reboot.
    import platform
    require(platform.release() == native.KERNEL, 'Expected OEM kernel')
    require(platform.freedesktop_os_release().get('VERSION','').startswith('24.04.5 '), 'Unexpected OS')
    require(native.run(['powerprofilesctl','get']) == 'power-saver', 'Expected power-saver')
    native.idle()
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ').lower()
    label = 'rocm-manual-'+stamp+'-'+os.urandom(2).hex(); kit = STAGE/'runs'/label
    native.build(native.SOURCE,native.PHYSICAL,kit,label,current_boot)
    selection = dict(label=label,kit=str(kit),boot_id=current_boot,utc=utc())
    native.save(kit/'SESSION_SELECTION.json',selection)
    save(folder/'RECOVERY_SUCCESSOR.json',dict(selection=selection,
        predecessor_case=str(CASE),protected_review_sha256=sha(folder/'PROTECTED_REVIEW.json'),
        original_teardown_completed=False, native_launcher_modified=False))
    # Recheck before changing the session selector; the original copy is in the
    # protected evidence. The old kit/case and collection stay in place.
    unchanged_selection(current_boot)
    temp = STAGE/('CURRENT.recovery-'+label+'.tmp')
    save(temp,selection); temp.replace(STAGE/'CURRENT.json')
    print('FRESH_RECOVERY_CAPTURE: '+label+'; enter sudo for the unchanged native capture.',flush=True)
    os.execvp('sudo',['sudo','/usr/bin/python3','-u','-I',str(kit/'capture_rocm_retest.py'),
                     '--label',label,'--seconds','7200'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--review',action='store_true')
    modes.add_argument('--capture',action='store_true')
    modes.add_argument('--check',action='store_true')
    parser.add_argument('--acknowledge-missing-teardown',action='store_true')
    args = parser.parse_args(); folder = Path(__file__).resolve().parent
    for name, expected in read(folder/'RECOVERY_SHA256.json')['files'].items():
        require(Path(name).name == name and sha(folder/name) == expected, 'Recovery helper pin mismatch')
    if args.review:
        review(folder,args.acknowledge_missing_teardown)
    elif args.capture:
        capture(folder)
    else:
        unchanged_selection(boot()); idle(); print('REBOOT_RECOVERY_STATIC_CHECK_PASS; no session changed')


if __name__ == '__main__':
    main()
