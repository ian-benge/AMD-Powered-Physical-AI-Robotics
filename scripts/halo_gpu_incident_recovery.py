#!/usr/bin/env python3
"""Review one preserved GPU incident before creating a fresh native capture.

Never clears incident flags, changes the failed teardown, or starts a workload.
The adjacent RECOVERY_INPUTS.json binds this helper to one boot and predecessor.
"""
import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import time

INCIDENT = re.compile(r'ring .*timeout|amdgpu_job_timedout|GPU reset|\bMES\b.*(?:fail|error)|'
                      r'VRAM is lost|GPU fault|GPU.*page fault|kernel panic|soft lockup|'
                      r'hard LOCKUP|out of memory|oom-kill|machine check|hardware error', re.I)
STAGE = Path('/home/ian/capstone-training/rover-sim')


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def sha(path):
    require(path.is_file() and not path.is_symlink(), 'Expected regular file: ' + str(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def boot():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def command(argv):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=60)
    return dict(argv=argv, returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)


def journal_rows(text, expected_boot):
    rows = [json.loads(line) for line in text.splitlines() if line.strip()]
    require(bool(rows), 'Empty protected journal')
    require(all(row.get('_BOOT_ID') == expected_boot.replace('-', '') and
                row.get('_TRANSPORT') == 'kernel' for row in rows), 'Wrong journal boot/transport')
    return rows


def incident_identity(rows):
    return sorted((int(row['__REALTIME_TIMESTAMP']), str(row['MESSAGE']))
                  for row in rows if INCIDENT.search(str(row.get('MESSAGE', ''))))


def compare_journal(current, preserved, expected_boot):
    before = journal_rows(preserved, expected_boot)
    after = journal_rows(current, expected_boot)
    require(any('ring sdma0 timeout' in str(r.get('MESSAGE', '')) for r in before), 'Missing known SDMA incident')
    require(any('GPU reset succeeded' in str(r.get('MESSAGE', '')) for r in before), 'Reset recovery unproven')
    require(incident_identity(after) == incident_identity(before), 'New or missing incident evidence; independent review required')
    return incident_identity(after)


def cooled(samples):
    if len(samples) < 7:
        return False
    tail = samples[-7:]
    times = [dt.datetime.fromisoformat(s['utc']) for s in tail]
    return ((times[-1] - times[0]).total_seconds() >= 30 and
            all(0 < (b - a).total_seconds() <= 10 for a, b in zip(times, times[1:])) and
            all(-20 <= s['tctl_c'] <= 70 for s in tail))


def tctl():
    values = []
    for device in Path('/sys/class/hwmon').glob('hwmon*'):
        if (device / 'name').read_text().strip() == 'k10temp':
            for label in device.glob('temp*_label'):
                if label.read_text().strip() == 'Tctl':
                    values.append(float(label.with_name(label.name.replace('_label', '_input')).read_text()) / 1000)
    require(len(values) == 1 and -20 <= values[0] < 90, 'Missing or unsafe Tctl')
    return values[0]


def idle():
    checks = [command(['docker', 'ps', '-q'])]
    checks += [command(['ss', '-ltnH', 'sport', '=', ':' + str(p)]) for p in (8774, 8775, 8777, 8778, 8782)]
    require(all(c['returncode'] == 0 and not c['stdout'].strip() for c in checks), 'Workloads present or idle inspection failed')
    kfd = command(['fuser', '-v', '/dev/kfd'])
    require(kfd['returncode'] == 1 and not (kfd['stdout'] + kfd['stderr']).strip(), 'KFD ownership inspection failed or owners present')
    return checks + [kfd]


def inputs(folder):
    value = json.loads((folder / 'RECOVERY_INPUTS.json').read_text())
    require(boot() == value['boot_id'], 'Boot changed; prepare a new review')
    for name, expected in value['files'].items():
        require(sha(Path(name)) == expected, 'Pinned recovery input changed: ' + name)
    case = Path(value['case'])
    state = json.loads((case / 'LATEST_CAPTURE_STATE.json').read_text())
    require(state['state'] in ('running', 'stopped') and state['boot_id'] == value['boot_id'], 'Predecessor capture identity mismatch')
    post = json.loads((case / 'POST_TEARDOWN.json').read_text())
    require(post['post_teardown_clean'] is False and (case / 'INCIDENT_DETECTED.txt').is_file(), 'Incident evidence mismatch')
    return value, case, post


def kernel():
    result = command(['journalctl', '-b', '-k', '--no-pager', '-o', 'json'])
    require(result['returncode'] == 0 and not result['stderr'].strip(), 'Protected kernel journal failed')
    return result['stdout']


def review(folder, acknowledged):
    require(os.geteuid() == 0 and os.isatty(0), 'Run --review with sudo in the Halo terminal')
    require(acknowledged, 'Explicit --acknowledge-incident is required')
    value, case, post = inputs(folder)
    require(not (folder / 'PROTECTED_REVIEW.json').exists(), 'Preserve previous review; use a new folder')
    report = dict(utc=utc(), case=str(case), boot_id=boot(), status='BLOCKED',
                  post_teardown_clean=False, monitoring_gap=True, root_cause_resolved=False,
                  input_sha256=sha(folder / 'RECOVERY_INPUTS.json'), evidence={})
    try:
        # A compositor crash can leave a stale "running" record. Prove the old
        # capture is absent under root; never rewrite its saved state to stopped.
        for process in Path('/proc').glob('[0-9]*'):
            try:
                argv = (process / 'cmdline').read_bytes().split(b'\0')
            except (FileNotFoundError, ProcessLookupError):
                continue
            require(not any(a.endswith(b'/capture_rocm_retest.py') for a in argv), 'A native capture is still alive')
        report['predecessor_capture_state_preserved'] = json.loads((case / 'LATEST_CAPTURE_STATE.json').read_text())['state']
        report['idle_before'] = idle()
        preserved = post['commands']['kernel']['stdout']
        current = kernel()
        report['incident_matches'] = compare_journal(current, preserved, boot())
        path = folder / 'protected-kernel-before.jsonl'
        with path.open('x') as stream:
            stream.write(current)
        report['evidence'][path.name] = sha(path)
        samples = []
        with (folder / 'protected-cooldown.jsonl').open('x') as stream:
            start = time.monotonic()
            while True:
                sample = dict(utc=utc(), tctl_c=tctl())
                samples.append(sample)
                stream.write(json.dumps(sample) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
                print(json.dumps(sample), flush=True)
                if time.monotonic() - start >= 60:
                    break
                time.sleep(5)
        require(cooled(samples), 'Cooldown failed')
        report['evidence']['protected-cooldown.jsonl'] = sha(folder / 'protected-cooldown.jsonl')
        report['idle_after'] = idle()
        final = kernel()
        compare_journal(final, preserved, boot())
        path = folder / 'protected-kernel-final.jsonl'
        with path.open('x') as stream:
            stream.write(final)
        report['evidence'][path.name] = sha(path)
        inputs(folder)
        report.update(status='REVIEWED_GPU_INCIDENT_FRESH_CAPTURE_ONLY', fresh_capture_permitted=True)
    except Exception as error:
        report['failure'] = str(error)
        raise
    finally:
        report['finished_utc'] = utc()
        save(folder / 'PROTECTED_REVIEW.json', report)
        for name in [*report['evidence'], 'PROTECTED_REVIEW.json']:
            path = folder / name
            os.chown(path, 0, case.stat().st_gid)
            path.chmod(0o640)
        print(json.dumps({k: report.get(k) for k in ('status', 'failure', 'fresh_capture_permitted')}), flush=True)


def capture(folder):
    value, case, _ = inputs(folder)
    require(os.geteuid() == case.stat().st_uid != 0, 'Run --capture as ian without sudo')
    path = folder / 'PROTECTED_REVIEW.json'
    info = path.stat()
    require(info.st_uid == 0 and not info.st_mode & (stat.S_IWGRP | stat.S_IWOTH), 'Review is not root protected')
    report = json.loads(path.read_text())
    require(report.get('status') == 'REVIEWED_GPU_INCIDENT_FRESH_CAPTURE_ONLY' and
            report.get('fresh_capture_permitted') is True and report.get('case') == str(case) and
            report.get('boot_id') == boot() and report.get('input_sha256') == sha(folder / 'RECOVERY_INPUTS.json') and
            report.get('post_teardown_clean') is False and report.get('monitoring_gap') is True, 'Protected review mismatch')
    age = (dt.datetime.now(dt.timezone.utc) - dt.datetime.fromisoformat(report['finished_utc'])).total_seconds()
    require(0 <= age < 600, 'Review expired; prepare a new review')
    for name, expected in report['evidence'].items():
        require(Path(name).name == name and sha(folder / name) == expected, 'Protected evidence changed')
    require(tctl() <= 70, 'Wait for cooldown')
    spec = importlib.util.spec_from_file_location('native_gpu_recovery', STAGE / 'manage.py')
    native = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(native)
    import platform
    require(platform.release() == native.KERNEL, 'Kernel changed')
    require(platform.freedesktop_os_release().get('VERSION', '').startswith('24.04.5 '), 'OS changed')
    require(native.run(['powerprofilesctl', 'get']) == 'power-saver', 'Power profile changed')
    native.idle()
    label = 'rocm-manual-' + dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ').lower() + '-' + os.urandom(2).hex()
    kit = STAGE / 'runs' / label
    native.build(native.SOURCE, native.PHYSICAL, kit, label, boot())
    adapter = folder / 'halo_remote_login_identity.py'
    (kit / adapter.name).write_bytes(adapter.read_bytes())
    for name in ('remote_session.py', 'local_session.py'):
        target = kit / name
        body = target.read_text()
        before = "    assert 'gnome-remote-de' in listener, 'Expected existing GNOME Remote Desktop listener on 3389'"
        require(body.count(before) == 1, 'Unexpected desktop guard source shape')
        target.write_text(body.replace(before, "    from halo_remote_login_identity import verify_listener\n"
            "    verify_listener(listener, command)", 1).replace(
            'GNOME RDP viewer sharing the existing desktop; SSH supports logging/services/timer',
            'Verified system GNOME Remote Login with current graphical session; SSH supports logging/services/timer'))
    # The baseline capture correctly refuses any incident anywhere in this boot.
    # This sibling admits only the exact root-reviewed history, retains full raw
    # journals and leaves live detection and protected finish unchanged.
    history = kit / 'reviewed_history.py'
    shutil_source = Path(__file__).read_text()
    (kit / 'gpu_incident_review.py').write_text(shutil_source)
    history.write_text("import datetime as dt,hashlib,json,stat\nfrom pathlib import Path\n"
        "from gpu_incident_review import compare_journal,require\n"
        "def validate(text,boot):\n"
        " p=Path(" + repr(str(path)) + ")\n"
        " require(p.stat().st_uid==0 and not p.stat().st_mode & (stat.S_IWGRP|stat.S_IWOTH),'Review protection changed')\n"
        " require(hashlib.sha256(p.read_bytes()).hexdigest()==" + repr(sha(path)) + ",'Review changed')\n"
        " r=json.loads(p.read_text())\n"
        " age=(dt.datetime.now(dt.timezone.utc)-dt.datetime.fromisoformat(r['finished_utc'])).total_seconds()\n"
        " require(0<=age<600 and r['boot_id']==boot and r['fresh_capture_permitted'] is True,'Review expired or mismatched')\n"
        " q=p.parent/'protected-kernel-final.jsonl'\n"
        " require(hashlib.sha256(q.read_bytes()).hexdigest()==r['evidence'][q.name],'Protected journal changed')\n"
        " return dict(protected_review=str(p),sha256=" + repr(sha(path)) + ",matches=compare_journal(text,q.read_text(),boot),\n"
        "             historical_incident_retained=True,root_cause_resolved=False)\n")
    source = kit / 'capture_rocm_retest.py'
    body = source.read_text()
    require(body.count('    assert not matches,repr(matches)') == 1, 'Unexpected capture source shape')
    body = body.replace('    assert not matches,repr(matches)',
                        "    from reviewed_history import validate\n    preflight['reviewed_historical_incidents']=validate(journal['stdout'],boot)", 1)
    source.write_text(body)
    # Update only the new kit's manifest. The baseline kit and launcher are intact.
    manifest = kit / 'KIT_MANIFEST.json'
    manifest.write_text(json.dumps(dict(files={str(p.relative_to(kit)): native.digest(p)
        for p in sorted(kit.rglob('*')) if p.is_file() and p != manifest}), indent=2) + '\n')
    native.verify_tree(kit)
    selection = dict(label=label, kit=str(kit), boot_id=boot(), utc=utc())
    native.save(kit / 'SESSION_SELECTION.json', selection)
    save(folder / 'RECOVERY_SUCCESSOR.json', dict(selection=selection, predecessor_case=str(case),
         protected_review_sha256=sha(path), predecessor_post_teardown_clean=False, monitoring_gap=True))
    # Atomic selection replacement is the only native pointer change. Old kit/case remain intact.
    inputs(folder)
    temporary = STAGE / ('CURRENT.gpu-recovery-' + label + '.tmp')
    save(temporary, selection)
    temporary.replace(STAGE / 'CURRENT.json')
    print('FRESH_NATIVE_CAPTURE: ' + label, flush=True)
    os.execvp('sudo', ['sudo', '/usr/bin/python3', '-u', '-I', str(kit / 'capture_rocm_retest.py'), '--label', label, '--seconds', '7200'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--review', action='store_true')
    mode.add_argument('--capture', action='store_true')
    parser.add_argument('--acknowledge-incident', action='store_true')
    args = parser.parse_args()
    folder = Path(__file__).resolve().parent
    if args.check:
        inputs(folder)
        print('PINNED_GPU_RECOVERY_CHECK_PASS')
    elif args.review:
        review(folder, args.acknowledge_incident)
    else:
        capture(folder)


if __name__ == '__main__':
    main()
