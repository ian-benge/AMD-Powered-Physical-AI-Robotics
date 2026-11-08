#!/usr/bin/env python3
"""Supervise an isolated fixture using the retained Halo capture/runtime gates.

No modification of default launcher pins. Requires a fresh halo-rover capture,
visible temperature monitor, and an attending operator. No navigation commands.
"""
import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

BASELINE = 'sha256:b976b898cc96ad898336310fcd630e2151af527656e549ed24b5f0967a8fc88b'


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2); stream.write('\n')
        stream.flush(); os.fsync(stream.fileno())


def command(args):
    return subprocess.check_output(args, text=True, timeout=20).strip()


def load_guard(kit):
    kit = Path(kit).resolve()
    for relative, expected in json.loads((kit/'KIT_MANIFEST.json').read_text())['files'].items():
        path = kit/relative
        if not path.resolve().is_relative_to(kit) or digest(path) != expected:
            raise RuntimeError('Retained operator kit changed: '+relative)
    sys.path.insert(0, str(kit))
    spec = importlib.util.spec_from_file_location('phase1_retained_trial', kit/'trial.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def live_guard(trial, case):
    trial.capture_guard(case)
    rows = trial.control_health(case)
    import remote_session
    remote_session.fan_guard(rows[-1])
    # Read the same labelled sensor directly as well as checking capture age.
    health = rows[-1]
    label = next(p for p, m in health['sensor_metadata'].items()
                 if m.get('driver') == 'k10temp' and m.get('label') == 'Tctl')
    tctl = float(Path(label).read_text())/1000
    if not -20 <= tctl < 90:
        raise RuntimeError('Stop: Tctl reached the 90 C experiment threshold or is invalid')
    if time.time()-(case/'temperature-live.jsonl').stat().st_mtime > 10:
        raise RuntimeError('Visible temperature monitor is stale')
    return dict(utc=trial.utc(), tctl_c=tctl, capture_utc=health['utc'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('evidence', type=Path)
    p.add_argument('name', help='New unique recording name, never reused')
    p.add_argument('--image', required=True, help='Immutable baseline or candidate image ID')
    p.add_argument('--fixture', required=True, choices=['fixture8', 'fixture20'])
    p.add_argument('--kit', type=Path, required=True)
    p.add_argument('--case', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--mode', choices=['stationary', 'approach_retreat', 'rotation'], default='stationary')
    p.add_argument('--probe', action='store_true', help='Build/run the material contract probe as well')
    p.add_argument('--attended', action='store_true', required=True)
    a = p.parse_args()
    if not re.fullmatch('[a-z0-9-]{1,48}', a.name) or not re.fullmatch('sha256:[a-f0-9]{64}', a.image):
        p.error('Unique lowercase name and immutable image ID required')
    if not 0 < a.seconds <= 600:
        p.error('Duration must be in (0, 600] seconds')
    evidence, case = a.evidence.resolve(), a.case.resolve()
    output = evidence/a.name; output.mkdir()
    name = 'rover-phase1-'+a.name
    child = capture = None
    identity = None; failure = None; capture_complete = False
    interrupted = False

    def stop_signal(*_):
        nonlocal interrupted
        interrupted = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop_signal)

    def inspect():
        info = json.loads(command(['docker', 'inspect', name]))[0]
        if info['Image'] != a.image or info['Config']['Labels'].get('rover.phase1.fixture') != 'true':
            raise RuntimeError('Unexpected fixture container identity')
        return info['Id']

    try:
        trial = load_guard(a.kit)
        import remote_session
        trial.guard(case)
        remote_session.fan_guard(trial.control_health(case)[-1])
        live_guard(trial, case)
        if command(['docker', 'ps', '-q']):
            raise RuntimeError('Stop and close the navigation/other container before fixture work')
        if command(['docker', 'image', 'inspect', '--format', '{{.Id}}', a.image]) != a.image:
            raise RuntimeError('Image identity mismatch')
        fixture = evidence/a.fixture
        metadata = json.loads((fixture/'fixture.json').read_text())
        if not metadata.get('evaluator_only') or metadata['camera']['far_m'] != 8 or metadata['camera']['near_m'] != .3:
            raise RuntimeError('Fixture violates the retained depth contract')
        save(output/'configuration.json', dict(utc=trial.utc(), baseline_image=BASELINE, image=a.image,
             kit=str(a.kit), kit_manifest_sha256=digest(a.kit/'KIT_MANIFEST.json'), case=str(case),
             boot_id=trial.policy.EXPECTED_BOOT, mode=a.mode, seconds=a.seconds, fixture=a.fixture,
             world_sha256=digest(fixture/'world.sdf'), fixture_sha256=digest(fixture/'fixture.json'),
             source_sha256={str(f.relative_to(evidence/'source')):digest(f) for f in sorted((evidence/'source').rglob('*'))
                            if f.is_file() and f.suffix in ('.py','.sh','.cc')},
             camera=metadata['camera'], evaluator_only=True, navigation_motion=False))
        env = dict(os.environ, P1_SUPERVISED='1')
        with (output/'simulator.log').open('xb') as simlog, (output/'capture.log').open('xb') as caplog, \
             (output/'guard-health.jsonl').open('x') as guardlog:
            child = subprocess.Popen(['bash', str(evidence/'source/scripts/phase1_fixture_container.sh'),
                                      name, a.image, str(evidence), a.fixture], env=env,
                                     stdout=simlog, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+40
            while identity is None:
                live_guard(trial, case)
                if interrupted or child.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Fixture startup interrupted or failed')
                result = subprocess.run(['docker', 'inspect', name], capture_output=True, text=True)
                if result.returncode == 0:
                    identity = inspect()
                else:
                    time.sleep(.5)
            save(output/'container.json', json.loads(command(['docker', 'inspect', identity]))[0])
            capture = subprocess.Popen(['docker', 'exec', identity, 'bash', '-c',
                'source /opt/ros/jazzy/setup.bash; exec python3 /evidence/source/scripts/phase1_capture_fixture.py "$@"',
                'fixture', '/evidence/'+a.name+'/capture', '--mode', a.mode, '--seconds', str(a.seconds)],
                stdout=caplog, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+a.seconds+45
            while capture.poll() is None:
                sample = live_guard(trial, case)
                remote_session.fan_guard(trial.control_health(case)[-1])
                guardlog.write(json.dumps(sample)+'\n'); guardlog.flush(); os.fsync(guardlog.fileno())
                if interrupted or child.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Fixture interrupted, stalled, or simulator exited')
                time.sleep(.5)
            if capture.returncode:
                raise RuntimeError('Capture failed; retain summary and logs')
            capture_complete = True
            result = subprocess.run(['docker','exec',identity,'python3',
                '/evidence/source/scripts/phase1_renderer_provenance.py'], capture_output=True,text=True,timeout=20)
            (output/'renderer-provenance.json').write_text(result.stdout)
            if result.returncode:
                (output/'renderer-provenance-error.txt').write_text(result.stderr)
                raise RuntimeError('Actual renderer/plugin provenance check failed')
            if a.probe:
                with (output/'material-probe.log').open('xb') as probelog:
                    capture = subprocess.Popen(['docker', 'exec', identity, 'bash', '-c',
                        'source /opt/ros/jazzy/setup.bash; '
                        'cmake -S /evidence/source/docker/rendering -B /evidence/probe-build && '
                        'cmake --build /evidence/probe-build -j 2 && '
                        '/evidence/probe-build/material_probe /evidence/fixture8/opaque_rgb.png '
                        '/evidence/fixture8/opaque_rgba.png /evidence/fixture8/translucent.png'],
                        stdout=probelog, stderr=subprocess.STDOUT)
                    deadline = time.monotonic()+60
                    while capture.poll() is None:
                        live_guard(trial, case)
                        if interrupted or time.monotonic() > deadline:
                            raise RuntimeError('Material probe interrupted or timed out')
                        time.sleep(.5)
                    save(output/'material-probe-result.json', dict(returncode=capture.returncode,
                         baseline_expected_to_fail=a.image == BASELINE))
                    if capture.returncode and a.image != BASELINE:
                        raise RuntimeError('Candidate material contract failed')
            # Keep actual renderer and backend provenance before removing the container.
            result = subprocess.run(['docker', 'exec', identity, 'bash', '-c',
                'cat /tmp/.gz/rendering/ogre.log; sha256sum /opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre.so.8.2.3'],
                capture_output=True, text=True, timeout=20)
            (output/'renderer-provenance.txt').write_text(result.stdout+result.stderr)
            live_guard(trial, case)
    except Exception as error:
        failure = str(error)
    finally:
        cleanup_errors=[]
        # Cover a fault between docker run and the first successful inspect.
        # Never stop a pre-existing or differently owned container by name.
        if child is not None and identity is None:
            try:
                identity=inspect()
            except Exception as error:
                cleanup_errors.append('Startup identity inspection: '+str(error))
        if identity is not None:
            try:
                subprocess.run(['docker','stop','--time','10',identity],capture_output=True,timeout=20)
            except Exception as error:
                cleanup_errors.append('Fixture stop: '+str(error))
        for process in (capture,child):
            if process and process.poll() is None:
                try:
                    process.terminate(); process.wait(timeout=10)
                except Exception as error:
                    cleanup_errors.append('Child stop: '+str(error))
        if cleanup_errors and failure is None: failure='Cleanup requires review'
        save(output/'session-result.json', dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(),
             capture_complete=capture_complete, failure=failure, interrupted=interrupted,
             cleanup_errors=cleanup_errors,
             acceptance='REVIEW_REQUIRED', limitation='Geometry, RGB, transparency, clipping and kernel evidence require review'))
    print(json.dumps(dict(output=str(output), capture_complete=capture_complete, failure=failure)), flush=True)
    return int(failure is not None)


if __name__ == '__main__':
    raise SystemExit(main())
