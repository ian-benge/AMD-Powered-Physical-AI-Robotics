#!/usr/bin/env python3
"""Build a separately tagged candidate under retained capture/thermal gates."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from phase1_run_fixture import BASELINE, command, digest, live_guard, load_guard, save

SOURCE_ARCHIVE = '5bfe18d164414e098b2ba6ef81bf9391f7ff8f2c39b8a4fa9f8543f778e921c8'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('context', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--tag', required=True)
    p.add_argument('--case', required=True, type=Path)
    p.add_argument('--kit', required=True, type=Path)
    p.add_argument('--attended', action='store_true', required=True)
    a = p.parse_args()
    if not re.fullmatch('rover-navigation-phase1-[a-z0-9-]+', a.tag):
        p.error('Separate phase1 candidate tag required')
    a.output.mkdir()
    trial = load_guard(a.kit); trial.guard(a.case)
    live_guard(trial, a.case)
    # Use a dedicated local alias because BuildKit interprets sha256:ID as
    # a registry repository/tag in FROM. Never edit/overwrite baseline tags.
    alias = 'rover-phase1-base-'+a.tag.removeprefix('rover-navigation-phase1-')
    if subprocess.run(['docker', 'image', 'inspect', a.tag], capture_output=True).returncode == 0:
        raise RuntimeError('Candidate tag already exists; use a new version')
    prior = subprocess.run(['docker', 'image', 'inspect', alias], capture_output=True)
    if prior.returncode:
        command(['docker', 'tag', BASELINE, alias])
    if command(['docker', 'image', 'inspect', '--format', '{{.Id}}', alias]) != BASELINE:
        raise RuntimeError('Separate baseline alias has unexpected contents')
    if digest(a.context/'gz-rendering-8.2.3.tar.gz') != SOURCE_ARCHIVE:
        raise RuntimeError('Pinned upstream archive differs')
    args = ['docker', 'build', '--pull=false', '--network', 'none', '--build-arg', 'BASE_IMAGE='+alias,
            '-t', a.tag, str(a.context.resolve())]
    save(a.output/'configuration.json', dict(command=args, baseline_image=BASELINE, baseline_alias=alias,
         case=str(a.case), kit=str(a.kit), source_sha256={f.name:digest(f) for f in a.context.iterdir() if f.is_file()}))
    interrupted = False
    def stop_signal(*_):
        nonlocal interrupted
        interrupted = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop_signal)
    failure = None
    with (a.output/'build.log').open('xb') as log, (a.output/'health.jsonl').open('x') as health:
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+1200
            while process.poll() is None:
                if interrupted or time.monotonic() > deadline:
                    raise RuntimeError('Build interrupted or timed out')
                health.write(json.dumps(live_guard(trial, a.case))+'\n'); health.flush(); os.fsync(health.fileno())
                time.sleep(.5)
            if process.returncode:
                raise RuntimeError('Build failed: '+str(process.returncode))
            if command(['docker', 'image', 'inspect', '--format', '{{.Id}}', alias]) != BASELINE:
                raise RuntimeError('Baseline alias changed during build')
        except Exception as error:
            failure = str(error)
            if process.poll() is None:
                process.terminate(); process.wait(timeout=30)
    result = dict(utc=trial.utc(), failure=failure, baseline=BASELINE, tag=a.tag,
                  image=command(['docker', 'image', 'inspect', '--format', '{{.Id}}', a.tag]) if failure is None else None)
    save(a.output/'result.json', result)
    print(json.dumps(result), flush=True)
    return int(failure is not None)


if __name__ == '__main__':
    raise SystemExit(main())
