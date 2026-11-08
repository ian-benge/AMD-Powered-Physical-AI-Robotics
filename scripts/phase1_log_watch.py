#!/usr/bin/env python3
"""Bounded read-only kernel/desktop and prompt device-dump collection.

Run in a root container with host journals and sysfs mounted read-only; write
only to a fresh evidence directory. Never clears or releases device dumps.
"""
import argparse
import datetime
import json
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=3600)
    parser.add_argument('--journal', default='/phase1-journal')
    parser.add_argument('--sys', default='/phase1-sys')
    args = parser.parse_args()
    if not 0 < args.seconds <= 7200:
        parser.error('duration must be in (0, 7200] seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    deadline = time.monotonic() + args.seconds
    streams = []
    for name, extra in [('kernel', ['-k']), ('desktop-system', [])]:
        log = (args.output / f'{name}.log').open('xb')
        process = subprocess.Popen(['journalctl', f'--directory={args.journal}', '-b',
                                    '-f', '-n', '100', '-o', 'short-iso-precise', *extra],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        streams.append((process, log))
    seen = set()
    try:
        with (args.output / 'events.jsonl').open('x') as events:
            def record(**values):
                events.write(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), **values))+'\n')
                events.flush()
            record(event='start', duration_s=args.seconds)
            while time.monotonic() < deadline:
                for data in sorted((Path(args.sys)/'class/devcoredump').glob('devcd*/data')):
                    identity = (str(data), data.stat().st_ino)
                    if identity in seen:
                        continue
                    seen.add(identity)
                    target = args.output / f'{data.parent.name}-{time.time_ns()}.bin'
                    record(event='dump_detected', source=str(data), target=target.name)
                    try:
                        with target.open('xb') as output:
                            result = subprocess.run(['timeout', '30', 'cat', str(data)], stdout=output,
                                                    stderr=subprocess.PIPE, timeout=35)
                        record(event='dump_copy', bytes=target.stat().st_size, returncode=result.returncode,
                               error=result.stderr.decode(errors='replace'))
                    except Exception as error:
                        record(event='dump_error', error=str(error))
                for process, _log in streams:
                    if process.poll() is not None:
                        raise RuntimeError(f'journal stream exited: {process.returncode}')
                time.sleep(.25)
            record(event='bounded_end')
    finally:
        for process, log in streams:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            log.close()


if __name__ == '__main__':
    main()
