#!/usr/bin/env python3
"""Bounded GET-only stationary qualification; sends no controls or heartbeats."""
import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import time
from urllib.request import urlopen


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def assess(status):
    guard = status.get('status', {})
    demo = status.get('demo_status', {})
    mapping = status.get('mapping', {})
    camera = status.get('camera_health', {})
    localization = status.get('localization', {})
    temperature = status.get('platform_temperature', {})
    p95 = mapping.get('publication_latency_p95_seconds')
    rtf = demo.get('real_time_factor')
    tctl = temperature.get('tctl_c')
    return dict(
        stopped=guard.get('mode') == 'stopped' and guard.get('command') == [0., 0.] and
                demo.get('actual_distance_m') == 0 and demo.get('phase') == 'ready' and not demo.get('goal_active'),
        navigation=demo.get('navigation_health', {}).get('healthy') is True,
        mapping=mapping.get('clearance_confirmed') is True and mapping.get('waiting_for_confirmation') is False and
                not mapping.get('compute_error') and finite(mapping.get('accepted_depth_stamp')),
        camera=camera.get('available') is True and finite(camera.get('age_seconds')) and 0 <= camera['age_seconds'] < 1.5,
        localization=localization.get('healthy') is True,
        temperature=finite(tctl) and -20 <= tctl < 87 and finite(temperature.get('age_seconds')) and
                    0 <= temperature['age_seconds'] < 5,
        publication=finite(p95) and 0 < p95 <= .2 and mapping.get('publication_latency_samples', 0) > 0,
        rtf=finite(rtf) and rtf >= .8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seconds', type=int, default=300)
    args = parser.parse_args()
    if args.seconds != 300:
        raise ValueError('This stationary prerequisite requires five minutes')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if (args.case / 'boot-id.txt').read_text().strip() != boot:
        raise RuntimeError('Case belongs to another boot')
    samples = []
    failure = None
    session = None
    with args.output.open('x') as stream:
        start = time.monotonic()
        try:
            for index in range(61):
                delay = start + index * 5 - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                with urlopen('http://127.0.0.1:8777/status', timeout=3) as response:
                    status = json.load(response)
                state = json.loads((args.case / 'LATEST_CAPTURE_STATE.json').read_text())
                utc = dt.datetime.now(dt.timezone.utc)
                age = (utc - dt.datetime.fromisoformat(state['utc'])).total_seconds()
                checks = assess(status)
                checks['native'] = (state['state'] == 'running' and state['boot_id'] == boot and 0 <= age < 20 and
                                    not (args.case / 'INCIDENT_DETECTED.txt').exists() and
                                    not (args.case / 'CONTROL_STOP_REQUIRED.txt').exists())
                current = status.get('demo_status', {}).get('session_id')
                if index == 0:
                    session = current
                checks['same_session'] = bool(session) and current == session
                sample = dict(utc=utc.isoformat(), elapsed_seconds=time.monotonic() - start,
                              checks=checks, status=status, native_health_utc=state['utc'])
                samples.append(sample)
                stream.write(json.dumps(sample, allow_nan=False) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
                if not all(checks.values()):
                    raise RuntimeError('Stationary prerequisite failed: ' + ', '.join(k for k, v in checks.items() if not v))
                if index % 6 == 0:
                    print(json.dumps(dict(utc=sample['utc'], samples=len(samples),
                          tctl_c=status['platform_temperature']['tctl_c'],
                          rolling_p95_ms=status['mapping']['publication_latency_p95_seconds'] * 1000,
                          rtf=status['demo_status']['real_time_factor'])), flush=True)
        except Exception as error:
            failure = str(error)
    intervals = [b['elapsed_seconds'] - a['elapsed_seconds'] for a, b in zip(samples, samples[1:])]
    passed = (failure is None and len(samples) == 61 and samples[-1]['elapsed_seconds'] >= 300 and
              all(0 < gap < 7.5 for gap in intervals))
    report = dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(), case=str(args.case), session=session,
                  passed=passed, failure=failure, samples=len(samples), raw=str(args.output),
                  raw_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                  scope='Five-minute stationary prerequisite only; no movement or endurance acceptance')
    if samples:
        temps = [s['status']['platform_temperature']['tctl_c'] for s in samples]
        report.update(start=samples[0]['utc'], end=samples[-1]['utc'], mean_tctl_c=statistics.mean(temps), peak_tctl_c=max(temps),
                      max_rolling_publication_p95_ms=max(s['status']['mapping']['publication_latency_p95_seconds'] * 1000 for s in samples),
                      median_reported_rtf=statistics.median(s['status']['demo_status']['real_time_factor'] for s in samples),
                      minimum_reported_rtf=min(s['status']['demo_status']['real_time_factor'] for s in samples),
                      maximum_sample_gap_seconds=max(intervals) if intervals else None)
    with args.output.with_suffix('.summary.json').open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
