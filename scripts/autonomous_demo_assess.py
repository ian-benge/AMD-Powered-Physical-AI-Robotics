#!/usr/bin/env python3
"""Isolated demo wrapper around the retained assessment/model lifecycle."""
import argparse
import importlib
import json
import math
from pathlib import Path
import sys
import time
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parent
TRAIN=Path('/home/ian/capstone-training/rover-navigation/navigation_sessions/capture5000-training-20261108')


def demo_settings(original,config,selected,profile,speed,fixture=''):
    if fixture not in ('','flat') or fixture and speed!=.05:
        raise ValueError('The stationary contact fixture is restricted to 0.05 m/s configuration')
    value=original(config,selected)
    rate=config.get('model_assessment',{}).get('camera_rate_hz',value.get('camera_rate_hz',15))
    if rate not in (8,10,15):raise ValueError('Demo camera profile must be explicitly 8, 10 or 15 Hz')
    value.update(demo_mode=not bool(fixture),demo_profile=profile,navigation_speed=speed,fixture=fixture,
        camera_rate_hz=rate,
        manual_initial_linear_speed=.05,manual_max_linear_speed=max(.10,speed),manual_max_angular_speed=.30,
        gui='false',rviz='false')
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--kit',type=Path);p.add_argument('--case',type=Path)
    p.add_argument('--representative',type=int,choices=(1,2,3),default=1)
    p.add_argument('--profile',choices=('rehearsal','full','survey'),default='rehearsal')
    p.add_argument('--speed',type=float,choices=(.05,.10,.15),default=.05)
    p.add_argument('--fixture',choices=('flat',),default='',help='Isolated stationary contact qualification; no demo mission')
    p.add_argument('--check',action='store_true');p.add_argument('--attended',action='store_true')
    a=p.parse_args()
    release=json.loads((ROOT/'ASSESSMENT_RELEASE.json').read_text())
    app=Path(release['application'])
    sys.path.insert(0,str(app/'source/src/rover_navigation'))
    from rover_navigation.platform_telemetry import check_demo_temperature, write_temperature
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    sys.path.insert(0,str(TRAIN));assess=importlib.import_module('capture5000_assess')
    assess.ROOT=ROOT
    sys.path.insert(0,str(Path(release['application'])/'management'))
    supervisor=importlib.import_module('phase2_supervise_live')
    native_live=supervisor.live_guard;last_snapshot=-math.inf
    output=None
    def live_guard(trial,case):
        nonlocal output,last_snapshot
        health=native_live(trial,case)
        check_demo_temperature(health['tctl_c'])
        write_temperature(app/'navigation_sessions/demo-platform-temperature.json',health,boot_id,case)
        if output is None:
            output=(ROOT/('observer-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'.jsonl')).open('x')
        if time.monotonic()-last_snapshot>=.5:
            try:
                with urlopen('http://127.0.0.1:8777/status',timeout=.4) as reply:status=json.load(reply)
                output.write(json.dumps(dict(utc=trial.utc(),wall_monotonic=time.monotonic(),status=status),allow_nan=False)+'\n');output.flush()
            except OSError:pass
            last_snapshot=time.monotonic()
        return health
    original_settings=assess.candidate_settings
    def settings(original,config,selected):
        return demo_settings(lambda c,s:original_settings(original,c,s),config,selected,a.profile,a.speed,a.fixture)
    assess.candidate_settings=settings;supervisor.live_guard=live_guard
    arguments=['--check'] if a.check else ['--kit',str(a.kit),'--case',str(a.case),
        '--representative',str(a.representative),'--seconds','600' if a.fixture else '2700','--attended']
    if not a.check and (not a.attended or a.kit is None or a.case is None):p.error('Use the checked attended starter')
    sys.argv=[str(__file__),*arguments]
    try:return assess.main()
    finally:
        if output:output.close()


if __name__=='__main__':raise SystemExit(main())
