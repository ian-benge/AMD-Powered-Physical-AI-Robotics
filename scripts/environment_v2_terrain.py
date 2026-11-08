#!/usr/bin/env python3
"""Attended candidate terrain service with native host/runtime guards and HIP 4.

Uses a new source tree and the original interpreter/libraries. Baseline kit and
deployed source are never edited. The sole KFD worker occupies the standard
terrain PID slot so native monitoring and protected finish retain ownership.
"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parent))
from phase1_run_fixture import load_guard,live_guard,digest
from phase1_supervise_live import verify_configuration

PYTHON=Path('/home/ian/capstone-training/venvs/terrain-rocm7141/bin/python')


def source_pins(root):
    return {str(p):digest(p) for p in [Path(__file__).resolve(),
        *[root/'source/src/rover_navigation/rover_navigation'/name for name in ('core.py','terrain_compute.py','terrain_service.py')]]}


def worker(args,trial,config,root):
    if Path(sys.prefix)!=PYTHON.parent.parent:raise RuntimeError('Wrong isolated terrain interpreter')
    if os.environ.get('OMP_NUM_THREADS')!='1' or os.environ.get('ROCM_SDK_TARGET_FAMILY')!='gfx1151':
        raise RuntimeError('Wrong qualified terrain environment')
    if any(k in ('LD_PRELOAD','LD_LIBRARY_PATH','PYTHONPATH') or k.startswith(('HSA_','HIP_','ROCR_')) for k in os.environ):
        raise RuntimeError('Unexpected terrain runtime override')
    if subprocess.check_output(['ss','-ltnH','sport','=',':8782'],text=True).strip():raise RuntimeError('Terrain port occupied')
    with (args.case/'terrain-pid.txt').open('x') as stream:stream.write(str(os.getpid())+'\n')
    trial.guard(args.case);trial.check_qualification(args.case);trial.npu_status()
    if (args.case/'terrain-scheduling.json').exists():raise RuntimeError('Terrain evidence is never overwritten')
    import hip_scheduling
    from terrain_entry import start_service
    libraries={}
    for relative,expected in hip_scheduling.LIBRARIES.items():
        file=hip_scheduling.SITE/relative
        if not file.is_file() or not file.resolve().is_relative_to(hip_scheduling.SITE.resolve()) or digest(file)!=expected:
            raise RuntimeError('Qualified runtime library changed: '+relative)
        libraries[relative]=dict(resolved=str(file.resolve()),sha256=expected)
    hip=ctypes.CDLL(str(hip_scheduling.SITE/'_rocm_sdk_core/lib/libamdhip64.so.7'))
    flags=hip_scheduling.apply_flags(hip)  # first HIP API, before candidate/Torch imports
    sys.path.insert(0,str(root/'source/src/rover_navigation'))
    from rover_navigation.terrain_compute import RocmTerrainCompute,MAX_GRID_SIDE
    from rover_navigation.terrain_service import create_server
    if MAX_GRID_SIDE!=1600:raise RuntimeError('Wrong candidate grid bound')
    pins=source_pins(root)
    def save(scheduling):
        trial.save(args.case/'terrain-scheduling.json',dict(utc=trial.utc(),case=str(args.case),boot_id=trial.policy.EXPECTED_BOOT,
            identity=trial.proc_identity(os.getpid()),flags=flags,after_warmup=scheduling,libraries=libraries,
            condition='environment-v2-1600-grid-bound-stock-7141-blocking-sync',
            application_compute_and_server_unchanged=False,candidate_source_pins=pins,max_memory_side=1600))
    start_service(RocmTerrainCompute,create_server,hip,8782,save)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('configuration',type=Path);p.add_argument('--kit',type=Path,required=True)
    p.add_argument('--case',type=Path,required=True);p.add_argument('--worker',action='store_true')
    p.add_argument('--module',choices=['rover_navigation.terrain_service'],required=True)
    a=p.parse_args();config,root=verify_configuration(a.configuration)
    trial=load_guard(a.kit)
    if a.worker:return worker(a,trial,config,root)
    trial.guard(a.case);live_guard(trial,a.case);trial.check_qualification(a.case);trial.npu_status()
    if not trial.policy.cooled(trial.control_health(a.case)):raise RuntimeError('Wait for retained cooldown <=70 C for 30 seconds')
    if (a.case/'terrain-pid.txt').exists():raise RuntimeError('Preserve this terrain attempt; finish the case before another attempt')
    pins=source_pins(root);stopped=False;failure=None
    def halt(*_):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,halt)
    env=dict(os.environ)
    for key in ('LD_PRELOAD','LD_LIBRARY_PATH','PYTHONPATH','HSA_OVERRIDE_GFX_VERSION'):env.pop(key,None)
    env.update(OMP_NUM_THREADS='1',ROCM_SDK_TARGET_FAMILY='gfx1151')
    command=[str(PYTHON),'-u','-I',str(Path(__file__).resolve()),str(a.configuration),'--kit',str(a.kit),
             '--case',str(a.case),'--worker','--module',a.module]
    child=None
    with (a.case/'environment-v2-terrain-startup.log').open('xb') as log:
        try:
            child=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            started=time.monotonic();ready=False
            while child.poll() is None and not stopped:
                live_guard(trial,a.case)
                if any(digest(path)!=sha for path,sha in pins.items()):raise RuntimeError('Candidate terrain source changed')
                if not ready:
                    try:
                        status=trial.terrain_status()
                        if status.get('max_memory_side')!=1600:raise RuntimeError('Wrong terrain capacity')
                        ready=True;print('ENVIRONMENT_V2_TERRAIN_READY: float64, HIP requested/observed 4, grid bound 1600',flush=True)
                    except (OSError,AssertionError):
                        if time.monotonic()-started>60:raise RuntimeError('Candidate terrain startup timed out')
                time.sleep(.5)
            if child.poll()==-signal.SIGTERM:stopped=True  # native protected finish stops its tracked worker
            elif child.poll() not in (None,0):raise RuntimeError('Candidate terrain worker exited; inspect startup log')
        except Exception as error:failure=str(error)
        finally:
            if child and child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=30)
            trial.save(a.case/'environment-v2-terrain-final.json',dict(utc=trial.utc(),failure=failure,
                operator_stop=stopped,worker_exit=child.returncode if child else None,candidate_source_pins=pins))
    if failure:raise RuntimeError(failure)


if __name__=='__main__':main()
