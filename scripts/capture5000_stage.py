"""Run a bounded post-training preparation command under the native capture guards."""
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from capture_training_dataset import sha,save
import start_capture5000_training as training

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('configuration',type=Path)
    args=p.parse_args();config=json.loads(args.configuration.read_text())
    training.verify_release()
    if not 0<config['seconds']<=5400:raise ValueError('Stage budget must fit a native session')
    folder=Path(config['output'])
    if not folder.resolve().is_relative_to(ROOT.resolve()):raise ValueError('Use an isolated training follow-up directory')
    folder.mkdir()
    sys.path.insert(0,str(training.CONTROL))
    import start_bulk10000_collection as starter
    kit,case,trial,live_guard=starter.session()
    token=str(uuid.uuid4())
    env=dict(os.environ,PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',ROVER_TRAINING_TOKEN=token,
        OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',MAX_JOBS='2',
        CUDA_VISIBLE_DEVICES='',HIP_VISIBLE_DEVICES='',**config.get('environment',{}))
    child=None;failure=None
    def check():
        starter.retry_session_scan(lambda:trial.guard(case))
        starter.retry_session_scan(lambda:live_guard(trial,case))
    save(folder/'INVOCATION.json',dict(utc=trial.utc(),configuration=config,configuration_sha256=sha(args.configuration),
        kit=str(kit),case=str(case),source_sha256=sha(Path(__file__))))
    try:
        check()
        with (folder/'stage.log').open('xb') as log:
            child=subprocess.Popen(config['command'],cwd=config.get('cwd',str(ROOT)),env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+config['seconds']
            while child.poll() is None:
                check()
                if time.monotonic()>deadline:raise RuntimeError('Stage exceeded its monitored budget')
                time.sleep(2)
        if child.returncode:raise RuntimeError('Preparation failed; preserve '+str(folder/'stage.log'))
        print('STAGE_PASS '+str(folder),flush=True)
    except BaseException as error:
        failure=type(error).__name__+': '+str(error);raise
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=30)
            except subprocess.TimeoutExpired:pass
        cleanup=training.cleanup_owned(token,os.getuid())
        if child is not None and child.poll() is None:child.wait(timeout=10)
        save(folder/'RESULT.json',dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(),failure=failure,
            exit_code=child.returncode if child else None,cleanup=cleanup))


if __name__=='__main__':main()
