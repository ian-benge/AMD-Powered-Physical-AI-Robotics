"""Retained native guards, with a bounded retry of a vanished loginctl session."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


def retry_session_scan(check, on_retry=None):
    """Repeat the whole check; never reuse a partial session inventory."""
    pattern=(r"\['loginctl', 'show-session', '([A-Za-z0-9_-]+)',[^\n]*\]: "
             r"Failed to get path for session '\1': No session '\1' known\s*")
    for attempt in range(3):
        try:
            return check()
        except AssertionError as error:
            if attempt==2 or not re.fullmatch(pattern,str(error)):
                raise
            if on_retry is not None:on_retry(str(error))
            time.sleep(.02)


def stop_owned_container(config, app):
    inspected=subprocess.run(['docker','container','inspect',config['container']],capture_output=True,text=True,timeout=15)
    if inspected.returncode:
        return
    value=json.loads(inspected.stdout)[0]
    if value['Image']!=config['image'] or not any(
        m['Destination']=='/work' and Path(m['Source']).resolve()==app for m in value['Mounts']):
        raise RuntimeError('Refusing cleanup of a different container identity')
    subprocess.run(['docker','stop','--time','10',value['Id']],capture_output=True,check=True,timeout=25)


def main():
    config=json.loads(Path(sys.argv[1]).read_text())
    app=Path(config['application']).resolve();out=Path(config['output']);out.mkdir()
    sys.path.insert(0,str(app/'management'))
    from phase1_run_fixture import load_guard,live_guard
    trial=load_guard(Path(config['kit']));case=Path(config['case'])
    stop=False;child=None;failure=None;cleanup_error=None;retries=0

    def halt(*_):
        nonlocal stop
        stop=True

    def retried(error):
        nonlocal retries
        retries+=1

    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,halt)
    with (out/'log.txt').open('xb') as log,(out/'health.jsonl').open('x') as health:
        try:
            retry_session_scan(lambda:trial.guard(case),retried)
            retry_session_scan(lambda:live_guard(trial,case),retried)
            if subprocess.check_output(['docker','ps','-q'],text=True).strip():
                raise RuntimeError('Other container running')
            child=subprocess.Popen(config['command'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+config['timeout_seconds']
            while child.poll() is None:
                if stop or time.monotonic()>deadline:raise RuntimeError('Step stopped or timed out')
                row=retry_session_scan(lambda:live_guard(trial,case),retried)
                row['session_scan_retries']=retries
                health.write(json.dumps(row)+'\n');health.flush();os.fsync(health.fileno())
                time.sleep(.5)
            if child.returncode:raise RuntimeError('Step command failed: '+str(child.returncode))
        except Exception as error:
            failure=str(error)
            if child and child.poll() is None:
                try:
                    stop_owned_container(config,app)
                except Exception as error:
                    cleanup_error=str(error)
                try:os.killpg(child.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=10)
                    cleanup_error=cleanup_error or 'Guard child required forced termination'
        result=dict(utc=trial.utc(),failure=failure,cleanup_error=cleanup_error,
            exit_code=child.returncode if child else None,case=str(case),command=config['command'],
            session_scan_retries=retries,native_guard_and_kit_unchanged=True)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    return int(failure is not None or cleanup_error is not None)


if __name__=='__main__':raise SystemExit(main())
