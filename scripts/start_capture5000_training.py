"""Train a separate candidate with the existing protected monitoring lifecycle."""
import argparse
import datetime as dt
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from capture_training_dataset import sha, save
from capture_training_worker import atomic

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')
HERE = Path(__file__).resolve().parent
CONTINUOUS = BASE/'bulk10000-continuous-20261107'
CONTROL = BASE/'bulk10000-collection-20261106'
PYTHON = '/usr/bin/python3'
RECIPE = dict(epochs=100,patience=20,imgsz=960,batch=4,seed=0,device=0,workers=2,
    rect=False,amp=False,compile=False,channels_last=False,cache=False,plots=True,
    optimizer='AdamW',lr0=.00125,weight_decay=.0005,deterministic=True,save=True,save_period=1)


def verify_release():
    value = json.loads((HERE/'TRAINING_RELEASE.json').read_text())
    if value.get('version') != 1 or value.get('recipe') != RECIPE:
        raise RuntimeError('Training release contract changed')
    for name, expected in value['files'].items():
        path = Path(name) if Path(name).is_absolute() else HERE/name
        if sha(path.resolve()) != expected:
            raise RuntimeError('Pinned training input changed: '+name)
    return value


def load_native():
    spec = importlib.util.spec_from_file_location('qualified_native_collection',CONTINUOUS/'start_bulk10000_continuous.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    module.verify_release()
    return module


def owned_processes(token, uid):
    found = []
    needle = ('ROVER_TRAINING_TOKEN='+token).encode()
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            fields = (proc/'stat').read_text().rsplit(')',1)[1].split()
            if proc.stat().st_uid == uid and fields[0] != 'Z' and needle in (proc/'environ').read_bytes().split(b'\0'):
                found.append(dict(pid=int(proc.name),start_ticks=int(fields[19]),uid=uid))
        except (FileNotFoundError,PermissionError,ProcessLookupError): pass
    return found


def cleanup_owned(token, uid):
    actions = []
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGKILL]:
        for item in owned_processes(token,uid):
            try:
                fd = os.pidfd_open(item['pid'])
            except ProcessLookupError: continue
            try:
                if item not in owned_processes(token,uid): continue
                signal.pidfd_send_signal(fd,sig)
                actions.append(dict(**item,signal=int(sig)))
            except ProcessLookupError: pass
            finally: os.close(fd)
        until = time.monotonic()+3
        while owned_processes(token,uid) and time.monotonic()<until: time.sleep(.05)
        if not owned_processes(token,uid): return dict(actions=actions,remaining=[])
    raise RuntimeError('Training processes survived bounded owned-token cleanup')


def process_identity(pid):
    proc = Path('/proc')/str(pid)
    fields = (proc/'stat').read_text().rsplit(')',1)[1].split()
    environment = [] if fields[0]=='Z' else (proc/'environ').read_bytes().split(b'\0')
    tokens = [row.split(b'=',1)[1].decode() for row in environment if row.startswith(b'ROVER_TRAINING_TOKEN=')]
    return dict(pid=pid,start_ticks=int(fields[19]),uid=proc.stat().st_uid,ppid=int(fields[1]),state=fields[0],
        token=tokens[0] if len(tokens)==1 else None)


def validate_compute_owners(owners, worker, inspect=process_identity):
    """Bind sampled owners to verified identities, including an already exited owner.

    Native capture samples precede this scan. A known dead PID cannot currently
    own a KFD descriptor; a live or reused PID must still pass the full identity
    and ancestry checks. Never infer a first-seen dead client's identity.
    """
    if not owners: return
    if worker is None: raise RuntimeError('Unexpected compute owner before training')
    keys=['pid','start_ticks','uid','ppid','token']
    known=worker.setdefault('_verified_processes',{worker['pid']:{key:worker[key] for key in keys}})
    retired=[]

    def observed(pid):
        previous=known.get(pid)
        try: current=inspect(pid)
        except (FileNotFoundError,ProcessLookupError):
            if previous is None:raise RuntimeError('Unidentified exited compute client')
            retired.append(pid);return dict(previous,retired=True)
        if current.get('state')=='Z':
            if previous is None or any(current[key]!=previous[key] for key in ['pid','start_ticks','uid']):
                raise RuntimeError('Exited compute client identity is unknown or PID was reused')
            retired.append(pid);return dict(previous,retired=True)
        return current

    root = observed(worker['pid'])
    if any(root[key] != worker[key] for key in ['pid','start_ticks','uid','token']):
        raise RuntimeError('Training worker identity changed')
    for pid in owners:
        current = observed(pid); seen = set();verified=[]
        if current.get('retired'):continue
        while True:
            if current['uid'] != worker['uid'] or current['token'] != worker['token']:
                raise RuntimeError('Unidentified compute client during training')
            verified.append({key:current[key] for key in keys})
            if current['pid'] == worker['pid']:
                if current['start_ticks'] != worker['start_ticks']:
                    raise RuntimeError('Training worker PID was reused')
                break
            if current['pid'] in seen or current['ppid'] <= 1:
                raise RuntimeError('Compute client is not a training descendant')
            seen.add(current['pid']);current=observed(current['ppid'])
        known.update({item['pid']:item for item in verified})
    worker['_retired_sampled_pids']=sorted(set(retired))


def training_live_check(starter, trial, live_guard, case, worker, log):
    """Retain native capture, sensor, fan and temperature checks with training identity."""
    class TrainingCapture:
        capture_guard = staticmethod(trial.capture_guard)
        utc = staticmethod(trial.utc)

        @staticmethod
        def control_health(selected):
            rows=trial.capture_state.read_health(selected,trial.policy.EXPECTED_BOOT,dt.datetime.now(dt.timezone.utc))
            trial.policy.health_guard(rows[-1],dt.datetime.now(dt.timezone.utc))
            owners=trial.policy.kfd_pids(rows[-1]['kfd_owners'])
            validate_compute_owners(owners,worker)
            return rows

    if worker is None:
        starter.retry_session_scan(lambda:trial.guard(case))
    result=starter.retry_session_scan(lambda:live_guard(TrainingCapture,case))
    with Path(log).open('a') as stream:
        stream.write(json.dumps(dict(**result,worker_pid=worker['pid'] if worker else None,
            training_compute_identity_checked=True,
            retired_previously_verified_sampled_pids=worker.get('_retired_sampled_pids',[]) if worker else []))+'\n')


def worker_configuration(release, work, deadline, probe=False):
    progress_path = HERE/'TRAINING_PROGRESS.json'
    progress = json.loads(progress_path.read_text()) if progress_path.exists() else {}
    resume = progress.get('checkpoint')
    if resume and progress.get('dataset_manifest_sha256') != release['dataset_manifest_sha256']:
        raise RuntimeError('Resume dataset differs from the frozen training snapshot')
    return dict(training_root=str(HERE),work=str(work),dataset=release['dataset'],
        dataset_manifest_sha256=release['dataset_manifest_sha256'],base_model=release['base_model'],
        base_model_sha256=release['base_model_sha256'],recipe=RECIPE,
        resume=resume,deadline_monotonic=deadline,probe=probe)


def execute_worker(release, work, check, child_options, interrupted=lambda:False, probe=False):
    work = Path(work)
    deadline = time.monotonic()+(600 if probe else 5400)
    config = worker_configuration(release,work,deadline,probe)
    save(work/'WORKER_CONFIGURATION.json',config)
    token = str(uuid.uuid4())
    environment = dict(child_options.get('env',os.environ),PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',
        OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',YOLO_CONFIG_DIR=str(HERE/'ultralytics-settings'),
        YOLO_OFFLINE='true',WANDB_MODE='disabled',ROVER_TRAINING_TOKEN=token)
    options = dict(child_options,env=environment)
    child = None
    try:
        check(None)
        with (work/'training.log').open('xb') as log:
            child = subprocess.Popen([release['training_python'],'-B',str(HERE/'capture_training_worker.py'),
                str(work/'WORKER_CONFIGURATION.json')],stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True,**options)
            identity=process_identity(child.pid)
            if identity['token']!=token or identity['uid']!=child_options.get('user',os.getuid()):
                raise RuntimeError('Launched worker lacks its expected user/token identity')
            save(work/'TRAINING_WORKER_IDENTITY.json',
                {key:identity[key] for key in ['pid','start_ticks','uid','ppid']})
            next_check = 0.
            while child.poll() is None:
                if interrupted(): raise KeyboardInterrupt
                if time.monotonic()>deadline+60: raise RuntimeError('Training worker exceeded its monitored budget')
                if time.monotonic()>=next_check:
                    check(identity); next_check=time.monotonic()+2
                time.sleep(.5)
        if child.returncode != 0:
            raise RuntimeError('Training worker failed; preserve '+str(work/'training.log'))
        result = json.loads((work/'WORKER_RESULT.json').read_text())
        if result['stage'] not in ['GPU_TRAINING_PROBE_PASS','PAUSED_FOR_MONITORING','TRAINING_COMPLETE_PENDING_EVALUATION']:
            raise RuntimeError('Worker lacks a recognized completion record')
        return result
    finally:
        uid = child_options.get('user',os.getuid())
        if child is not None:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try: child.wait(timeout=30)
                except subprocess.TimeoutExpired: pass
            clean = cleanup_owned(token,uid)
            if child.poll() is None: child.wait(timeout=10)
            save(work/'PROCESS_CLEANUP.json',clean)


def monitored_training(engine, release, max_windows=50):
    previous = 0
    if (HERE/'TRAINING_PROGRESS.json').exists():
        previous = json.loads((HERE/'TRAINING_PROGRESS.json').read_text())['completed_epochs']
    for index in range(1,max_windows+1):
        window = engine.open_window(index)
        error = None; result = None
        try:
            result = engine.train_window(window,release)
            if result.get('completed_epochs',0) <= previous:
                raise RuntimeError('Training made no checkpointed progress; automatic renewal refused')
        except BaseException as failure: error = failure
        closed = engine.close_window(window,reason='training_failure' if error else result['stage'])
        if error is not None: raise error
        if not closed: raise RuntimeError('Protected training closeout failed; automatic renewal refused')
        if result['stage'] == 'TRAINING_COMPLETE_PENDING_EVALUATION': return result
        if result['stage'] != 'PAUSED_FOR_MONITORING': raise RuntimeError('Unexpected worker exit; no automatic retry')
        previous = result['completed_epochs']
    raise RuntimeError('Bounded training monitoring-window limit reached; checkpoints preserved')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--run',action='store_true');g.add_argument('--check',action='store_true')
    g.add_argument('--status',action='store_true');g.add_argument('--probe',action='store_true')
    args=p.parse_args()
    if args.status:
        out={}
        for name in ['TRAINING_STATUS.json','TRAINING_PROGRESS.json','BATCH_PROGRESS.json']:
            if (HERE/name).exists():out[name]=json.loads((HERE/name).read_text())
        print(json.dumps(out,indent=2));return 0
    release=verify_release();native=load_native()
    if args.check:
        print('CAPTURE5000_TRAINING_STATIC_CHECK_PASS');return 0
    if args.probe:
        sys.path.insert(0,str(CONTROL))
        import start_bulk10000_collection as starter
        import pwd
        view=native.NativeEngine.__new__(native.NativeEngine);view.account=pwd.getpwnam('ian')
        environment=view.desktop_environment()
        kit,case,trial,live_guard=starter.session()
        work=Path(tempfile.mkdtemp(prefix='gpu-probe-',dir=HERE))
        def check(worker):
            training_live_check(starter,trial,live_guard,case,worker,work/'training-guard.jsonl')
            subprocess.run([PYTHON,str(CONTROL/'desktop_render_guard.py'),'--probe'],env=environment,
                check=True,capture_output=True,timeout=7)
        print(json.dumps(execute_worker(release,work,check,dict(env=environment),probe=True)),flush=True)
        return 0
    if os.geteuid()!=0:
        p.error('Use local sudo from the Halo desktop for protected training monitoring')
    import fcntl,pwd
    account=pwd.getpwnam('ian')
    if os.environ.get('SUDO_UID')!=str(account.pw_uid):raise RuntimeError('Launch with local sudo as ian')
    status_path=HERE/'TRAINING_STATUS.json'
    if status_path.exists():
        previous=json.loads(status_path.read_text())
        if previous.get('stage')=='TRAINING_COMPLETE_PENDING_EVALUATION':
            if sha(Path(previous['best_model']))!=previous['best_model_sha256']:
                raise RuntimeError('Completed model changed')
            print('TRAINING_ALREADY_COMPLETE_PENDING_EVALUATION');return 0
    class TrainingEngine(native.NativeEngine):
        def event(self,stage,**values):
            value=dict(utc=native.utc(),stage=stage,run=str(self.output),**values)
            with (self.output/'events.jsonl').open('a') as stream:stream.write(json.dumps(value)+'\n')
            atomic(HERE/'TRAINING_STATUS.json',value)
            print(stage,json.dumps(values),flush=True)

        def command(self,arguments,protected=False,timeout=180):
            verify_release()
            return super().command(arguments,protected,timeout)

        def train_window(self,window,release):
            sys.path.insert(0,str(CONTROL))
            import start_bulk10000_collection as starter
            trial,live_guard,_=starter.load_session_modules(window['kit'])
            work=window['folder']/'training';work.mkdir()
            os.chown(work,account.pw_uid,account.pw_gid);work.chmod(0o755)
            self.event('TRAINING',window=window['index'],case=str(window['case']),log=str(work/'training.log'))
            next_display_check=0.
            last_epoch=None
            def check(worker):
                nonlocal next_display_check,last_epoch
                if self.interrupted:raise KeyboardInterrupt
                self.checkpoint(window);self.display_temperatures(window)
                training_live_check(starter,trial,live_guard,window['case'],worker,work/'training-guard.jsonl')
                if time.monotonic()>=next_display_check:
                    self.command([PYTHON,str(CONTROL/'desktop_render_guard.py'),'--probe'],timeout=7)
                    next_display_check=time.monotonic()+10
                progress_path=HERE/'TRAINING_PROGRESS.json'
                if progress_path.exists():
                    progress=json.loads(progress_path.read_text())
                    if progress['completed_epochs']!=last_epoch:
                        last_epoch=progress['completed_epochs']
                        self.event('TRAINING',window=window['index'],case=str(window['case']),
                            completed_epochs=last_epoch,log=str(work/'training.log'))
            return execute_worker(release,work,check,self.as_user(),lambda:self.interrupted)

    with (HERE/'training.lock').open('a') as lock,(CONTINUOUS/'continuous.lock').open('r') as collection_lock, \
            (CONTROL/'collection.lock').open('a') as manual_lock:
        for held in [lock,collection_lock,manual_lock]:fcntl.flock(held,fcntl.LOCK_EX|fcntl.LOCK_NB)
        output=Path(tempfile.mkdtemp(prefix='training-monitors-',dir=HERE));output.chmod(0o755)
        engine=TrainingEngine(native.verify_release(),Path(release['source_collection']),output,account)
        save(output/'RUN_IDENTITY.json',dict(utc=native.utc(),release_sha256=sha(HERE/'TRAINING_RELEASE.json'),
            dataset_manifest_sha256=release['dataset_manifest_sha256'],recipe=RECIPE,
            training_uid=account.pw_uid,privileged_scope='Native monitoring/module setup/protected closeout only'))
        def interrupt(*_):engine.interrupted=True
        for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,interrupt)
        try:
            result=monitored_training(engine,release)
            engine.event(result['stage'],**{k:v for k,v in result.items() if k!='stage'})
            return 0
        except BaseException as error:
            engine.event('STOPPED_REVIEW_REQUIRED',error=type(error).__name__+': '+str(error),
                monitoring_left_open=engine.active is not None)
            return 130 if isinstance(error,KeyboardInterrupt) else 1


if __name__=='__main__':raise SystemExit(main())
