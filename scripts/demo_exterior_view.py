#!/usr/bin/env python3
"""Bounded existing-Gazebo GUI for stationary inspection; never moves the rover."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from urllib.request import urlopen
import uuid


def stationary(status):
    guard=status.get('status',{});command=guard.get('command',[])
    # A viewer may inspect a stopped faulted rover. It never acknowledges or
    # clears a latched motion fault, arms a mode, or publishes motion.
    return (guard.get('mode')=='stopped' and
            len(command)==2 and all(isinstance(v,(int,float)) and math.isfinite(v) and v==0 for v in command) and
            0<=guard.get('age_seconds',math.inf)<=.5 and
            not (status.get('demo_status') or {}).get('goal_active',False) and
            status.get('localization',{}).get('healthy') is True and
            status.get('camera_health',{}).get('available') is True and
            status.get('camera_health',{}).get('age_seconds',math.inf)<=.6)


def run_gui(argv,seconds,stopped):
    """Use the tested pidfd cleanup loop with a GUI-only inherited token.

    The viewer shares Gazebo's partition with the sensor server. Cleanup must
    therefore use its own token, never kill other processes in that partition.
    """
    from mars_training_environment.process_cleanup import CaptureProcessScope,process_identity
    tag=uuid.uuid4().hex
    marker=('ROVER_DEMO_GUI_TOKEN='+tag).encode()
    class Scope(CaptureProcessScope):
        def __init__(self,process):
            self.partition='exterior-'+tag
            self.leader=dict(pid=process.pid,uid=os.getuid(),start_ticks=None)
        def processes(self):
            owned=[]
            for directory in Path('/proc').glob('[0-9]*'):
                try:
                    identity=process_identity(int(directory.name))
                    if identity['uid']==os.getuid() and identity['state']!='Z' and marker in (directory/'environ').read_bytes().split(b'\0'):
                        owned.append(identity)
                except (OSError,PermissionError):pass
            return owned
        def signal(self,identity,sig):
            try:fd=os.pidfd_open(identity['pid'])
            except ProcessLookupError:return
            try:
                current=process_identity(identity['pid'])
                if (current['start_ticks'],current['uid'])!=(identity['start_ticks'],identity['uid']):
                    raise RuntimeError('Viewer PID identity changed')
                if marker not in Path(f"/proc/{identity['pid']}/environ").read_bytes().split(b'\0'):
                    raise RuntimeError('Refusing cleanup outside the owned GUI token')
                signal.pidfd_send_signal(fd,sig)
            except (FileNotFoundError,ProcessLookupError):pass
            finally:os.close(fd)
    child=None;scope=None
    try:
        if stopped.is_set():return 'cancelled_before_launch'
        child=subprocess.Popen(argv,start_new_session=True,env=dict(os.environ,ROVER_DEMO_GUI_TOKEN=tag))
        scope=Scope(child)
        deadline=time.monotonic()+seconds
        while child.poll() is None and not stopped.is_set() and time.monotonic()<deadline:
            stopped.wait(.05)
        if child.poll() is not None:
            if child.returncode:raise RuntimeError('Gazebo GUI exited: '+str(child.returncode))
            return 'window_closed'
        return 'operator_stop' if stopped.is_set() else 'bounded_timeout'
    finally:
        if scope is not None:scope.cleanup(stage_seconds=.5)
        if child is not None:child.wait(timeout=5)


def start_ticks(pid):
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]


def token_path(token):
    if not re.fullmatch('[a-f0-9]{32}',token):raise ValueError('Invalid viewer token')
    return Path('/tmp')/('demo-exterior-'+token+'.json')


def stop_worker(token):
    path=token_path(token)
    until=time.monotonic()+3
    while not path.exists() and time.monotonic()<until:time.sleep(.05)
    if not path.exists():raise RuntimeError('Viewer worker did not publish its identity')
    if path.is_symlink() or path.stat().st_uid!=os.getuid():raise RuntimeError('Unowned viewer record')
    value=json.loads(path.read_text());pid=value['pid']
    try:
        proc=Path('/proc')/str(pid)
        if proc.stat().st_uid!=os.getuid() or start_ticks(pid)!=value['start_ticks']:
            raise RuntimeError('Viewer process identity changed')
        command=(proc/'cmdline').read_bytes().split(b'\0')
        environment=(proc/'environ').read_bytes().split(b'\0')
        if (b'--worker' not in command or token.encode() not in command or
                ('ROVER_DEMO_VIEW_TOKEN='+token).encode() not in environment):
            raise RuntimeError('Viewer launch token mismatch')
        os.kill(pid,signal.SIGTERM)
    except FileNotFoundError:pass


def worker(token,seconds):
    import fcntl
    if os.environ.get('ROVER_DEMO_VIEW_TOKEN')!=token:raise RuntimeError('Missing worker identity')
    stopped=threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.set())
    path=token_path(token)
    fd=os.open('/tmp/demo-exterior.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    try:
        if os.fstat(fd).st_uid!=os.getuid():raise RuntimeError('Unowned viewer lock')
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with path.open('x') as stream:
            os.chmod(path,0o600)
            json.dump(dict(pid=os.getpid(),start_ticks=start_ticks(os.getpid())),stream)
        print('EXTERIOR_VIEW_COMPLETE: '+run_gui(['gz','sim','-g'],seconds,stopped),flush=True)
    finally:os.close(fd)


def worker_command(identity,token,*arguments):
    # docker exec inherits container configuration, not exports made by the
    # original launch shell. Prepare the same qualified ROS/installation paths.
    return ['docker','exec','-e','ROVER_DEMO_VIEW_TOKEN='+token,identity,'bash','-c',
            'source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && exec python3 -B "$@"',
            'demo-exterior','/work/source/scripts/demo_exterior_view.py',*arguments]


def host(args):
    root=Path(__file__).resolve().parent
    subprocess.run([sys.executable,'-B',str(root/'autonomous_demo_assess.py'),'--check'],check=True)
    release=json.loads((root/'ASSESSMENT_RELEASE.json').read_text());app=Path(release['application'])
    config=json.loads(Path(release['configuration']).read_text())
    sys.path.insert(0,str(app/'management'))
    import phase2_supervise_live as supervisor
    trial=supervisor.load_guard(args.kit)
    if not os.environ.get('DISPLAY') or not args.attended:raise RuntimeError('Use the attended desktop starter')
    value=json.loads(subprocess.check_output(['docker','container','inspect','rover-navigation'],text=True))[0]
    if not supervisor.belongs_to_candidate(value,config,app):raise RuntimeError('Container is not this candidate')
    identity=value['Id'];token=uuid.uuid4().hex
    record=Path(tempfile.mkdtemp(prefix='exterior-',dir=app/'navigation_sessions'))
    stopped=threading.Event();failure=None;child=None;cleanup=[]
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.set())
    def check():
        trial.guard(args.case);health=supervisor.live_guard(trial,args.case)
        if not math.isfinite(health['tctl_c']) or health['tctl_c']>=87:raise RuntimeError('Exterior check reached 87 C')
        with urlopen('http://127.0.0.1:8777/status',timeout=1) as response:status=json.load(response)
        if not stationary(status):raise RuntimeError('Exterior inspection requires stopped/disarmed rover and fresh camera')
        session=status.get('session','')
        if not re.fullmatch('[a-zA-Z0-9_-]+',session):raise RuntimeError('Missing current session')
        current=json.loads((app/'navigation_sessions'/session/'session.json').read_text())
        if current.get('gui')!='false':raise RuntimeError('Use this viewer only with the GUI-disabled profile')
        return session
    session=check()
    command=worker_command(identity,token,'--worker','--token',token,'--seconds',str(args.seconds))
    (record/'configuration.json').write_text(json.dumps(dict(container=identity,session=session,seconds=args.seconds,
        command=command,stationary_only=True),indent=2)+'\n')
    try:
        with (record/'viewer.log').open('xb') as log:
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+args.seconds+10
            while child.poll() is None and not stopped.is_set():
                if check()!=session:raise RuntimeError('Scene changed during exterior inspection')
                if time.monotonic()>deadline:raise RuntimeError('Exterior worker missed its bound')
                stopped.wait(.5)
            if child.poll() not in (None,0):raise RuntimeError('Exterior worker failed; inspect viewer.log')
    except BaseException as error:
        failure=type(error).__name__+': '+str(error)
        raise
    finally:
        if child is not None and child.poll() is None:
            result=subprocess.run(worker_command(identity,token,'--stop','--token',token),
                                  capture_output=True,text=True,timeout=10)
            if result.returncode:cleanup.append(result.stderr or result.stdout)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:cleanup.append('GUI worker remains bounded by its own deadline; review before continuing')
        (record/'result.json').write_text(json.dumps(dict(failure=failure,cleanup_errors=cleanup,
            worker_exit=child.poll() if child else None,operator_stop=stopped.is_set(),
            motion_requested=False,thermal_savings_verified=False),indent=2)+'\n')
        if cleanup and failure is None:raise RuntimeError('Exterior cleanup needs review: '+str(cleanup))
    print('EXTERIOR_INSPECTION_CLOSED: '+str(record),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    m=p.add_mutually_exclusive_group();m.add_argument('--worker',action='store_true');m.add_argument('--stop',action='store_true')
    p.add_argument('--token',default='');p.add_argument('--seconds',type=int,default=60)
    p.add_argument('--kit',type=Path);p.add_argument('--case',type=Path);p.add_argument('--attended',action='store_true')
    args=p.parse_args()
    if not 5<=args.seconds<=90:p.error('Exterior inspection must be bounded to 5..90 seconds')
    if args.worker:return worker(args.token,args.seconds)
    if args.stop:return stop_worker(args.token)
    if args.kit is None or args.case is None:p.error('Use view_autonomous_demo.sh')
    return host(args)


if __name__=='__main__':main()
