#!/usr/bin/env python3
"""Build a separate demo candidate on Halo; never launches rendering or motion."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


IMAGE='sha256:0b607598f65a9355057a28a8e36dedcab72baf33ee8955882fd43460314477fa'
BASE=Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')
TRAIN=BASE/'capture5000-training-20261108'


def sha(path):
    if path.is_symlink() or not path.is_file():raise ValueError('Expected regular pinned file: '+str(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')


def native():
    spec=importlib.util.spec_from_file_location('demo_native','/home/ian/capstone-training/rover-sim/manage.py')
    manager=importlib.util.module_from_spec(spec);spec.loader.exec_module(manager)
    _,kit,case=manager.selected();manager.acknowledge(kit,case)
    trial=manager.kit_import(kit,'trial')
    return trial,case


def check_health(trial,case):
    trial.guard(case)
    h=trial.control_health(case)[-1]
    for name,meta in h['sensor_metadata'].items():
        if meta.get('label')=='Tctl' and float(h['sensors'][name])/1000>=87:
            raise RuntimeError('Demo qualification stops early at 87 C')


def monitored(argv,log,trial,case,cwd=None):
    check_health(trial,case)
    with log.open('x') as f:
        child=subprocess.Popen(argv,stdout=f,stderr=subprocess.STDOUT,cwd=cwd)
        try:
            while child.poll() is None:
                check_health(trial,case);time.sleep(2)
            if child.returncode:raise RuntimeError('Check/build failed; preserve '+str(log))
        finally:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:child.kill();child.wait()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',required=True,type=Path);p.add_argument('--application',required=True,type=Path)
    p.add_argument('--archive',required=True,type=Path);p.add_argument('--archive-sha256',required=True)
    a=p.parse_args();source=a.source.resolve();app=a.application.resolve()
    if app.parent!=BASE/'autonomous-demo-20261109' or app.exists():raise ValueError('Use a new demo application directory')
    if sha(a.archive)!=a.archive_sha256:raise ValueError('Source archive changed')
    trial,case=native();check_health(trial,case)
    previous=json.loads((TRAIN/'ASSESSMENT_RELEASE.json').read_text());old=Path(previous['application'])
    config=json.loads(Path(previous['configuration']).read_text())
    for rel,expected in config['files'].items():
        if sha(old/rel)!=expected:raise ValueError('Retained application changed: '+rel)
    app.mkdir();shutil.copytree(source,app/'source')
    for name in ('management','testdeps','scripts'):shutil.copytree(old/name,app/name)
    # Preserve the qualified CPU/thread and mount settings; adapt only its DDS
    # file path to this candidate's separate source tree. Never fall back silently.
    overrides={}
    profile=app/'source/src/rover_navigation/config/fastdds.xml'
    if sha(profile)!=sha(old/'src/rover_navigation/config/fastdds.xml'):
        raise ValueError('Qualified DDS transport contents changed')
    for rel in ('scripts/halo_navigation.sh','source/scripts/halo_navigation.sh'):
        original=(old/rel).read_text()
        corrected=original.replace('/work/src/rover_navigation/config/fastdds.xml','/work/source/src/rover_navigation/config/fastdds.xml')
        if '/work/source/src/rover_navigation/config/fastdds.xml' not in corrected:
            raise ValueError('Unexpected retained launcher transport path')
        (app/rel).write_text(corrected)
        overrides[rel]=dict(source=str(old/rel),parent_sha256=sha(old/rel),sha256=sha(app/rel),
            dds_profile_sha256=sha(profile),delta='DDS path only; resource and renderer settings retained')
    common=['docker','run','--rm','--init','--network','none','--cpuset-cpus=0,1,2,3,4,5,6,7',
        '--user',f'{os.getuid()}:{os.getgid()}','-e','HOME=/tmp','-e','PYTHONDONTWRITEBYTECODE=1',
        '-e','OMP_NUM_THREADS=2','-e','OPENBLAS_NUM_THREADS=2',
        '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/work/source/src/rover_navigation/config/fastdds.xml',
        '-v',str(app)+':/work','-w','/work',IMAGE,'bash','-c']
    (app/'checks').mkdir()
    for package in ('rover_navigation','rover_perception','rover_training','mars_training_environment'):
        shell='source /opt/ros/jazzy/setup.bash && export PYTHONPATH='+':'.join('/work/source/src/'+n for n in
            ('rover_navigation','rover_perception','rover_training','mars_training_environment'))+':/work/testdeps:$PYTHONPATH && exec python3 -m unittest discover -s source/src/'+package+'/test -v'
        monitored(common+[shell],app/'checks'/f'{package}.log',trial,case)
    shell='source /opt/ros/jazzy/setup.bash && exec colcon build --base-paths source/src --packages-up-to rover_navigation --executor sequential --event-handlers console_direct+'
    monitored(common+[shell],app/'checks/build.log',trial,case)
    shell='source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && python3 -c "from rover_navigation_interfaces.srv import DemoCommand; from rover_navigation.demo_node import DemoMission; assert DemoCommand.Request().sequence == 0; print(\'DEMO_TYPED_INTERFACE_IMPORT_PASS\')"'
    monitored(common+[shell],app/'checks/installed-import.log',trial,case)
    shell='source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && exec python3 /work/source/scripts/probe_autonomous_demo.py'
    monitored(common+[shell],app/'checks/actual-ros-probe.log',trial,case)
    shell='source /opt/ros/jazzy/setup.bash && exec python3 /work/source/scripts/probe_demo_transport.py --output /work/checks/transport-throughput.json'
    monitored(common+[shell],app/'checks/transport-throughput.log',trial,case)
    config.update(project=str(app),source_archive_sha256=a.archive_sha256,source_archive_name=a.archive.name,
        phase='autonomous-demo',qualification_status='IMPLEMENTED_NOT_LIVE_QUALIFIED',
        demo_mode=True,demo_profile='rehearsal',qualification_speed_mps=.05,qualification_stop_c=87,
        retained_launch_overrides=overrides)
    config['model_assessment']['display_profile'].update(gazebo_gui=False,
        exterior_view='bounded_stationary_inspection_only',sensor_headless=False)
    config['files']={str(f.relative_to(app)):sha(f) for tree in ('source','install','management','scripts','testdeps')
        for f in sorted((app/tree).rglob('*')) if f.is_file() and not f.is_symlink() and '__pycache__' not in f.parts and f.suffix!='.pyc'}
    configuration=app/'DEMO_CONFIGURATION.json';save(configuration,config)
    release=dict(previous,application=str(app),configuration=str(configuration))
    release['files']=dict(previous['files']);release['files'][str(configuration)]=sha(configuration)
    save(app/'ASSESSMENT_RELEASE.json',release)
    save(app/'BUILD_RECEIPT.json',dict(image=IMAGE,source_sha256=a.archive_sha256,case=str(case),
        installed_interface=True,tests='See checks/*.log; GPU parity remains a separate live gate',
        qualification_status='NOT_LIVE_QUALIFIED',overrides=overrides))
    print('DEMO_CANDIDATE_BUILD_PASS: '+str(app),flush=True)


if __name__=='__main__':main()
