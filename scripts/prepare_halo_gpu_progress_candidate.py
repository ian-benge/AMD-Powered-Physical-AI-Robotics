#!/usr/bin/env python3
"""Pin V26 with rotation-aware progress and the unchanged V25 GPU/motion policy."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109')
ALLOWED = {'src/rover_navigation/config/nav2.yaml', 'scripts/probe_demo_progress.py', 'scripts/probe_demo_progress.cpp'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overlay', required=True, type=Path)
    args = parser.parse_args()
    parent, app, runtime = BASE / 'application-v25', BASE / 'application-v26', BASE / 'runtime-v26'
    if app.exists() or runtime.exists():
        raise ValueError('Preserve all earlier candidates')
    config_path = parent / 'DEMO_CONFIGURATION.json'
    config = json.loads(config_path.read_text())
    release = json.loads((BASE / 'runtime-v25/ASSESSMENT_RELEASE.json').read_text())
    original = dict(config['files'])
    for name, expected in release['files'].items():
        if sha(Path(name)) != expected:
            raise ValueError('Parent release changed: ' + name)
    for name, expected in original.items():
        if sha(parent / name) != expected:
            raise ValueError('Parent source/install changed: ' + name)
    overlay = json.loads((args.overlay / 'OVERLAY.json').read_text())
    if set(overlay['files']) != ALLOWED:
        raise ValueError('Only navigation progress configuration and its native probe are allowed')
    for name, expected in overlay['files'].items():
        if sha(args.overlay / name) != expected:
            raise ValueError('Overlay changed: ' + name)
    app.mkdir()
    for tree in ('source', 'install', 'management', 'scripts', 'testdeps', 'demo-scenes'):
        shutil.copytree(parent / tree, app / tree, symlinks=True)
    for name in ALLOWED:
        shutil.copyfile(args.overlay / name, app / 'source' / name)
    install_configs = [name for name in original if name.startswith('install/') and name.endswith('/config/nav2.yaml')]
    if len(install_configs) != 1:
        raise ValueError('Expected one installed navigation configuration')
    shutil.copyfile(app / 'source/src/rover_navigation/config/nav2.yaml', app / install_configs[0])
    for name, expected in original.items():
        if name != install_configs[0] and name.startswith('install/') and sha(app / name) != expected:
            raise ValueError('Unexpected installed delta: ' + name)
    checks = app / 'checks'
    checks.mkdir()
    common = ['docker', 'run', '--rm', '--init', '--network', 'none', '--cpuset-cpus=0,1,2,3,4,5,6,7',
              '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp', '-e', 'PYTHONDONTWRITEBYTECODE=1',
              '-e', 'OMP_NUM_THREADS=2', '-e', 'OPENBLAS_NUM_THREADS=2', '-e', 'ROS_DOMAIN_ID=88',
              '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/work/source/src/rover_navigation/config/fastdds.xml',
              '-v', str(app) + ':/work', '-w', '/work']
    def run(name, command, extra=()):
        print('CHECK_START ' + name, flush=True)
        with (checks / (name + '.log')).open('xb') as stream:
            subprocess.run(common + list(extra) + [config['image'], 'bash', '-c',
                'source /opt/ros/jazzy/setup.bash && ' + command], stdout=stream,
                stderr=subprocess.STDOUT, check=True, timeout=600)
        print('CHECK_PASS ' + name, flush=True)
    run('progress-configuration', "python3 -c \"import yaml; p=yaml.safe_load(open('/work/source/src/rover_navigation/config/nav2.yaml'))['controller_server']['ros__parameters']['progress_checker']; assert p==dict(plugin='nav2_controller::PoseProgressChecker',required_movement_radius=.1,required_movement_angle=.5,movement_time_allowance=20.)\"")
    source_paths = ':'.join('/work/source/src/' + name for name in
                           ('rover_navigation', 'rover_perception', 'rover_training', 'mars_training_environment'))
    run('navigation-source', 'export PYTHONPATH=' + source_paths + ':/work/testdeps:$PYTHONPATH && '
        'exec python3 -m unittest discover -s source/src/rover_navigation/test -v')
    events = parent / 'navigation_sessions/restart-20261117T202038Z-8dbcab/demo-events.jsonl'
    run('actual-progress-replay', 'exec python3 -B /work/source/scripts/probe_demo_progress.py '
        '--events /evidence/demo-events.jsonl --output /work/checks/progress-replay.json',
        ['-v', str(events.parent) + ':/evidence:ro'])
    for entry in config['qualified_scenes']:
        for key in ('bundle', 'scene_file'):
            path = Path(entry[key])
            if path.is_relative_to(parent):
                entry[key] = str(app / path.relative_to(parent))
    config.update(project=str(app), parent_application=str(parent), parent_configuration_sha256=sha(config_path),
        progress_repair=dict(overlay=overlay, plugin='nav2_controller::PoseProgressChecker',
            required_movement_radius_m=.1, required_movement_angle_rad=.5, movement_time_allowance_seconds=20.,
            recorded_return_sha256=sha(events)), qualification_status='PROGRESS_FIX_NOT_LIVE_QUALIFIED')
    config['files'] = {str(p.relative_to(app)): sha(p)
        for tree in ('source', 'install', 'management', 'scripts', 'testdeps', 'demo-scenes')
        for p in sorted((app / tree).rglob('*')) if p.is_file() and not p.is_symlink() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    save(app / 'DEMO_CONFIGURATION.json', config)
    installed = 'source /work/install/setup.bash && '
    run('actual-ros', installed + 'exec python3 /work/source/scripts/probe_autonomous_demo.py')
    run('actual-tf2', installed + 'exec python3 /work/source/scripts/probe_demo_transforms.py')
    run('transport', installed + 'exec python3 /work/source/scripts/probe_demo_transport.py --rate-hz 10 --output /work/checks/transport.json')
    for index, scene in enumerate(config['qualified_scenes'], 1):
        run('launch-' + str(index), installed + 'exec python3 /work/source/scripts/check_autonomous_demo_launch.py --representative ' + str(index),
            ['-v', scene['bundle'] + ':/scenes:ro'])
    release.update(application=str(app), configuration=str(app / 'DEMO_CONFIGURATION.json'))
    release['files'] = dict(release['files'])
    release['files'][str(app / 'DEMO_CONFIGURATION.json')] = sha(app / 'DEMO_CONFIGURATION.json')
    save(app / 'ASSESSMENT_RELEASE.json', release)
    save(app / 'BUILD_RECEIPT.json', dict(installed_interface=True,
        only_installed_delta=install_configs[0], retained_code_unchanged=True,
        heartbeat_freshness_motion_thermal_limits_unchanged=True, no_motion_timeout_unchanged=True,
        progress_rotation_threshold_rad=.5, rendering_started=False, motion_requested=False))
    runtime.mkdir()
    for name in ('autonomous_demo_assess.py', 'start_autonomous_demo.sh', 'demo_exterior_view.py', 'view_autonomous_demo.sh'):
        target = runtime / name
        shutil.copyfile(BASE / 'runtime-v25' / name, target)
        release['files'][str(target)] = sha(target)
    save(runtime / 'ASSESSMENT_RELEASE.json', release)
    subprocess.run(['python3', '-B', str(runtime / 'autonomous_demo_assess.py'), '--check'], check=True)
    for name, expected in original.items():
        if sha(parent / name) != expected:
            raise ValueError('Rollback changed: ' + name)
    print('PROGRESS_REPAIR_CANDIDATE_READY: ' + str(runtime), flush=True)


if __name__ == '__main__':
    main()
