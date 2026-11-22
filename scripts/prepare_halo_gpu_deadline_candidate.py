#!/usr/bin/env python3
"""Pin a diagnostic sibling without changing heartbeat or freshness thresholds."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109')
ALLOWED = {'src/rover_navigation/rover_navigation/demo_node.py', 'src/rover_navigation/test/test_demo_adapter.py'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overlay', type=Path, required=True)
    args = parser.parse_args()
    parent, app, runtime = BASE / 'application-v24', BASE / 'application-v25', BASE / 'runtime-v25'
    if app.exists() or runtime.exists():
        raise ValueError('Preserve previous candidates')
    configuration = parent / 'DEMO_CONFIGURATION.json'
    config = json.loads(configuration.read_text())
    release = json.loads((BASE / 'runtime-v24/ASSESSMENT_RELEASE.json').read_text())
    for name, expected in release['files'].items():
        if sha(Path(name)) != expected:
            raise ValueError('Parent release changed: ' + name)
    for name, expected in config['files'].items():
        if sha(parent / name) != expected:
            raise ValueError('Parent source changed: ' + name)
    overlay = json.loads((args.overlay / 'OVERLAY.json').read_text())
    if set(overlay['files']) != ALLOWED:
        raise ValueError('Only deadline diagnostics and their adapter regressions are allowed')
    for name, expected in overlay['files'].items():
        if sha(args.overlay / name) != expected:
            raise ValueError('Overlay changed: ' + name)
    app.mkdir()
    for tree in ('source', 'install', 'management', 'scripts', 'testdeps', 'demo-scenes'):
        shutil.copytree(parent / tree, app / tree, symlinks=True)
    for name in ALLOWED:
        shutil.copyfile(args.overlay / name, app / 'source' / name)
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
    source_paths = ':'.join('/work/source/src/' + name for name in
                           ('rover_navigation', 'rover_perception', 'rover_training', 'mars_training_environment'))
    run('navigation-source', 'export PYTHONPATH=' + source_paths + ':/work/testdeps:$PYTHONPATH && '
        'exec python3 -m unittest discover -s source/src/rover_navigation/test -v')
    installed = 'source /work/install/setup.bash && '
    run('build', installed + 'exec colcon build --base-paths source/src --packages-select rover_navigation '
        '--executor sequential --event-handlers console_direct+')
    for name, expected in config['files'].items():
        if name.startswith('install/') and not name.endswith('/rover_navigation/demo_node.py') and sha(app / name) != expected:
            raise ValueError('Unexpected installed change: ' + name)
    for entry in config['qualified_scenes']:
        for key in ('bundle', 'scene_file'):
            path = Path(entry[key])
            if path.is_relative_to(parent):
                entry[key] = str(app / path.relative_to(parent))
    config.update(project=str(app), parent_application=str(parent), parent_configuration_sha256=sha(configuration),
                  deadline_diagnostics=overlay, qualification_status='DIAGNOSTICS_NOT_LIVE_QUALIFIED')
    config['files'] = {str(p.relative_to(app)): sha(p)
        for tree in ('source', 'install', 'management', 'scripts', 'testdeps', 'demo-scenes')
        for p in sorted((app / tree).rglob('*')) if p.is_file() and not p.is_symlink() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    save(app / 'DEMO_CONFIGURATION.json', config)
    run('actual-ros', installed + 'exec python3 /work/source/scripts/probe_autonomous_demo.py')
    run('actual-tf2', installed + 'exec python3 /work/source/scripts/probe_demo_transforms.py')
    run('transport', installed + 'exec python3 /work/source/scripts/probe_demo_transport.py --rate-hz 10 --output /work/checks/transport.json')
    for i, scene in enumerate(config['qualified_scenes'], 1):
        run('launch-' + str(i), installed + 'exec python3 /work/source/scripts/check_autonomous_demo_launch.py --representative ' + str(i),
            ['-v', scene['bundle'] + ':/scenes:ro'])
    release.update(application=str(app), configuration=str(app / 'DEMO_CONFIGURATION.json'))
    release['files'] = dict(release['files'])
    release['files'][str(app / 'DEMO_CONFIGURATION.json')] = sha(app / 'DEMO_CONFIGURATION.json')
    save(app / 'ASSESSMENT_RELEASE.json', release)
    save(app / 'BUILD_RECEIPT.json', dict(installed_interface=True, retained_components_unchanged=True,
        only_installed_delta='rover_navigation/demo_node.py: stop-time diagnostics',
        heartbeat_limit_unchanged=True, freshness_limits_unchanged=True, rendering_started=False, motion_requested=False))
    runtime.mkdir()
    for name in ('autonomous_demo_assess.py', 'start_autonomous_demo.sh', 'demo_exterior_view.py', 'view_autonomous_demo.sh'):
        target = runtime / name
        shutil.copyfile(BASE / 'runtime-v24' / name, target)
        release['files'][str(target)] = sha(target)
    save(runtime / 'ASSESSMENT_RELEASE.json', release)
    subprocess.run(['python3', '-B', str(runtime / 'autonomous_demo_assess.py'), '--check'], check=True)
    for name, expected in json.loads(configuration.read_text())['files'].items():
        if sha(parent / name) != expected:
            raise ValueError('Rollback changed: ' + name)
    print('DEADLINE_DIAGNOSTIC_CANDIDATE_READY: ' + str(runtime), flush=True)


if __name__ == '__main__':
    main()
