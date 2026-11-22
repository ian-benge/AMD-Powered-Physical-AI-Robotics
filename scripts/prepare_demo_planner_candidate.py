#!/usr/bin/env python3
"""Build a pinned planner-only sibling of the retained GUI-off demo candidate."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess


BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions/autonomous-demo-20261109')
ALLOWED = {
    'src/rover_navigation/rover_navigation/demo_core.py',
    'src/rover_navigation/rover_navigation/demo_node.py',
    'src/rover_navigation/test/test_demo_core.py',
    'src/rover_navigation/test/test_demo_adapter.py',
    'scripts/demo_exterior_view.py',
    'scripts/prepare_demo_planner_candidate.py',
}
RARE_FILES = {
    'src/mars_training_environment/config/mixed_reachable_v2_demo_rare.json',
    'src/mars_training_environment/mars_training_environment/common.py',
    'src/mars_training_environment/mars_training_environment/environment_v2.py',
    'src/mars_training_environment/mars_training_environment/reachability.py',
    'src/mars_training_environment/mars_training_environment/candidates.py',
    'src/mars_training_environment/mars_training_environment/generate.py',
    'src/mars_training_environment/test/test_environment_v2.py',
    'scripts/prepare_demo_rare_scenes.py',
    'scripts/test_demo_rare_scenes.py',
}
LOW_LOAD_FILES = {
    'scripts/autonomous_demo_assess.py',
    'scripts/view_autonomous_demo.sh',
    'scripts/check_autonomous_demo_launch.py',
    'scripts/probe_demo_transport.py',
    'scripts/probe_demo_transforms.py',
    'scripts/test_demo_exterior_view.py',
    'scripts/test_autonomous_demo_tools.py',
    'src/rover_navigation/launch/navigation.launch.py',
    'src/rover_navigation/test/test_launch.py',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overlay', type=Path, required=True)
    parser.add_argument('--parent-runtime', type=Path, required=True)
    parser.add_argument('--application', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--rare-sites', action='store_true')
    parser.add_argument('--low-load', action='store_true',help='Separate 10 Hz RGB-D candidate with ten-second exterior inspection')
    args = parser.parse_args()
    app, runtime = args.application.resolve(), args.runtime.resolve()
    if app.parent != BASE or runtime.parent != BASE or app.exists() or runtime.exists() or app == runtime:
        raise ValueError('Use new sibling application/runtime directories; preserve all earlier attempts')
    parent_release = json.loads((args.parent_runtime / 'ASSESSMENT_RELEASE.json').read_text())
    parent = Path(parent_release['application'])
    parent_configuration = Path(parent_release['configuration'])
    config = json.loads(parent_configuration.read_text())
    spec = importlib.util.spec_from_file_location('demo_builder', parent / 'source/scripts/prepare_autonomous_demo.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    for name, expected in parent_release['files'].items():
        if builder.sha(Path(name)) != expected:
            raise ValueError('Parent release changed: ' + name)
    for name, expected in config['files'].items():
        if builder.sha(parent / name) != expected:
            raise ValueError('Parent application changed: ' + name)
    overlay = args.overlay.resolve()
    manifest = json.loads((overlay / 'OVERLAY_SHA256.json').read_text())
    if args.rare_sites and args.low_load:raise ValueError('Qualify scene and workload changes in separate candidates')
    if set(manifest['files']) != ALLOWED | (RARE_FILES if args.rare_sites else set()) | (LOW_LOAD_FILES if args.low_load else set()):
        raise ValueError('Overlay must contain only the declared planner, tests and corrected viewer')
    for name, expected in manifest['files'].items():
        if builder.sha(overlay / name) != expected:
            raise ValueError('Overlay changed: ' + name)
    if (config['model_assessment']['camera_rate_hz'] != 15 or
            config['model_assessment']['display_profile']['gazebo_gui'] is not False):
        raise ValueError('Expected the retained 15 Hz GUI-off profile')
    trial, case = builder.native()
    builder.check_health(trial, case)
    app.mkdir()
    for tree in ('source', 'install', 'management', 'scripts', 'testdeps'):
        shutil.copytree(parent / tree, app / tree, symlinks=True)
    if not args.rare_sites and (parent/'demo-scenes').exists():
        # Copy rather than mutate the parent's pinned presentation scenes.
        shutil.copytree(parent/'demo-scenes',app/'demo-scenes')
        for entry in config['qualified_scenes']:
            for key in ('bundle','scene_file'):
                path=Path(entry[key])
                if path.is_relative_to(parent):entry[key]=str(app/path.relative_to(parent))
    if args.low_load:
        config['model_assessment']['camera_rate_hz']=10
        config['model_assessment']['display_profile'].update(
            camera_rate_changed=True,exterior_max_seconds=10,thermal_savings_verified=False)
        config['low_load_profile']=dict(camera_rate_hz=10,previous_camera_rate_hz=15,
            exterior_max_seconds=10,resolution_unchanged=True,geometry_unchanged=True,
            safety_policy_unchanged=True,qualification_status='NOT_LIVE_QUALIFIED')
    for name in manifest['files']:
        target = app / 'source' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(overlay / name, target)
    checks = app / 'checks'
    checks.mkdir()
    common = ['docker', 'run', '--rm', '--init', '--network', 'none',
        '--cpuset-cpus=0,1,2,3,4,5,6,7', '--user', f'{os.getuid()}:{os.getgid()}',
        '-e', 'HOME=/tmp', '-e', 'PYTHONDONTWRITEBYTECODE=1',
        '-e', 'OMP_NUM_THREADS=2', '-e', 'OPENBLAS_NUM_THREADS=2', '-e', 'ROS_DOMAIN_ID=88',
        '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/work/source/src/rover_navigation/config/fastdds.xml',
        '-v', str(app) + ':/work', '-w', '/work']
    prefix = 'source /opt/ros/jazzy/setup.bash && '
    def run(name, shell, extra=()):
        print('CHECK_START ' + name, flush=True)
        builder.monitored(common + list(extra) + [config['image'], 'bash', '-c', prefix + shell],
                          checks / (name + '.log'), trial, case)
        print('CHECK_PASS ' + name, flush=True)
    source_paths = ':'.join('/work/source/src/' + package for package in
                           ('rover_navigation', 'rover_perception', 'rover_training', 'mars_training_environment'))
    run('navigation-source', 'export PYTHONPATH=' + source_paths + ':/work/testdeps:$PYTHONPATH && '
        'exec python3 -m unittest discover -s source/src/rover_navigation/test -v')
    if args.low_load:
        run('low-load-tools','export PYTHONPATH=/work/source/scripts:' + source_paths + ':/work/testdeps:$PYTHONPATH && '
            'exec python3 -m unittest test_autonomous_demo_tools test_demo_exterior_view -v')
    if args.rare_sites:
        run('environment-source', 'export PYTHONPATH=' + source_paths + ':/work/testdeps:$PYTHONPATH && '
            'exec python3 -m unittest discover -s source/src/mars_training_environment/test -v')
        run('rare-tools', 'export PYTHONPATH=/work/source/scripts:' + source_paths + ':/work/testdeps:$PYTHONPATH && '
            'exec python3 -m unittest test_demo_rare_scenes -v')
    packages = 'mars_training_environment rover_navigation' if args.rare_sites else 'rover_navigation'
    run('build', 'source /work/install/setup.bash && exec colcon build --base-paths source/src '
        '--packages-select ' + packages + ' --executor sequential --event-handlers console_direct+')
    installed = 'source /work/install/setup.bash && '
    run('actual-ros', installed + 'exec python3 /work/source/scripts/probe_autonomous_demo.py')
    if args.low_load:run('actual-tf2',installed+'exec python3 /work/source/scripts/probe_demo_transforms.py')
    run('transport', installed + 'exec python3 /work/source/scripts/probe_demo_transport.py --output /work/checks/transport.json' +
        (' --rate-hz 10' if args.low_load else ''))
    # The guard, terrain and perception implementations must remain byte-identical.
    allowed_installed = ('/rover_navigation/demo_core.py', '/rover_navigation/demo_node.py')
    if args.low_load:allowed_installed+=('/launch/navigation.launch.py',)
    if args.rare_sites:
        allowed_installed += tuple('/mars_training_environment/' + name + '.py' for name in
            ('common', 'environment_v2', 'reachability', 'candidates', 'generate'))
    for name, expected in config['files'].items():
        if name.startswith('install/') and not name.endswith(allowed_installed):
            if args.rare_sites and name.startswith('install/mars_training_environment/') and name.endswith('/SOURCES.txt'):
                continue
            if builder.sha(app / name) != expected:
                raise ValueError('Unexpected installed delta: ' + name)
    config.update(project=str(app), parent_application=str(parent),
        parent_configuration_sha256=builder.sha(parent_configuration),
        planner_overlay=manifest, qualification_status='PLANNER_IMPLEMENTED_NOT_LIVE_QUALIFIED')
    if args.rare_sites:
        builder.save(app / 'PARENT_SCENES.json', config['qualified_scenes'])
        mounts = []
        for i, scene in enumerate(config['qualified_scenes'], 1):
            mounts += ['-v', scene['bundle'] + ':/parent-' + str(i) + ':ro']
        run('rare-scenes', installed + 'exec python3 /work/source/scripts/prepare_demo_rare_scenes.py', mounts)
        scenes = json.loads((checks / 'RARE_SCENES.json').read_text())
        for entry in scenes['selected']:
            for key in ('bundle', 'scene_file'):
                entry[key] = str(app) + entry[key].removeprefix('/work')
        config['qualified_scenes'] = scenes['selected']
        config['sample_site_recipe'] = 'rare_demo_v1'
    pinned_trees = ('source', 'install', 'management', 'scripts', 'testdeps', 'demo-scenes')
    config['files'] = {str(p.relative_to(app)): builder.sha(p)
        for tree in pinned_trees
        for p in sorted((app / tree).rglob('*'))
        if p.is_file() and not p.is_symlink() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    configuration = app / 'DEMO_CONFIGURATION.json'
    builder.save(configuration, config)
    release = dict(parent_release, application=str(app), configuration=str(configuration), files=dict(parent_release['files']))
    release['files'][str(configuration)] = builder.sha(configuration)
    builder.save(app / 'ASSESSMENT_RELEASE.json', release)
    for i, scene in enumerate(config['qualified_scenes'], 1):
        run('launch-' + str(i), installed + 'exec python3 /work/source/scripts/check_autonomous_demo_launch.py '
            '--representative ' + str(i), ['-v', scene['bundle'] + ':/scenes:ro'])
    builder.save(app / 'BUILD_RECEIPT.json', dict(image=config['image'], case=str(case),
        installed_interface=True, qualification_status='NOT_LIVE_QUALIFIED', overlay=manifest,
        parent_application=str(parent), retained_installed_components_unchanged=True,
        checks=[p.name for p in sorted(checks.glob('*.log'))], rendering_started=False, motion_requested=False))
    subprocess.run(['python3', '-B', str(app / 'source/scripts/install_autonomous_demo_runtime.py'),
                    str(app), str(runtime)], check=True)
    for name, expected in json.loads(parent_configuration.read_text())['files'].items():
        if builder.sha(parent / name) != expected:
            raise ValueError('Rollback changed during build: ' + name)
    print('DEMO_PLANNER_CANDIDATE_READY: ' + str(runtime / 'start_autonomous_demo.sh'), flush=True)


if __name__ == '__main__':
    main()
