#!/usr/bin/env python3
"""Prepare a separate process-scoped SDMA-off terrain and v23 demo candidate.

CPU/ROS checks have no GPU devices. No hardware workload starts in this builder.
"""
import ast
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

BASE = Path('/home/ian/capstone-training/rover-navigation/navigation_sessions')
DEMO = BASE / 'autonomous-demo-20261109'
TRAIN = BASE / 'capture5000-training-20261108'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def once(body, before, after):
    if body.count(before) != 1:
        raise ValueError('Unexpected retained terrain source shape: ' + before)
    return body.replace(before, after, 1)


def terrain_candidate(body):
    body = once(body, "sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'bulk10000-collection-20261106'))",
                "sys.path.insert(0, '/home/ian/capstone-training/rover-navigation/navigation_sessions/bulk10000-collection-20261106')")
    body = once(body, "    if any(k in ('LD_PRELOAD','LD_LIBRARY_PATH','PYTHONPATH') or k.startswith(('HSA_','HIP_','ROCR_')) for k in os.environ):",
                "    if os.environ.get('HSA_ENABLE_SDMA') != '0':raise RuntimeError('Explicit SDMA-off candidate required')\n"
                "    if any(k in ('LD_PRELOAD','LD_LIBRARY_PATH','PYTHONPATH') or (k.startswith(('HSA_','HIP_','ROCR_')) and k != 'HSA_ENABLE_SDMA') for k in os.environ):")
    body = once(body, "    env.update(OMP_NUM_THREADS='1',ROCM_SDK_TARGET_FAMILY='gfx1151')",
                "    env.update(OMP_NUM_THREADS='1',ROCM_SDK_TARGET_FAMILY='gfx1151',HSA_ENABLE_SDMA='0')")
    body = once(body, "condition='environment-v2-1600-grid-bound-stock-7141-blocking-sync',",
                "condition='environment-v2-1600-grid-bound-stock-7141-blocking-sync-sdma-off',\n"
                "            copy_policy=dict(HSA_ENABLE_SDMA=os.environ['HSA_ENABLE_SDMA'],scope='this terrain worker only',\n"
                "                             effective_engine_activity_verified=False),")
    body = once(body, '    start_service(RocmTerrainCompute,create_server,hip,8782,save)',
                "    class SdmaOffTerrainCompute(RocmTerrainCompute):\n"
                "        def __init__(self):\n"
                "            super().__init__()\n"
                "            self.metadata['copy_policy'] = dict(HSA_ENABLE_SDMA=os.environ['HSA_ENABLE_SDMA'],\n"
                "                scope='this terrain worker only',effective_engine_activity_verified=False)\n"
                "    start_service(SdmaOffTerrainCompute,create_server,hip,8782,save)")
    body = once(body, 'ENVIRONMENT_V2_TERRAIN_READY: float64, HIP requested/observed 4, grid bound 1600',
                'ENVIRONMENT_V2_TERRAIN_READY: SDMA-off candidate, float64, HIP requested/observed 4, grid bound 1600')
    ast.parse(body)
    return body


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', type=int, default=23)
    parser.add_argument('--terrain-version', type=int, default=1)
    args = parser.parse_args()
    if args.version < 23 or args.terrain_version < 1:
        raise ValueError('Use separate new candidate versions')
    folder = DEMO / 'gpu-repair-20261117'
    candidate = folder / ('terrain-sdma-off-v' + str(args.terrain_version))
    app, runtime = DEMO / ('application-v' + str(args.version)), DEMO / ('runtime-v' + str(args.version))
    if any(p.exists() for p in (candidate, app, runtime)):
        raise ValueError('Preserve prior candidates; choose new identities')
    parent = DEMO / 'application-v22'
    parent_config = parent / 'DEMO_CONFIGURATION.json'
    config = json.loads(parent_config.read_text())
    release = json.loads((DEMO / 'runtime-v22/ASSESSMENT_RELEASE.json').read_text())
    for name, expected in release['files'].items():
        if sha(Path(name)) != expected:
            raise ValueError('Parent release changed: ' + name)
    for name, expected in config['files'].items():
        if sha(parent / name) != expected:
            raise ValueError('Parent application changed: ' + name)
    candidate.mkdir()
    retained = TRAIN / 'capture5000_terrain.py'
    generated = candidate / 'terrain_sdma_off.py'
    generated.write_text(terrain_candidate(retained.read_text()))
    config_path = config['terrain_service_runtime_configuration']
    starter = candidate / 'start_terrain.sh'
    starter.write_text('#!/usr/bin/env bash\nset -euo pipefail\n'
        'ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)\n'
        'python3 -B "$ROOT/check_candidate.py"\n'
        'eval "$(python3 -I /home/ian/capstone-training/rover-sim/manage.py context)"\n'
        'CASE=$(cat "/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103/capture-$SIM_LABEL.path")\n'
        'exec python3 /home/ian/capstone-training/rover-navigation/navigation_sessions/environment-v2-20261105T225758Z/application-v6/management/phase2_desktop_launch.py '
        '"$ROOT/terrain_sdma_off.py" ' + config_path + ' --kit "$SIM_KIT" --case "$CASE" --module rover_navigation.terrain_service\n')
    checker = candidate / 'check_candidate.py'
    checker.write_text("import hashlib,json,sys\nfrom pathlib import Path\nfrom urllib.request import urlopen\nr=Path(__file__).resolve().parent\n"
        "v=json.loads((r/'CANDIDATE.json').read_text())\n"
        "for n,h in v['files'].items():\n"
        " if hashlib.sha256(Path(n).read_bytes()).hexdigest()!=h:raise RuntimeError('Candidate changed: '+n)\n"
        "if '--live' in sys.argv:\n"
        " with urlopen('http://127.0.0.1:8782/status',timeout=3) as s:v=json.load(s)\n"
        " if v.get('backend')!='rocm' or v.get('geometry')!='float64' or v.get('copy_policy',{}).get('HSA_ENABLE_SDMA')!='0':raise RuntimeError('Live SDMA-off ROCm terrain required')\n"
        "print('SDMA_OFF_CANDIDATE_PINS_PASS')\n")
    save(candidate / 'CANDIDATE.json', dict(policy={'HSA_ENABLE_SDMA': '0'},
         scope='terrain worker only; rendering and system policy unchanged', qualification_status='NOT_HARDWARE_QUALIFIED',
         parent_terrain_wrapper_sha256=sha(retained), files={str(p): sha(p) for p in
         (retained, generated, starter, checker, Path(config_path), Path(__file__).resolve())}))
    subprocess.run(['bash', '-n', str(starter)], check=True)
    subprocess.run(['python3', '-B', str(checker)], check=True)
    app.mkdir()
    for tree in ('source', 'install', 'management', 'scripts', 'testdeps', 'demo-scenes'):
        shutil.copytree(parent / tree, app / tree, symlinks=True)
    for entry in config['qualified_scenes']:
        for key in ('bundle', 'scene_file'):
            path = Path(entry[key])
            if path.is_relative_to(parent):
                entry[key] = str(app / path.relative_to(parent))
    # Bind the exact terrain policy to this candidate; v22 remains available.
    config.update(project=str(app), parent_application=str(parent), parent_configuration_sha256=sha(parent_config),
                  qualification_status='GPU_REPAIR_CANDIDATE_NOT_HARDWARE_QUALIFIED',
                  terrain_copy_candidate=str(candidate / 'CANDIDATE.json'),
                  terrain_copy_candidate_sha256=sha(candidate / 'CANDIDATE.json'))
    save(app / 'DEMO_CONFIGURATION.json', config)
    release.update(application=str(app), configuration=str(app / 'DEMO_CONFIGURATION.json'))
    release['files'] = dict(release['files'])
    release['files'][str(app / 'DEMO_CONFIGURATION.json')] = sha(app / 'DEMO_CONFIGURATION.json')
    release['files'][str(candidate / 'CANDIDATE.json')] = sha(candidate / 'CANDIDATE.json')
    release['files'].update(json.loads((candidate / 'CANDIDATE.json').read_text())['files'])
    save(app / 'ASSESSMENT_RELEASE.json', release)
    checks = app / 'checks'
    checks.mkdir()
    common = ['docker', 'run', '--rm', '--init', '--network', 'none', '--cpuset-cpus=0,1,2,3,4,5,6,7',
              '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp', '-e', 'PYTHONDONTWRITEBYTECODE=1',
              '-e', 'OMP_NUM_THREADS=2', '-e', 'OPENBLAS_NUM_THREADS=2', '-e', 'ROS_DOMAIN_ID=88',
              '-e', 'FASTRTPS_DEFAULT_PROFILES_FILE=/work/source/src/rover_navigation/config/fastdds.xml',
              '-v', str(app) + ':/work', '-w', '/work']
    def run(name, shell, extra=()):
        print('CHECK_START ' + name, flush=True)
        with (checks / (name + '.log')).open('xb') as stream:
            subprocess.run(common + list(extra) + [config['image'], 'bash', '-c',
                           'source /opt/ros/jazzy/setup.bash && ' + shell], stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=600)
        print('CHECK_PASS ' + name, flush=True)
    source_paths = ':'.join('/work/source/src/' + package for package in
                           ('rover_navigation', 'rover_perception', 'rover_training', 'mars_training_environment'))
    run('navigation-source', 'export PYTHONPATH=' + source_paths + ':/work/testdeps:$PYTHONPATH && '
        'exec python3 -m unittest discover -s source/src/rover_navigation/test -v')
    installed = 'source /work/install/setup.bash && '
    run('actual-ros', installed + 'exec python3 /work/source/scripts/probe_autonomous_demo.py')
    run('actual-tf2', installed + 'exec python3 /work/source/scripts/probe_demo_transforms.py')
    run('transport', installed + 'exec python3 /work/source/scripts/probe_demo_transport.py --rate-hz 10 --output /work/checks/transport.json')
    for i, scene in enumerate(config['qualified_scenes'], 1):
        run('launch-' + str(i), installed + 'exec python3 /work/source/scripts/check_autonomous_demo_launch.py --representative ' + str(i),
            ['-v', scene['bundle'] + ':/scenes:ro'])
    save(app / 'BUILD_RECEIPT.json', dict(installed_interface=True, installed_files_unchanged=True,
         retained_file_count=len(config['files']), parent_application=str(parent),
         rendering_started=False, motion_requested=False, qualification_status='NOT_HARDWARE_QUALIFIED',
         terrain_candidate=str(candidate / 'CANDIDATE.json')))
    subprocess.run(['python3', '-B', str(app / 'source/scripts/install_autonomous_demo_runtime.py'), str(app), str(runtime)], check=True)
    start = runtime / 'start_autonomous_demo.sh'
    start.write_text(once(start.read_text(), 'REP=${1:-1}',
                         'python3 -B ' + str(checker) + ' --live\nREP=${1:-1}'))
    runtime_release = runtime / 'ASSESSMENT_RELEASE.json'
    value = json.loads(runtime_release.read_text())
    value['files'][str(start)] = sha(start)
    runtime_release.write_text(json.dumps(value, indent=2) + '\n')
    subprocess.run(['bash', '-n', str(start)], check=True)
    subprocess.run(['python3', '-B', str(runtime / 'autonomous_demo_assess.py'), '--check'], check=True)
    for name, expected in config['files'].items():
        if sha(parent / name) != expected or sha(app / name) != expected:
            raise ValueError('Retained or copied file changed: ' + name)
    print('GPU_REPAIR_CANDIDATE_READY: ' + str(runtime), flush=True)


if __name__ == '__main__':
    main()
