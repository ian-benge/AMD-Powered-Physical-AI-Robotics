"""Container worker: prepare the reserved pilot scenes and capture; never train."""
import argparse
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time

from mars_training_environment import dataset
from mars_training_environment.capture_attempts import verify_completed
from mars_training_environment.capture_attempts import FatalRendererError
from mars_training_environment.common import digest, read_config, write_json
from mars_training_environment.generate import make_scene
from mars_training_environment.planning import class_coverage
from mars_training_environment.validate import validate_scene
from rover_training.geometry import CLASS_NAMES
from process_cleanup import (CaptureProcessScope, verify_scene_renderer,
                             prepare_dataset_output, verify_runtime_qualification)
from desktop_render_guard import DesktopUnavailable, require_desktop, stop_path, watch_desktop

ROOT = Path('/collection')
PROFILE = Path('/work/source/src/mars_training_environment/config/pilot200_environment_v2.json')


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def check_profile(config):
    assert len(set(config['dataset']['seeds'])) == 10
    assert len(config['dataset']['empty_seeds']) == 2
    assert set(config['dataset']['empty_seeds']) <= set(config['dataset']['seeds'])
    assert config['capture']['views_per_scene'] == 20
    assert config['capture']['target_schema'] == 'four_class_export'
    assert list(config['capture']['target_class_views'].values()) == [5]*4
    assert config['camera']['render_engine'] == 'ogre'
    assert (config['camera']['width'], config['camera']['height']) == (960, 540)
    for name in ['require_depth_check', 'require_bbox_check', 'require_visibility_check']:
        assert config['capture'][name]


def plan_bundle(config, resume):
    bundle = ROOT/'bundle'; marker = ROOT/'BUNDLE_PINS.json'
    if resume:
        if not marker.is_file():
            raise ValueError('Incomplete scene generation retained; start a new collection directory')
        pins = json.loads(marker.read_text())
        if pins['config_sha256'] != digest(config):
            raise ValueError('Resume configuration changed')
        for name, expected in pins['files'].items():
            if sha(bundle/name) != expected:
                raise ValueError('Resume bundle hash mismatch: '+name)
        return bundle
    bundle.mkdir()
    shutil.copytree('/prepared-assets', bundle/'assets', copy_function=shutil.copy2)
    assets = json.loads((bundle/'assets/catalog.json').read_text())
    ground = json.loads((bundle/'assets/ground/catalog.json').read_text())
    scenes = []
    for index, seed in enumerate(config['dataset']['seeds'], 1):
        print(f'Planning scene {index}/10, seed {seed}', flush=True)
        scene = make_scene(bundle, config, seed, assets, ground)
        errors = validate_scene(scene, assets)
        if errors or scene['split'] != 'train' or len(scene['views']) != 20:
            raise ValueError(dict(seed=seed, errors=errors, split=scene['split'], views=len(scene['views'])))
        scenes.append(scene)
        write_json(ROOT/'GENERATION_PROGRESS.json', dict(planned_scenes=len(scenes), planned_frames=len(scenes)*20))
    coverage = class_coverage(scenes)
    if coverage['errors']:
        raise ValueError(coverage['errors'])
    plan = dict(schema_version=4, class_names=list(CLASS_NAMES), config_sha256=digest(config), config=config,
        scenes=[dict(id=s['id'], seed=s['seed'], split=s['split'], content_sha256=s['content_sha256'],
                     views=len(s['views']), kind=s['kind']) for s in scenes],
        reserved_meshes=[a['id'] for a in assets if a['held_out']], class_coverage=coverage)
    write_json(bundle/'plan.json', plan)
    write_json(bundle/'splits.json', {s['scene_group']:s['split'] for s in scenes})
    # Launch rewrites only these path-dependent descriptions, not scene/asset identities.
    files = {p.relative_to(bundle).as_posix():sha(p) for p in sorted(bundle.rglob('*'))
             if p.is_file() and p.name not in ['world.sdf', 'robot_capture.urdf', 'bridge.yaml']}
    write_json(marker, dict(config_sha256=digest(config), files=files))
    print('PILOT200_SCENE_PLAN_READY', flush=True)
    return bundle


def qualified_process(original, root, invocation):
    """Keep desktop lifetime, renderer provenance and owned cleanup in one scope."""
    run = json.loads(invocation.read_text())['utc'].replace(':', '').replace('+', '-')
    desktop_stop = stop_path(root, invocation)
    counter = 0

    @contextmanager
    def checked_process(command, log, qualified_desktop=False):
        nonlocal counter
        counter += 1
        proof = root/'renderer-provenance'/run/f'launch-{counter:03d}'; proof.mkdir(parents=True)
        finished = threading.Event(); success = []
        world = Path(next(arg[7:] for arg in command if arg.startswith('scene:='))).with_name('world.sdf')
        scope = None
        launch_wall_time = time.time()
        try:
            with original(command, log, qualified_desktop=qualified_desktop) as process:
                scope = CaptureProcessScope(process)
                def observe():
                    end = time.monotonic()+60; attempt = 0
                    while not finished.is_set() and time.monotonic() < end:
                        attempt += 1
                        result = subprocess.run([sys.executable, '/work/source/scripts/phase1_renderer_provenance.py'],
                            capture_output=True, text=True, timeout=20)
                        (proof/f'attempt-{attempt:03d}.stderr').write_text(result.stderr)
                        (proof/f'attempt-{attempt:03d}.json').write_text(result.stdout)
                        if result.returncode == 0:
                            try:
                                identity = verify_scene_renderer(json.loads(result.stdout), scope, world, launch_wall_time)
                            except Exception as error:
                                (proof/f'attempt-{attempt:03d}.scope-error').write_text(str(error))
                            else:
                                write_json(proof/'PASS.json', dict(attempt=attempt,
                                    qualified_desktop=qualified_desktop,
                                    observation_sha256=sha(proof/f'attempt-{attempt:03d}.json'), **identity))
                                success.append(True); return
                        finished.wait(.5)
                thread = threading.Thread(target=observe); thread.start()
                try:
                    yield process
                finally:
                    finished.set(); thread.join(timeout=25)
                if thread.is_alive() or not success:
                    raise RuntimeError('Mapped hardware renderer provenance was not verified; attempt retained')
        finally:
            if scope is not None:
                write_json(proof/'CLEANUP.json', scope.cleanup())
        write_json(Path(log.name).parent/'RUNTIME_QUALIFICATION.json', dict(version=2,
            world=str(world), proof=str(proof.relative_to(root)),
            scene_sha256=json.loads(world.with_name('scene.json').read_text())['content_sha256'],
            proof_sha256=sha(proof/'PASS.json'), cleanup_sha256=sha(proof/'CLEANUP.json')))
    @contextmanager
    def guarded_process(command, log, qualified_desktop=False):
        try:
            require_desktop(desktop_stop)
            with checked_process(command, log, qualified_desktop=qualified_desktop) as process:
                with watch_desktop(process, desktop_stop):
                    yield process
        except DesktopUnavailable as error:
            # Use the existing fatal-attempt path: preserve this attempt and stop
            # the campaign, rather than starting three renderers on a lost output.
            raise FatalRendererError(str(error)) from error

    return guarded_process


def capture(bundle, resume):
    dataset.simulator_process = qualified_process(dataset.simulator_process, ROOT,
        Path('/invocation/INVOCATION.json'))
    output = prepare_dataset_output(ROOT)
    print('Using generated scenes with scene-bound proof. Any legacy capture is preserved separately.', flush=True)
    for report_path in (output/'reports').glob('scene_*.json'):
        report = json.loads(report_path.read_text())
        verify_runtime_qualification(ROOT, output, report)
    dataset.capture_bundle(bundle, output, resume=resume)
    plan = json.loads((bundle/'plan.json').read_text())
    for entry in plan['scenes']:
        report = json.loads((output/'reports'/(entry['id']+'.json')).read_text())
        verify_completed(output, report, entry)
        verify_runtime_qualification(ROOT, output, report)
    validation = json.loads((output/'validation.json').read_text())
    if validation['errors'] or validation['frames'] != 200 or validation['empty_frames'] != 40:
        raise ValueError('Pilot frame count, backgrounds or validation failed')
    write_json(ROOT/'COLLECTION_RESULT.json', dict(status='PILOT200_CAPTURE_COMPLETE_PENDING_REVIEW',
        frames=200, scenes=10, populated_frames=160, background_frames=40, validation=validation,
        source_class_names=list(CLASS_NAMES), four_class_export_review_required=True,
        training_started=False, training_approved=False))
    print('PILOT200_CAPTURE_COMPLETE: 200 frames; review labels before export/training.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--check-config', type=Path, help='Check configuration only; no generation/capture')
    args = parser.parse_args()
    config = read_config(args.check_config or PROFILE); check_profile(config)
    if args.check_config:
        print('PILOT200_PROFILE_CHECK_PASS'); return
    capture(plan_bundle(config, args.resume), args.resume)


if __name__ == '__main__':
    main()
