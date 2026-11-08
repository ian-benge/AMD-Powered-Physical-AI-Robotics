"""One reserved 200-frame batch, using the accepted pilot capture implementation."""
import json
from pathlib import Path
import shutil
import time

import bulk10000_plan as policy
import pilot200_capture_worker as pilot
from mars_training_environment.common import digest, read_config, write_json as _write_json
from mars_training_environment.generate import make_scene
from mars_training_environment.planning import class_coverage
from mars_training_environment.validate import validate_scene
from mars_training_environment import capture_attempts
from process_cleanup import verify_runtime_qualification

ROOT = Path('/collection')


def write_json(path, value):
    path = Path(path)
    temp = path.with_name(path.name+'.tmp')
    _write_json(temp, value)
    temp.replace(path)


def bundle_for(profile, batch, reservation_sha256):
    bundle = ROOT/'bundle'
    marker = ROOT/'BUNDLE_PINS.json'
    if marker.exists():
        pins = json.loads(marker.read_text())
        if pins['reservation_sha256'] != reservation_sha256 or pins['batch'] != batch:
            raise ValueError('Saved bundle belongs to another reservation')
        policy.verify_files(bundle, pins['files'])
        return bundle
    bundle.mkdir(exist_ok=True)
    preserved = ROOT/'generation-attempts'
    preserved.mkdir(exist_ok=True)
    if not (bundle/'assets').exists():
        # An interrupted copy remains separate; the next invocation gets a new attempt.
        stage = preserved/f'assets-{time.time_ns()}'
        shutil.copytree('/prepared-assets', stage)
        stage.rename(bundle/'assets')
    source = Path('/prepared-assets')
    source_pins = {p.relative_to(source).as_posix():policy.sha(p)
                   for p in source.rglob('*') if p.is_file()}
    if set(source_pins) != {p.relative_to(bundle/'assets').as_posix()
                           for p in (bundle/'assets').rglob('*') if p.is_file()}:
        raise ValueError('Prepared asset copy has a different inventory')
    policy.verify_files(bundle/'assets', source_pins)
    assets = json.loads((bundle/'assets/catalog.json').read_text())
    ground = json.loads((bundle/'assets/ground/catalog.json').read_text())
    checkpoints = ROOT/'generation-checkpoints'
    checkpoints.mkdir(exist_ok=True)
    scenes = []
    for index, entry in enumerate(batch['scenes'], 1):
        config = policy.scene_configuration(profile, batch, entry)
        folder = bundle/'scenes'/entry['id']
        checkpoint = checkpoints/(entry['id']+'.json')
        if checkpoint.exists():
            saved = json.loads(checkpoint.read_text())
            if saved['config_sha256'] != digest(config):
                raise ValueError('Interrupted generation configuration changed')
            policy.verify_files(bundle, saved['files'])
            scene = json.loads((folder/'scene.json').read_text())
        else:
            if folder.exists():
                folder.rename(preserved/f'{entry["id"]}-partial-{time.time_ns()}')
            print(f'Planning {batch["id"]} scene {index}/10: {entry["id"]}', flush=True)
            try:
                scene = make_scene(bundle, config, entry['seed'], assets, ground)
            except Exception as error:
                write_json(preserved/f'{entry["id"]}-failure-{time.time_ns()}.json',
                           dict(seed=entry['seed'], error=str(error), replacement_seed_allowed=False))
                raise
            files = {p.relative_to(bundle).as_posix():policy.sha(p)
                     for p in folder.rglob('*') if p.is_file()}
            write_json(checkpoint, dict(config_sha256=digest(config), files=files))
        errors = validate_scene(scene, assets)
        if (errors or scene['split'] != entry['split'] or scene['scene_group'] != entry['scene_group'] or
                scene['empty'] != entry['empty'] or len(scene['views']) != 20):
            raise ValueError(dict(scene=entry['id'], errors=errors, reservation=entry))
        scenes.append(scene)
        write_json(ROOT/'GENERATION_PROGRESS.json', dict(batch=batch['id'],
                   planned_scenes=len(scenes), planned_frames=len(scenes)*20))
    coverage = class_coverage(scenes)
    if coverage['errors']:
        raise ValueError(coverage['errors'])
    # Each scene carries its own reserved density/sunlight configuration.
    batch_config = policy.scene_configuration(profile, batch, batch['scenes'][0])
    plan = dict(schema_version=4, class_names=list(pilot.CLASS_NAMES),
        config_sha256=digest(batch_config), config=batch_config, scene_recipes=batch['scenes'],
        reservation_sha256=reservation_sha256, batch_id=batch['id'],
        scenes=[dict(id=s['id'], seed=s['seed'], split=s['split'],
                     content_sha256=s['content_sha256'], views=20, kind=s['kind']) for s in scenes],
        reserved_meshes=[a['id'] for a in assets if a['held_out']], class_coverage=coverage)
    write_json(bundle/'plan.json', plan)
    write_json(bundle/'splits.json', {s['scene_group']:s['split'] for s in scenes})
    files = {p.relative_to(bundle).as_posix():policy.sha(p) for p in bundle.rglob('*')
             if p.is_file() and p.name not in ['world.sdf','robot_capture.urdf','bridge.yaml']}
    write_json(marker, dict(reservation_sha256=reservation_sha256, batch=batch, files=files))
    return bundle


def main():
    invocation = Path('/invocation')
    recipe = json.loads((invocation/'RECIPE.json').read_text())
    batch, plan_sha = recipe['batch'], recipe['reservation_sha256']
    identity = json.loads((ROOT/'BATCH_IDENTITY.json').read_text())
    if identity != recipe:
        raise ValueError('Batch identity differs from the invocation')
    if policy.sha(invocation/'pilot200_environment_v2.json') != recipe['profile_sha256']:
        raise ValueError('Qualified camera/environment profile changed')
    if (ROOT/'BATCH_RESULT.json').exists():
        policy.verify_batch(ROOT, batch, plan_sha)
        print('BULK_BATCH_VERIFIED_REUSE', flush=True)
        return
    profile = read_config(invocation/'pilot200_environment_v2.json')
    pilot.check_profile(profile)
    bundle = bundle_for(profile, batch, plan_sha)
    # Require the scoped renderer/cleanup evidence before canonical commit,
    # including when recovering a completed attempt after an interruption.
    original_commit = capture_attempts.commit_attempt

    def checked_commit(attempt, output, report, entry):
        verify_runtime_qualification(ROOT, output, report)
        return original_commit(attempt, output, report, entry)

    capture_attempts.commit_attempt = checked_commit
    pilot.capture(bundle, resume=True)
    validation = json.loads((ROOT/'dataset/validation.json').read_text())
    if validation['split_counts'] != {batch['split']:200}:
        raise ValueError('Batch split counts differ from the reserved scene groups')
    # ROS log writers may still flush; accepted outputs and proofs are closed here.
    files = {p.relative_to(ROOT).as_posix():policy.sha(p) for p in ROOT.rglob('*')
             if p.is_file() and 'roslogs' not in p.relative_to(ROOT).parts
             and p.name not in ['BATCH_RESULT.json','BATCH_PINS.json']}
    write_json(ROOT/'BATCH_PINS.json', dict(files=files))
    write_json(ROOT/'BATCH_RESULT.json', dict(status='BULK_BATCH_COMPLETE_PENDING_REVIEW',
        batch_id=batch['id'], reservation_sha256=plan_sha, split=batch['split'],
        scene_ids=[e['id'] for e in batch['scenes']], frames=200, background_frames=40,
        pins_sha256=policy.sha(ROOT/'BATCH_PINS.json'), training_approved=False))
    print('BULK_BATCH_COMPLETE_PENDING_REVIEW: '+batch['id'], flush=True)


if __name__ == '__main__':
    main()
