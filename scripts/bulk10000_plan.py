"""Predeclared full-collection reservations; no rendering or training."""
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path

SPLITS = {'train': 400, 'val': 50, 'test': 50}
DENSITIES = {'sparse': [76, 110], 'standard': [152, 220], 'cluttered': [228, 330]}
SUNLIGHT = ('low', 'medium', 'high')
VIEWS = 20


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def split_for_seed(seed):
    # Identical to the accepted generator: existing seed groups never move.
    bucket = int(hashlib.sha256(f'mars-scene:{seed}'.encode()).hexdigest()[:8], 16) % 10
    return 'train' if bucket < 6 else ('val' if bucket < 8 else 'test')


def reservation(profile_sha256):
    batches = []
    seed = 4320000  # Separate from inspected pilot, mapping and qualification seeds.
    for split, count in SPLITS.items():
        scenes = []
        while len(scenes) < count:
            candidate = seed
            seed += 1
            if split_for_seed(candidate) != split:
                continue
            index = len(scenes)
            scenes.append(dict(id=f'scene_{candidate:06d}', seed=candidate,
                scene_group=f'seed_{candidate}', split=split, views=VIEWS,
                empty=index % 5 == 4, density=tuple(DENSITIES)[index % 3],
                sunlight=SUNLIGHT[index // 3 % 3],
                appearance_family=f'{split}:seed_{candidate}'))
        for start in range(0, count, 10):
            batches.append(dict(id=f'batch-{len(batches)+1:03d}', split=split,
                                scenes=scenes[start:start+10], frames=200, backgrounds=40))
    value = dict(version=1, planned_images=10000, views_per_scene=VIEWS,
                 split_images={name: count*VIEWS for name, count in SPLITS.items()},
                 profile_sha256=profile_sha256, pilot_reused=False,
                 source_schema='three_class', export_schema='four_class_metadata_conversion',
                 training_approved=False, batches=batches)
    validate_reservation(value)
    return value


def validate_reservation(plan):
    scenes = [entry for batch in plan['batches'] for entry in batch['scenes']]
    if (plan['version'] != 1 or plan['planned_images'] != 10000 or
            len(plan['batches']) != 50 or len(scenes) != 500 or
            len({e['seed'] for e in scenes}) != 500 or
            Counter(e['split'] for e in scenes) != Counter(SPLITS)):
        raise ValueError('Full collection reservation has wrong counts or duplicate seeds')
    for batch in plan['batches']:
        if len(batch['scenes']) != 10 or sum(e['empty'] for e in batch['scenes']) != 2:
            raise ValueError('Each bounded batch must have eight populated and two background scenes')
        for e in batch['scenes']:
            if (e['seed'] < 4320000 or e['id'] != f'scene_{e["seed"]:06d}' or
                    e['scene_group'] != f'seed_{e["seed"]}' or
                    e['split'] != batch['split'] or split_for_seed(e['seed']) != e['split'] or
                    e['views'] != VIEWS or e['density'] not in DENSITIES or
                    e['sunlight'] not in SUNLIGHT or
                    e['appearance_family'] != f'{e["split"]}:seed_{e["seed"]}'):
                raise ValueError('Scene reservation changes split, recipe or frame identity')


def scene_configuration(profile, batch, entry):
    config = deepcopy(profile)
    config['dataset'] = dict(seeds=[e['seed'] for e in batch['scenes']],
        empty_seeds=[e['seed'] for e in batch['scenes'] if e['empty']], rocks_only_seeds=[])
    config['terrain'].update(density=entry['density'], sunlight=entry['sunlight'])
    config['rocks']['count_range'] = DENSITIES[entry['density']][:]
    return config


def inside(root, relative):
    root = Path(root).resolve()
    value = Path(relative)
    path = (root/value).resolve()
    if value.is_absolute() or '..' in value.parts or not path.is_relative_to(root):
        raise ValueError('Evidence path escapes collection')
    return path


def verify_files(root, files):
    for name, expected in files.items():
        if sha(inside(root, name)) != expected:
            raise ValueError('Saved evidence changed: '+name)


def verify_batch(root, batch, plan_sha256):
    root = Path(root)
    result = json.loads((root/'BATCH_RESULT.json').read_text())
    if (result['status'] != 'BULK_BATCH_COMPLETE_PENDING_REVIEW' or
            result['batch_id'] != batch['id'] or result['reservation_sha256'] != plan_sha256 or
            result['frames'] != 200 or result['background_frames'] != 40 or
            result['split'] != batch['split'] or result['scene_ids'] != [e['id'] for e in batch['scenes']]):
        raise ValueError('Batch result differs from full collection reservation')
    pins = root/'BATCH_PINS.json'
    if sha(pins) != result['pins_sha256']:
        raise ValueError('Batch evidence manifest changed')
    files = json.loads(pins.read_text())['files']
    if not files:
        raise ValueError('Batch has no qualified evidence')
    verify_files(root, files)
    validation = json.loads((root/'dataset/validation.json').read_text())
    sensor = validation.get('sensor_depth_bbox_check', {})
    if (validation['errors'] or validation['frames'] != 200 or validation['empty_frames'] != 40 or
            validation['split_counts'] != {batch['split']:200} or
            sensor.get('backend') != 'sensor_depth_v1' or
            sensor.get('frames_checked') != 200 or sensor.get('frames_agreeing') != 200):
        raise ValueError('Batch validation does not pass')
    records = set()
    for entry in batch['scenes']:
        report = json.loads((root/'dataset/reports'/(entry['id']+'.json')).read_text())
        names = report['records']
        if report['frames'] != 20 or len(names) != 20 or len(set(names)) != 20:
            raise ValueError('Batch report has missing or repeated frame identities')
        for name in names:
            record = json.loads(inside(root/'dataset',name).read_text())
            if (name in records or record['scene_id'] != entry['id'] or
                    record['split'] != batch['split'] or
                    record['scene_sha256'] != report['scene_sha256'] or
                    not record.get('capture_checks',{}).get('passed') or
                    not record.get('bbox_check',{}).get('agrees')):
                raise ValueError('Captured frame differs from its scene or acceptance checks')
            records.add(name)
            if any('dataset/'+p not in files for p in (name,record['image'],record['label'])):
                raise ValueError('Batch manifest omits a required captured file')
        qualification = json.loads((inside(root/'dataset',report['attempt'])/'RUNTIME_QUALIFICATION.json').read_text())
        proof = inside(root,qualification['proof'])
        passed = json.loads((proof/'PASS.json').read_text())
        cleanup = json.loads((proof/'CLEANUP.json').read_text())
        world = '/collection/bundle/scenes/'+entry['id']+'/world.sdf'
        if (qualification.get('version') != 2 or passed.get('version') != 2 or
                qualification['world'] != world or passed['world'] != world or
                qualification['scene_sha256'] != report['scene_sha256'] or
                sha(proof/'PASS.json') != qualification['proof_sha256'] or
                sha(proof/'CLEANUP.json') != qualification['cleanup_sha256'] or
                cleanup.get('remaining') != [] or cleanup.get('partition') != passed['partition'] or
                sha(proof/f'attempt-{passed["attempt"]:03d}.json') != passed['observation_sha256']):
            raise ValueError('Scene-bound renderer proof or clean teardown is incomplete')
    if len(records) != 200:
        raise ValueError('Batch does not contain 200 distinct accepted frames')
    return result
