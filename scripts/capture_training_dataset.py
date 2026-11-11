"""Freeze accepted capture batches and reviewed legacy data for candidate training.

Source pixels, labels, splits, receipts and model checkpoints are never modified.
The new export records geometry validation separately from human visual review.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile

NAMES = ('rock', 'pale_soil_patch', 'yellow_crystal_rock', 'mudstone_slab')
SLABS = {'cheyava_rock', 'veined_lookalike'}
LEGACY_AUDIT_SHA = '88ace1ac788ac4784679c484e3f29377c4465356764e4a3866c422462fbae068'


def sha(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError('Expected a regular input file: '+str(path))
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')


def inside(root, relative):
    root = Path(root).resolve(); relative = Path(relative)
    path = root/relative
    if relative.is_absolute() or not path.resolve().is_relative_to(root) or path.is_symlink():
        raise ValueError('Input path escapes its dataset')
    return path


def label_text(labels, width, height, classes=4):
    lines = []
    for item in labels:
        cid = item['class_id']
        box = [item[k] for k in ('xmin', 'ymin', 'xmax', 'ymax')]
        if (type(cid) is not int or not 0 <= cid < classes or
                any(not math.isfinite(v) for v in box) or
                not 0 <= box[0] < box[2] <= width or not 0 <= box[1] < box[3] <= height):
            raise ValueError('Invalid class or box coordinates')
        x1, y1, x2, y2 = box
        lines.append(f'{cid} {(x1+x2)/(2*width):.6f} {(y1+y2)/(2*height):.6f} '
                     f'{(x2-x1)/width:.6f} {(y2-y1)/height:.6f}')
    return '\n'.join(lines)+ ('\n' if lines else '')


def four_class_labels(labels):
    converted, changes = [], []
    for item in labels:
        result = copy.deepcopy(item)
        cid, appearance = item['class_id'], item.get('appearance_name')
        slab = appearance in SLABS or item.get('asset_id') == 'cand_slab'
        if type(cid) is not int or not 0 <= cid < 3 or item.get('class_name') != NAMES[cid]:
            raise ValueError('New capture does not use the original three-class schema')
        if slab:
            if cid != 0:
                raise ValueError('Slab identity conflicts with its source class')
            result.update(class_id=3, class_name=NAMES[3])
            changes.append(item['name'])
        elif cid == 0 and not appearance:
            raise ValueError('Rock lacks metadata identity for mudstone conversion')
        converted.append(result)
    return converted, changes


def check_isolation(rows):
    groups, images, identities = {}, {}, set()
    for row in rows:
        split = row['split']; ident = row['id']; group = row['scene_group']; pixels = row['image_sha256']
        if split not in ('train', 'val', 'test') or ident in identities:
            raise ValueError('Invalid split or repeated frame identity')
        identities.add(ident)
        if groups.setdefault(group, split) != split:
            raise ValueError('Scene group crosses dataset splits')
        if images.setdefault(pixels, split) != split:
            raise ValueError('Identical RGB crosses dataset splits')


def copy_checked(source, target, expected):
    if sha(source) != expected:
        raise ValueError('Source changed before copy: '+str(source))
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open('rb') as src, target.open('xb') as dest:
        shutil.copyfileobj(src, dest)
    if sha(target) != expected or sha(source) != expected:
        raise ValueError('Source or destination changed during copy')


def verify_export(root):
    root = Path(root)
    manifest = json.loads((root/'DATASET_MANIFEST.json').read_text())
    if manifest.get('schema') != 1 or manifest.get('class_names') != list(NAMES):
        raise ValueError('Unexpected training snapshot schema')
    rows = manifest['frames']; check_isolation(rows)
    if dict(Counter(row['split'] for row in rows)) != manifest['split_counts']:
        raise ValueError('Snapshot split counts changed')
    files = manifest['files']
    for name, expected in files.items():
        if sha(inside(root, name)) != expected:
            raise ValueError('Training snapshot hash changed: '+name)
    actual = {p.relative_to(root).as_posix() for area in ('images', 'labels', 'metadata')
              for p in (root/area).rglob('*') if p.is_file() and p.suffix != '.cache'}
    expected_artifacts = {name for name in files if name.split('/')[0] in ('images', 'labels', 'metadata')}
    if actual != expected_artifacts:
        raise ValueError('Unpinned image, label or metadata appeared in the snapshot')
    for row in rows:
        for name, expected in [(row['image'], row['image_sha256']), (row['label'], row['label_sha256']),
                               (row['metadata'], row['metadata_sha256'])]:
            if files.get(name) != expected:
                raise ValueError('Frame evidence omitted from snapshot pins')
    return manifest


def verify_pixels(root):
    from PIL import Image
    root=Path(root);manifest=verify_export(root);shapes=Counter()
    for row in manifest['frames']:
        meta=json.loads((root/row['metadata']).read_text())
        with Image.open(root/row['image']) as image:
            image.load()
            expected=tuple(int(meta['intrinsics'][key]) for key in ('width','height'))
            if image.size!=expected or image.mode!='RGB':
                raise ValueError('Image pixels differ from their metadata dimensions/RGB format')
            shapes[f'{image.width}x{image.height}']+=1
    return dict(frames=len(manifest['frames']),decoded_shapes=dict(shapes),all_images_decoded=True)


def prepare(campaign, legacy, legacy_source, output, controller):
    campaign, legacy, legacy_source, output, controller = map(lambda p: Path(p).resolve(),
        (campaign, legacy, legacy_source, output, controller))
    if output.exists() or any(output.is_relative_to(p) or p.is_relative_to(output)
                              for p in [campaign, legacy, legacy_source, controller]):
        raise ValueError('Choose a fresh output separate from all inputs')
    sys.path.insert(0, str(controller))
    import start_bulk10000_collection as starter
    starter.pilot.check_candidate()
    reservation = json.loads((campaign/'SCENE_RESERVATIONS.json').read_text())
    identity = json.loads((campaign/'COLLECTION_IDENTITY.json').read_text())
    starter.policy.validate_reservation(reservation)
    if starter.policy.digest(reservation) != identity['reservation_sha256']:
        raise ValueError('Campaign reservation changed')
    audit_path = legacy/'review_export.json'
    if sha(audit_path) != LEGACY_AUDIT_SHA:
        raise ValueError('Previously reviewed evaluation export changed')
    audit = json.loads(audit_path.read_text())
    if audit['class_names'] != list(NAMES) or audit['frames'] != 500:
        raise ValueError('Previously reviewed four-class export is incomplete')
    legacy_catalog = legacy_source.parent/'coverage500_balanced_scenes/assets/catalog.json'
    protected_assets = {item['id'] for item in json.loads(legacy_catalog.read_text()) if item.get('held_out')}
    if not protected_assets:
        raise ValueError('Legacy held-out asset identities are missing')
    if (campaign/'batches/batch-026/dataset/reports').exists() and list((campaign/'batches/batch-026/dataset/reports').glob('*.json')):
        raise ValueError('New committed data appeared after the 5,000-image freeze')
    completed, seen = [], {}
    for batch in reservation['batches']:
        if starter.verify_prior(campaign, batch, identity['reservation_sha256'], seen):
            completed.append(batch)
            print('VERIFIED_SOURCE_BATCH', batch['id'], flush=True)
    if len(completed) != 25 or any(b['split'] != 'train' for b in completed):
        raise ValueError('Expected exactly 25 completed training batches')
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=output.name+'.preparing-', dir=output.parent))
    rows, source_pins, recipes = [], {}, {}
    coverage = {s: Counter() for s in ('train', 'val', 'test')}
    review_flags = []

    def add(stem, split, image, image_sha, metadata, labels, origin, flags, original_label=None):
        width, height = (int(metadata['intrinsics'][k]) for k in ('width', 'height'))
        expected = (960, 540) if origin['kind'] == 'geometry_validated_capture' else (1920, 1080)
        if (width, height) != expected:
            raise ValueError('Source camera dimensions differ from their qualified capture')
        text = label_text(labels, width, height)
        destination_image = f'images/{split}/{stem}{image.suffix}'
        destination_label = f'labels/{split}/{stem}.txt'
        destination_metadata = f'metadata/{split}/{stem}.json'
        copy_checked(image, stage/destination_image, image_sha)
        target = stage/destination_label; target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding='utf-8')
        if original_label is not None and target.read_bytes() != original_label.read_bytes():
            raise ValueError('Previously reviewed label bytes changed')
        record = copy.deepcopy(metadata)
        record.update(image=destination_image, label=destination_label, labels=labels)
        record['training_source'] = origin
        save(stage/destination_metadata, record)
        row = dict(id=stem, split=split, scene_group=metadata.get('scene_group', metadata['scene_id']),
            image=destination_image, label=destination_label, metadata=destination_metadata,
            image_sha256=image_sha, label_sha256=sha(target), metadata_sha256=sha(stage/destination_metadata),
            source=origin, review_flags=flags)
        rows.append(row); coverage[split].update(item['class_name'] for item in labels)
        if flags: review_flags.append(dict(id=stem, flags=flags))

    try:
        for batch in completed:
            folder = campaign/'batches'/batch['id']; dataset = folder/'dataset'
            source_pins[batch['id']] = dict(receipt_sha256=sha(campaign/'batch-receipts'/(batch['id']+'.json')),
                batch_pins_sha256=sha(folder/'BATCH_PINS.json'), result_sha256=sha(folder/'BATCH_RESULT.json'))
            pins = json.loads((folder/'BATCH_PINS.json').read_text())['files']
            for entry in batch['scenes']:
                recipes[entry['id']] = entry
                report = json.loads((dataset/'reports'/(entry['id']+'.json')).read_text())
                for name in report['records']:
                    metadata_path = inside(dataset, name); meta = json.loads(metadata_path.read_text())
                    if (meta['split'] != 'train' or meta['scene_group'] != entry['scene_group'] or
                            not meta['capture_checks']['passed'] or not meta['bbox_check']['agrees'] or
                            not meta['depth_quality']['visibility_check']['checked']):
                        raise ValueError('Capture annotation checks did not pass')
                    width, height = (meta['intrinsics'][k] for k in ('width', 'height'))
                    original_label = inside(dataset, meta['label'])
                    if original_label.read_text() != label_text(meta['labels'], width, height, classes=3):
                        raise ValueError('Source label file differs from captured geometry metadata')
                    labels, changes = four_class_labels(meta['labels'])
                    if any(item.get('asset_id') in protected_assets for item in labels):
                        raise ValueError('New training labels contain a held-out rock asset')
                    flags = [k for k in ('view_recoveries', 'framing_adjustments') if meta.get(k)]
                    if changes: flags.append('metadata_mudstone_conversion')
                    meta['four_class_relabel'] = dict(changed_objects=changes, review_required=bool(changes),
                        source_metadata_sha256=sha(metadata_path), issues=[])
                    origin = dict(kind='geometry_validated_capture', batch=batch['id'], source_metadata=str(metadata_path),
                        metadata_sha256=sha(metadata_path), original_label_sha256=sha(original_label),
                        original_class_names=list(NAMES[:3]), human_reviewed=False)
                    add(metadata_path.stem, 'train', inside(dataset, meta['image']), pins['dataset/'+meta['image']],
                        meta, labels, origin, flags)
            print('COPIED_SOURCE_BATCH', batch['id'], flush=True)
        for item in audit['audit']:
            split, stem = item['split'], item['id']
            meta_path = inside(legacy_source, f'metadata/{split}/{stem}.json')
            meta = json.loads(meta_path.read_text())
            if meta['split'] != split:
                raise ValueError('Previously reviewed frame split changed')
            label = inside(legacy, f'labels/{split}/{stem}.txt')
            if sha(label) != item['label_sha256']:
                raise ValueError('Previously reviewed labels changed')
            labels = copy.deepcopy(item['labels'])
            for box in labels: box['class_name'] = NAMES[box['class_id']]
            add(stem, split, inside(legacy, item['source_image']), item['image_sha256'], meta, labels,
                dict(kind='previous_reviewed_export', revision=audit['revision'], audit_sha256=sha(audit_path),
                     source_metadata_sha256=sha(meta_path), human_reviewed=bool(item.get('reviewed'))), [], label)
        check_isolation(rows)
        if dict(Counter(row['split'] for row in rows)) != dict(train=5300, val=100, test=100):
            raise ValueError('Unexpected combined dataset counts')
        (stage/'classes.txt').write_text('\n'.join(NAMES)+'\n', encoding='utf-8')
        (stage/'data.yaml').write_text('path: '+output.as_posix()+'\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n'+
            ''.join(f'  {i}: {n}\n' for i,n in enumerate(NAMES)), encoding='utf-8')
        save(stage/'REVIEW_STATUS.json', dict(geometry_checks_passed=True, uncertain_conversions=0,
            human_review_complete=False, training_scope='experimental_candidate_requested_by_owner',
            promotion_approved=False, flags=review_flags,
            limitation='The new captures are not a completed human-reviewed dataset; legacy evaluation does not establish new-environment generalization.'))
        save(stage/'SCENE_RECIPES.json', recipes)
        files = {p.relative_to(stage).as_posix():sha(p) for p in stage.rglob('*') if p.is_file()}
        manifest = dict(schema=1, class_names=list(NAMES), split_counts=dict(train=5300,val=100,test=100),
            frames=rows, files=files, class_instances={s:dict(v) for s,v in coverage.items()},
            capture_batches=source_pins, source_collection=str(campaign),
            source_reservation_sha256=sha(campaign/'SCENE_RESERVATIONS.json'),
            legacy_export=str(legacy), legacy_audit_sha256=sha(audit_path),
            source_split_assignments_unchanged=True, human_review_complete=False,
            protected_asset_ids=sorted(protected_assets),legacy_asset_catalog_sha256=sha(legacy_catalog),
            training_authorization='Owner explicitly requested preparing these images and beginning a new model',
            deployment_approved=False)
        save(stage/'DATASET_MANIFEST.json', manifest)
        verify_export(stage)
        stage.rename(output)
        print('TRAINING_SNAPSHOT_READY', output, flush=True)
        return manifest
    except BaseException as error:
        save(stage/'PREPARATION_FAILED.json', dict(error=str(error), sources_modified=False))
        raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--campaign', type=Path); p.add_argument('--legacy', type=Path)
    p.add_argument('--legacy-source', type=Path); p.add_argument('--controller', type=Path)
    p.add_argument('--output', type=Path); p.add_argument('--verify', type=Path)
    p.add_argument('--verify-pixels', type=Path)
    args = p.parse_args()
    if args.verify_pixels:
        print(json.dumps(verify_pixels(args.verify_pixels),indent=2));return
    if args.verify:
        result = verify_export(args.verify)
    else:
        if any(getattr(args,k) is None for k in ['campaign','legacy','legacy_source','controller','output']):
            p.error('Provide all five source/output arguments, or --verify')
        import fcntl
        with (args.controller/'collection.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
            result = prepare(args.campaign,args.legacy,args.legacy_source,args.output,args.controller)
    print(json.dumps(dict(split_counts=result['split_counts'], class_instances=result['class_instances'],
                          human_review_complete=False, deployment_approved=False), indent=2))


if __name__ == '__main__': main()
