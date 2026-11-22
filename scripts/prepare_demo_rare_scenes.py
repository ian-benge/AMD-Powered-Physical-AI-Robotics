#!/usr/bin/env python3
"""Generate three separate rare-site demo bundles and audit geometry; no capture."""
import hashlib
import json
import os
from pathlib import Path
import shutil

from mars_training_environment.common import digest
from mars_training_environment.environment_v2 import configuration, export_class_id
from mars_training_environment.generate import make_scene
from mars_training_environment.reachability_audit import audit_layout
from rover_navigation.scene_generation import generate


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_asset(source, target):
    try:
        os.link(source, target)
        return target
    except OSError:
        return shutil.copy2(source, target)


def counts(scene):
    result = [0] * 4
    for obj in scene['objects'] + scene['regions']:
        result[export_class_id(obj)] += 1
    return result


def geometry_record(scene, folder):
    # Material baking adds render-only fields to scene objects. Geometry's
    # authoritative hash predates those additions; use the saved layout.
    layout = json.loads((folder / 'layout-attempts' /
                         f"attempt-{scene['layout_attempt']:02d}.json").read_text())
    result = {key: scene[key] for key in ('seed', 'layout_seed', 'layout_attempt', 'config_sha256')}
    result.update({key: layout[key] for key in ('terrain', 'objects', 'regions')})
    if digest(result) != scene['geometry_sha256']:
        raise ValueError('Saved pre-material layout does not match the geometry identity')
    return result


def main():
    root = Path('/work')
    old = json.loads((root / 'PARENT_SCENES.json').read_text())
    report, selected = [], []
    for index, entry in enumerate(old, 1):
        parent = Path('/parent-' + str(index))
        template = json.loads((parent / 'scenes' / entry['scene'] / 'scene.json').read_text())
        output = root / 'demo-scenes' / str(template['seed'])
        output.mkdir(parents=True, exist_ok=False)
        shutil.copytree(parent / 'assets', output / 'assets', copy_function=copy_asset)
        original_counts = counts(template)
        template['config'] = configuration(template['config'], manual=True, sample_site_recipe='rare_demo_v1')
        source = output / 'template.json'
        source.write_text(json.dumps(template, allow_nan=False) + '\n')
        print('RARE_SCENE_GENERATION_START', entry['density'], template['seed'], flush=True)
        generate(output, source, template['seed'])
        scene_file = output / 'scenes' / entry['scene'] / 'scene.json'
        scene = json.loads(scene_file.read_text())
        assets = json.loads((output / 'assets/catalog.json').read_text())
        geometry = geometry_record(scene, scene_file.parent)
        audit = audit_layout(output, geometry, scene['reachability'], assets)
        assert len(audit['routes']) == 8 and audit['result'] == 'PASS'
        regen_root = root / 'checks/regeneration' / str(scene['seed'])
        shutil.copytree(output / 'assets', regen_root / 'assets', copy_function=copy_asset)
        regenerated = make_scene(regen_root, scene['config'], scene['seed'], assets, validation_only=True)
        assert regenerated['geometry_sha256'] == scene['geometry_sha256']
        actual = counts(scene)
        assert 3 <= actual[1] <= 6 and 2 <= actual[2] <= 4 and actual[3] == 2
        assert scene['config']['rocks'] == template['config']['rocks']
        assert actual[0] > sum(actual[1:])
        row = dict(entry, bundle=str(output), scene_file=str(scene_file),
                   scene_sha256=sha(scene_file), geometry_sha256=scene['geometry_sha256'],
                   sample_site_recipe='rare_demo_v1')
        selected.append(row)
        report.append(dict(seed=scene['seed'], density=entry['density'], original_export_counts=original_counts,
            export_counts=actual, actual_special_target_reduction=1 - sum(actual[1:]) / sum(original_counts[1:]),
            layout_attempt=scene['layout_attempt'], layout_failures=scene['layout_failures'],
            audit=audit, deterministic_geometry_passed=True, capture_requested=False))
        print('RARE_SCENE_AUDIT_PASS', entry['density'], actual, flush=True)
    (root / 'checks/RARE_SCENES.json').write_text(json.dumps(dict(
        passed=True, expected_target_reduction=1 - 9.5 / 34, scenes=report,
        selected=selected, live_qualified=False, capture_requested=False), indent=2) + '\n')


if __name__ == '__main__':
    main()
