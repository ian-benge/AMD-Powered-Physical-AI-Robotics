"""Generate one navigation scene from an existing asset library, without capture."""
import argparse
from copy import deepcopy
import json
from pathlib import Path


def generate(output, template, seed):
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 1_000_000_000:
        raise ValueError('Scene seed must be an integer below one billion')
    output, template = Path(output), Path(template)
    scene_id = f'scene_{seed:06d}'
    if (output/'scenes'/scene_id).exists():
        raise ValueError('Scene already exists; use a fresh output directory')
    source = json.loads(template.read_text())
    config = deepcopy(source['config'])
    from mars_training_environment.reachability import mixed_reachable_config
    config=mixed_reachable_config(config)
    # Reuse the environment generator's terrain/object constraints. Dataset
    # capture poses and class quotas are unnecessary for a manual drive scene.
    config['dataset']['seeds'] = [seed]
    config['dataset']['empty_seeds'] = []
    config['dataset']['rocks_only_seeds'] = []
    config['capture']['views_per_scene'] = 0
    config['capture'].pop('target_class_views', None)
    assets = json.loads((output/'assets/catalog.json').read_text())
    ground_path = output/'assets/ground/catalog.json'
    ground = json.loads(ground_path.read_text()) if ground_path.exists() else []
    from mars_training_environment.generate import make_scene
    scene = make_scene(output, config, seed, assets, ground)
    record = dict(schema_version=2, purpose='manual_navigation', seed=seed,
                  terrain_profile='mixed_reachable',terrain_profile_version=config['terrain']['profile_version'],
                  geometry_sha256=scene.get('geometry_sha256'),
                  scene=scene_id, source_scene=source['id'],
                  source_content_sha256=source.get('content_sha256'),
                  content_sha256=scene['content_sha256'], capture_requested=False)
    (output/'generation.json').write_text(json.dumps(record, indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--seed', type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.output, args.template, args.seed)))


if __name__ == '__main__':
    main()
