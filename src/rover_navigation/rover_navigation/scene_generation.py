"""Generate one navigation scene from an existing asset library, without capture."""
import argparse
from copy import deepcopy
import json
from pathlib import Path


def generate(output, template, seed, *, mars_survey=False, perimeter_objects=False):
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
    if mars_survey:
        if config['terrain'].get('profile_version')!=2:
            raise ValueError('Mars survey scenes need the shared version-2 terrain')
        config['terrain']['surface_style']='jezero_ripples'
        # Explicit ranges keep repeated scene generation from compounding scale.
        config['rocks'].update(sampling_size_m=[.025,.14],obstacle_size_m=[.17,.53])
    if mars_survey or perimeter_objects:
        if config['terrain'].get('profile_version')!=2:
            raise ValueError('Perimeter placement needs shared version-2 terrain')
        config['terrain']['perimeter_objects']=True
        from .demo_core import DEMO_MAP_EDGE_SETBACK_M
        config['terrain']['perimeter_soil_staging_setback_m']=DEMO_MAP_EDGE_SETBACK_M
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
    if mars_survey:
        record['surface_style']=config['terrain']['surface_style']
    if config['terrain'].get('perimeter_objects'):
        record['perimeter_objects']=True
    (output/'generation.json').write_text(json.dumps(record, indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--mars-survey', action='store_true')
    args = parser.parse_args()
    print(json.dumps(generate(args.output, args.template, args.seed, mars_survey=args.mars_survey)))


if __name__ == '__main__':
    main()
