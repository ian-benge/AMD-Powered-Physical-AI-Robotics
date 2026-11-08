"""Version-2 environment policy; geometry and export identities are evaluator-only."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from rover_training.geometry import CLASS_NAMES, FOUR_CLASS_NAMES, class_id_for
from .serialization import digest

DENSITIES = {'sparse': (76, 110), 'standard': (152, 220), 'cluttered': (228, 330)}
SUNLIGHT = {'low': (.35, .65), 'medium': (.65, .95), 'high': (.95, 1.2)}
QUALIFICATION_SEEDS = tuple(range(4311000, 4311050))


def is_v2(config):
    t = config.get('terrain', config)
    return t.get('profile') == 'mixed_reachable' and t.get('profile_version') == 2


def stream(seed, purpose):
    return np.random.RandomState(int(digest({'seed': int(seed), 'purpose': purpose, 'version': 2})[:8], 16))


def target_names(capture):
    return FOUR_CLASS_NAMES if capture.get('target_schema') == 'four_class_export' else CLASS_NAMES


def export_class_id(obj):
    # Match reclassify.py exactly, without changing legacy source class IDs.
    if obj.get('appearance_name', obj.get('class_name')) in {'cheyava_rock', 'veined_lookalike'} or obj.get('asset_id') == 'cand_slab':
        return 3
    return class_id_for(obj.get('class_name', 'rock'))


def configuration(source, density=None, sunlight=None, manual=False):
    from .common import config_path
    result = deepcopy(source)
    updates = json.loads(config_path().with_name('mixed_reachable_v2.json').read_text())
    for section, values in updates.items():
        result[section].update(values)
    t = result['terrain']
    t['density'] = density or (source['terrain'].get('density') if is_v2(source) else None) or 'standard'
    t['sunlight'] = sunlight or (source['terrain'].get('sunlight') if is_v2(source) else None) or 'medium'
    result['rocks']['count_range'] = list(DENSITIES[t['density']])
    if manual:
        result['capture']['views_per_scene'] = 0
        result['capture']['target_class_views'] = {}
    return result


def qualification_recipe(index):
    return dict(seed=QUALIFICATION_SEEDS[index], density=tuple(DENSITIES)[index % 3],
                sunlight=tuple(SUNLIGHT)[index // 3 % 3])


def apron_mesh(field, width=20.):
    """A joined ring outside the operating mesh, sharing exact boundary vertices."""
    inner = np.linspace(-field.size/2, field.size/2, field.n)
    outer = field.size/2 + np.arange(5., width+1e-9, 5.)
    coords = np.concatenate([-outer[::-1], inner, outer])
    x, y = np.meshgrid(coords, coords)
    distance = np.maximum(np.maximum(abs(x), abs(y))-field.size/2, 0.)
    blend = np.clip(distance/width, 0., 1.)
    blend = blend*blend*(3-2*blend)
    z = field.height(np.clip(x, -field.size/2, field.size/2), np.clip(y, -field.size/2, field.size/2))*(1-blend)
    n = len(coords)
    r, c = np.indices((n-1, n-1))
    outside = ((coords[c] < -field.size/2) | (coords[c+1] > field.size/2) |
               (coords[r] < -field.size/2) | (coords[r+1] > field.size/2))
    a = (r[outside]*n+c[outside]).ravel()
    faces = np.concatenate([np.column_stack([a, a+1, a+n]), np.column_stack([a+1, a+n+1, a+n])])
    vertices = np.column_stack([x.ravel(), y.ravel(), z.ravel()])
    used, inverse = np.unique(faces, return_inverse=True)
    return vertices[used], inverse.reshape(faces.shape)


def operating_map_size(scene, requested='auto'):
    size = 40. if is_v2(scene.get('config', {})) else 20.
    value = size if str(requested) == 'auto' else float(requested)
    if not np.isfinite(value) or value not in (20., 40.) or value < size:
        raise ValueError('Map size must cover the qualified operating area (20 or 40 m)')
    return value
