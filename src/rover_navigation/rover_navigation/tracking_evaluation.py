"""Independent evaluator only. Never imported by live navigation/tracking."""
import math
from collections import Counter
from rover_training.geometry import CLASS_NAMES, class_id_for


def revisit_evidence(before, after):
    """Persistence alone is insufficient: require a fresh associated re-sighting."""
    initial = {item['track_id']: item for item in before.get('objects', [])}
    final = {item['track_id']: item for item in after.get('objects', [])}
    stamp = after.get('accepted_stamp')
    reobserved = []
    for identifier, old in initial.items():
        item = final.get(identifier)
        if (item is not None and stamp is not None and item['last_stamp'] > old['last_stamp'] and
                -.05 <= stamp-item['last_stamp'] <= .6):
            reobserved.append(identifier)
    return dict(passed=bool(initial) and set(reobserved) == set(initial),
                initial_ids=sorted(initial), retained_ids=sorted(set(initial) & set(final)),
                freshly_reobserved_ids=sorted(reobserved), accepted_stamp=stamp,
                criterion='same confirmed ID, increased last observation stamp, <=0.6 simulated seconds old')


def reference_matches(objects, start, scene):
    truth, estimate = start['truth'], start['estimate']
    yaw = truth['yaw']-estimate['yaw']
    c, s = math.cos(yaw), math.sin(yaw)
    eligible = []
    for obj in scene['objects']:
        label = class_id_for(obj.get('class_name', 'rock'))
        if label is not None and CLASS_NAMES[label] in ('rock', 'yellow_crystal_rock'):
            eligible.append((obj, CLASS_NAMES[label]))
    matches, false = [], []
    for item in objects:
        x, y = item['odom_position'][:2]
        dx, dy = x-estimate['x'], y-estimate['y']
        world = [truth['x']+c*dx-s*dy, truth['y']+s*dx+c*dy]
        choices = []
        for obj, label in eligible:
            distance = math.dist(world, obj['xyz'][:2])
            if label == item['class_name'] and distance <= obj['radius_m']+.10:
                choices.append((distance, obj))
        if not choices:
            false.append(dict(track_id=item['track_id'], world_xy=world, class_name=item['class_name']))
        else:
            distance, obj = min(choices, key=lambda value: value[0])
            matches.append(dict(track_id=item['track_id'], reference=obj['name'], world_xy=world,
                                center_distance_m=distance, reference_radius_m=obj['radius_m']))
    counts = Counter(item['reference'] for item in matches)
    return dict(matches=matches, false_confirmed=false,
                duplicate_references=[name for name, count in counts.items() if count > 1],
                unconfirmed_scene_references=[obj['name'] for obj, _ in eligible if obj['name'] not in counts],
                position_scope='planar surface location within reference radius +0.10 m; not a collection pose',
                recall_scope='unconfirmed list includes out-of-view objects; visibility-qualified recall not measured')
