"""Renderer-specific box verification, separate from geometry-derived labels.

Ogre has RGB-D support but no BoundingBoxCamera. Its check compares retained
geometry masks with measured sensor depth; it is not a renderer instance-ID
camera. RGB-only portions beyond usable depth retain their geometry labels.
"""
import math

import numpy as np


SENSOR_DEPTH = 'sensor_depth_v1'
GAZEBO_BOXES = 'gazebo_bbox'


def box_check_backend(camera):
    engine = camera.get('render_engine', 'ogre2')
    if engine not in ('ogre', 'ogre2'):
        raise ValueError('Unknown capture renderer: ' + str(engine))
    return SENSOR_DEPTH if engine == 'ogre' else GAZEBO_BOXES


def capture_box_contract(config):
    backend = box_check_backend(config['camera'])
    capture = config['capture']
    required = bool(capture.get('require_bbox_check', True))
    if backend == SENSOR_DEPTH and required and not (
            capture.get('require_depth_check') and capture.get('require_visibility_check')):
        raise ValueError('Ogre box verification requires synchronized depth and per-object visibility checks')
    return dict(version=1, backend=backend, required=required,
                renderer=config['camera'].get('render_engine', 'ogre2'),
                renderer_instance_camera=backend == GAZEBO_BOXES,
                geometry_only_beyond_usable_depth=backend == SENSOR_DEPTH)


def _bounds(mask):
    y, x = np.nonzero(mask)
    return [int(x.min()), int(y.min()), int(x.max()+1), int(y.max()+1)] if len(x) else None


def _iou(a, b):
    if a is None or b is None:
        return 0.
    overlap = max(0., min(a[2], b[2])-max(a[0], b[0])) * max(0., min(a[3], b[3])-max(a[1], b[1]))
    union = max(0., a[2]-a[0])*max(0., a[3]-a[1]) + max(0., b[2]-b[0])*max(0., b[3]-b[1]) - overlap
    return overlap/union if union > 0 else 0.


def sensor_depth_box_check(labels, objects, instances, expected, observed, policy, near, far):
    """Verify box extents and rendered-depth support without inventing detections.

    Geometry supplies identity/masks, measured depth supplies independent surface
    support. The usable-depth part is compared separately so a box crossing the
    depth limit keeps its complete RGB extent. A received all-far depth frame is
    legitimate when no geometry in the frame is expected inside usable depth.
    """
    expected, instances = np.asarray(expected), np.asarray(instances)
    errors, checks = [], []
    result = dict(backend=SENSOR_DEPTH, checked=False, agrees=False, objects=checks,
                  errors=errors, matched=0, median_iou=None,
                  mask_source='geometry_projection', support_source='measured_sensor_depth',
                  usable_depth_far_m=float(far))
    if expected.ndim != 2 or instances.shape != expected.shape:
        errors.append('geometry_mask_shape_mismatch')
        return result
    if observed is None:
        errors.append('sensor_depth_missing_or_shape_mismatch')
        return result
    observed = np.asarray(observed)
    # decode_ros_depth returns row-major values, while evaluator fixtures use 2D.
    if observed.ndim == 1 and observed.size == expected.size:
        observed = observed.reshape(expected.shape)
    if observed.shape != expected.shape:
        errors.append('sensor_depth_missing_or_shape_mismatch')
        return result
    result['checked'] = True
    eligible_frame = np.isfinite(expected) & (expected > near) & (expected < far)
    valid_frame = np.isfinite(observed) & (observed > near) & (observed < far)
    result.update(expected_usable_depth_pixels=int(eligible_frame.sum()),
                  observed_usable_depth_pixels=int(valid_frame.sum()))
    if eligible_frame.any() and not valid_frame.any():
        errors.append('sensor_depth_no_usable_pixels')
    lookup = {obj['name']: index for index, obj in enumerate(objects, 1)}
    if len(lookup) != len(objects) or len({label['name'] for label in labels}) != len(labels):
        errors.append('duplicate_object_identity')
    threshold = float(policy['bbox_min_iou'])
    required = float(policy.get('depth_support_fraction', .9))
    tolerance = float(policy.get('depth_support_tolerance_m', .03))
    difference = np.full(expected.shape, np.inf)
    np.subtract(observed, expected, out=difference, where=valid_frame & eligible_frame)
    for label in labels:
        name = label['name']
        mask = instances == lookup[name] if name in lookup else np.zeros(expected.shape, bool)
        eligible = mask & eligible_frame
        support = eligible & valid_frame & (np.abs(difference) <= tolerance)
        count, usable, supported = int(mask.sum()), int(eligible.sum()), int(support.sum())
        box = [float(label[key]) for key in ('xmin', 'ymin', 'xmax', 'ymax')]
        geometry_iou = _iou(box, _bounds(mask)) if all(math.isfinite(v) for v in box) else 0.
        sensor_iou = _iou(_bounds(eligible), _bounds(support)) if usable else None
        geometry_only = bool(count and not usable and np.isfinite(expected[mask]).all()
                             and (expected[mask] >= far).all())
        reasons = []
        if not count or geometry_iou < threshold:
            reasons.append('label_box_geometry_extent_mismatch')
        if usable:
            if supported/usable < required:
                reasons.append('insufficient_sensor_depth_box_support')
            if sensor_iou < threshold:
                reasons.append('sensor_depth_box_extent_mismatch')
        elif not geometry_only:
            reasons.append('invalid_geometry_depth_scope')
        check = dict(name=name, visible_pixels=count, usable_depth_expected_pixels=usable,
                     supported_pixels=supported, support_fraction=supported/usable if usable else None,
                     geometry_box_iou=geometry_iou, sensor_box_iou=sensor_iou,
                     depth_check_scope='geometry_only_beyond_usable_depth' if geometry_only else 'usable_depth',
                     passed=not reasons, reasons=reasons)
        checks.append(check)
        errors.extend(name+': '+reason for reason in reasons)
    result['agrees'] = not errors
    result['matched'] = sum(check['passed'] for check in checks)
    scores = [min(check['geometry_box_iou'], check['sensor_box_iou'])
              if check['sensor_box_iou'] is not None else check['geometry_box_iou'] for check in checks]
    result['median_iou'] = float(np.median(scores)) if scores else None
    return result


def persisted_box_check_errors(check, labels, quality, config):
    """Reject missing, mismatched or incomplete new checks during saved-data audit."""
    required = config['capture'].get('require_bbox_check', True)
    if check is None:
        return ['bounding_box_check_missing'] if required else []
    backend = box_check_backend(config['camera'])
    if check.get('backend', GAZEBO_BOXES) != backend:
        return ['bounding_box_check_backend_mismatch']
    if backend != SENSOR_DEPTH:
        return []  # Historical Gazebo checks retain their recorded comparison policy.
    errors = []
    if not check.get('checked') or not check.get('agrees') or check.get('errors'):
        errors.append('sensor_depth_box_check_failed')
    rows = check.get('objects', [])
    if len(rows) != len(labels) or {r.get('name') for r in rows} != {label['name'] for label in labels}:
        errors.append('sensor_depth_box_checks_incomplete')
    visibility = {v['name']: v for v in quality.get('visibility_check', {}).get('objects', [])}
    policy = config['labels']
    for row in rows:
        name = row.get('name')
        geometry_iou = row.get('geometry_box_iou')
        if not row.get('passed') or not isinstance(geometry_iou, (int, float)) or not math.isfinite(geometry_iou) or geometry_iou < policy['bbox_min_iou']:
            errors.append(str(name)+': invalid_geometry_box_check')
        reference = visibility.get(name)
        if reference is None or row.get('usable_depth_expected_pixels') != reference.get('usable_depth_expected_pixels'):
            errors.append(str(name)+': sensor_depth_box_scope_mismatch')
        if row.get('usable_depth_expected_pixels', 0):
            for field, minimum in [('support_fraction', policy.get('depth_support_fraction', .9)),
                                   ('sensor_box_iou', policy['bbox_min_iou'])]:
                value = row.get(field)
                if not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum:
                    errors.append(str(name)+': invalid_'+field)
        elif row.get('depth_check_scope') != 'geometry_only_beyond_usable_depth' or reference is None or reference.get('depth_check_scope') != row.get('depth_check_scope'):
            errors.append(str(name)+': invalid_geometry_only_box_scope')
    return errors
