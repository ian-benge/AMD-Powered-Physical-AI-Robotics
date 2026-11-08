"""Bounded camera-pose corrections for undersized targets in existing plans."""
import math

import numpy as np

from .terrain import Terrain


MAX_REFRAMES = 3
SIZE_FAILURE = 'target_too_small_at_training_resolution'
MARGIN_FAILURE = 'target_outside_frame_margin'
FRAMING_FAILURES = {SIZE_FAILURE, MARGIN_FAILURE, 'target_too_occluded',
                    'target_clipped', 'target_not_visible'}
VIEW_ANGLES = (0., .045, -.045, .09, -.09, .18, -.18, .30, -.30, .48, -.48)


def framing_headroom(framing, capture, camera, difficult=False):
    """Planning preference only; actual exports retain target_framing's gates."""
    if not framing or not framing.get('passed'):
        return False
    fraction = framing.get('visible_fraction', float('nan'))
    side = framing.get('side_at_training_px', float('nan'))
    minimum = capture['target_min_side_px'] * (.5 if difficult else 1)
    visible = .3 if difficult else capture['target_min_visible_fraction']
    if not (math.isfinite(fraction) and fraction >= min(1., visible+.04) and
            math.isfinite(side) and side >= minimum+2):
        return False
    box = framing.get('bbox_xyxy', [])
    if len(box) != 4 or not all(math.isfinite(v) for v in box):
        return False
    if difficult:
        return True
    w,h = camera['width'],camera['height']
    margin = capture['target_margin_fraction']
    return (box[0] >= w*margin+4 and box[1] >= h*margin+4 and
            box[2] <= w*(1-margin)-4 and box[3] <= h*(1-margin)-4)


def clearer_target_view(scene, view, checks, measured_xyz, predict, diagnostics=None):
    """Search a finite set of safe poses for a robustly framed primary target.

    `predict` evaluates geometry only, at capture resolution. It never supplies
    sensor evidence or authorizes export. Capture must acquire and validate a
    fresh synchronized frame after every proposed pose. Scene plans are immutable.
    """
    config = scene['config']
    camera,cap = config['camera'],config['capture']
    if (camera.get('render_engine') != 'ogre' or
            config['terrain'].get('profile') != 'mixed_reachable' or
            config['terrain'].get('profile_version') != 2 or not view.get('target_name')):
        return None
    preflight = checks is None
    if preflight:
        planned = view.get('planned_framing')
        if not planned or framing_headroom(planned,cap,camera,view.get('difficult',False)):
            return None
    elif (not checks.get('errors') or
          not set(checks['errors']) <= FRAMING_FAILURES):
        # Sensor, box, class, calibration and global failures cannot be hidden
        # by a target-only framing correction.
        return None
    history = list(view.get('framing_adjustments', []))
    if len(history) >= MAX_REFRAMES:
        return None
    target = next((obj for obj in scene['objects']+scene.get('regions', [])
                   if obj['name'] == view['target_name']),None)
    if target is None:
        return None
    xy = np.asarray(measured_xyz[:2],dtype=float)
    target_xy = np.asarray(target['xyz'][:2],dtype=float)
    offset = xy-target_xy
    distance = float(np.linalg.norm(offset))
    if (not np.isfinite(xy).all() or not math.isfinite(distance) or distance <= 0 or
            not all(math.isfinite(v) for v in view['rpy'])):
        return None
    field = Terrain(scene['terrain']['base_heights'],scene['terrain']['size_m'])
    low,high = cap['distance_m']
    old_poses = [np.asarray(other['xyz']+other['rpy'],dtype=float)
                 for other in scene.get('views',[]) if other['id'] != view['id']]
    predictions = 0
    # Same azimuth and small orbit changes come first. Retain the target,
    # distance policy, off-axis difficult-view offset and export-class schedule.
    for scale in (1., .94, .86):
        for angle in VIEW_ANGLES:
            if not preflight and scale == 1. and angle == 0.:
                continue
            ca,sa = math.cos(angle),math.sin(angle)
            shifted = np.array([ca*offset[0]-sa*offset[1],sa*offset[0]+ca*offset[1]])*scale
            x,y = map(float,target_xy+shifted)
            yaw = float(view['rpy'][2]+angle)
            if max(abs(x),abs(y)) > field.size/2-1.2:
                continue
            if any(math.hypot(x-o['xyz'][0],y-o['xyz'][1]) < .42+o['radius_m']
                   for o in scene['objects']):
                continue
            cy,sy = math.cos(yaw),math.sin(yaw)
            mx,my = camera['mount_xyz'][:2]
            lens = np.array([x+mx*cy-my*sy,y+mx*sy+my*cy])
            gap = float(np.linalg.norm(lens-target_xy))
            if not low <= gap <= high:
                continue
            if any(np.linalg.norm(lens-np.asarray(o['xyz'][:2])) < o['radius_m']+camera['near_m']+.08
                   for o in scene['objects']):
                continue
            gx,gy = field.gradient(x,y)
            if math.hypot(gx,gy) > math.tan(math.radians(config['terrain']['max_slope_deg'])):
                continue
            roll,pitch = math.atan(-gx*sy+gy*cy),-math.atan(gx*cy+gy*sy)
            candidate = dict(view,xyz=[x,y,float(field.height(x,y))+.09],rpy=[roll,pitch,yaw])
            pose = np.asarray(candidate['xyz']+candidate['rpy'])
            if any(np.allclose(pose,other,rtol=0,atol=1e-5) for other in old_poses):
                continue
            # Predict settled height, then test small height/pitch perturbations.
            # Reject a pose that merely moves the same threshold rounding issue.
            predictions += 1
            nominal = dict(candidate,xyz=[x,y,candidate['xyz'][2]-.01])
            framing = predict(nominal)
            if not framing_headroom(framing,cap,camera,view.get('difficult',False)):
                continue
            robust = True
            for dz,dp in ((-.006,-.002),(.006,.002)):
                probe = dict(nominal,xyz=[x,y,nominal['xyz'][2]+dz],
                             rpy=[roll,pitch+dp,yaw])
                predictions += 1
                if not framing_headroom(predict(probe),cap,camera,view.get('difficult',False)):
                    robust = False
                    break
            if not robust:
                continue
            adjustment = dict(reason='target_framing_preflight' if preflight else 'target_framing_recovery',
                rejected_errors=[] if preflight else list(checks['errors']),
                rejected_diagnostics=str(diagnostics) if diagnostics is not None else None,
                from_measured_xyz=list(map(float,measured_xyz)),to_xyz=candidate['xyz'],
                orbit_angle_rad=angle,distance_scale=scale,predicted_framing=framing,
                geometry_predictions=predictions,settling_probes=3,
                acceptance_policy_unchanged=True,fresh_sensor_acceptance_required=True)
            return dict(candidate,framing_adjustments=history+[adjustment])
    return None


def closer_target_view(scene, view, checks, measured_xyz):
    """Propose a closer teleport pose only for an otherwise valid frame.

    The captured image must pass every normal check after this move. Keep the
    original target, azimuth, difficult-view offset and configured distance
    range. Reject endpoints that violate the planner's collision/slope limits.
    No view or scene manifest is modified in place.
    """
    config = scene['config']
    cap, camera = config['capture'], config['camera']
    framing = checks.get('target_framing') or {}
    margin_recovery = checks.get('errors') == [MARGIN_FAILURE]
    if margin_recovery:
        box = framing.get('bbox_xyxy')
        margin = cap['target_margin_fraction']
        w,h = camera['width'],camera['height']
        if (camera.get('render_engine') != 'ogre' or
                config['terrain'].get('profile_version') != 2 or view.get('difficult') or
                not box or len(box) != 4 or
                not (w*margin <= box[0] < box[2] <= w*(1-margin) and
                     h*margin-8 <= box[1] < h*margin and box[3] <= h*(1-margin))):
            return None
    elif checks.get('errors') != [SIZE_FAILURE]:
        return None
    history = list(view.get('framing_adjustments', []))
    if len(history) >= MAX_REFRAMES:
        return None
    target = next((obj for obj in scene['objects'] + scene.get('regions', [])
                   if obj['name'] == view.get('target_name')), None)
    if target is None:
        return None
    side = framing.get('side_at_training_px', 0)
    if not math.isfinite(side) or side <= 0:
        return None
    minimum = cap['target_min_side_px'] * (.5 if view.get('difficult', False) else 1)
    target_xy = np.asarray(target['xyz'][:2], dtype=float)
    xy = np.asarray(measured_xyz[:2], dtype=float)
    offset = xy-target_xy
    distance = float(np.linalg.norm(offset))
    lens = float(np.linalg.norm(camera['mount_xyz'][:2]))
    low, high = cap['distance_m']
    # Triangle inequality keeps the lens at least low metres from the target,
    # including the difficult-view yaw offset and camera's forward mount.
    lower = max(low, target['radius_m']+camera['near_m']+.25) + lens
    if not math.isfinite(distance) or distance <= lower+.03:
        return None
    # Aim above the acceptance floor so rounding/settling do not cause another
    # marginal result; move at most 25% of the current distance per correction.
    factor = .95 if margin_recovery else float(np.clip(side/(minimum+2), .75, .90))
    desired = max(lower, distance*factor)
    if distance-desired < .03:
        return None
    terrain = scene['terrain']
    field = Terrain(terrain['base_heights'], terrain['size_m'])
    yaw = view['rpy'][2]
    cy, sy = math.cos(yaw), math.sin(yaw)
    mount_x, mount_y = camera['mount_xyz'][:2]
    camera_offset = np.array([mount_x*cy-mount_y*sy, mount_x*sy+mount_y*cy])
    # If the preferred endpoint is blocked, inspect closer endpoints within
    # the same configured range. This is a teleport, not a driving route.
    for candidate_distance in np.linspace(desired, lower, 5):
        candidate = target_xy + offset/distance*candidate_distance
        x, y = map(float, candidate)
        if max(abs(x), abs(y)) > field.size/2-1.2:
            continue
        if any(np.linalg.norm(candidate-np.asarray(obj['xyz'][:2])) < .42+obj['radius_m']
               for obj in scene['objects']):
            continue
        camera_xy = candidate+camera_offset
        gap = float(np.linalg.norm(camera_xy-target_xy))
        if not low <= gap <= high:
            continue
        if any(np.linalg.norm(camera_xy-np.asarray(obj['xyz'][:2])) < obj['radius_m']+camera['near_m']+.08
               for obj in scene['objects']):
            continue
        gx, gy = field.gradient(x, y)
        if math.hypot(gx, gy) > math.tan(math.radians(config['terrain']['max_slope_deg'])):
            continue
        roll = math.atan(-gx*sy+gy*cy)
        pitch = -math.atan(gx*cy+gy*sy)
        new_xyz = [x, y, float(field.height(x, y))+.09]
        adjustment = {'reason':MARGIN_FAILURE if margin_recovery else SIZE_FAILURE,
                      'side_at_training_px':float(side),
                      'minimum_side_at_training_px':float(minimum),
                      'from_measured_xyz':list(map(float, measured_xyz)), 'to_xyz':new_xyz}
        return dict(view, xyz=new_xyz, rpy=[roll, pitch, yaw],
                    framing_adjustments=history+[adjustment])
    return None
