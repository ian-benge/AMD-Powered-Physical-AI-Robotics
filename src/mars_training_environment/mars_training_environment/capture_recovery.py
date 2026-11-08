"""Bounded alternative views for localized incidental depth-support rejections.

This changes only the proposed capture pose. Geometry labels and every frame
acceptance threshold stay unchanged; missing sensors and global errors stop.
"""
import math

import numpy as np

from .terrain import Terrain

YAW_OFFSETS = (.035, -.035, .070, -.070)
EDGE_YAW_OFFSETS = (.035, .070, .105, .140)
LOCALIZED_OFFSETS = (.035,-.035,.18,-.18,.35,-.35,.60,-.60)
SUPPORT_REASONS = {'insufficient_sensor_depth_support', 'insufficient_sensor_depth_box_support'}
CONTINUATION_FRAMING = {'target_too_occluded','target_too_small_at_training_resolution',
                        'target_outside_frame_margin','target_clipped'}


def localized_depth_context(scene, view, checks, rendered):
    """Prove a small incidental failure against the complete measured frame.

    A high fraction of tiny objects is not necessarily a widespread sensor
    fault. Use pixel coverage and independent global depth agreement instead.
    This permits a pose proposal only; every original export gate still applies.
    """
    config=scene['config'];camera=config['camera'];policy=config['labels']
    continuing=bool(view.get('view_recoveries'))
    if (camera.get('render_engine')!='ogre' or config['terrain'].get('profile')!='mixed_reachable' or
            config['terrain'].get('profile_version')!=2 or
            (not (checks.get('target_framing') or {}).get('passed') and not continuing)):
        return None
    failures=set()
    for error in checks.get('errors',[]):
        if continuing and error in CONTINUATION_FRAMING:continue
        name,separator,reason=error.partition(': ')
        if not separator or reason not in SUPPORT_REASONS:return None
        failures.add(name)
    visibility=rendered['quality'].get('visibility_check',{})
    rows={row['name']:row for row in visibility.get('objects',[])}
    if (not visibility.get('checked') or not failures or view.get('target_name') in failures or
            not rows.get(view.get('target_name'),{}).get('passed')):
        return None
    expected=np.asarray(rendered['expected']);instances=np.asarray(rendered['instances'])
    observed=rendered.get('observed')
    shape=(int(camera['height']),int(camera['width']))
    if observed is None or expected.shape!=shape or instances.shape!=shape:return None
    observed=np.asarray(observed)
    if observed.size!=expected.size:return None
    observed=observed.reshape(shape)
    near,far=camera['near_m'],camera['far_m']
    eligible=np.isfinite(expected)&(expected>near)&(expected<far)
    valid=eligible&np.isfinite(observed)&(observed>near)&(observed<far)
    count=int(eligible.sum())
    if count<10000 or int(valid.sum())/count<.95:return None
    errors=np.abs(observed[valid]-expected[valid])
    median,p95=map(float,np.percentile(errors,[50,95]))
    support=float((errors<=policy['depth_support_tolerance_m']).mean())
    if median>.002 or p95>.01 or support<.98:return None
    identities={obj['name']:i for i,obj in enumerate(rendered['objects'],1)}
    affected=0;objects={}
    for name in failures:
        row=rows.get(name,{})
        fraction,object_median=row.get('support_fraction'),row.get('median_error_m')
        mask=eligible&(instances==identities.get(name,-1))
        pixels=int(mask.sum())
        if (not pixels or not isinstance(fraction,(int,float)) or not math.isfinite(fraction) or
                not 0.<=fraction<policy['depth_support_fraction'] or
                not isinstance(object_median,(int,float)) or not math.isfinite(object_median) or
                object_median<0 or int((mask&valid).sum())/pixels<.9):
            return None
        affected+=pixels
        objects[name]=dict(usable_pixels=pixels,rejected_support_fraction=fraction,
                           median_error_m=object_median)
    if affected/count>.02:return None
    return dict(version=2,kind='localized_incidental_depth',objects=objects,
        expected_usable_pixels=count,valid_fraction=float(valid.sum()/count),
        global_support_fraction=support,global_median_error_m=median,global_p95_error_m=p95,
        affected_pixel_fraction=affected/count,full_frame_acceptance_required=True,
        primary_sensor_passed=True,continuing_after_framing_rejection=not (checks.get('target_framing') or {}).get('passed',False))


def terrain_visibility_context(scene, view, checks, rendered, terrain_mesh):
    """Identify a far, clipped incidental surface rendered as terrain instead.

    This authorizes another viewpoint, never an export. The raw frame still
    fails its original checks. Require independent terrain agreement at the
    rejected pixels, valid sensor coverage, a good primary target and a single
    horizontal edge shared by the affected incidental objects.
    """
    from .annotation import rasterize
    from rover_training.geometry import camera_rigid_from_base

    config = scene['config']
    camera = config['camera']
    if (camera.get('render_engine') != 'ogre' or
            config['terrain'].get('profile') != 'mixed_reachable' or
            config['terrain'].get('profile_version') != 2 or
            not (checks.get('target_framing') or {}).get('passed')):
        return None
    failures = set()
    for error in checks.get('errors', []):
        name, separator, reason = error.partition(': ')
        if not separator or reason not in SUPPORT_REASONS:
            return None
        failures.add(name)
    visibility = rendered['quality'].get('visibility_check', {})
    rows = {row['name']:row for row in visibility.get('objects', [])}
    if (not visibility.get('checked') or not failures or not view.get('target_name') or
            view['target_name'] in failures or not rows.get(view['target_name'], {}).get('passed') or
            len(failures) > max(1, len(rows)//4)):
        return None
    labels = {label['name']:label for label in rendered['labels']}
    identities = {obj['name']:i for i,obj in enumerate(rendered['objects'],1)}
    expected, instances = map(np.asarray, (rendered['expected'], rendered['instances']))
    observed = rendered.get('observed')
    shape = (int(camera['height']), int(camera['width']))
    if expected.shape != shape or instances.shape != shape or observed is None:
        return None
    observed = np.asarray(observed)
    if observed.size != expected.size:
        return None
    observed = observed.reshape(shape)
    near,far = camera['near_m'],camera['far_m']
    tolerance = config['labels']['depth_support_tolerance_m']
    history = view.get('view_recoveries', [])
    previous = history[0].get('terrain_visibility_context', {}) if history else {}
    previous_objects = previous.get('objects', {})
    evidence = {}; masks = {}
    for name in failures:
        row, label = rows.get(name, {}), labels.get(name, {})
        fraction = row.get('support_fraction')
        if (not label or not isinstance(fraction, (int,float)) or not math.isfinite(fraction) or
                not .5 <= fraction < config['labels']['depth_support_fraction']):
            return None
        bounds = label.get('bbox_xyxy', [])
        if len(bounds) != 4:
            return None
        if label.get('truncated') and bounds[0] == 0 and bounds[2] < shape[1]:
            side = 'left'
        elif label.get('truncated') and bounds[2] == shape[1] and bounds[0] > 0:
            side = 'right'
        else:
            # Later bounded views may already have brought the original object
            # fully inside the frame. Never extend this to new failing objects.
            side = previous_objects.get(name, {}).get('side')
        if side not in ('left','right'):
            return None
        mask = instances == identities.get(name, -1)
        depths = expected[mask]
        eligible = mask & np.isfinite(expected) & (expected > near) & (expected < far)
        valid = eligible & np.isfinite(observed) & (observed > near) & (observed < far)
        if (not eligible.any() or not np.isfinite(depths).all() or
                depths.min() < .9*far or int(valid.sum())/int(eligible.sum()) < .9):
            return None
        difference = np.zeros(shape)
        np.subtract(observed, expected, out=difference, where=valid)
        bad = valid & (np.abs(difference) > tolerance)
        if not bad.any() or not (difference[bad] > tolerance).all():
            return None
        evidence[name] = dict(side=side, bbox_xyxy=list(bounds),
            expected_depth_min_m=float(depths.min()), expected_depth_max_m=float(depths.max()),
            usable_pixels=int(eligible.sum()), valid_pixels=int(valid.sum()),
            rejected_support_fraction=float(fraction), mismatched_pixels=int(bad.sum()))
        masks[name] = bad
    if len({row['side'] for row in evidence.values()}) != 1:
        return None
    xyz,quat = rendered['pose']
    r,t = map(np.asarray,camera_rigid_from_base(xyz,quat,rendered['extrinsics']))
    vertices,faces = terrain_mesh
    ground = rasterize(vertices@r.T+t,faces,rendered['intrinsics'],
        1e-4 if camera.get('near_clip_mode') == 'occlude' else near,
        camera.get('rgb_far_m',far),far_clip_mode=camera.get('far_clip_mode','radial'))
    threshold = min(.003,tolerance/10)
    for name,bad in masks.items():
        difference = np.abs(ground[bad]-observed[bad])
        agreement = float((difference <= threshold).mean())
        if not np.isfinite(difference).all() or agreement < .95:
            return None
        evidence[name].update(terrain_agreement_fraction=agreement,
            terrain_agreement_tolerance_m=threshold,
            terrain_median_error_m=float(np.median(difference)))
    return dict(version=1,objects=evidence,full_frame_acceptance_required=True)


def incidental_depth_view(scene, view, checks, quality, measured_xyz, diagnostics,
                          recovery_context=None):
    config = scene['config']
    if (config['camera'].get('render_engine') != 'ogre' or
            config['terrain'].get('profile') != 'mixed_reachable' or
            config['terrain'].get('profile_version') != 2):
        return None
    history = list(view.get('view_recoveries', []))
    continuing=(bool(history) and (recovery_context or {}).get('version')==2 and
                (recovery_context or {}).get('kind')=='localized_incidental_depth' and
                (recovery_context or {}).get('primary_sensor_passed') is True)
    if not view.get('target_name') or (not (checks.get('target_framing') or {}).get('passed') and not continuing):
        return None
    failures = set()
    for error in checks.get('errors', []):
        if continuing and error in CONTINUATION_FRAMING:continue
        name, separator, reason = error.partition(': ')
        if not separator or reason not in SUPPORT_REASONS:
            return None
        failures.add(name)
    visibility = quality.get('visibility_check', {})
    rows = visibility.get('objects', [])
    weighted=((recovery_context or {}).get('version')==2 and
              (recovery_context or {}).get('kind')=='localized_incidental_depth' and
              (recovery_context or {}).get('full_frame_acceptance_required') is True and
              set((recovery_context or {}).get('objects',{}))==failures and
              (recovery_context or {}).get('expected_usable_pixels',0)>=10000 and
              (recovery_context or {}).get('valid_fraction',0)>=.95 and
              (recovery_context or {}).get('global_support_fraction',0)>=.98 and
              (recovery_context or {}).get('global_median_error_m',float('inf'))<=.002 and
              (recovery_context or {}).get('global_p95_error_m',float('inf'))<=.01 and
              0<(recovery_context or {}).get('affected_pixel_fraction',0)<=.02)
    if not (checks.get('target_framing') or {}).get('passed') and not weighted:
        return None
    if (not visibility.get('checked') or not failures or view['target_name'] in failures or
            (not weighted and len(failures) > max(1, len(rows)//4))):
        return None
    policy = scene['config']['labels']
    lookup = {r['name']:r for r in rows}
    if not lookup.get(view['target_name'], {}).get('passed'):
        return None
    edge_rows = (recovery_context or {}).get('objects', {})
    edge = ((recovery_context or {}).get('version') == 1 and
            (recovery_context or {}).get('full_frame_acceptance_required') is True and
            set(edge_rows) == failures and bool(edge_rows) and
            len({row.get('side') for row in edge_rows.values()}) == 1 and
            all(row.get('side') in ('left','right') and
                row.get('terrain_agreement_fraction',0) >= .95 for row in edge_rows.values()))
    if history and history[0].get('reason') == 'clipped_incidental_terrain_visibility' and not edge and not weighted:
        return None
    for name in failures:
        row = lookup.get(name, {})
        usable = row.get('usable_depth_expected_pixels', 0)
        fraction = row.get('support_fraction')
        median = row.get('median_error_m')
        if (not usable or not isinstance(fraction, (int, float)) or
                not math.isfinite(fraction) or not (0. if weighted else .5 if edge else .75) <= fraction < policy['depth_support_fraction'] or
                row.get('sensor_valid_depth_pixels', 0)/usable < .9 or
                not isinstance(median, (int, float)) or not math.isfinite(median) or
                median < 0 or median > (float('inf') if weighted else policy['depth_support_tolerance_m'] if edge else
                          min(.01, policy['depth_support_tolerance_m']/3))):
            return None
    offsets = EDGE_YAW_OFFSETS if edge else LOCALIZED_OFFSETS if weighted else YAW_OFFSETS
    if len(history) >= len(offsets):
        return None
    target = next((o for o in scene['objects']+scene.get('regions', [])
                   if o['name'] == view['target_name']), None)
    if target is None:
        return None
    original_yaw = history[0]['original_yaw'] if history else view['rpy'][2]
    original_xyz = history[0].get('original_xyz',view['xyz']) if history else view['xyz']
    first = history[-1].get('candidate_index',len(history)-1)+1 if history else 0
    field = Terrain(scene['terrain']['base_heights'], scene['terrain']['size_m'])
    camera = scene['config']['camera']
    low,high = scene['config']['capture']['distance_m']
    for index in range(first,len(offsets)):
        offset=offsets[index]
        orbit=weighted and not edge and index>=2
        if edge and next(iter(edge_rows.values()))['side']=='right':offset=-offset
        yaw=original_yaw+offset
        x,y,z=map(float,original_xyz if orbit else view['xyz'])
        if orbit:
            dx,dy=x-target['xyz'][0],y-target['xyz'][1]
            ca,sa=math.cos(offset),math.sin(offset)
            x,y=target['xyz'][0]+ca*dx-sa*dy,target['xyz'][1]+sa*dx+ca*dy
            z=float(field.height(x,y))+.09
        if max(abs(x),abs(y)) > field.size/2-1.2:continue
        if any(math.hypot(x-o['xyz'][0],y-o['xyz'][1]) < .42+o['radius_m'] for o in scene['objects']):continue
        cy,sy=math.cos(yaw),math.sin(yaw)
        mx,my=camera['mount_xyz'][:2]
        lens=np.array([x+mx*cy-my*sy,y+mx*sy+my*cy])
        if not low<=float(np.linalg.norm(lens-np.asarray(target['xyz'][:2])))<=high:continue
        if any(np.linalg.norm(lens-np.asarray(o['xyz'][:2])) < o['radius_m']+camera['near_m']+.08
               for o in scene['objects']):continue
        gx,gy=field.gradient(x,y)
        if math.hypot(gx,gy)>math.tan(math.radians(scene['config']['terrain']['max_slope_deg'])):continue
        roll,pitch=math.atan(-gx*sy+gy*cy),-math.atan(gx*cy+gy*sy)
        recovery=dict(reason='clipped_incidental_terrain_visibility' if edge else 'localized_incidental_depth_support',
            original_yaw=float(original_yaw),original_xyz=list(map(float,original_xyz)),
            yaw_offset=offset,candidate_index=index,orbit_about_target=orbit,
            candidate_limit=len(offsets),
            from_measured_xyz=list(map(float,measured_xyz)),rejected_errors=list(checks['errors']),
            rejected_diagnostics=str(diagnostics),acceptance_policy_unchanged=True)
        if edge:recovery['terrain_visibility_context']=recovery_context
        if weighted:recovery['localized_depth_context']=recovery_context
        return dict(view,xyz=[x,y,z],rpy=[roll,pitch,yaw],view_recoveries=history+[recovery])
    return None
