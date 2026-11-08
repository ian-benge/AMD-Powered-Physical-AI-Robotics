"""Independent review of captured publications and world-frame ground geometry."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from rover_navigation.core import depth_points, rotation
from rover_navigation.terrain_compute import CpuTerrainCompute


def slerp(a, b, fraction):
    a, b = np.asarray(a, float), np.asarray(b, float)
    a, b = a/np.linalg.norm(a), b/np.linalg.norm(b)
    dot = float(a@b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > .9995:
        q = a+(b-a)*fraction
        return q/np.linalg.norm(q)
    angle = math.acos(min(1., dot))
    return (math.sin((1-fraction)*angle)*a+math.sin(fraction*angle)*b)/math.sin(angle)


def ground_heights(values, size, points):
    h = np.asarray(values, float)
    n = len(h)-1
    u, v = (points[:, 0]+size/2)*n/size, (points[:, 1]+size/2)*n/size
    inside = (u >= 0) & (v >= 0) & (u < n) & (v < n)
    u, v = np.clip(u, 0, n-1e-9), np.clip(v, 0, n-1e-9)
    i, j = u.astype(int), v.astype(int)
    a, b = u-i, v-j
    z00, z10, z01, z11 = h[j, i], h[j, i+1], h[j+1, i], h[j+1, i+1]
    z = np.where(a+b <= 1, z00+a*(z10-z00)+b*(z01-z00),
                 z11+(1-a)*(z01-z11)+(1-b)*(z10-z11))
    return z, inside


def review(audit, session, scene_file, output):
    source = json.loads(audit.read_text())
    scene = json.loads(scene_file.read_text())
    surface = json.loads((scene_file.parent/'terrain.json').read_text())
    traces = {r['stamp']: r for line in (session/'terrain-evidence-events.jsonl').read_text().splitlines()
              if (r := json.loads(line))['kind'] == 'terrain_frame'}
    compute, rows, alignment = CpuTerrainCompute(), [], None
    for capture in source['captures']:
        prefix = audit.parent/capture['evidence']
        metadata = json.loads(prefix.with_suffix('.json').read_text())
        with np.load(prefix.with_suffix('.npz')) as stored:
            arrays = {name: stored[name] for name in stored.files}
        stamp = metadata['stamp']
        row = dict(stamp=stamp, evidence=capture['evidence'])
        trace = traces.get(stamp)
        if trace is None:
            row['integrity_error'] = 'No successful publication trace at this exact input stamp'
            rows.append(row)
            continue
        raw = arrays['depth_bytes'].tobytes()
        checks = dict(input=hashlib.sha256(raw).hexdigest() == trace['input_sha256'],
                      local=hashlib.sha256(arrays['local'].tobytes()).hexdigest() == trace['local']['sha256'],
                      global_map=hashlib.sha256(arrays['map'].tobytes()).hexdigest() == trace['global_map']['sha256'],
                      local_causes=bool(np.array_equal(arrays['local'] >= 90, arrays['local_reasons'] > 0)),
                      global_causes=bool(np.array_equal(arrays['map'] >= 90, arrays['reasons'] > 0)))
        depth, calibration = metadata['depth'], metadata['calibration']['k']
        camera = trace['camera_to_odom']
        origin = trace['current_frame']['origin']
        shape = trace['current_frame']['shape']
        resolution = trace['current_frame']['resolution']
        size = shape[0]*resolution
        center = [v+size/2+resolution/2 for v in origin]
        frame, layers, _ = compute.depth(data=raw, encoding=depth['encoding'], width=depth['width'],
            height=depth['height'], step=depth['step'], bigendian=depth['is_bigendian'], k=calibration,
            translation=camera['translation'], quaternion=camera['quaternion'],
            center=center, resolution=resolution, size=size, max_slope=12., max_step=.03)
        checks['cpu_current_frame'] = hashlib.sha256(frame.data.tobytes()).hexdigest() == trace['current_frame']['sha256']
        row.update(integrity=checks, current_hazards=int((frame.data >= 90).sum()),
                   current_slope_hazards=int(((frame.data >= 90) & (layers['slope'] > 12.)).sum()),
                   current_height_hazards=int(((frame.data >= 90) & (layers['height_span'] > .03)).sum()),
                   clearing=trace['terrain_evidence'])
        truth = metadata.get('evaluator_truth', {})
        if truth.get('available'):
            before, after = truth['before'], truth['after']
            extrinsic = truth['camera_to_base']
            if before['frame'] != 'world' or before['child'] != extrinsic['parent']:
                row['geometry_error'] = 'Truth/extrinsic frames do not describe world to the same base link'
                rows.append(row)
                continue
            fraction = (stamp-before['stamp'])/(after['stamp']-before['stamp']) if after['stamp'] > before['stamp'] else 0.
            position = np.array(before['position'])+(np.array(after['position'])-before['position'])*fraction
            row['evaluator_position_world_m'] = position.tolist()
            base_r = rotation(slerp(before['quaternion'], after['quaternion'], fraction))
            camera_r = base_r @ rotation(extrinsic['rotation_xyzw'])
            camera_p = position+base_r @ np.array(extrinsic['translation'])
            points = depth_points(raw, depth['encoding'], depth['width'], depth['height'], depth['step'],
                                  depth['is_bigendian'], calibration)
            world = points @ camera_r.T+camera_p
            expected_z, valid = ground_heights(surface['heights'], surface['size_m'], world)
            for obj in scene['objects']:
                valid &= np.hypot(world[:, 0]-obj['xyz'][0], world[:, 1]-obj['xyz'][1]) > obj['radius_m']+.15
            valid &= np.hypot(world[:, 0]-position[0], world[:, 1]-position[1]) > .5
            residual = world[valid, 2]-expected_z[valid]
            if not residual.size:
                row['geometry_error'] = 'No observed background ground outside object/rover envelopes'
                rows.append(row)
                continue
            row['world_ground'] = dict(samples=len(residual), bias_m=float(np.median(residual)),
                rms_m=float(np.sqrt(np.mean(residual**2))), p95_abs_m=float(np.percentile(np.abs(residual), 95)),
                method='Image-time world/base pose bracket, static camera extrinsic, exact terrain triangles; no fitted offset or orientation')
            odom_r, odom_p = rotation(camera['quaternion']), np.array(camera['translation'])
            if alignment is None:
                world_from_odom_r = camera_r @ odom_r.T
                world_from_odom_p = camera_p-world_from_odom_r@odom_p
                alignment = dict(stamp=stamp, rotation=world_from_odom_r.tolist(), translation=world_from_odom_p.tolist(),
                                 method='Single first-pose alignment, fixed for the entire recording; no trajectory fit')
            estimate_r = np.array(alignment['rotation']) @ odom_r
            estimate_p = np.array(alignment['rotation']) @ odom_p+alignment['translation']
            angle = math.acos(float(np.clip((np.trace(camera_r.T@estimate_r)-1)/2, -1, 1)))
            row['vio_from_first_pose'] = dict(position_error_m=float(np.linalg.norm(estimate_p-camera_p)),
                                            orientation_error_deg=math.degrees(angle))
        else:
            row['geometry_error'] = truth.get('reason', 'Image-time evaluator truth is unavailable')
        rows.append(row)
    physical = [r['world_ground'] for r in rows if 'world_ground' in r]
    vio = [r['vio_from_first_pose'] for r in rows if 'vio_from_first_pose' in r]
    complete = bool(rows) and all('integrity' in r and all(r['integrity'].values()) for r in rows)
    geometry = bool(rows) and len(physical) == len(rows) and all(
        r['samples'] >= 1000 and r['p95_abs_m'] <= .015 for r in physical)
    outer = [r for r in rows if 'world_ground' in r and
             max(abs(v) for v in r['evaluator_position_world_m'][:2]) >= 12.]
    terrain_config = scene.get('config', {}).get('terrain', {})
    requires_outer = terrain_config.get('profile_version') == 2
    outer_pass = not requires_outer or bool(outer)
    report = dict(result='PUBLICATION_AND_RENDERED_GEOMETRY_REVIEW_PASS' if complete and geometry and outer_pass else 'REVIEW_REQUIRED',
        audit=str(audit), scene=str(scene_file), frames=len(rows), paired_world_geometry_frames=len(physical),
        publication_integrity_pass=complete, world_ground_geometry_pass=geometry,
        all_frames_have_world_geometry=bool(rows) and len(physical) == len(rows),
        outer_area_required=requires_outer, outer_area_frames=len(outer), outer_area_review_pass=outer_pass,
        max_ground_p95_abs_m=max((r['p95_abs_m'] for r in physical), default=None),
        max_vio_position_error_m=max((r['position_error_m'] for r in vio), default=None),
        max_vio_orientation_error_deg=max((r['orientation_error_deg'] for r in vio), default=None),
        fixed_alignment=alignment, rows=rows,
        limitations=['Simulator truth is evaluator-only and never supplied to mapping/control.',
                     'Ground statistics exclude conservative object envelopes and the rover footprint; they do not qualify every rock/depth edge.',
                     'Controlled clearing/loss acceptance and protected final kernel review remain separate records.'])
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('rows','fixed_alignment')}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('audit','session','scene','output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    review(args.audit, args.session, args.scene, args.output)
