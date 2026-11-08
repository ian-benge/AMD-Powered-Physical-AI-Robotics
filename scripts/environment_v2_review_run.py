#!/usr/bin/env python3
"""Read-only timing, speed, contact and connected-traversal review.

Uses recorded evaluator truth for measurement only. Ground error and fixed
first-pose localization drift remain the separate rendered-publication review.
"""
import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def contact_wiring(session):
    """Gazebo Contact publishes events only; silence needs verified wiring."""
    paths = [session/'world.sdf', session/'robot.urdf', session/'bridge.yaml',
             session.parent/'control'/(session.name+'.log')]
    result = dict(verified=False, source_sha256={},
        semantics='The Gazebo Contact system publishes only nonempty contact events.',
        source='https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc')
    try:
        import yaml
        world, robot = ET.parse(paths[0]).getroot(), ET.parse(paths[1]).getroot()
        sensors = [g.find('sensor[@type="contact"]') for g in robot.findall('gazebo')
                   if g.get('reference') == 'base_link']
        sensors = [s for s in sensors if s is not None]
        bridges = yaml.safe_load(paths[2].read_text())
        log = paths[3].read_text(errors='replace')
        chassis_collision = robot.find('link[@name="base_link"]/collision[@name="base_link_collision"]') is not None
        if not chassis_collision:
            # URDF permits unnamed collisions; inspect the actual sdformat
            # conversion used by Gazebo instead of guessing its generated name.
            converted = subprocess.run(['gz', 'sdf', '-p', str(paths[1])],
                capture_output=True, text=True, timeout=20, check=True)
            converted_root = ET.fromstring(converted.stdout)
            chassis_collision = converted_root.find('.//link[@name="base_link"]/collision[@name="base_link_collision"]') is not None
            result['urdf_to_sdf_sha256'] = hashlib.sha256(converted.stdout.encode()).hexdigest()
        checks = dict(
            contact_system=any(p.get('name') == 'gz::sim::systems::Contact' for p in world.findall('.//plugin')),
            chassis_collision=chassis_collision,
            chassis_sensor=any(s.findtext('always_on') == 'true' and
                s.findtext('contact/collision') == 'base_link_collision' and
                s.findtext('contact/topic') == '/navigation/chassis_contacts' for s in sensors),
            bridge_configuration=any(b.get('direction') == 'GZ_TO_ROS' and
                b.get('gz_topic_name') == b.get('ros_topic_name') == '/navigation/chassis_contacts' and
                b.get('gz_type_name') == 'gz.msgs.Contacts' and
                b.get('ros_type_name') == 'ros_gz_interfaces/msg/Contacts' for b in bridges),
            bridge_started=any('Creating GZ->ROS Bridge:' in line and
                '/navigation/chassis_contacts (gz.msgs.Contacts)' in line and
                '/navigation/chassis_contacts (ros_gz_interfaces/msg/Contacts)' in line for line in log.splitlines()))
        result.update(checks=checks, verified=all(checks.values()),
            source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    except (OSError, ValueError, TypeError, ET.ParseError, ImportError, subprocess.SubprocessError) as error:
        result['error'] = str(error)
    return result


def realtime_windows(truth, warmup=10., window=5.):
    if not truth:return []
    wall=np.array([r['wall_monotonic'] for r in truth])
    sim=np.array([r['sim_time'] for r in truth])
    start=wall[0]+warmup
    boundaries=np.arange(start,wall[-1]-window+1e-9,window)
    return [float((np.interp(t+window,wall,sim)-np.interp(t,wall,sim))/window) for t in boundaries]


def review(session, baseline=None, traversal=True):
    event_file=session/'mission-events.jsonl';terrain_file=session/'terrain-evidence-events.jsonl'
    events=rows(event_file);terrain=[r for r in rows(terrain_file) if r['kind']=='terrain_frame']
    truth=[r for r in events if r['kind']=='truth']
    status=[r for r in events if r['kind']=='status']
    if not truth or not terrain or not status:raise ValueError('Missing truth, publication or guard records')
    moving=[r for r in status if any(abs(v)>1e-8 for v in r.get('command',[]))]
    first_motion=moving[0]['sim_time'] if moving else math.inf
    contact_records=[r for r in events if r['kind']=='chassis_contacts']
    contacts=sum(r['count'] for r in contact_records)
    contact_contract=contact_wiring(session)
    faults={}
    for r in status:
        fault=r.get('fault')
        when=fault.get('sim_time') if fault else None
        if isinstance(when,(int,float)) and math.isfinite(when) and when>=first_motion:
            faults[(fault.get('reason'),fault.get('sim_time'))]=fault
    warm_stamp=terrain[0]['stamp']+10.
    latencies=[r['depth_to_local_publication_seconds'] for r in terrain if r['stamp']>=warm_stamp]
    rtf=realtime_windows(truth)
    if not latencies or not rtf:raise ValueError('Insufficient steady-state timing records')
    poses=np.array([r['pose'][:2] for r in truth]);axis=np.abs(poses).max(axis=1)
    reached=np.flatnonzero(axis>=12.)
    returned=bool(len(reached) and np.linalg.norm(poses[-1])<=1.3 and reached[0]<len(poses)-1)
    stages=[];last=-math.inf;previous=None
    for speed in (.05,.10,.20):
        samples=[r for r in moving if r['sim_time']>last and abs(r['command'][0]-speed)<1e-6]
        first=samples[0] if samples else None
        stopped_between=previous is None or bool(first and any(
            r.get('mode')=='stopped' and not any(abs(v)>1e-8 for v in r.get('command',[]))
            and previous['wall_monotonic']<r['wall_monotonic']<first['wall_monotonic'] for r in status))
        stages.append(dict(speed_mps=speed,observed=bool(samples),
            first_stamp=first['sim_time'] if first else None,stopped_before_next_stage=stopped_between))
        if first:last=first['sim_time'];previous=first
    cap=all(0<=r['command'][0]<=.2+1e-9 and abs(r['command'][1])<=.3+1e-9 for r in moving)
    duration=truth[-1]['wall_monotonic']-events[0]['wall_monotonic']
    median=float(np.median(rtf));p95=float(np.percentile(latencies,95))
    relative=None
    if baseline:
        reference=json.loads(baseline.read_text())
        relative=median/reference['median_real_time_factor']
    checks=dict(real_time_factor=median>=.8,depth_to_local_publication=p95<=.2,
        matched_baseline_ratio=relative is None or relative>=.8,manual_caps=cap,
        no_contact=contacts==0 and (bool(contact_records) or contact_contract['verified']),
        no_live_guard_faults=not faults,bounded_run=duration<=600.)
    if traversal:checks.update(progressive_speed_stages=all(r['observed'] for r in stages),
        stopped_between_speed_stages=all(r['stopped_before_next_stage'] for r in stages),
        explicit_manual_mode=bool(moving) and all(r.get('mode')=='manual' for r in moving),
        reached_twelve_m_axis=bool(len(reached)),returned_to_launch_region=returned)
    return dict(result='RUN_METRICS_PASS_RENDERED_REVIEW_STILL_REQUIRED' if all(checks.values()) else 'REVIEW_REQUIRED',
        checks=checks,session=str(session),source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (event_file,terrain_file,Path(__file__))},
        median_real_time_factor=median,rtf_five_second_windows=rtf,matched_baseline_ratio=relative,
        steady_state_depth_to_local_p95_seconds=p95,latency_samples=len(latencies),
        latency_scope='Mapper application callback receipt through successful local-map publication; includes TF waiting and requested compute/publication work.',
        maximum_axis_coordinate_m=float(axis.max()),maximum_radial_distance_m=float(np.linalg.norm(poses,axis=1).max()),
        initial_truth_xy=poses[0].tolist(),final_truth_xy=poses[-1].tolist(),final_center_distance_m=float(np.linalg.norm(poses[-1])),
        returned_to_launch_region=returned,launch_region_radius_m=1.3,wall_duration_seconds=duration,
        speed_stages=stages,contacts=contacts,contact_samples=len(contact_records),guard_faults=list(faults.values()),
        event_only_contact_wiring=contact_contract,
        limitations=['Simulator truth is evaluator-only.',
            'Known-ground traversal relies on the unchanged production swept-footprint guard; no evaluator route is sent to control.',
            'Review rendered-depth ground error, localization drift, native health/kernel evidence and operator observations separately.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--baseline',type=Path);p.add_argument('--baseline-run',action='store_true')
    a=p.parse_args();report=review(a.session,a.baseline,not a.baseline_run)
    with a.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256','rtf_five_second_windows','guard_faults')}))
