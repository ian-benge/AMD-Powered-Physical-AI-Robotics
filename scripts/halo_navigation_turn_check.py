#!/usr/bin/env python3
"""Measure turn response through the guarded manual input (flat fixture only).

Commands at most 0.3 rad/s and 0.05 m/s for at most 3 wall seconds per motion,
then stops. Truth measures the response; navigation never sees it. The guard's
swept-footprint check predicts motion from the commanded rates, so the rate
ratio and the body point that does not slide sideways (the pivot) are
safety-model calibrations, not tuning.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist

from halo_navigation_mission_check import MissionCheck
from rover_navigation.core import drive_pose


def response(window):
    """Yaw rate, forward speed and pivot x from base_link truth.

    A rigid point at body x = p has lateral velocity v_lat + w*p; the pivot is
    the least-squares p where that vanishes.
    """
    t = np.array([r['stamp'] for r in window])
    x, y = np.array([r['x'] for r in window]), np.array([r['y'] for r in window])
    yaw = np.unwrap([r['yaw'] for r in window])
    dt, heading = np.diff(t), (yaw[1:]+yaw[:-1])/2
    keep = dt > 1e-4
    vx, vy, w = np.diff(x)[keep]/dt[keep], np.diff(y)[keep]/dt[keep], np.diff(yaw)[keep]/dt[keep]
    forward = np.cos(heading[keep])*vx+np.sin(heading[keep])*vy
    lateral = -np.sin(heading[keep])*vx+np.cos(heading[keep])*vy
    return (float(np.polyfit(t-t[0], yaw, 1)[0]), float(np.mean(forward)),
            float(-np.sum(lateral*w)/np.sum(w*w)))


def turn(node, speed, rate, duration):
    start = time.monotonic()
    node.sample('turn_start', dict(speed=speed, rate=rate))
    message = Twist()
    message.linear.x, message.angular.z = speed, rate
    while time.monotonic()-start < duration:
        node.ready('manual')
        node.pub.publish(message)
        node.spin_for(.05)
    node.sample('turn_end', dict(speed=speed, rate=rate))
    end = time.monotonic()
    for _ in range(20):
        node.pub.publish(Twist())
        node.spin_for(.05)
    # Exclude acceleration: use the second half of the commanded window.
    window = [r for r in node.records if r['kind'] == 'truth' and start+duration/2 <= r['wall'] <= end]
    if len(window) < 10:
        raise RuntimeError('Too few truth samples while turning')
    measured, forward, pivot = response(window)
    drift = max(math.hypot(r['x']-window[0]['x'], r['y']-window[0]['y']) for r in window)
    centres = [drive_pose(r['x'], r['y'], r['yaw']) for r in window]
    centre_drift = max(math.dist(c[:2], centres[0][:2]) for c in centres)
    return dict(commanded_m_s=speed, measured_m_s=forward, commanded_rad_s=rate, measured_rad_s=measured,
                ratio=measured/rate, pivot_x_m=pivot, translation_m=drift, drive_center_translation_m=centre_drift)


def run(args):
    session, output = args.session.resolve(), args.output.resolve()
    if json.loads((session/'session.json').read_text()).get('fixture') != 'flat':
        raise ValueError('Turn check is restricted to the flat fixture')
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='in_place_turn_response', passed=False, session=str(session), scope='simulation only')
    with output.open('x') as summary, output.with_suffix('.jsonl').open('x') as events:
        rclpy.init()
        node = MissionCheck(events)
        try:
            report['discovery_wall_s'] = node.discover()
            node.ready('stopped')
            owners = node.owners()
            if owners['/cmd_vel'] != ['navigation_motion_guard'] or owners['/navigation/manual_cmd_vel'] != [node.get_name()]:
                raise RuntimeError('Unexpected command publisher ownership: '+json.dumps(owners))
            answer = node.call('confirm_clear_start')
            if not answer['success'] and not answer['message'].startswith('Launch clearance already confirmed;'):
                raise RuntimeError(answer['message'])
            if not node.call('manual')['success']:
                raise RuntimeError('Manual selection failed')
            node.spin_for(.3)
            motions = [(0., args.rate), (0., -args.rate)]
            if args.arcs:
                motions += [(.05, args.rate), (.05, -args.rate), (.05, args.rate/2), (.05, -args.rate/2)]
            report['turns'] = [turn(node, speed, rate, args.duration) for speed, rate in motions]
            report['mean_ratio'] = sum(t['ratio'] for t in report['turns'])/len(report['turns'])
            report['pivot_x_range_m'] = [min(t['pivot_x_m'] for t in report['turns']),
                                         max(t['pivot_x_m'] for t in report['turns'])]
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
        finally:
            node.pub.publish(Twist())
            try:
                report['cleanup_stop'] = node.call('stop')
                node.spin_for(.3)
            except Exception as error:
                report['cleanup_error'] = str(error)
                report['passed'] = False
            report['last_status'] = node.latest.get('status')
            node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--rate', type=float, default=.3)
    parser.add_argument('--duration', type=float, default=3.)
    parser.add_argument('--arcs', action='store_true', help='Also drive four 0.05 m/s arcs')
    args = parser.parse_args()
    if not .05 <= args.rate <= .3 or not 1. <= args.duration <= 3.:
        parser.error('Limit rate to [0.05,0.3] rad/s and duration to [1,3] wall seconds')
    raise SystemExit(run(args))
