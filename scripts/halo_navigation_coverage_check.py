#!/usr/bin/env python3
"""Bounded Phase Two simulation check through the existing guarded manual HTTP API.

Truth is recorded only here for independent evaluation. --drive-seconds permits
forward input at 0.05 m/s; this script never publishes a ROS velocity or goal.
"""
import argparse
import json
import math
from pathlib import Path
import time
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rover_navigation.ros_utils import stamp_seconds


def post(route, data=None):
    payload = json.dumps(data).encode() if data is not None else b''
    with urlopen(Request('http://127.0.0.1:8774/'+route, data=payload, method='POST',
                         headers={'Content-Type': 'application/json'}), timeout=2.) as reply:
        result = json.load(reply)
    if not result.get('success'):
        raise RuntimeError(result.get('message', 'Control request failed'))
    return result


class Observer(Node):
    def __init__(self, stream):
        super().__init__('navigation_coverage_evaluator')
        self.stream, self.latest, self.records = stream, {}, []
        for name in ('status', 'mapping_status'):
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE) if name == 'mapping_status' else 1
            self.create_subscription(String, '/navigation/'+name,
                                     lambda m, n=name: self.record(n, json.loads(m.data)), qos)
        self.create_subscription(Odometry, '/odom_gt', lambda m: self.odom('truth', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom('estimate', m), qos_profile_sensor_data)

    def odom(self, kind, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        valid = (all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)) and
                 .9 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.1)
        if kind == 'estimate':
            valid = valid and msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link' and all(
                                  math.isfinite(msg.pose.covariance[i]) and 0 <= msg.pose.covariance[i] <= 1.
                                  for i in (0, 7, 14, 21, 28, 35))
        self.record(kind, dict(stamp=stamp_seconds(msg.header.stamp), valid=valid,
                              x=p.x if math.isfinite(p.x) else None, y=p.y if math.isfinite(p.y) else None,
                              yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) if valid else None))

    def record(self, kind, data):
        row = dict(kind=kind, wall=time.monotonic(), **data)
        self.latest[kind] = row
        self.records.append(row)
        self.stream.write(json.dumps(row, allow_nan=False)+'\n')

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(.01, max(0., end-time.monotonic())))

    def fresh(self, kind, maximum=.6):
        row = self.latest.get(kind, {})
        return row if time.monotonic()-row.get('wall', -math.inf) < maximum else None


def compare(start, end):
    """Relative displacement comparison avoids confusing spawn height with planar drift."""
    truth, estimate = end['truth'], end['estimate']
    tx, ty = truth['x']-start['truth']['x'], truth['y']-start['truth']['y']
    ex, ey = estimate['x']-start['estimate']['x'], estimate['y']-start['estimate']['y']
    # Coordinate origins can have a fixed initial yaw. Compare in each start's
    # body axes, so that initial orientation is not itself counted as drift.
    def relative(x, y, yaw):
        return math.cos(yaw)*x+math.sin(yaw)*y, -math.sin(yaw)*x+math.cos(yaw)*y
    tx, ty = relative(tx, ty, start['truth']['yaw'])
    ex, ey = relative(ex, ey, start['estimate']['yaw'])
    angle = (estimate['yaw']-start['estimate']['yaw'])-(truth['yaw']-start['truth']['yaw'])
    angle = math.atan2(math.sin(angle), math.cos(angle))
    return dict(truth_displacement_m=math.hypot(tx, ty), estimated_displacement_m=math.hypot(ex, ey),
                relative_endpoint_error_m=math.hypot(ex-tx, ey-ty), relative_yaw_error_deg=abs(math.degrees(angle)),
                final_receipt_skew_wall_seconds=abs(estimate['wall']-truth['wall']))


def run(args):
    session, output = args.session.resolve(), args.output.resolve()
    manifest = json.loads((session/'session.json').read_text())
    fixture = manifest.get('fixture', '')
    if args.expect_known_edge and fixture != 'flat_edge':
        raise ValueError('Known-edge driving is restricted to flat_edge')
    if args.drive_seconds and fixture not in ('flat', 'flat_edge', ''):
        raise ValueError('Drive only inspected flat/flat_edge fixtures or scene 50000')
    if not fixture and Path(manifest['scene']).parent.name != 'scene_050000':
        raise ValueError('Martian trials are restricted to scene 50000')
    if args.confirm_clear_launch and fixture == 'slope':
        raise ValueError('The slope fixture intersects the 1.1 m seed; observe it without clearance seeding')
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='phase2_coverage', passed=False, fixture=fixture, camera=manifest['camera'],
                  session=str(session), drive_seconds=args.drive_seconds, motion_commanded=False)
    with output.open('x') as summary, output.with_suffix('.jsonl').open('x') as stream:
        rclpy.init()
        node, token, started = Observer(stream), None, time.monotonic()
        try:
            post('stop')
            confirmed = False
            while time.monotonic()-started < args.settle:
                node.spin(.1)
                if args.confirm_clear_launch and not confirmed and node.fresh('estimate') and node.latest['estimate']['valid']:
                    post('confirm')
                    confirmed = True
            report['settled_wall_seconds'] = time.monotonic()-started
            report['settled_status'] = node.latest.get('status')
            report['initial_mapping'] = node.latest.get('mapping_status')
            start = {key: dict(node.fresh(key) or {}) for key in ('truth', 'estimate')}
            if not all(row.get('valid') for row in start.values()):
                raise RuntimeError('No fresh valid truth/visual odometry after bounded settling')
            report['start'] = start
            if args.drive_seconds:
                status = node.fresh('status') or {}
                if not status.get('ready') or status.get('mode') != 'stopped':
                    raise RuntimeError('Guard is not stopped and ready')
                if [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')] != ['navigation_motion_guard']:
                    raise RuntimeError('Unexpected /cmd_vel publisher ownership')
                token = post('manual')['session']
                node.spin(.25)  # receive the mode selection before inspecting status
                if (node.fresh('status') or {}).get('mode') != 'manual':
                    raise RuntimeError('Manual selection was interrupted before the first command')
                drive_start, sequence = time.monotonic(), 0
                while time.monotonic()-drive_start < args.drive_seconds:
                    node.spin(.01)
                    status = node.fresh('status') or {}
                    if status.get('mode') == 'stopped':
                        report['drive_stop_reason'] = status.get('reason')
                        break
                    truth = node.fresh('truth')
                    if not truth or not truth['valid']:
                        raise RuntimeError('Independent truth observation was lost; bounded trial aborted')
                    if fixture == 'flat_edge' and truth['x'] > 1.28:
                        raise RuntimeError('Independent edge limit reached before guard stop; trial aborted')
                    sequence += 1
                    try:
                        post('drive', dict(session=token, sequence=sequence, linear=.05, angular=0.))
                    except Exception:
                        node.spin(.3)
                        if (node.fresh('status') or {}).get('mode') == 'stopped':
                            report['drive_stop_reason'] = node.latest['status'].get('reason')
                            break
                        raise
                    report['motion_commanded'] = True
                    node.spin(.04)
                report['actual_drive_wall_seconds'] = time.monotonic()-drive_start
                post('stop')
                node.spin(1.)
            end = {key: dict(node.fresh(key) or {}) for key in ('truth', 'estimate')}
            if not all(row.get('valid') for row in end.values()):
                raise RuntimeError('Final pose observations are unavailable')
            report['end'], report['comparison'] = end, compare(start, end)
            reason = report.get('drive_stop_reason')
            if args.expect_known_edge and reason != 'unsafe_or_unobserved_ground':
                raise RuntimeError('Expected known-ground boundary stop was not recorded')
            if reason and not args.expect_known_edge:
                raise RuntimeError('Drive was interrupted by '+reason)
            if args.drive_seconds and report['comparison']['truth_displacement_m'] < min(.2, .035*args.drive_seconds):
                raise RuntimeError('Insufficient independently observed movement')
            if report['comparison']['relative_endpoint_error_m'] > .10 or report['comparison']['relative_yaw_error_deg'] > 5.:
                raise RuntimeError('Pose disagreement exceeds Phase Two short-trial limits (0.10 m / 5 degrees)')
            coverage = (node.latest.get('mapping_status') or {}).get('coverage')
            if not (coverage and node.fresh('mapping_status', 1.) and node.fresh('status') and
                    -.05 <= node.latest['status']['sim_time']-coverage['stamp'] <= .6):
                raise RuntimeError('Fresh-depth coverage is unavailable')
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
        finally:
            try:
                report['cleanup_stop'] = post('stop')
                node.spin(.3)
            except Exception as error:
                report['cleanup_error'] = str(error)
                report['passed'] = False
            report['final_status'], report['final_mapping'] = node.latest.get('status'), node.latest.get('mapping_status')
            report['recorded_wall_seconds'] = time.monotonic()-started
            node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps({k: report.get(k) for k in ('passed', 'error', 'comparison', 'drive_stop_reason')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--settle', type=float, default=15.)
    parser.add_argument('--drive-seconds', type=float, default=0.)
    parser.add_argument('--confirm-clear-launch', action='store_true')
    parser.add_argument('--expect-known-edge', action='store_true')
    args = parser.parse_args()
    if not 5 <= args.settle <= 45 or not 0 <= args.drive_seconds <= 30:
        parser.error('Use settle 5..45 seconds and drive 0..30 seconds')
    raise SystemExit(run(args))
