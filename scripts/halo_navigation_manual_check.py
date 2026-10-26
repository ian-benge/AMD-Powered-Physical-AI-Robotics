#!/usr/bin/env python3
"""Bounded browser-input acceptance on flat/flat_edge only; no velocity publisher."""
import argparse
import json
import math
from pathlib import Path
import time
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts


class Observer(Node):
    def __init__(self, stream):
        super().__init__('navigation_browser_acceptance')
        self.stream, self.records, self.latest = stream, [], {}
        self.create_subscription(String, '/navigation/status', lambda m: self.record('status', json.loads(m.data)), 1)
        self.create_subscription(Odometry, '/odom_gt', self.truth, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.record('command', dict(v=m.linear.x, w=m.angular.z)), 1)
        self.create_subscription(Twist, '/navigation/manual_cmd_vel', lambda m: self.record('input', dict(v=m.linear.x, w=m.angular.z)), 1)
        self.create_subscription(Contacts, '/navigation/chassis_contacts', lambda m: self.record('contact', dict(count=len(m.contacts))), qos_profile_sensor_data)

    def truth(self, msg):
        p = msg.pose.pose.position
        self.record('truth', dict(x=p.x, y=p.y))

    def record(self, kind, data):
        row = dict(kind=kind, wall=time.monotonic(), **data)
        self.latest[kind] = row
        self.records.append(row)
        self.stream.write(json.dumps(row, allow_nan=False)+'\n')

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(.01, max(0., end-time.monotonic())))


def post(name, data=None):
    body = json.dumps(data).encode() if data is not None else b''
    request = Request('http://127.0.0.1:8774/'+name, data=body, method='POST',
                      headers={'Content-Type': 'application/json'} if data is not None else {})
    with urlopen(request, timeout=4.) as response:
        result = json.loads(response.read())
    if not result['success']:
        raise RuntimeError(result['message'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--edge', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((args.session/'session.json').read_text())
    if manifest['fixture'] != ('flat_edge' if args.edge else 'flat'):
        parser.error('Use a fresh matching flat/flat_edge fixture')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='browser_known_edge' if args.edge else 'browser_drive_release', passed=False)
    with args.output.open('x') as summary, args.output.with_suffix('.jsonl').open('x') as stream:
        rclpy.init()
        node = Observer(stream)
        try:
            end = time.monotonic()+30.
            while time.monotonic() < end:
                node.spin(.1)
                guard = node.latest.get('status', {})
                if guard.get('ready') and guard.get('mode') == 'stopped' and 'truth' in node.latest:
                    break
            else:
                raise RuntimeError('Stopped ready preflight did not become available')
            # DDS endpoint discovery can precede the matching node-name graph.
            discovery_end = time.monotonic()+5.
            while True:
                report['command_owners'] = [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
                if report['command_owners'] == ['navigation_motion_guard'] or time.monotonic() >= discovery_end:
                    break
                node.spin(.1)
            if report['command_owners'] != ['navigation_motion_guard']:
                raise RuntimeError('Unexpected command ownership')
            home = dict(node.latest['truth'])
            if math.hypot(home['x'], home['y']) > .1:
                raise RuntimeError('Use the fresh inspected fixture launch position')
            confirmation = post('confirm')
            report['clearance'] = confirmation
            node.spin(.8)
            token = post('manual')['session']
            start = time.monotonic()
            sequence = 0
            next_send = start
            end = start+(35. if args.edge else 1.)
            while time.monotonic() < end:
                node.spin(.005)
                guard = node.latest.get('status', {})
                if args.edge and guard.get('reason') == 'unsafe_or_unobserved_ground':
                    break
                if time.monotonic() >= next_send:
                    sequence += 1
                    try:
                        post('drive', dict(session=token, sequence=sequence, linear=.05, angular=0.))
                    except Exception:
                        node.spin(.3)
                        if args.edge and node.latest.get('status', {}).get('reason') == 'unsafe_or_unobserved_ground':
                            break
                        raise
                    next_send = time.monotonic()+.05
            released = time.monotonic()
            post('release', dict(session=token))
            node.spin(1.)
            report['guard_after_release'] = dict(node.latest['status'])
            report['truth_displacement_m'] = math.hypot(node.latest['truth']['x']-home['x'], node.latest['truth']['y']-home['y'])
            report['input_messages'] = sequence
            report['nonzero_inputs'] = sum(r['kind'] == 'input' and bool(r['v'] or r['w']) for r in node.records)
            zeros = [r['wall'] for r in node.records if r['kind'] == 'command' and r['wall'] >= released and not r['v'] and not r['w']]
            report['release_to_zero_wall_s'] = min(zeros)-released if zeros else None
            expected = 'unsafe_or_unobserved_ground' if args.edge else 'command_timeout'
            if node.latest['status']['reason'] != expected or node.latest['status']['command'] != [0., 0.]:
                raise RuntimeError('Expected latched stop was not established')
            if args.edge:
                if not .3 < report['truth_displacement_m'] < 1.4:
                    raise RuntimeError('Unexpected platform-edge displacement')
            elif not .01 < report['truth_displacement_m'] < .12 or report['release_to_zero_wall_s'] is None or report['release_to_zero_wall_s'] > .4:
                raise RuntimeError('Movement/release timing outside bounded acceptance')
            if any(r['kind'] == 'contact' and r['count'] for r in node.records):
                raise RuntimeError('Chassis contact observed')
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
        finally:
            try:
                report['cleanup_stop'] = post('stop')
            except Exception as error:
                report['cleanup_error'] = str(error)
                report['passed'] = False
            node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
