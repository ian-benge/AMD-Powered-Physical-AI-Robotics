#!/usr/bin/env python3
"""Record a preflight or bounded manual smoke test in the flat Gazebo fixture.

Run inside rover-navigation after sourcing its ROS installation. Default mode
is read-only. --drive allows <=1 wall second at <=0.05 m/s through the existing
manual guard input. Independent truth is used only to evaluate the test.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from std_srvs.srv import Trigger


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def pause_world(paused):
    result = subprocess.run(['gz', 'service', '-s', '/world/mars_sampling/control',
                             '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                             '--timeout', '2000', '--req', 'pause: '+str(paused).lower()],
                            capture_output=True, text=True, timeout=4.)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError('World pause/resume request failed: '+result.stdout+result.stderr)
    return result.stdout.strip()


class Check(Node):
    def __init__(self, events):
        super().__init__('navigation_manual_smoke')
        self.events = events
        self.latest, self.counts, self.statuses = {}, Counter(), []
        self.pub = self.create_publisher(Twist, '/navigation/manual_cmd_vel', 1)
        self.trigger_clients = {name: self.create_client(Trigger, '/navigation/'+name)
                                for name in ('stop', 'manual', 'confirm_clear_start')}
        self.create_subscription(String, '/navigation/status', self.status, 1)
        self.create_subscription(String, '/navigation/mission_status', self.mission, 1)
        self.create_subscription(OccupancyGrid, '/navigation/local_ground',
                                 lambda m: self.sample('ground', {'frame': m.header.frame_id}), 1)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom('estimate', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom_gt', lambda m: self.odom('truth', m), qos_profile_sensor_data)
        self.create_subscription(Contacts, '/navigation/chassis_contacts',
                                 lambda m: self.sample('contacts', {'count': len(m.contacts)}), qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel',
                                 lambda m: self.sample('guard_command', {'v': m.linear.x, 'w': m.angular.z}), 1)

    def sample(self, kind, values):
        record = dict(kind=kind, wall=time.monotonic(), **values)
        self.latest[kind] = record
        self.counts[kind] += 1
        if kind == 'contacts' and values['count']:
            self.counts['nonempty_contacts'] += 1
        self.events.write(json.dumps(record, allow_nan=False)+'\n')

    def status(self, message):
        values = json.loads(message.data)
        self.sample('status', values)
        self.statuses.append(self.latest['status'])

    def mission(self, message):
        self.sample('mission', json.loads(message.data))

    def odom(self, kind, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self.counts['invalid_'+kind] += 1
            return
        self.sample(kind, dict(x=p.x, y=p.y,
                              yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                              stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9))

    def spin_for(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(.02, max(0., end-time.monotonic())))

    def call(self, name, timeout=5.):
        client = self.trigger_clients[name]
        end = time.monotonic()+timeout
        while not client.service_is_ready() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
        if not client.service_is_ready():
            raise RuntimeError('Service unavailable: '+name)
        future = client.call_async(Trigger.Request())
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
        if not future.done():
            raise RuntimeError('Service timeout: '+name)
        response = future.result()
        result = dict(success=response.success, message=response.message)
        self.sample('service_'+name, result)
        return result

    def fresh(self, kind, age):
        if kind not in self.latest or time.monotonic()-self.latest[kind]['wall'] > age:
            raise RuntimeError('Missing/stale '+kind)
        return self.latest[kind]

    def ready(self, mode):
        status = self.fresh('status', .6)
        if status.get('mode') != mode or not status.get('ready'):
            raise RuntimeError('Guard not ready in '+mode+': '+json.dumps(status))
        # Harmonic contacts are event-only; absence is not a heartbeat or proof
        # of zero collisions. Connection is checked separately through Gazebo.
        if self.counts['nonempty_contacts']:
            raise RuntimeError('Chassis contact observed')
        self.fresh('truth', .6)
        self.fresh('estimate', .6)

    def owners(self):
        return {topic: sorted(info.node_name for info in self.get_publishers_info_by_topic(topic))
                for topic in ('/cmd_vel', '/navigation/manual_cmd_vel', '/diff_drive_controller/cmd_vel')}


def run(args):
    session = args.session.resolve()
    manifest = json.loads((session/'session.json').read_text())
    if manifest.get('fixture') != 'flat':
        raise ValueError('This smoke test is restricted to the flat Gazebo fixture')
    models = ET.parse(session/'world.sdf').getroot().find('world').findall('model')
    if any(m.get('name') != 'ground' and not m.get('name', '').startswith('paint_') for m in models):
        raise ValueError('Unexpected physical model in flat fixture')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='stationary_clock_pause' if args.pause_check else 'flat_manual_smoke',
                  drive_requested=args.drive, session=str(session),
                  duration_wall_s=args.duration, speed_m_s=args.speed, stop_method=args.stop_method,
                  passed=False, scope='manual_simulation_only')
    # Reserve both files before creating a ROS node or allowing any motion.
    with output.open('x', encoding='utf-8') as summary, \
            output.with_suffix('.jsonl').open('x', encoding='utf-8') as events:
        rclpy.init()
        node = None
        armed = False
        paused_requested = False
        try:
            node = Check(events)
            node.spin_for(args.warmup)
            report['owners'] = node.owners()
            expected = {'/cmd_vel': ['navigation_motion_guard'],
                        '/navigation/manual_cmd_vel': [node.get_name()],
                        '/diff_drive_controller/cmd_vel': ['twist_stamper']}
            if report['owners'] != expected:
                raise RuntimeError('Unexpected command publisher ownership')
            transport = subprocess.run(['gz', 'topic', '-i', '-t', '/navigation/chassis_contacts'],
                                       capture_output=True, text=True, timeout=6.)
            report['contact_transport'] = transport.stdout.strip()
            report['collision_acceptance'] = 'physical_contact_test_pending'
            if (transport.returncode or 'No publishers' in transport.stdout
                    or 'Publishers' not in transport.stdout or 'gz.msgs.Contacts' not in transport.stdout):
                raise RuntimeError('Native Gazebo contact publisher is not connected at bridge topic')
            # Service/transport discovery must not leave an old sensor snapshot
            # as the baseline for enabling a mode.
            node.spin_for(.5)
            node.ready('stopped')
            if node.fresh('mission', 2.)['mission'] != 'idle':
                raise RuntimeError('Mission must be idle')
            command = node.fresh('guard_command', .3)
            if command['v'] or command['w']:
                raise RuntimeError('Baseline guard command is not zero')
            baseline = {kind: dict(node.latest[kind]) for kind in ('truth', 'estimate')}
            report['baseline'] = baseline
            if args.drive:
                # The inspected flat fixture has a 10m ground platform and no
                # collision obstacles. Stay far enough from its outer edge to
                # establish the existing 1.1m operator-clearance seed.
                if max(abs(baseline['truth'][axis]) for axis in ('x', 'y')) > 3.5:
                    raise RuntimeError('Rover too close to platform boundary')
                response = node.call('confirm_clear_start')
                report['clear_start'] = response
                if not response['success'] and not response['message'].startswith('Launch clearance already confirmed;'):
                    raise RuntimeError('Clear-start confirmation failed: '+response['message'])
                # An uncertain service response must still attempt a stop.
                armed = True
                response = node.call('manual')
                if not response['success']:
                    raise RuntimeError('Manual selection failed: '+response['message'])
                end = time.monotonic()+2.
                while node.latest['status']['mode'] != 'manual' and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.02)
                node.ready('manual')
                baseline = {kind: dict(node.latest[kind]) for kind in ('truth', 'estimate')}
                report['baseline'] = baseline
                report['drive_start_wall'] = time.monotonic()
                node.sample('drive_start', dict(duration=args.duration, speed=args.speed))
                end = time.monotonic()+args.duration
                next_send = time.monotonic()
                while time.monotonic() < end:
                    node.ready('manual')
                    now = time.monotonic()
                    if now >= next_send:
                        message = Twist()
                        message.linear.x = args.speed
                        node.pub.publish(message)
                        node.counts['manual_commands_sent'] += 1
                        next_send = now+.05
                    rclpy.spin_once(node, timeout_sec=.005)
                report['last_command_wall'] = time.monotonic()
                node.sample('drive_publication_ended', {})
                if args.stop_method == 'lease':
                    # No further manual messages; the existing .35s lease must stop.
                    node.spin_for(.8)
                    status = node.fresh('status', .6)
                    report['lease_status'] = dict(status)
                    if status['mode'] != 'stopped' or status['reason'] != 'command_timeout':
                        raise RuntimeError('Expected command-timeout stop was not observed')
                response = node.call('stop')
                report['stop_response'] = response
                if not response['success']:
                    raise RuntimeError('Stop service rejected the request')
                node.spin_for(1.5)
                command = node.fresh('guard_command', .3)
                if command['v'] or command['w']:
                    raise RuntimeError('Guard did not command zero after stop')
                report['final'] = {kind: dict(node.fresh(kind, .6)) for kind in ('truth', 'estimate')}
                distances = {}
                for kind, last in report['final'].items():
                    first = baseline[kind]
                    distances[kind] = math.hypot(last['x']-first['x'], last['y']-first['y'])
                    report[kind+'_yaw_change_rad'] = angle(last['yaw']-first['yaw'])
                report['distance_m'] = distances
                report['distance_disagreement_m'] = abs(distances['truth']-distances['estimate'])
                if not .01 <= distances['truth'] <= .12:
                    raise RuntimeError('Unexpected displacement in bounded drive')
                if report['distance_disagreement_m'] > .03:
                    raise RuntimeError('Odometry disagrees with truth by more than 3cm')
                if abs(report['truth_yaw_change_rad']) > math.radians(10):
                    raise RuntimeError('Unexpected turning in straight drive')
                if node.counts['nonempty_contacts']:
                    raise RuntimeError('Chassis contact observed')
            if args.pause_check:
                armed = True
                response = node.call('manual')
                if not response['success']:
                    raise RuntimeError('Manual selection failed: '+response['message'])
                node.spin_for(.3)
                node.ready('manual')
                paused_requested = True
                report['pause_response'] = pause_world(True)
                node.spin_for(.9)
                status = node.fresh('status', .6)
                report['paused_status'] = dict(status)
                if (status['mode'] != 'stopped' or status['reason'] != 'sensor_or_clock_timeout'
                        or status['command'] != [0., 0.] or status['clock_age_seconds'] < .5):
                    raise RuntimeError('Paused clock did not latch a zero-command safety stop')
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
        finally:
            if node is not None:
                if armed:
                    # The existing wall watchdogs also cover a disconnected or
                    # killed test process; this is the normal cleanup path.
                    node.pub.publish(Twist())
                    try:
                        report['cleanup_stop'] = node.call('stop')
                        node.spin_for(.3)
                    except Exception as error:
                        report['cleanup_error'] = str(error)
                        report['passed'] = False
                if paused_requested:
                    try:
                        report['resume_response'] = pause_world(False)
                        node.spin_for(1.)
                        if node.latest.get('status', {}).get('mode') != 'stopped':
                            raise RuntimeError('Motion mode resumed without operator selection')
                    except Exception as error:
                        report['resume_error'] = str(error)
                        report['passed'] = False
                report['counts'] = dict(node.counts)
                report['last_status'] = node.latest.get('status')
                report['ready_false_samples'] = sum(not s.get('ready') for s in node.statuses)
                if node.statuses:
                    report['max_ground_age_s'] = max(s.get('depth_age_seconds') or 0. for s in node.statuses)
                node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps(report, indent=2, allow_nan=False), flush=True)
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--drive', action='store_true')
    mode.add_argument('--pause-check', action='store_true', help='Pause/resume while stationary; no drive commands')
    parser.add_argument('--duration', type=float, default=1.)
    parser.add_argument('--speed', type=float, default=.05)
    parser.add_argument('--warmup', type=float, default=5.)
    parser.add_argument('--stop-method', choices=('service', 'lease'), default='service')
    args = parser.parse_args()
    if not 0 < args.duration <= 1. or not 0 < args.speed <= .05 or not 1. <= args.warmup <= 30.:
        parser.error('Limit duration to (0,1]s, speed to (0,0.05]m/s and warmup to [1,30]s')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
