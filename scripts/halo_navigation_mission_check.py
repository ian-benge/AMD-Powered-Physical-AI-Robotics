#!/usr/bin/env python3
"""Explicit physical-contact or waypoint acceptance; existing smoke limits stay intact.

Run in the navigation container with ROS sourced. --start authorizes the selected
waypoint and automatic return. Truth is used only for evaluation/fixture placement.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
from urllib.request import urlopen

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from std_srvs.srv import Trigger

from halo_navigation_smoke import Check
from rover_navigation.metrics import summarize_trials


def create_box(name, center, size, dynamic=False):
    dimensions = ' '.join(map(str, size))
    xyz = ' '.join(map(str, center))
    sdf = f'''<sdf version="1.9"><model name="{name}"><static>{str(not dynamic).lower()}</static>
<pose>{xyz} 0 0 0</pose><link name="probe"><inertial><mass>.01</mass><inertia>
<ixx>.000002</ixx><iyy>.000002</iyy><izz>.000002</izz></inertia></inertial>
<collision name="solid"><geometry><box><size>{dimensions}</size></box></geometry></collision>
<visual name="surface"><geometry><box><size>{dimensions}</size></box></geometry>
<material><ambient>.8 .1 .1 1</ambient><diffuse>.8 .1 .1 1</diffuse></material></visual></link></model></sdf>'''
    reply = subprocess.run(['gz', 'service', '-s', '/world/mars_sampling/create',
                            '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '2000', '--req', 'sdf: '+json.dumps(sdf)],
                           capture_output=True, text=True, timeout=4.)
    if reply.returncode or 'data: true' not in reply.stdout:
        raise RuntimeError('Physical probe creation failed: '+reply.stdout+reply.stderr)
    return dict(name=name, center=center, size=size, dynamic=dynamic, sdf=sdf)


class MissionCheck(Check):
    def __init__(self, events):
        self.records = []
        super().__init__(events)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.goal_pub = self.create_publisher(PoseStamped, '/navigation/selected_goal', 1)
        self.trigger_clients['start_selected_goal'] = self.create_client(Trigger, '/navigation/start_selected_goal')
        self.preview_count = 0
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        for topic in ('preview_outward', 'preview_return'):
            self.create_subscription(PathMessage, '/navigation/'+topic,
                                     lambda msg, t=topic: self.path(t, msg), durable)

    def sample(self, kind, values):
        super().sample(kind, values)
        self.records.append(self.latest[kind])

    def discover(self, timeout=20.):
        """Bounded wait for DDS discovery; endpoints of one node can match seconds after another's."""
        kinds = ('status', 'mission', 'truth', 'estimate', 'ground', 'guard_command')
        start = time.monotonic()

        def missing():
            now = time.monotonic()
            return ([k for k in kinds if k not in self.latest or now-self.latest[k]['wall'] > .6] +
                    [name for name, client in self.trigger_clients.items() if not client.service_is_ready()])

        self.spin_for(3.)
        while missing() and time.monotonic()-start < timeout:
            self.spin_for(.1)
        if missing():
            raise RuntimeError('Discovery incomplete after %.0f s: %s' % (timeout, ', '.join(missing())))
        return time.monotonic()-start

    def path(self, topic, msg):
        self.sample(topic, dict(poses=[[p.pose.position.x, p.pose.position.y] for p in msg.poses]))
        if msg.poses:
            self.preview_count += 1

    def map_pose(self):
        tf = self.tf.lookup_transform('map', 'drive_center', Time())
        p, q = tf.transform.translation, tf.transform.rotation
        return p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def select(self, goal):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = float(goal[0]), float(goal[1])
        message.pose.orientation.z, message.pose.orientation.w = math.sin(goal[2]/2), math.cos(goal[2]/2)
        self.goal_pub.publish(message)
        self.sample('selection_sent', dict(goal=goal))


def camera(output, label):
    try:
        with urlopen('http://127.0.0.1:8773/frame.png', timeout=2.) as response:
            output.with_name(output.stem+'-'+label+'.png').write_bytes(response.read())
    except Exception:
        pass


def contact_probe(node, session, output, report):
    if json.loads((session/'session.json').read_text()).get('fixture') != 'flat':
        raise ValueError('Contact probe requires a fresh flat fixture')
    if math.hypot(node.fresh('truth', .6)['x'], node.latest['truth']['y']) > .2:
        raise ValueError('Contact probe requires the stationary launch position')
    if not node.call('manual')['success']:
        raise RuntimeError('Could not select stationary manual mode')
    native_path = output.with_name(output.stem+'-native-contacts.txt')
    with native_path.open('x') as native:
        process = subprocess.Popen(['gz', 'topic', '-e', '-t', '/navigation/chassis_contacts'], stdout=native, stderr=subprocess.STDOUT)
        try:
            node.spin_for(1.)
            truth = node.fresh('truth', .6)
            report['physical_probe'] = create_box('navigation_contact_probe', (truth['x'], truth['y'], .23), (.025, .025, .025), True)
            end = time.monotonic()+8.
            while time.monotonic() < end and not node.counts['nonempty_contacts']:
                node.spin_for(.02)
            node.spin_for(.5)
            contacts = [r for r in node.records if r['kind'] == 'contacts' and r['count']]
            if not contacts:
                raise RuntimeError('No bridged physical chassis contact observed')
            first = contacts[0]['wall']
            zeros = [r for r in node.records if r['kind'] == 'guard_command' and r['wall'] >= first and not r['v'] and not r['w']]
            latency = min(r['wall'] for r in zeros)-first if zeros else math.inf
            report['contact_to_zero_command_wall_s'] = latency if math.isfinite(latency) else None
            report['contact_status'] = dict(node.fresh('status', .6))
            rejected = node.call('manual')
            report['resume_attempt'] = rejected
            if latency > .2 or report['contact_status']['reason'] != 'chassis_contact' or rejected['success']:
                raise RuntimeError('Contact stop/rearm rejection did not meet acceptance')
        finally:
            process.terminate()
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    evidence = native_path.read_text()
    if 'navigation_contact_probe' not in evidence or 'base_link_collision' not in evidence:
        raise RuntimeError('Native contact evidence does not identify probe and chassis')
    report['collision_scope'] = 'Stationary chassis contact and guard reaction; wheel coverage not established'
    report['restart_required'] = True


def run(args):
    session, output = args.session.resolve(), args.output.resolve()
    manifest = json.loads((session/'session.json').read_text())
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='physical_contact' if args.contact_probe else 'waypoint_and_return', passed=False,
                  session=str(session), fixture=manifest.get('fixture'), min_outward_m=args.min_outward)
    with output.open('x') as summary, output.with_suffix('.jsonl').open('x') as events:
        rclpy.init()
        node = MissionCheck(events)
        try:
            report['discovery_wall_s'] = node.discover()
            node.ready('stopped')
            owners = node.owners()
            report['owners'] = owners
            if owners['/cmd_vel'] != ['navigation_motion_guard'] or owners['/diff_drive_controller/cmd_vel'] != ['twist_stamper']:
                raise RuntimeError('Unexpected motion publisher ownership')
            camera(output, 'before')
            if args.contact_probe:
                contact_probe(node, session, output, report)
            else:
                if not args.start:
                    raise ValueError('Use --start to authorize the selected waypoint and return')
                evaluation = session/'mission-events.jsonl'
                if not evaluation.is_file() or not evaluation.stat().st_size or 'navigation_evaluator' not in node.get_node_names():
                    raise RuntimeError('Start the independent evaluator before the mission (mission_controls:=true)')
                if args.confirm_flat_start:
                    if manifest.get('fixture') != 'flat':
                        raise ValueError('Automatic clearance confirmation is restricted to the inspected flat fixture')
                    answer = node.call('confirm_clear_start')
                    if not answer['success'] and not answer['message'].startswith('Launch clearance already confirmed;'):
                        raise RuntimeError(answer['message'])
                    node.spin_for(1.)
                start_map = node.map_pose()
                start_truth = dict(node.fresh('truth', .6))
                if args.goal_forward is not None:
                    goal = (start_map[0]+args.goal_forward*math.cos(start_map[2]),
                            start_map[1]+args.goal_forward*math.sin(start_map[2]), start_map[2])
                else:
                    goal = args.goal
                if goal:
                    node.select(goal)
                    node.spin_for(.5)
                end = time.monotonic()+25.
                while time.monotonic() < end:
                    state = node.latest.get('mission', {})
                    if state.get('selection_state') == 'ready':
                        break
                    if state.get('selection_state') == 'rejected':
                        raise RuntimeError('Goal rejected: '+state.get('selection_reason', ''))
                    node.spin_for(.05)
                else:
                    raise RuntimeError('No verified selected goal; select one in RViz')
                report['selection'] = dict(node.latest['mission'])
                trial = node.latest['mission'].get('trial', 0)+1
                result = node.call('start_selected_goal', timeout=28.)
                if not result['success']:
                    raise RuntimeError(result['message'])
                injected = False
                end = time.monotonic()+args.timeout
                while time.monotonic() < end:
                    node.spin_for(.05)
                    state = node.latest.get('mission', {})
                    if state.get('trial') == trial and state.get('mission') == 'idle' and state.get('result'):
                        break
                    if args.obstruction and not injected:
                        truth = node.fresh('truth', .6)
                        if math.hypot(truth['x']-start_truth['x'], truth['y']-start_truth['y']) >= .3:
                            if manifest.get('fixture') != 'flat' or args.goal_forward is None or args.goal_forward < 2.:
                                raise ValueError('Obstruction injection requires flat fixture and --goal-forward >= 2')
                            # Truth places the physical fixture; it never supplies navigation data.
                            heading = start_truth['yaw']
                            center = (start_truth['x']+1.3*math.cos(heading), start_truth['y']+1.3*math.sin(heading), .2)
                            size = (.3, .35 if args.obstruction == 'detour' else 10., .4)
                            report['obstruction'] = create_box('navigation_obstruction', center, size)
                            injected = True
                else:
                    raise RuntimeError('Mission wall-time limit exceeded')
                node.spin_for(1.5)
                with evaluation.open() as stream:
                    rows = [json.loads(line) for line in stream if line.strip()]
                trial_reports = summarize_trials(rows, args.min_outward)
                report['trials'] = trial_reports
                current = next((r for r in trial_reports if r['trial'] == trial), None)
                if args.obstruction == 'blocked':
                    if not injected or state.get('result') == 'Home reached' or node.latest['status']['mode'] != 'stopped' or node.counts['nonempty_contacts']:
                        raise RuntimeError('Blocked route did not cancel and stop safely')
                    report['expected_blocked_stop'] = state.get('result')
                elif not current or not current['passed']:
                    raise RuntimeError('Waypoint/return metrics did not meet acceptance')
                if args.obstruction == 'detour':
                    lateral = max(abs(-(r['x']-start_truth['x'])*math.sin(start_truth['yaw'])+
                                      (r['y']-start_truth['y'])*math.cos(start_truth['yaw']))
                                  for r in node.records if r['kind'] == 'truth')
                    report['max_lateral_displacement_m'] = lateral
                    if not injected or lateral < .35:
                        raise RuntimeError('No independently observed obstacle detour')
            command = node.fresh('guard_command', .3)
            if command['v'] or command['w'] or node.latest['status']['mode'] != 'stopped':
                raise RuntimeError('Final stopped state was not established')
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
        finally:
            try:
                report['cleanup_stop'] = node.call('stop')
                node.spin_for(.3)
            except Exception as error:
                report['cleanup_error'] = str(error)
                report['passed'] = False
            camera(output, 'after')
            report['guard_faults'] = []
            for status in node.statuses:
                if status.get('fault') and status['fault'] not in report['guard_faults']:
                    report['guard_faults'].append(status['fault'])
            report['last_status'] = node.latest.get('status')
            report['max_ground_age_s'] = max((r.get('depth_age_seconds') or 0. for r in node.statuses), default=None)
            node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps(report, indent=2, allow_nan=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--contact-probe', action='store_true')
    mode.add_argument('--start', action='store_true')
    goal = parser.add_mutually_exclusive_group()
    goal.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'YAW'))
    goal.add_argument('--goal-forward', type=float)
    parser.add_argument('--confirm-flat-start', action='store_true')
    parser.add_argument('--obstruction', choices=('detour', 'blocked'))
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--min-outward', type=float, default=.3)
    args = parser.parse_args()
    if (not 0 < args.timeout <= 300. or not .3 <= args.min_outward <= 3. or
            (args.goal and not all(math.isfinite(x) for x in args.goal)) or
            (args.goal_forward is not None and not .3 <= args.goal_forward <= 3.)):
        parser.error('Use finite goals, timeout (0,300], minimum outward [0.3,3], forward goal [0.3,3]')
    raise SystemExit(run(args))
