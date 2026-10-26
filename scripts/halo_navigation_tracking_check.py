#!/usr/bin/env python3
"""Phase Three: bounded manual translation, 360-degree revisit and independent scoring.

Use only a fresh scene 50000 session with an inspected clear launch pad.
Commands use the guarded HTTP API; simulator truth is evaluation/abort evidence.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from halo_navigation_coverage_check import Observer, compare, post
from rover_navigation.tracking_evaluation import reference_matches, revisit_evidence


class TrackingObserver(Observer):
    def __init__(self, stream):
        super().__init__(stream)
        self.create_subscription(String, '/navigation/sample_sites',
            lambda m: self.record('sites', json.loads(m.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))


def run(args):
    session, output = args.session.resolve(), args.output.resolve()
    manifest = json.loads((session/'session.json').read_text())
    source = Path(manifest['scene'])
    if manifest.get('fixture') or source.parent.name != 'scene_050000' or manifest.get('database'):
        raise ValueError('Only fresh mapping sessions in scene 50000 are allowed')
    scene = json.loads(source.read_text())  # evaluator only; never passed to tracker/control
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='phase3_tracking', passed=False, session=str(session), configuration=manifest,
                  limits=dict(wall_seconds=90, forward_m_s=.05, turn_rad_s=.30, translation_m=.45,
                              localization_error_m=.10, localization_error_deg=5.), motion_commanded=False)
    with output.open('x') as summary, output.with_suffix('.jsonl').open('x') as stream:
        rclpy.init()
        node, token, sequence = TrackingObserver(stream), None, 0
        started = time.monotonic()
        try:
            post('stop')
            confirmed_launch = False
            settle_end = started+args.settle
            while time.monotonic() < settle_end:
                node.spin(.05)
                if args.confirm_clear_launch and not confirmed_launch and (node.fresh('estimate') or {}).get('valid'):
                    post('confirm')
                    confirmed_launch = True
            start = {kind: dict(node.fresh(kind) or {}) for kind in ('truth', 'estimate')}
            if not all(row.get('valid') for row in start.values()):
                raise RuntimeError('No fresh valid truth and odometry after bounded settling')
            report['start'] = start
            if [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')] != ['navigation_motion_guard']:
                raise RuntimeError('Unexpected /cmd_vel publisher ownership')
            sites = node.fresh('sites', 1.) or {}
            if sites.get('schema_version') != 2:
                raise RuntimeError('Confirmation tracker is not installed')
            report['stationary_sites'] = sites
            if sites.get('objects'):
                raise RuntimeError('Objects confirmed without the required translated viewpoints')
            token = post('manual')['session']

            def command(linear, angular):
                nonlocal sequence
                if time.monotonic()-started >= 90.:
                    raise RuntimeError('90 wall-second trial limit reached')
                node.spin(.01)
                status = node.fresh('status') or {}
                if status.get('mode') == 'stopped' and sequence > 2:
                    raise RuntimeError('Safety interruption: '+status.get('reason', 'status unavailable'))
                current = {kind: dict(node.fresh(kind) or {}) for kind in ('truth', 'estimate')}
                if not all(row.get('valid') for row in current.values()):
                    raise RuntimeError('Independent pose observation unavailable')
                disagreement = compare(start, current)
                report['maximum_position_disagreement_m'] = max(report.get('maximum_position_disagreement_m', 0.), disagreement['relative_endpoint_error_m'])
                report['maximum_yaw_disagreement_deg'] = max(report.get('maximum_yaw_disagreement_deg', 0.), disagreement['relative_yaw_error_deg'])
                if disagreement['relative_endpoint_error_m'] > .10 or disagreement['relative_yaw_error_deg'] > 5.:
                    raise RuntimeError('Localization comparison exceeds 0.10 m / 5 degrees')
                sequence += 1
                post('drive', dict(session=token, sequence=sequence, linear=linear, angular=angular))
                report['motion_commanded'] |= bool(linear or angular)
                node.spin(.035)

            for _ in range(5):
                command(0., 0.)
            initial = dict(node.latest['estimate'])
            forward_start = time.monotonic()
            while math.hypot(node.latest['estimate']['x']-initial['x'], node.latest['estimate']['y']-initial['y']) < .45:
                if time.monotonic()-forward_start > 18.:
                    raise RuntimeError('Translation did not reach 0.45 m within 18 wall seconds')
                command(.05, 0.)
            for _ in range(12):
                command(0., 0.)
            report['before_turn_sites'] = dict(node.latest.get('sites', {}))
            initial_ids = {item['track_id'] for item in report['before_turn_sites'].get('objects', [])}
            rotation, previous = 0., node.latest['estimate']['yaw']
            turn_start = time.monotonic()
            while rotation < 2*math.pi:
                if time.monotonic()-turn_start > 45.:
                    raise RuntimeError('Full turn did not finish within 45 wall seconds')
                command(0., .30)
                yaw = node.latest['estimate']['yaw']
                rotation += math.atan2(math.sin(yaw-previous), math.cos(yaw-previous))
                previous = yaw
            for _ in range(24):
                command(0., 0.)
            report['estimated_rotation_rad'] = rotation
            report['after_turn_sites'] = dict(node.latest.get('sites', {}))
            final_objects = report['after_turn_sites'].get('objects', [])
            final_ids = {item['track_id'] for item in final_objects}
            report['revisit_ids_retained'] = sorted(initial_ids & final_ids)
            report['revisit_evidence'] = revisit_evidence(report['before_turn_sites'], report['after_turn_sites'])
            report['reference_evaluation'] = reference_matches(final_objects, start, scene)
            report['tentative_count'] = len(report['after_turn_sites'].get('tentative_objects', []))
            if not initial_ids:
                raise RuntimeError('No rock confirmed before the full turn; this cannot establish revisit acceptance')
            if not initial_ids <= final_ids:
                raise RuntimeError('A previously confirmed rock did not retain its confirmed ID on revisit')
            if not report['revisit_evidence']['passed']:
                raise RuntimeError('Persistent markers were not freshly reobserved with their existing IDs after the turn')
            scoring = report['reference_evaluation']
            if scoring['false_confirmed'] or scoring['duplicate_references']:
                raise RuntimeError('Independent reference matching found false or duplicate confirmed rocks')
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
        finally:
            try:
                report['cleanup_stop'] = post('stop')
                node.spin(.5)
            except Exception as error:
                report['cleanup_error'] = str(error)
                report['passed'] = False
            report['final_status'] = node.latest.get('status')
            if (report['final_status'] or {}).get('command') != [0., 0.]:
                report['passed'] = False
                report['cleanup_error'] = 'Final zero guard command not observed'
            report['final_sites'] = node.latest.get('sites')
            if 'start' in report:
                current = {kind: dict(node.fresh(kind) or {}) for kind in ('truth', 'estimate')}
                if all(row.get('valid') for row in current.values()):
                    report['comparison'] = compare(report['start'], current)
            report['recorded_wall_seconds'] = time.monotonic()-started
            node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps({key: report.get(key) for key in ('passed', 'error', 'comparison', 'revisit_ids_retained', 'tentative_count')}, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--settle', type=float, default=15.)
    parser.add_argument('--confirm-clear-launch', action='store_true', help='Only after inspecting the 1.1 m launch pad')
    args = parser.parse_args()
    if not 5 <= args.settle <= 30:
        parser.error('Use settle 5..30 wall seconds')
    raise SystemExit(run(args))
