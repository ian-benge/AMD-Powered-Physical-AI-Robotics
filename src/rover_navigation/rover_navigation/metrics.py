"""Summarize independent trajectory evidence without importing ROS."""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def stamped(record):
    value = record.get('sim_time')
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def summarize(records):
    truth = sorted((r for r in records if r.get('kind') == 'truth' and stamped(r)), key=lambda r: r['sim_time'])
    estimated = sorted((r for r in records if r.get('kind') == 'estimate' and stamped(r)), key=lambda r: r['sim_time'])
    missions = [r for r in records if r.get('kind') == 'mission_status' and stamped(r)]
    starts = [r for r in missions if r.get('mission') in ('waypoints', 'exploring')]
    report = dict(truth_samples=len(truth), estimate_samples=len(estimated),
                  collision_acceptance='not_established_by_trajectory_logging',
                  mission_result=missions[-1].get('result', '') if missions else '',
                  home_reference_available=False)
    contacts = [r for r in records if r.get('kind') == 'chassis_contacts']
    if contacts:
        count = sum(r['count'] for r in contacts)
        report.update(chassis_contact_samples=len(contacts), chassis_contact_events=count,
                      collision_acceptance='chassis_contact_observed' if count else 'no_chassis_contact_observed')
    if not truth or not starts:
        return report
    t = np.array([r['sim_time'] for r in truth])
    p = np.array([r['pose'] for r in truth])
    start = starts[0]['sim_time']
    if not t[0] <= start <= t[-1]:
        return report
    yaw = np.unwrap(p[:, 2])
    home = np.array([np.interp(start, t, p[:, 0]), np.interp(start, t, p[:, 1]), np.interp(start, t, yaw)])
    position_error = float(np.linalg.norm(p[-1, :2]-home[:2]))
    yaw_error = abs(math.atan2(math.sin(p[-1, 2]-home[2]), math.cos(p[-1, 2]-home[2])))
    report.update(home_reference_available=True, final_home_error_m=position_error,
                  final_home_yaw_error_deg=math.degrees(yaw_error),
                  return_within_tolerance=position_error <= .15 and yaw_error <= math.radians(10),
                  mission_duration_sim_seconds=float(t[-1]-start))
    if estimated:
        valid = [r for r in estimated if t[0] <= r['sim_time'] <= t[-1]]
        if valid:
            te = np.array([r['sim_time'] for r in valid])
            e = np.array([r['pose'] for r in valid])
            aligned_truth = np.column_stack([np.interp(te, t, p[:, i]) for i in (0, 1)])
            angle = np.interp(te[0], t, yaw)-e[0, 2]
            rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
            aligned = (e[:, :2]-e[0, :2]) @ rotation.T+aligned_truth[0]
            errors = np.linalg.norm(aligned-aligned_truth, axis=1)
            report.update(aligned_odometry_rmse_m=float(np.sqrt(np.mean(errors**2))),
                          aligned_odometry_max_error_m=float(errors.max()))
    return report


def summarize_trials(records, min_outward=.3):
    """Evaluate every trial against the FIRST mission's truth home, not its restart pose."""
    truth = sorted((r for r in records if r.get('kind') == 'truth' and stamped(r)), key=lambda r: r['sim_time'])
    missions = [r for r in records if r.get('kind') == 'mission_status' and stamped(r)]
    starts = [r for r in missions if r.get('mission') in ('waypoints', 'exploring')]
    if not truth or not starts:
        return []
    t = np.array([r['sim_time'] for r in truth])
    p = np.array([r['pose'] for r in truth])
    yaw = np.unwrap(p[:, 2])
    def at(stamp):
        if not t[0] <= stamp <= t[-1]:
            raise ValueError('Truth does not bracket the mission event')
        return np.array([np.interp(stamp, t, p[:, 0]), np.interp(stamp, t, p[:, 1]), np.interp(stamp, t, yaw)])
    try:
        home = at(starts[0]['sim_time'])
    except ValueError:
        return []
    reports = []
    for trial in sorted({r.get('trial', 0) for r in starts}):
        events = [r for r in missions if r.get('trial', 0) == trial]
        active = [r for r in events if r.get('mission') != 'idle']
        ends = [r for r in events if r.get('mission') == 'idle' and r.get('result')]
        report = dict(trial=trial, passed=False, result='incomplete', collision_scope='chassis sensor only')
        reports.append(report)
        if not active or not ends:
            continue
        start, end = active[0]['sim_time'], ends[0]['sim_time']
        try:
            final = at(end)
        except ValueError:
            continue
        reached = [r['sim_time'] for r in events if r.get('outbound_reached') and start <= r['sim_time'] <= end]
        # A wandering return must not count as outward travel.
        route = p[(t >= start) & (t <= (reached[0] if reached else end))]
        displacement = float(np.linalg.norm(route[:, :2]-home[:2], axis=1).max()) if len(route) else 0.
        distance = float(np.linalg.norm(final[:2]-home[:2]))
        heading = math.degrees(abs(math.atan2(math.sin(final[2]-home[2]), math.cos(final[2]-home[2]))))
        contacts = sum(r['count'] for r in records if r.get('kind') == 'chassis_contacts' and stamped(r) and start <= r['sim_time'] <= end)
        statuses = [r for r in records if r.get('kind') == 'status' and stamped(r) and start <= r['sim_time'] <= end]

        def fault_in_trial(status):
            fault = status.get('fault')
            return bool(fault) and (not stamped(fault) or start <= fault['sim_time'] <= end)

        faults = [r for r in statuses if not r.get('ready') or fault_in_trial(r) or
                  (r.get('mode') == 'stopped' and r.get('reason') not in ('operator_stop', ''))]
        outbound = bool(reached)
        report.update(result=ends[0]['result'], outward_displacement_m=displacement,
                      final_home_error_m=distance, final_home_yaw_error_deg=heading,
                      outbound_reached=outbound, chassis_contact_events=contacts, safety_fault_samples=len(faults),
                      guard_faults=sorted({r['fault']['reason'] for r in statuses if fault_in_trial(r)}))
        report['passed'] = bool(report['result'] == 'Home reached' and outbound and displacement >= min_outward
                                and distance <= .15 and heading <= 10. and not contacts and not faults)
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.recording.read_text().splitlines() if line.strip()]
    print(json.dumps(dict(**summarize(records), trials=summarize_trials(records)), indent=2, allow_nan=False))
