#!/usr/bin/env python3
"""Fail closed when a final 600-second visibility recording is incomplete.

Offline RGB/geometry gates; protected capture/kernel teardown is a separate
required review. Tolerances are evaluator choices, not navigation limit changes.
"""
import argparse
import hashlib
import json
from pathlib import Path
from phase1_pose_sync import measured_pose_bracket


def metric_failures(metrics):
    failures=[]
    for key,limit in [('p95_error_m',.03),('fraction_over_3cm',.01),
                      ('expected_finite_missing_fraction',.005),('unexpected_usable_fraction',.001)]:
        value=metrics.get(key)
        if value is None or not 0<=value<=limit: failures.append(key)
    for item in metrics.get('visibility',[]):
        if item['predicted_pixels']>=100 and item.get('rgb_chroma_support_fraction',0)<.90:
            failures.append('rgb_visibility:'+item['name'])
    if metrics.get('near_comparison_pixels',metrics.get('near_occluded_pixels',0))>=100 and metrics.get('near_background_support_fraction',0)<.90:
        failures.append('near_clipping')
    panel=metrics.get('transparent_panel',{})
    if panel.get('sky_pixels',0)>=100 and panel.get('intermediate_alpha_fraction',0)<.95:
        failures.append('transparent_panel_blending')
    return failures


def evaluate(recording, geometry):
    recording=Path(recording); capture=recording/'capture'
    failures=[]
    session=json.loads((recording/'session-result.json').read_text())
    summary=json.loads((capture/'summary.json').read_text())
    configuration=json.loads((recording/'configuration.json').read_text())
    if not session.get('capture_complete') or session.get('failure') or session.get('interrupted'):
        failures.append('session_incomplete_or_failed')
    if not summary.get('capture_complete') or summary.get('failure') or summary.get('requested_seconds')!=600 or summary.get('duration_wall_s',0)<600:
        failures.append('full_600_seconds_not_completed')
    if summary.get('saved_frames',0)<540: failures.append('insufficient_saved_frame_coverage')
    for key in ('uniform_rgb','empty_depth','unsynchronized_pose'):
        if summary['counts'].get(key)!=0: failures.append(key)
    camera=configuration['camera']
    if (camera['near_m'],camera['far_m'],camera['rgb_far_m'])!=(.3,8,20): failures.append('unqualified_camera_limits')
    frames=[json.loads(row) for row in (capture/'frames.jsonl').read_text().splitlines()]
    health=[json.loads(row) for row in (capture/'frame-health.jsonl').read_text().splitlines()]
    if len(health)!=summary['counts'].get('matched'): failures.append('frame_health_count_mismatch')
    if len(frames)!=summary['saved_frames']: failures.append('saved_frame_count_mismatch')
    for frame in frames:
        index=frame['index']
        if not frame.get('synchronized_pose') or frame.get('pose_gap_s',99)>.05:
            failures.append('unsynchronized_saved_pose:'+str(index))
        if summary.get('pose_policy')=='equal_measured_poses_bracketing_frame':
            bracket=measured_pose_bracket(frame.get('pose_bracket',[]),frame['stamp_ns'])
            if not frame.get('stable_pose_bracket') or not bracket or not bracket['stable'] or bracket['pose']!=frame['pose']:
                failures.append('ambiguous_saved_pose:'+str(index))
        if not all((capture/f'{kind}-{index:05d}.{extension}').is_file()
                   for kind,extension in [('rgb','png'),('depth','npz')]):
            failures.append('missing_raw_frame:'+str(index))
    data=json.loads(Path(geometry).read_text())
    if data.get('sampled') or not data.get('calibration_sha256'):
        failures.append('incomplete_or_uncalibrated_geometry_review')
    if len(data['frames'])!=len(frames) or {f['index'] for f in data['frames']}!={f['index'] for f in frames}:
        failures.append('geometry_frame_coverage_mismatch')
    for frame in data['frames']:
        if 'error' in frame or 'axial' not in frame.get('modes',{}):
            failures.append('geometry_error:'+str(frame['index'])); continue
        failures.extend(str(frame['index'])+':'+reason for reason in metric_failures(frame['modes']['axial']))
    return dict(verdict='FAIL_OR_INCOMPLETE' if failures else 'VISIBILITY_CHECKS_PASS_PENDING_PROTECTED_TEARDOWN',
                failures=failures,mode=summary['mode'],image=configuration['image'],
                duration_wall_s=summary['duration_wall_s'],saved_frames=len(frames),checked_rgbd_pairs=len(health),
                geometry_sha256=hashlib.sha256(Path(geometry).read_bytes()).hexdigest(),
                evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                limitation='Protected capture/kernel/teardown review required; fixture replay is not navigation acceptance')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path); parser.add_argument('--geometry',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=evaluate(args.recording,args.geometry or args.recording/'capture/geometry-comparison.json')
    with args.output.open('x') as stream: json.dump(result,stream,indent=2)
    print(json.dumps(result))
    raise SystemExit(bool(result['failures']))
