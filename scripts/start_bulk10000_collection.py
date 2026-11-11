#!/usr/bin/env python3
"""Attended, resumable 10,000-image collection; never trains or approves labels."""
import argparse
from collections import Counter
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import bulk10000_plan as policy
import start_pilot200_collection as pilot
from bulk_guarded_step import retry_session_scan
from desktop_render_guard import DesktopUnavailable, stop_path

CONTROL = Path(__file__).resolve().parent
LATEST = CONTROL/'LATEST_COLLECTION.path'
TRANSFER = ('start_bulk10000_collection.py', 'bulk10000_capture_worker.py',
            'bulk10000_plan.py', 'pilot200_capture_worker.py', 'process_cleanup.py',
            'pilot200_environment_v2.json','bulk_guarded_step.py','desktop_render_guard.py')


def capture_application(pins):
    manifest=CONTROL/'CAPTURE_RUNTIME.json'
    if not manifest.exists():
        return pilot.APP
    runtime=json.loads(manifest.read_text())
    app=Path(runtime['application']).resolve()
    expected=pilot.BASE/'bulk10000-capture-recovery-20261106'
    revision=app.name.removeprefix('application-v')
    if (runtime.get('version') != 1 or app.parent != expected or
            not app.name.startswith('application-v') or not revision.isdigit() or int(revision)<1 or
            runtime['base_source_archive_sha256'] != pins['source_archive_sha256'] or
            runtime['image'] != pins['image']):
        raise ValueError('Capture-only runtime has a different base or identity')
    prefixes=['source/src/mars_training_environment/mars_training_environment/',
              'install/mars_training_environment/lib/python3.12/site-packages/mars_training_environment/']
    changed={prefix+name for prefix in prefixes for name in ['capture.py','review.py','reframing.py']}
    changed.add('source/src/mars_training_environment/test/test_reframing.py')
    added={prefix+'capture_recovery.py' for prefix in prefixes}
    added.add('source/src/mars_training_environment/test/test_capture_recovery.py')
    added.add('source/src/mars_training_environment/test/test_capture_pose_feedback.py')
    files=runtime['files']
    if set(files)-set(pins['files'])-added:
        raise ValueError('Capture runtime adds an unexpected file')
    for name,hash_ in pins['files'].items():
        if name not in files or (name not in changed and files[name] != hash_):
            raise ValueError('Capture runtime changes a protected application file: '+name)
    policy.verify_files(app,files)
    print('CAPTURE_RECOVERY_RUNTIME_CHECK_PASS: unchanged renderer, geometry and acceptance policy',flush=True)
    return app


def atomic(path, value):
    path = Path(path)
    temp = path.with_name(path.name+'.tmp')
    with temp.open('w') as stream:
        json.dump(value, stream, indent=2); stream.write('\n')
        stream.flush(); os.fsync(stream.fileno())
    temp.replace(path)


def root_for(path=None):
    root = Path(path or LATEST.read_text().strip()).resolve()
    if root.parent != pilot.BASE or not root.name.startswith('bulk10000-'):
        raise ValueError('Collection must be a bulk10000 directory under navigation_sessions')
    return root


def status(root):
    plan = json.loads((root/'SCENE_RESERVATIONS.json').read_text())
    receipts = list((root/'batch-receipts').glob('batch-*.json'))
    counts = Counter()
    for file in receipts:
        value = json.loads(file.read_text())
        counts[value['split']] += value['frames']
    progress = json.loads((root/'PROGRESS.json').read_text()) if (root/'PROGRESS.json').exists() else {}
    current = {}
    batch_id = progress.get('batch')
    if batch_id in {batch['id'] for batch in plan['batches']}:
        dataset = root/'batches'/batch_id/'dataset'
        current = dict(batch=batch_id,planned_images=200,
            committed_images=len(list((dataset/'metadata').glob('*/*.json'))),
            committed_scenes=len(list((dataset/'reports').glob('scene_*.json'))))
        attempts = list((dataset/'attempts').glob('*/attempt-*/launch.log'))
        if attempts:
            latest = max(attempts,key=lambda path:path.stat().st_mtime).parent
            current.update(scene=latest.parent.name,attempt=latest.name,
                attempt_saved_views=len(list((latest/'metadata').glob('*/*.json'))))
    return dict(collection=str(root), recorded_images=sum(counts.values()), planned_images=10000,
                completed_batches=len(receipts), planned_batches=len(plan['batches']),
                split_images=dict(counts), progress=progress,current_batch=current,
                training_approved=False, human_review_required=True)


def capture_time_remaining(case, now=None):
    """Use the native capture's declared duration; never extend or restart it."""
    with (case/'capture-events.jsonl').open() as stream:
        first = next(json.loads(line) for line in stream if '"capture_started"' in line)
    duration = first['duration_seconds']
    started = dt.datetime.fromisoformat(first['utc'])
    if type(duration) is not int or not 60 <= duration <= 7200 or started.tzinfo is None:
        raise ValueError('Native capture timing evidence is invalid')
    clock = now or dt.datetime.now(dt.timezone.utc)
    elapsed = (clock-started).total_seconds()
    if elapsed < -5:
        raise ValueError('Native capture starts in the future')
    return max(0.,duration-elapsed)


def session_budget(root, case, now=None, batch=None):
    """Leave time for a whole batch and native guard margin between sessions."""
    durations = []
    receipts = sorted((root/'batch-receipts').glob('batch-*.json'))[-3:]
    for path in receipts:
        receipt = json.loads(path.read_text())
        result = policy.inside(root,receipt['guard_result'])
        value = json.loads(result.read_text())
        started = dt.datetime.strptime(result.parent.parent.name,'%Y%m%dT%H%M%S%fZ').replace(tzinfo=dt.timezone.utc)
        ended = dt.datetime.fromisoformat(value['utc'])
        durations.append(max(0.,(ended-started).total_seconds()))
    work = min(5400.,max([1800.]+durations)*1.3)
    remaining_scenes=None;generation_reused=False
    if batch is not None:
        folder=root/'batches'/batch['id'];marker=folder/'BUNDLE_PINS.json'
        if marker.exists():
            pins=json.loads(marker.read_text())
            reservation=json.loads((root/'SCENE_RESERVATIONS.json').read_text())
            if pins['batch']!=batch or pins['reservation_sha256']!=policy.digest(reservation):
                raise ValueError('Cached budget bundle belongs to another reservation')
            policy.verify_files(folder/'bundle',pins['files'])
            plan=json.loads((folder/'bundle/plan.json').read_text())
            if {entry['id'] for entry in plan['scenes']}!={entry['id'] for entry in batch['scenes']}:
                raise ValueError('Cached budget scene identities changed')
            complete=0
            for entry in plan['scenes']:
                report=folder/'dataset/reports'/(entry['id']+'.json')
                if report.exists():
                    value=json.loads(report.read_text())
                    records=value.get('records',[])
                    if (value.get('scene_sha256')!=entry['content_sha256'] or value.get('frames')!=entry['views'] or
                            len(records)!=entry['views'] or len(set(records))!=len(records) or
                            not value.get('evidence_sha256')):
                        raise ValueError('Cached budget capture report is incomplete')
                    policy.verify_files(folder/'dataset',value['evidence_sha256'])
                    complete+=1
            remaining_scenes=len(plan['scenes'])-complete;generation_reused=True
            work=max(900.,work*remaining_scenes/max(1,len(plan['scenes'])))
    required = work+180.
    remaining = capture_time_remaining(case,now)
    return dict(remaining_seconds=round(remaining,1),required_seconds=round(required,1),
                start_batch=remaining >= required,native_duration_limit_unchanged=True,
                generation_reused=generation_reused,remaining_scenes=remaining_scenes)


def load_session_modules(kit):
    sys.path.insert(0, str(pilot.APP/'management'))
    from phase1_run_fixture import load_guard, live_guard
    # The verified kit loader adds the kit's own modules to sys.path.
    trial = load_guard(kit)
    import remote_session
    return trial, live_guard, remote_session


def session():
    context = dict(line.split('=', 1) for line in subprocess.check_output(
        ['python3', '-I', '/home/ian/capstone-training/rover-sim/manage.py', 'context'],
        text=True).splitlines() if '=' in line)
    kit = Path(context['SIM_KIT'])
    pointer = Path('/home/ian/capstone-training/halo-crash-tools/oem617-trial-20261103')/('capture-'+context['SIM_LABEL']+'.path')
    case = Path(pointer.read_text().strip())
    if (case/'MANUAL_SESSION_FINAL.json').exists():
        raise ValueError('Start fresh capture and temperature terminals first')
    trial, live_guard, remote_session = load_session_modules(kit)
    inventory = retry_session_scan(lambda:remote_session.inventory(require_view=True))
    acknowledgement = case/'local-capture-confirmed.json'
    if not acknowledgement.exists():
        pilot.save(acknowledgement, dict(utc=trial.utc(), case=str(case),
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            delivery='local-files-only', offhost_delivery_verified=False,
            operator_confirmation='Operator invoked attended full collection from the Halo desktop',
            **inventory))
    retry_session_scan(lambda:trial.guard(case))
    retry_session_scan(lambda:live_guard(trial,case))
    if subprocess.check_output(['docker','ps','-q'], text=True).strip():
        raise ValueError('Stop other workloads; leave NPU, terrain and rover simulation off')
    return kit, case, trial, live_guard


def execute_batch(root, batch, plan_sha, pins, kit, case, trial, application=None):
    application=application or pilot.APP
    folder = root/'batches'/batch['id']
    folder.mkdir(parents=True, exist_ok=True)
    recipe = dict(batch=batch, reservation_sha256=plan_sha,
                  profile_sha256=policy.sha(CONTROL/'pilot200_environment_v2.json'))
    identity = folder/'BATCH_IDENTITY.json'
    if identity.exists():
        if json.loads(identity.read_text()) != recipe:
            raise ValueError('Existing batch has a different reservation')
    else:
        pilot.save(identity, recipe)
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    run = root/'invocations'/stamp
    run.mkdir(parents=True)
    for name in TRANSFER:
        shutil.copy2(CONTROL/name, run/name)
    runtime=CONTROL/'CAPTURE_RUNTIME.json'
    if runtime.exists():
        shutil.copy2(runtime,run/runtime.name)
    pilot.save(run/'RECIPE.json', recipe)
    pilot.save(run/'INVOCATION.json', dict(utc=trial.utc(), case=str(case), kit=str(kit),
        batch=batch['id'], runtime_version=2,capture_application=str(application),
        capture_runtime_sha256=policy.sha(runtime) if runtime.exists() else None,
        source_sha256={name:policy.sha(run/name) for name in TRANSFER}))
    try:
        display, authority = pilot.desktop_authority()
    except DesktopUnavailable as error:
        atomic(root/'PROGRESS.json', dict(stage='DESKTOP_RECONNECT_REQUIRED', batch=batch['id'], reason=str(error)))
        raise
    name = 'rover-bulk10000-'+stamp.lower()
    command = ['docker','run','--rm','--init','--name',name,'--network','host',
        '--shm-size=1g','--user',f'{os.getuid()}:{os.getgid()}', '--device','/dev/dri','--group-add','992']
    for value in ['HOME=/tmp','ROS_LOG_DIR=/collection/roslogs','DISPLAY='+display,
                  'XAUTHORITY=/tmp/bulk10000.xauth','LIBGL_ALWAYS_SOFTWARE=0','ROS_DOMAIN_ID=231',
                  'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
                  'PYTHONPATH=/work/source/scripts:/work/management:/work/testdeps',
                  'PYTHONDONTWRITEBYTECODE=1','PYTHONUNBUFFERED=1',
                  'OMP_NUM_THREADS=2','OPENBLAS_NUM_THREADS=2']:
        command += ['-e',value]
    for value in ['/tmp/.X11-unix:/tmp/.X11-unix:ro',str(authority)+':/tmp/bulk10000.xauth:ro',
                  str(application)+':/work:ro',str(pilot.ASSETS)+':/prepared-assets:ro',
                  str(folder)+':/collection',str(run)+':/invocation:ro']:
        command += ['-v',value]
    command += ['-w','/work/source',pins['image'],'bash','-c',
        'source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && '
        'exec python3 /invocation/bulk10000_capture_worker.py']
    config = run/'guard-configuration.json'
    pilot.save(config, dict(application=str(application), output=str(run/'guarded-step'),
        case=str(case), kit=str(kit), image=pins['image'], container=name,
        timeout_seconds=5400, command=command))
    atomic(root/'PROGRESS.json',dict(stage='RUNNING', batch=batch['id'], split=batch['split'],
        progress_log=str(run/'guarded-step/log.txt'), batch_directory=str(folder)))
    print('CURRENT_BATCH='+batch['id'], flush=True)
    print('PROGRESS_LOG='+str(run/'guarded-step/log.txt'), flush=True)
    try:
        code = pilot.run_guarded(['python3',str(run/'bulk_guarded_step.py'),str(config)], name, pins['image'],application)
    finally:
        authority.unlink(missing_ok=True)
    if code:
        desktop_stop = stop_path(folder, run/'INVOCATION.json')
        detail = dict(stage='STOPPED_REVIEW_LOG')
        if desktop_stop.exists():
            value = json.loads(desktop_stop.read_text())
            if value.get('stage') == 'DESKTOP_RECONNECT_REQUIRED':
                detail = dict(stage=value['stage'], reason=value['reason'], desktop_stop=str(desktop_stop))
        atomic(root/'PROGRESS.json',dict(**detail, batch=batch['id'],
            progress_log=str(run/'guarded-step/log.txt'), exit_code=code))
        print('Outputs preserved. Close monitoring with halo-rover finish before a fresh session.',flush=True)
        return code
    guarded = run/'guarded-step/result.json'
    guard_result = json.loads(guarded.read_text())
    if guard_result['failure'] is not None or guard_result['exit_code'] != 0:
        raise ValueError('Batch lacks a successful guarded completion')
    result = policy.verify_batch(folder, batch, plan_sha)
    pilot.save(root/'batch-receipts'/(batch['id']+'.json'), dict(batch_id=batch['id'],
        split=batch['split'], frames=result['frames'], backgrounds=result['background_frames'],
        result_sha256=policy.sha(folder/'BATCH_RESULT.json'),
        guard_result=guarded.relative_to(root).as_posix(), guard_sha256=policy.sha(guarded),
        utc=trial.utc(), case=str(case)))
    atomic(root/'PROGRESS.json',dict(stage='BATCH_COMPLETE',batch=batch['id'],
        split=batch['split'],progress_log=str(run/'guarded-step/log.txt')))
    return 0


def verify_prior(root, batch, plan_sha, image_splits):
    receipt = root/'batch-receipts'/(batch['id']+'.json')
    if not receipt.exists():
        return False
    value = json.loads(receipt.read_text())
    folder = root/'batches'/batch['id']
    if (value['batch_id'] != batch['id'] or value['split'] != batch['split'] or
            value['frames'] != 200 or value['backgrounds'] != 40 or
            value['result_sha256'] != policy.sha(folder/'BATCH_RESULT.json')):
        raise ValueError('Completed batch receipt changed')
    guard = policy.inside(root, value['guard_result'])
    if policy.sha(guard) != value['guard_sha256']:
        raise ValueError('Saved guard evidence changed')
    evidence = json.loads(guard.read_text())
    if evidence['failure'] is not None or evidence['exit_code'] != 0:
        raise ValueError('Saved guard did not pass')
    policy.verify_batch(folder, batch, plan_sha)
    for name, hash_ in json.loads((folder/'BATCH_PINS.json').read_text())['files'].items():
        if name.startswith('dataset/images/'):
            previous = image_splits.setdefault(hash_, batch['split'])
            if previous != batch['split']:
                raise ValueError('Identical RGB occurs across splits; review before continuing')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--check',action='store_true',help='Static checks; no container')
    group.add_argument('--status',action='store_true',help='Read saved progress; no container')
    group.add_argument('--run',action='store_true',help='Start or resume the reserved full collection')
    parser.add_argument('--collection',type=Path,help='Explicit prior collection instead of LATEST')
    parser.add_argument('--batches',type=int,default=50,help='Maximum batches this invocation (1..50)')
    args = parser.parse_args()
    if not 1 <= args.batches <= 50:
        parser.error('--batches must be from 1 to 50')
    if args.status:
        if not args.collection and not LATEST.exists():
            print('No full collection started.');return 0
        print(json.dumps(status(root_for(args.collection)),indent=2));return 0
    pins = pilot.check_candidate()
    application = capture_application(pins)
    plan = json.loads((CONTROL/'SCENE_RESERVATIONS.json').read_text())
    profile_sha = policy.sha(CONTROL/'pilot200_environment_v2.json')
    policy.validate_reservation(plan)
    if plan != policy.reservation(profile_sha):
        raise ValueError('Predeclared full collection reservation changed')
    if args.check:
        print('BULK10000_STATIC_CHECK_PASS: 500 scenes, 10000 images, 50 resumable batches')
        return 0
    import fcntl
    with (CONTROL/'collection.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        kit, case, trial, live_guard = session()
        if args.collection or LATEST.exists():
            root = root_for(args.collection)
        else:
            if shutil.disk_usage(pilot.BASE).free < 120*1024**3:
                raise ValueError('Reserve at least 120 GiB free for scenes, captures and preserved attempts')
            stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            root = pilot.BASE/('bulk10000-'+stamp);root.mkdir()
            pilot.save(root/'SCENE_RESERVATIONS.json',plan)
            pilot.save(root/'COLLECTION_IDENTITY.json',dict(source_archive_sha256=pins['source_archive_sha256'],
                reservation_sha256=policy.digest(plan), candidate=str(pilot.CANDIDATE),
                planned_images=10000, prior_pilot_modified=False, training_approved=False))
            (root/'batch-receipts').mkdir()
            pointer = LATEST.with_name(LATEST.name+'.tmp')
            pointer.write_text(str(root)+'\n')
            pointer.replace(LATEST)
        if json.loads((root/'SCENE_RESERVATIONS.json').read_text()) != plan:
            raise ValueError('Collection belongs to a different scene reservation')
        identity = json.loads((root/'COLLECTION_IDENTITY.json').read_text())
        if (identity['source_archive_sha256'] != pins['source_archive_sha256'] or
                identity['reservation_sha256'] != policy.digest(plan)):
            raise ValueError('Collection camera candidate or reservation changed')
        print('COLLECTION_DIR='+str(root),flush=True)
        completed_this_time=0;image_splits={}
        for batch in plan['batches']:
            retry_session_scan(lambda:live_guard(trial,case))
            if verify_prior(root,batch,identity['reservation_sha256'],image_splits):
                continue
            if completed_this_time >= args.batches:
                print('Requested batch limit reached; use the same --run command to resume.',flush=True)
                return 0
            if shutil.disk_usage(root).free < 15*1024**3:
                raise ValueError('Less than 15 GiB remains; outputs retained, free storage before resuming')
            budget = session_budget(root,case,batch=batch)
            if not budget['start_batch']:
                previous = json.loads((root/'PROGRESS.json').read_text()) if (root/'PROGRESS.json').exists() else {}
                atomic(root/'PROGRESS.json',dict(stage='SESSION_RENEWAL_REQUIRED',batch=batch['id'],
                    progress_log=previous.get('progress_log'),native_case=str(case),**budget))
                print('SESSION_RENEWAL_REQUIRED: outputs preserved; no new batch started. '
                      'Run halo-rover finish with monitors through cooldown, start fresh capture/temps, '
                      'then use the same --run command.',flush=True)
                print(json.dumps(status(root)),flush=True)
                return 0
            code=execute_batch(root,batch,identity['reservation_sha256'],pins,kit,case,trial,application)
            if code:
                return code
            verify_prior(root,batch,identity['reservation_sha256'],image_splits)
            completed_this_time+=1
            print(json.dumps(status(root)),flush=True)
        atomic(root/'PROGRESS.json',dict(stage='FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW'))
        atomic(root/'COLLECTION_RESULT.json',dict(status='FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW',
            frames=10000,scenes=500,background_frames=2000,
            split_images=dict(train=8000,val=1000,test=1000),
            four_class_export_review_required=True,training_started=False,training_approved=False))
        print('FULL10000_CAPTURE_COMPLETE_PENDING_REVIEW. Run halo-rover finish; keep monitors through cooldown.',flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
