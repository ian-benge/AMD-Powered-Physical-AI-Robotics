#!/usr/bin/env python3
"""One-session protected reboot review; never reconstructs a clean teardown.

Only permits a fresh unchanged native capture after root-reviewed journals,
idle checks and cooldown. Does not start rendering, NPU, terrain or motion.
"""
import argparse
import datetime as dt
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time

BASE=Path('/home/ian/capstone-training')
STAGE=BASE/'rover-sim'
CASE=BASE/'rover-navigation/navigation_sessions/oem617-7141-rocm-manual-20261109t212959z-21ab-20261109T213005Z-1mk81tgq'
OLD_BOOT='36049adc-2956-4aed-8091-b1fcd3470630'
BOOT_CHAIN=[OLD_BOOT,'3e465269-cd09-4760-ab02-51213fdc2624','9d20c7d3-3bc5-4e0d-90a4-d1e4ce458438']
SELECTION_SHA='7bdbfba3592a7c7131960b62b42f3aece965f7577141e3c6366dd205c3691f75'
NATIVE_PINS={'manage.py':'3d24fd08fbe8d94acc86f94980382310bd8e0b80ae3ef53a02c7da1afb1394b1',
             'rover-sim.sh':'30c3f4fdfb5702d9e993ec4b90aa8fb8b1b13ecd1cb71e5611c08f90bfff90c2'}
INCIDENT=re.compile(r'ring .*timeout|amdgpu_job_timedout|GPU reset begin|GPU reset succeeded|\bMES\b.*(?:fail|error)|VRAM is lost|GPU fault|kernel panic|soft lockup|hard LOCKUP|out of memory|oom-kill|machine check|hardware error',re.I)


def require(condition,message):
    if not condition:raise RuntimeError(message)


def utc():return dt.datetime.now(dt.timezone.utc).isoformat()
def boot():return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
def read(path):return json.loads(path.read_text())


def sha(path):
    require(not path.is_symlink() and path.is_file(),'Expected regular file: '+str(path))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())


def unchanged():
    require(boot()==BOOT_CHAIN[-1],'Another reboot occurred; this one-session review cannot be reused')
    require(sha(STAGE/'CURRENT.json')==SELECTION_SHA,'Native selection changed')
    require(read(STAGE/'CURRENT.json')['boot_id']==OLD_BOOT,'Predecessor boot mismatch')
    require((CASE/'boot-id.txt').read_text().strip()==OLD_BOOT,'Predecessor case mismatch')
    state=read(CASE/'LATEST_CAPTURE_STATE.json')
    require(state['boot_id']==OLD_BOOT and state['state'] in ('running','stopped'),'Unexpected saved capture state')
    require(not any((CASE/n).exists() for n in ['POST_TEARDOWN.json','MANUAL_SESSION_FINAL.json','INCIDENT_DETECTED.txt','CONTROL_STOP_REQUIRED.txt']),
            'New final evidence or an incident marker needs independent review')
    for name,expected in NATIVE_PINS.items():require(sha(STAGE/name)==expected,'Native helper changed: '+name)


def command(args):
    r=subprocess.run(args,capture_output=True,text=True,timeout=60)
    return dict(argv=args,returncode=r.returncode,stdout=r.stdout,stderr=r.stderr)


def idle():
    checks=[command(['docker','ps','-q'])]+[command(['ss','-ltnH','sport','=',':'+str(p)]) for p in [8774,8775,8777,8778,8782]]
    require(all(c['returncode']==0 and not c['stdout'].strip() for c in checks),'Workloads remain or inspection failed')
    kfd=command(['fuser','-v','/dev/kfd'])
    require(kfd['returncode']==1 and not kfd['stdout'].strip() and not kfd['stderr'].strip(),'KFD owners remain or inspection failed')
    return checks+[kfd]


def tctl():
    values=[]
    for device in Path('/sys/class/hwmon').glob('hwmon*'):
        if (device/'name').read_text().strip()!='k10temp':continue
        for label in device.glob('temp*_label'):
            if label.read_text().strip()=='Tctl':values.append(float(label.with_name(label.name.replace('_label','_input')).read_text())/1000)
    require(len(values)==1 and -20<=values[0]<90,'Missing, ambiguous or unsafe Tctl')
    return values[0]


def journal_review(text,expected,since=None):
    rows=[json.loads(line) for line in text.splitlines() if line.strip()]
    require(bool(rows),'No protected kernel journal; independent investigation required')
    require(all(r.get('_BOOT_ID')==expected.replace('-','') and r.get('_TRANSPORT')=='kernel' for r in rows),'Journal boot or transport mismatch')
    if since is not None:
        cutoff=dt.datetime.fromisoformat(since).timestamp()*1e6
        rows=[r for r in rows if int(r['__REALTIME_TIMESTAMP'])>=cutoff]
    return [r for r in rows if INCIDENT.search(str(r.get('MESSAGE','')))]


def cooldown_pass(samples):
    if len(samples)<7:return False
    times=[dt.datetime.fromisoformat(s['utc']) for s in samples[-7:]]
    return ((times[-1]-times[0]).total_seconds()>=30 and
        all(0<(b-a).total_seconds()<=10 for a,b in zip(times,times[1:])) and
        all(-20<=s['tctl_c']<=70 for s in samples[-7:]))


def review(folder,acknowledged,reason):
    require(os.geteuid()==0 and os.isatty(0),'Run --review with sudo in the Halo terminal')
    require(acknowledged,'Explicitly acknowledge the missing original teardown')
    unchanged();require(not (folder/'PROTECTED_REVIEW.json').exists(),'Preserve the previous protected review')
    evidence=Path(tempfile.mkdtemp(prefix='protected-review-',dir=folder))
    report=dict(utc=utc(),case=str(CASE),old_boot_id=OLD_BOOT,recovery_boot_id=boot(),boot_chain=BOOT_CHAIN,
        selection_sha256=SELECTION_SHA,original_teardown_completed=False,post_teardown_clean=False,
        monitoring_gap=True,operator_reboot_reason=reason,status='BLOCKED',files={},commands={})
    try:
        report['idle_before']=idle()
        report['saved_predecessor_state']=read(CASE/'LATEST_CAPTURE_STATE.json')['state']
        sources=[STAGE/'CURRENT.json',CASE/'capture-events.jsonl',CASE/'temperature-live.jsonl',
                 CASE/'LATEST_CAPTURE_STATE.json',CASE/'CAPTURE_PREFLIGHT.json',CASE/'boot-id.txt']
        for index,source in enumerate(sources):
            dest=evidence/(str(index)+'-'+source.name);shutil.copyfile(source,dest)
            require(sha(dest)==sha(source),'Evidence changed during preservation')
            report['files'][str(dest.relative_to(folder))]=sha(dest)
        since=read(CASE/'CAPTURE_PREFLIGHT.json')['utc']
        for index,ident in enumerate(BOOT_CHAIN):
            args=['journalctl','_BOOT_ID='+ident.replace('-',''),'_TRANSPORT=kernel','--no-pager','-o','json']
            result=command(args);dest=evidence/f'kernel-{index}.jsonl';dest.write_text(result['stdout'])
            report['files'][str(dest.relative_to(folder))]=sha(dest)
            report['commands'][f'kernel-{index}']={k:v for k,v in result.items() if k!='stdout'}
            require(result['returncode']==0,'Protected journal inspection failed')
            matches=journal_review(result['stdout'],ident,since if ident==OLD_BOOT else None)
            report[f'kernel_{index}_incident_window_since']=since if ident==OLD_BOOT else 'boot start'
            report[f'kernel_{index}_incident_matches']=matches
            require(not matches,'Kernel incident detected; no fresh capture is permitted by this helper')
        samples=[];start=time.monotonic()
        print('Recording current-boot idle cooldown; original teardown and monitoring gap remain incomplete.',flush=True)
        while True:
            sample=dict(utc=utc(),tctl_c=tctl());samples.append(sample);print(json.dumps(sample),flush=True)
            if time.monotonic()-start>=60:break
            time.sleep(5)
        report['cooldown_samples']=samples;require(cooldown_pass(samples),'Retained cooldown requirement failed')
        report['idle_after']=idle()
        result=command(['journalctl','_BOOT_ID='+boot().replace('-',''),'_TRANSPORT=kernel','--no-pager','-o','json'])
        dest=evidence/'final-current-kernel.jsonl';dest.write_text(result['stdout']);report['files'][str(dest.relative_to(folder))]=sha(dest)
        require(result['returncode']==0 and not journal_review(result['stdout'],boot()),'Current kernel incident during cooldown')
        unchanged()
        group=CASE.stat().st_gid
        for name in report['files']:
            f=folder/name;os.chown(f,-1,group);f.chmod(0o640)
        os.chown(evidence,-1,group);evidence.chmod(0o750)
        report.update(status='REVIEWED_REBOOT_WITH_MISSING_TEARDOWN',fresh_capture_preparation_permitted=True)
    except Exception as error:report['failure']=str(error);raise
    finally:
        report['finished_utc']=utc();path=folder/'PROTECTED_REVIEW.json';save(path,report)
        os.chown(path,-1,CASE.stat().st_gid);path.chmod(0o640)
        print(json.dumps({k:report.get(k) for k in ['status','failure','post_teardown_clean','monitoring_gap','fresh_capture_preparation_permitted']}),flush=True)


def verify_report(report):
    require(report.get('status')=='REVIEWED_REBOOT_WITH_MISSING_TEARDOWN' and
        report.get('fresh_capture_preparation_permitted') is True and report.get('case')==str(CASE) and
        report.get('recovery_boot_id')==BOOT_CHAIN[-1] and report.get('boot_chain')==BOOT_CHAIN and
        report.get('selection_sha256')==SELECTION_SHA and report.get('post_teardown_clean') is False and
        report.get('monitoring_gap') is True and report.get('original_teardown_completed') is False,'Protected review identity/status mismatch')
    require(0<=(dt.datetime.now(dt.timezone.utc)-dt.datetime.fromisoformat(report['finished_utc'])).total_seconds()<600,'Protected review is older than ten minutes')


def capture(folder):
    require(os.geteuid()==CASE.stat().st_uid!=0,'Run --capture as the original Halo user without sudo')
    unchanged();path=folder/'PROTECTED_REVIEW.json';info=path.stat()
    require(not path.is_symlink() and info.st_uid==0 and not info.st_mode&0o022,'Root-owned protected review required')
    report=read(path);verify_report(report);require(bool(report['files']),'Missing preserved evidence')
    for name,expected in report['files'].items():
        dest=folder/name
        require(not Path(name).is_absolute() and '..' not in Path(name).parts and dest.resolve().is_relative_to(folder.resolve()),'Invalid evidence path')
        require(sha(dest)==expected,'Protected evidence changed')
    require(tctl()<=70,'Wait for cooldown before capture preparation')
    spec=importlib.util.spec_from_file_location('native_demo_recovery',STAGE/'manage.py');native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
    import platform
    require(platform.release()==native.KERNEL,'Select the retained OEM kernel')
    require(platform.freedesktop_os_release().get('VERSION','').startswith('24.04.5 '),'Unexpected OS')
    require(native.run(['powerprofilesctl','get'])=='power-saver','Retain power-saver')
    native.idle()
    label='rocm-manual-'+dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ').lower()+'-'+os.urandom(2).hex()
    kit=STAGE/'runs'/label;native.build(native.SOURCE,native.PHYSICAL,kit,label,boot())
    selection=dict(label=label,kit=str(kit),boot_id=boot(),utc=utc());native.save(kit/'SESSION_SELECTION.json',selection)
    save(folder/'RECOVERY_SUCCESSOR.json',dict(selection=selection,predecessor_case=str(CASE),
        protected_review_sha256=sha(path),original_teardown_completed=False,native_launcher_modified=False))
    unchanged();temp=STAGE/('CURRENT.demo-recovery-'+label+'.tmp');save(temp,selection);temp.replace(STAGE/'CURRENT.json')
    print('FRESH_NATIVE_CAPTURE: '+label,flush=True)
    os.execvp('sudo',['sudo','/usr/bin/python3','-u','-I',str(kit/'capture_rocm_retest.py'),'--label',label,'--seconds','7200'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);m=p.add_mutually_exclusive_group(required=True)
    m.add_argument('--check',action='store_true');m.add_argument('--review',action='store_true');m.add_argument('--capture',action='store_true')
    p.add_argument('--acknowledge-missing-teardown',action='store_true');p.add_argument('--reason',choices=('intentional','crash','unknown'),default='unknown')
    a=p.parse_args();folder=Path(__file__).resolve().parent
    manifest=read(folder/'RECOVERY_SHA256.json')
    for name,expected in manifest['files'].items():require(Path(name).name==name and sha(folder/name)==expected,'Recovery helper pin mismatch')
    if a.check:unchanged();print('PINNED_REBOOT_RECOVERY_CHECK_PASS')
    elif a.review:review(folder,a.acknowledge_missing_teardown,a.reason)
    else:capture(folder)
