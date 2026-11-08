#!/usr/bin/env python3
"""Bounded candidate preview with original runtime gates and explicit new pins.

Run from the attended Halo desktop after fresh halo-rover capture/temps/npu/
terrain readiness. It never confirms clearance or sends motion commands.
Ctrl+C stops this candidate; halo-rover stop/finish then close host services.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import urllib.request

from phase1_run_fixture import BASELINE, command, digest, live_guard, load_guard, save


def verify_configuration(path):
    config=json.loads(Path(path).read_text())
    if config.get('baseline_image') != BASELINE or config.get('image') == BASELINE:
        raise ValueError('Separate candidate and retained baseline identities required')
    if not re.fullmatch('sha256:[a-f0-9]{64}',config.get('image','')):
        raise ValueError('Immutable candidate image required')
    root=Path(config['project']).resolve()
    if not config.get('files'): raise ValueError('Candidate source/install pins required')
    for relative,sha in config['files'].items():
        file=root/relative
        if not file.resolve().is_relative_to(root) or file.is_symlink() or digest(file)!=sha:
            raise ValueError('Candidate source/install changed: '+relative)
    if digest(config['scene_file']) != config['scene_sha256']:
        raise ValueError('Retained scene changed')
    if config.get('camera') != dict(width=960,height=540,near_m=.3,far_m=8.,rgb_far_m=20.,tilt_rad=.4,
                                    far_clip_mode='axial',near_clip_mode='occlude',render_engine='ogre'):
        raise ValueError('Unqualified candidate camera profile')
    return config,root


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('configuration',type=Path)
    p.add_argument('--kit',type=Path,required=True); p.add_argument('--case',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=600)
    p.add_argument('--attended',action='store_true',required=True)
    p.add_argument('--verify-only',action='store_true')
    a=p.parse_args()
    if not 0<a.seconds<=660: p.error('Bounded preview duration must be in (0,660]')
    config,root=verify_configuration(a.configuration)
    trial=load_guard(a.kit); trial.guard(a.case); live_guard(trial,a.case)
    trial.check_qualification(a.case)
    trial.npu_status(); trial.terrain_status()
    for name in ('npu','terrain'): trial.service_identity(a.case,name)
    if command(['docker','ps','-q']): raise RuntimeError('Stop other containers first')
    if command(['ss','-ltnH','sport','=',':8777']): raise RuntimeError('Dashboard port is already occupied')
    if command(['docker','image','inspect','--format','{{.Id}}',config['image']])!=config['image']:
        raise RuntimeError('Candidate image is missing or changed')
    if not trial.policy.cooled(trial.control_health(a.case)):
        raise RuntimeError('Wait for the retained 30-second cooldown at Tctl <=70 C')
    if a.verify_only:
        print('CANDIDATE_CONFIGURATION_AND_RUNTIME_GATES_PASS'); return 0
    if not os.environ.get('DISPLAY') or os.environ.get('XDG_SESSION_TYPE') not in ('wayland','x11'):
        raise RuntimeError('Run inside the attended graphical desktop terminal')
    session='phase1-live-'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())
    output=root/'navigation_sessions'/session
    output.mkdir(parents=True)
    args=['bash','source/scripts/halo_navigation.sh',config['bundle'],config['scene'],session,
          'gui:=true','rviz:=true','rviz_software_rendering:=1','sensor_render_engine:=ogre','sensor_headless:=false',
          'camera_pitch:=0.4','camera_rgb_far:=20','terrain_backend:=rocm_http','terrain_url:=http://127.0.0.1:8782',
          'adaptive_terrain:=true','mission_controls:=true','controls_port:=8777','preview:=true',
          'mapping_on_confirm:=true','detector_backend:=remote_http','detector_url:=http://127.0.0.1:8775',
          'detector_model:='+config['checkpoint'],'budget_seconds:=3600.0']
    # ROS launch owns the session directory itself; keep supervisor evidence
    # separate so an existing session can never be overwritten.
    record=root/'supervisor-sessions'/session; record.mkdir(parents=True)
    output.rmdir()
    save(record/'configuration.json',dict(candidate=config,command=args,case=str(a.case),kit=str(a.kit),seconds=a.seconds))
    env=dict(os.environ,NAV_IMAGE=config['image'],NAV_INSTALL_REL='install',NAV_CONTAINER_NAME='rover-navigation',
             NAV_SOFTWARE_RENDERING='0',NAV_GZ_PARTITION='rover-phase1-live',NAV_ROS_DOMAIN_ID='87')
    stopped=False
    def stop_signal(*_):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGINT,signal.SIGTERM): signal.signal(sig,stop_signal)
    identity=None; failure=None; proven=False; started=time.monotonic()
    with (record/'launch.log').open('xb') as log,(record/'health.jsonl').open('x') as health:
        child=subprocess.Popen(args,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+a.seconds
        try:
            while child.poll() is None and not stopped and time.monotonic()<deadline:
                health.write(json.dumps(live_guard(trial,a.case))+'\n'); health.flush(); os.fsync(health.fileno())
                info=subprocess.run(['docker','inspect','rover-navigation'],capture_output=True,text=True)
                if info.returncode==0:
                    container=json.loads(info.stdout)[0]
                    if container['Image']!=config['image'] or (identity and identity!=container['Id']):
                        raise RuntimeError('Candidate container was replaced; preserve evidence')
                    identity=container['Id']
                if identity and not proven:
                    try:
                        status=json.load(urllib.request.urlopen('http://127.0.0.1:8777/status',timeout=1))
                    except OSError:
                        status={}
                    camera=status.get('camera_health',{})
                    if camera.get('available') and camera.get('age_seconds',99)<1:
                        if camera.get('low_detail'): raise RuntimeError('Uniform/low-detail candidate camera')
                        result=subprocess.run(['docker','exec',identity,'python3',
                            '/work/management/phase1_renderer_provenance.py'],capture_output=True,text=True,timeout=20)
                        (record/'renderer-provenance.json').write_text(result.stdout)
                        if result.returncode:
                            (record/'renderer-provenance-error.txt').write_text(result.stderr)
                            raise RuntimeError('Candidate renderer/plugin provenance failed')
                        proof=json.loads(result.stdout)
                        plugins={lib['sha256'] for proc in proof['processes'] for lib in proc['libraries']
                                 if 'libgz-rendering8-ogre.so' in lib['path']}
                        if plugins!={config['ogre_plugin_sha256']}:
                            raise RuntimeError('Mapped Ogre backend differs from the candidate build')
                        proven=True
                        print('RENDERER_PROVENANCE_READY: inspect clearance in the dashboard before manual control',flush=True)
                    elif time.monotonic()-started>60:
                        raise RuntimeError('Candidate camera did not become ready within 60 seconds')
                time.sleep(.5)
            if child.poll() not in (None,0): raise RuntimeError('Candidate launch exited: '+str(child.returncode))
            if not proven and not stopped: raise RuntimeError('Candidate exited before renderer readiness')
        except Exception as error: failure=str(error)
        finally:
            cleanup_errors=[]
            if identity is None:
                inspected=subprocess.run(['docker','inspect','rover-navigation'],capture_output=True,text=True)
                if inspected.returncode==0:
                    value=json.loads(inspected.stdout)[0]
                    if (value['Image']==config['image'] and
                        any(m['Destination']=='/work' and Path(m['Source']).resolve()==root for m in value.get('Mounts',[]))):
                        identity=value['Id']
                    else: cleanup_errors.append('Unrecognized startup container; inspect without replacing it')
            if identity:
                try: subprocess.run(['docker','stop','--time','15',identity],capture_output=True,timeout=25)
                except Exception as error: cleanup_errors.append(str(error))
            if child.poll() is None:
                try: child.terminate(); child.wait(timeout=15)
                except Exception as error: cleanup_errors.append(str(error))
            if cleanup_errors and failure is None: failure='Cleanup requires review'
    save(record/'result.json',dict(utc=trial.utc(),failure=failure,operator_stop=stopped,renderer_provenance_ready=proven,
         bounded_preview=True,cleanup_errors=cleanup_errors,
         limitation='Review live camera, localization and runtime provenance; no automatic acceptance'))
    print(json.dumps(dict(evidence=str(record),failure=failure)))
    return int(failure is not None)


if __name__=='__main__': raise SystemExit(main())
