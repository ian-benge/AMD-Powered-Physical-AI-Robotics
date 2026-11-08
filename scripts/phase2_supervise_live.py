#!/usr/bin/env python3
"""Attended, bounded Phase 2 scene control under the retained Halo guards.

Runs in a Halo desktop terminal. Never confirms launch clearance or sends drive
commands. Every replacement container repeats image/library/camera checks.
"""
import argparse
from http.server import ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import threading
import time
from urllib.request import urlopen

from phase1_run_fixture import command, digest, live_guard, load_guard, save
from phase1_supervise_live import verify_configuration


def inspect_container(name):
    # The generic `docker inspect` also resolves image tags. There is a retained
    # image named rover-navigation even while its container does not exist.
    result=subprocess.run(['docker','container','inspect',name],capture_output=True,text=True,timeout=5)
    if result.returncode:return None
    value=json.loads(result.stdout)[0]
    if 'Image' not in value or 'Mounts' not in value:raise RuntimeError('Invalid container inspection')
    return value


def belongs_to_candidate(value,config,root):
    return (value['Image']==config['image'] and any(m['Destination']=='/work' and
            Path(m['Source']).resolve()==root for m in value.get('Mounts',[])))


def scene_settings(config,selected):
    """The same explicit settings feed live launch and CPU launch-graph review."""
    result=dict(scene='/scenes/scenes/'+selected['scene']+'/scene.json',camera=config['camera'],
        container_image=config['image'],install_relative='install',gui='true',rviz='true',
        fixture='',terrain_collision='scene',navigation_speed=.05,detector_model=config['checkpoint'],
        manual_max_linear_speed=.05,manual_max_angular_speed=.3,
        detector_backend='remote_http',detector_url='http://127.0.0.1:8775',terrain_backend='rocm_http',
        terrain_url='http://127.0.0.1:8782',adaptive_terrain='true',sensor_headless='false',sensor_render_engine='ogre')
    if config.get('environment_version')==2:
        result.update(map_size_m=40.,manual_max_linear_speed=.20,manual_initial_linear_speed=.05)
    return result


def validate_scene(config, bundle, scene, expected_sha=None):
    path=Path(bundle)/'scenes'/scene/'scene.json'
    if expected_sha and digest(path)!=expected_sha:raise ValueError('Qualified scene changed')
    value=json.loads(path.read_text())
    terrain=value['terrain'];routes=value.get('reachability',{})
    version=2 if config.get('environment_version')==2 else 1
    required=({'ordinary_rock','pale_soil','crystal_rock'} if version==1 else
              {name+':'+band for name in ('rock','pale_soil_patch','yellow_crystal_rock','mudstone_slab') for band in ('near','outer')})
    if (terrain.get('profile')!='mixed_reachable' or terrain.get('profile_version')!=version or
            not terrain.get('shared_surface') or value['config']['capture']['views_per_scene']!=0 or
            routes.get('truth_to_navigation') is not False or not routes.get('evaluator_only') or
            {r['category'] for r in routes.get('routes',[])}!=required):
        raise ValueError('Scene does not satisfy the Phase 2 profile/capture/truth contract')
    from mars_training_environment.serialization import digest as content_digest
    copy=dict(value);expected=copy.pop('content_sha256')
    if content_digest(copy)!=expected:raise ValueError('Scene content hash failed')
    return value


def main(phase=2, record_terrain=False):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('configuration',type=Path)
    parser.add_argument('--kit',type=Path,required=True);parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--representative',type=int,choices=(1,2,3),default=1)
    parser.add_argument('--seconds',type=float,default=1800)
    parser.add_argument('--attended',action='store_true',required=True)
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    if not 0<args.seconds<=3600:parser.error('Duration must be in (0,3600] seconds')
    config,root=verify_configuration(args.configuration)
    sys.path[:0]=[str(root/'source/src/rover_navigation'),str(root/'source/src/mars_training_environment'),
                  str(root/'source/src/rover_training')]
    from rover_navigation.scene_control import SceneControl,handler,gateway_handler
    selected=config['qualified_scenes'][args.representative-1]
    validate_scene(config,selected['bundle'],selected['scene'],selected['scene_sha256'])
    trial=load_guard(args.kit)
    trial.guard(args.case);live_guard(trial,args.case);trial.check_qualification(args.case)
    trial.npu_status();trial.terrain_status()
    for name in ('npu','terrain'):trial.service_identity(args.case,name)
    if command(['docker','ps','-q']):raise RuntimeError('Stop other containers before Phase 2')
    for port in (8774,8776,8777):
        if command(['ss','-ltnH','sport','=',':'+str(port)]):raise RuntimeError('Scene/dashboard port occupied')
    if command(['docker','image','inspect','--format','{{.Id}}',config['image']])!=config['image']:
        raise RuntimeError('Candidate image missing/changed')
    if not trial.policy.cooled(trial.control_health(args.case)):
        raise RuntimeError('Wait for retained 30-second cooldown at Tctl <=70 C')
    if args.verify_only:print(f'PHASE{phase}_CONFIGURATION_AND_RUNTIME_GATES_PASS');return 0
    if not os.environ.get('DISPLAY') or os.environ.get('XDG_SESSION_TYPE') not in ('wayland','x11'):
        raise RuntimeError('Run inside the attended Halo graphical desktop')
    stamp=time.strftime('%Y%m%dT%H%M%SZ',time.gmtime());bootstrap=f'phase{phase}-control-'+stamp
    record=root/'supervisor-sessions'/bootstrap;record.mkdir(parents=True)
    session=root/'navigation_sessions'/bootstrap;session.mkdir(parents=True)
    settings=scene_settings(config,selected)
    save(session/'session.json',settings)
    save(record/'configuration.json',dict(candidate=config,case=str(args.case),kit=str(args.kit),
         seconds=args.seconds,representative=args.representative))
    halted=threading.Event();transition_lock=threading.Lock();failure=None;seen={};cleanup=[];recorders={}
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:halted.set())
    os.environ.update(NAV_IMAGE=config['image'],NAV_INSTALL_REL='install',NAV_CONTAINER_NAME='rover-navigation',
                      NAV_SOFTWARE_RENDERING='0',NAV_GZ_PARTITION=f'rover-phase{phase}-live',NAV_ROS_DOMAIN_ID='87')

    class GuardedScenes(SceneControl):
        def start_process(self,command,log):
            with transition_lock:
                if halted.is_set():raise RuntimeError('Supervisor stopped before process launch')
                live_guard(trial,args.case)
                self.process=super().start_process(command,log)
                return self.process

        def launch_command(self,session,bundle=None,scene=None,generated=False):
            if halted.is_set():raise RuntimeError('Supervisor has stopped')
            verify_configuration(args.configuration)
            live_guard(trial,args.case)
            value=validate_scene(config,bundle or self.bundle,scene or self.scene)
            save(record/(session+'-scene.json'),dict(bundle=str(bundle or self.bundle),scene=value['id'],
                 content_sha256=value['content_sha256'],geometry_sha256=value['geometry_sha256']))
            return super().launch_command(session,bundle,scene,generated)+['rviz_software_rendering:=1']

        def restart(self,new_scene=False):
            if halted.is_set():return dict(success=False,message='Phase 2 supervisor stopped',**self.status())
            try:live_guard(trial,args.case)
            except Exception as error:
                halted.set();return dict(success=False,message=str(error),**self.status())
            return super().restart(new_scene)

    manager=GuardedScenes(root,selected['bundle'],bootstrap)
    servers=[ThreadingHTTPServer(('127.0.0.1',8776),handler(manager)),
             ThreadingHTTPServer(('127.0.0.1',8774),gateway_handler(manager))]
    for server in servers:
        server.daemon_threads=True
        threading.Thread(target=server.serve_forever,daemon=True).start()
    manager.restart()
    deadline=time.monotonic()+args.seconds
    try:
        with (record/'health.jsonl').open('x') as health:
            while not halted.is_set() and time.monotonic()<deadline:
                health.write(json.dumps(live_guard(trial,args.case))+'\n');health.flush();os.fsync(health.fileno())
                state=manager.status()
                if state['phase']=='failed':raise RuntimeError('Scene control failed: '+state['error'])
                container=inspect_container('rover-navigation')
                if container is not None:
                    identity=container['Id']
                    if not belongs_to_candidate(container,config,root):
                        raise RuntimeError('Unexpected container image/checkout')
                    if identity not in seen:
                        launched=record/(state['session']+'-scene.json')
                        if state['phase'] not in ('starting','stopping','ready') or not launched.is_file():
                            raise RuntimeError('Container replaced outside a supervised scene change')
                        seen[identity]=False;save(record/(identity[:12]+'-container.json'),container)
                    if state['phase']=='ready' and not seen[identity]:
                        status=json.load(urlopen('http://127.0.0.1:8777/status',timeout=2))
                        camera=status.get('camera_health',{})
                        if camera.get('available') and camera.get('age_seconds',99)<1:
                            if camera.get('low_detail'):raise RuntimeError('Uniform/low-detail camera')
                            proof=subprocess.run(['docker','exec',identity,'python3',
                                '/work/management/phase1_renderer_provenance.py'],capture_output=True,text=True,timeout=20)
                            (record/(identity[:12]+'-renderer.json')).write_text(proof.stdout)
                            if proof.returncode:raise RuntimeError('Renderer provenance failed: '+proof.stderr)
                            value=json.loads(proof.stdout)
                            plugins={lib['sha256'] for proc in value['processes'] for lib in proc['libraries']
                                     if 'libgz-rendering8-ogre.so' in lib['path']}
                            if plugins!={config['ogre_plugin_sha256']}:raise RuntimeError('Mapped renderer changed')
                            seen[identity]=True
                            print('RENDERER_PROVENANCE_READY: '+state['session']+'; inspect clearance before manual control',flush=True)
                            if record_terrain:
                                output=record/(identity[:12]+'-terrain-audit.json')
                                log=(record/(identity[:12]+'-terrain-audit.log')).open('xb')
                                args_capture=['python3','/work/source/scripts/halo_navigation_terrain_audit.py',
                                    '--session','/work/navigation_sessions/'+state['session'],
                                    '--output','/work/'+str(output.relative_to(root)),
                                    '--wait','40','--duration',('300' if config.get('environment_version')==2 else '120'),
                                    '--interval',('2.5' if config.get('environment_version')==2 else '1')]
                                shell='source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && '+shlex.join(args_capture)
                                process=subprocess.Popen(['docker','exec',identity,'bash','-c',shell],
                                    stdout=log,stderr=subprocess.STDOUT)
                                recorders[identity]=dict(process=process,log=log,output=output,complete=False)
                elif state['phase']=='ready':raise RuntimeError('Ready scene container disappeared')
                for recorder in recorders.values():
                    if recorder['process'].poll() is not None and not recorder['complete']:
                        recorder['log'].flush()
                        if (recorder['process'].returncode or not recorder['output'].is_file() or
                                not json.loads(recorder['output'].read_text()).get('passed')):
                            raise RuntimeError('Read-only synchronized terrain recording failed; inspect its log')
                        recorder['complete']=True
                        prefix='ENVIRONMENT_V2' if config.get('environment_version')==2 else 'PHASE3'
                        print(prefix+'_SYNC_CAPTURE_COMPLETE: '+str(recorder['output']),flush=True)
                time.sleep(.5)
    except Exception as error:failure=str(error)
    finally:
        halted.set()
        with transition_lock:
            if manager.process and manager.process.poll() is None:
                try:os.killpg(manager.process.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                except Exception as error:cleanup.append(str(error))
        try:
            # A generator is CPU-only but must stop on the same thermal gate.
            generators=command(['docker','ps','-q','--filter','label=rover.scene-generation=true']).split()
            for identity in generators:
                value=inspect_container(identity)
                if value and any(m['Destination']=='/work' and Path(m['Source']).resolve()==root for m in value.get('Mounts',[])):
                    subprocess.run(['docker','stop','--time','5',identity],capture_output=True,timeout=15)
            # Cover a Docker create request already in flight when the launch
            # process group was stopped. No later launch can pass the lock gate.
            until=time.monotonic()+5
            while time.monotonic()<until:
                value=inspect_container('rover-navigation')
                if value:
                    if not belongs_to_candidate(value,config,root):
                        cleanup.append('Unrecognized container during cleanup; preserved for review');break
                    seen.setdefault(value['Id'],False)
                    stopped=subprocess.run(['docker','stop','--time','15',value['Id']],capture_output=True,text=True,timeout=25)
                    if stopped.returncode and 'No such container' not in stopped.stderr:cleanup.append(stopped.stderr)
                time.sleep(.2)
            if manager.process:
                try:manager.process.wait(timeout=15)
                except subprocess.TimeoutExpired:cleanup.append('Navigation launch did not exit')
        except Exception as error:cleanup.append(str(error))
        for recorder in recorders.values():
            try:recorder['process'].wait(timeout=10)
            except subprocess.TimeoutExpired:
                recorder['process'].terminate()
                cleanup.append('Read-only recorder did not exit after container stop')
            recorder['log'].close()
        for server in servers:server.shutdown();server.server_close()
        save(record/'result.json',dict(utc=trial.utc(),failure=failure,cleanup_errors=cleanup,
             read_only_recordings={identity:dict(output=str(r['output']),complete=r['complete'],
                 exit_code=r['process'].poll()) for identity,r in recorders.items()},
             containers=seen,manual_traversal_verdict='OPERATOR_AND_PROTECTED_REVIEW_REQUIRED',
             limitation='No automatic clearance, motion, traversal acceptance or endurance claim'))
    print(json.dumps(dict(evidence=str(record),failure=failure)))
    return int(failure is not None or bool(cleanup))


if __name__=='__main__':raise SystemExit(main())
