"""Bounded attended browser assessment of a separately verified NPU candidate."""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from urllib.request import Request,urlopen
import uuid

from capture_training_dataset import NAMES,sha,save
import start_capture5000_training as training

ROOT=Path(__file__).resolve().parent
PORT=8778


def verify_status(status,release):
    model=status.get('model',{})
    expected=release['candidate']
    required=dict(backend='npu',precision='a16w8',output_format='yolo_dfl',model_family='YOLOv8',
        source_checkpoint=expected['checkpoint'],source_checkpoint_sha256=expected['checkpoint_sha256'],
        model_sha256=expected['model_sha256'],manifest=expected['manifest'],
        class_names=list(NAMES),input_size=[576,960],proof_state='npu_executed',
        computation_offload_verified=True,cache_key=expected['cache_key'])
    if status.get('ready') is not True or status.get('error') or any(model.get(k)!=v for k,v in required.items()):
        raise RuntimeError('Candidate NPU service is not the verified selected model')
    evidence=model.get('compile_evidence',{})
    if evidence.get('verified') is not True or evidence.get('sha256')!=expected['compile_report_sha256']:
        raise RuntimeError('Candidate NPU compilation proof changed')
    if model.get('runtime_fingerprint')!=expected['runtime_fingerprint']:
        raise RuntimeError('Candidate NPU runtime identity changed')
    return status


def candidate_settings(original,config,selected):
    settings=original(config,selected)
    settings['detector_url']=f'http://127.0.0.1:{PORT}'
    # The browser already supplies the map and camera. RViz was rendering the
    # same large maps with llvmpipe; retain the qualified Gazebo/Ogre preview.
    settings['rviz']='false'
    settings['camera_rate_hz']=config.get('model_assessment',{}).get('camera_rate_hz',30)
    return settings


def check_terrain_result(case):
    """A finished terrain attempt is a stop; report its cause before PID lookup."""
    path=Path(case)/'environment-v2-terrain-final.json'
    if path.is_symlink():raise RuntimeError('Terrain stop evidence must be a regular file')
    if path.exists():
        result=json.loads(path.read_text())
        reason=result.get('failure') or ('operator stop' if result.get('operator_stop') else
            'worker exit '+str(result.get('worker_exit')))
        raise RuntimeError('Terrain service stopped: '+reason+'; preserved evidence: '+str(path))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--check',action='store_true');p.add_argument('--service-check',action='store_true');p.add_argument('--kit',type=Path)
    p.add_argument('--case',type=Path);p.add_argument('--seconds',type=int,default=1800)
    p.add_argument('--representative',type=int,choices=(1,2,3),default=1)
    p.add_argument('--attended',action='store_true')
    args=p.parse_args()
    release=json.loads((ROOT/'ASSESSMENT_RELEASE.json').read_text())
    def verify_pins():
        for name,expected in release['files'].items():
            if sha(Path(name))!=expected:raise RuntimeError('Assessment input changed: '+name)
    verify_pins();training.verify_release()
    if args.check:print('CAPTURE5000_ASSESSMENT_PINS_PASS');return 0
    if (not args.service_check and not args.attended) or args.kit is None or args.case is None or not 0<args.seconds<=3600:
        p.error('Use the attended checked starter with a current kit/case and bounded duration')
    if not args.service_check and (not os.environ.get('DISPLAY') or os.environ.get('XDG_SESSION_TYPE') not in ('x11','wayland')):
        raise RuntimeError('Launch from the connected Halo graphical desktop')
    app=Path(release['application']);sys.path.insert(0,str(app/'management'))
    import phase2_supervise_live as supervisor
    sys.path.insert(0,str(training.CONTROL))
    from bulk_guarded_step import retry_session_scan
    trial=supervisor.load_guard(args.kit)
    if not args.service_check:check_terrain_result(args.case)
    retry_session_scan(lambda:trial.guard(args.case))
    retry_session_scan(lambda:supervisor.live_guard(trial,args.case))
    trial.check_qualification(args.case);trial.npu_status()
    if not args.service_check:trial.terrain_status()
    if subprocess.check_output(['ss','-ltnH','sport','=',':'+str(PORT)],text=True).strip():
        raise RuntimeError('Candidate NPU port is occupied; identify it before another launch')
    folder=Path(tempfile.mkdtemp(prefix='assessment-session-',dir=ROOT))
    candidate=release['candidate'];token=str(uuid.uuid4())
    env=dict(os.environ,ROVER_TRAINING_TOKEN=token,PYTHONUNBUFFERED='1',PYTHONDONTWRITEBYTECODE='1',
        CUDA_VISIBLE_DEVICES='',HIP_VISIBLE_DEVICES='')
    exports=dict(PYTHONPATH=str(args.kit/'overlay')+':/home/ian/capstone-training/rover-navigation/src/rover_training',
        PYTHONNOUSERSITE='1',ROVER_NPU_CACHE_ROOT=candidate['cache_root'],ROVER_NPU_CACHE_MODE='reuse')
    command=['/home/ian/capstone-training/venvs/rover-npu/bin/python','-B','-m',
        'rover_perception.detector_service','--model',candidate['checkpoint'],
        '--backend','npu','--model-manifest',candidate['manifest'],'--port',str(PORT)]
    shell='source /home/ian/capstone-training/mars-pilot50-v2/scripts/halo_npu_env.sh && export '+ \
        ' '.join(key+'='+shlex.quote(value) for key,value in exports.items())+' && exec '+shlex.join(command)
    child=None;failure=None;result=1
    original_live_guard=supervisor.live_guard
    original_settings=supervisor.scene_settings
    original_load_guard=supervisor.load_guard
    def load_guard(kit):
        retained=original_load_guard(kit)
        native_guard=retained.guard
        def guarded(case):
            check_terrain_result(case)
            return retry_session_scan(lambda:native_guard(case))
        retained.guard=guarded
        return retained
    def candidate_status():
        if child is None or child.poll() is not None:raise RuntimeError('Candidate NPU process stopped')
        with urlopen(f'http://127.0.0.1:{PORT}/status',timeout=3) as response:status=json.load(response)
        return verify_status(status,release)
    def live_guard(t,case):
        if not args.service_check:check_terrain_result(case)
        health=retry_session_scan(lambda:original_live_guard(t,case))
        candidate_status()
        return health
    with (folder/'candidate-npu.log').open('xb') as log:
        try:
            child=subprocess.Popen(['bash','-c',shell],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+120
            while True:
                retry_session_scan(lambda:original_live_guard(trial,args.case))
                if child.poll() is not None:raise RuntimeError('Candidate NPU startup failed; inspect '+str(folder/'candidate-npu.log'))
                try:status=candidate_status();break
                except OSError:
                    if time.monotonic()>=deadline:raise RuntimeError('Candidate NPU readiness timed out')
                    time.sleep(.5)
            save(folder/'candidate-ready.json',status)
            print('CANDIDATE_EPOCH22_NPU_READY: verified selected model on port 8778; baseline retained on 8775',flush=True)
            if args.service_check:
                request=Request(f'http://127.0.0.1:{PORT}/predict',data=bytes(960*540*3),method='POST',
                    headers={'Content-Type':'application/octet-stream','X-Image-Width':'960','X-Image-Height':'540',
                        'X-Confidence':'.25','X-Model-Sha256':candidate['model_sha256']})
                with urlopen(request,timeout=10) as response:prediction=json.load(response)
                after=candidate_status()
                if (prediction.get('backend')!='npu' or prediction.get('model_sha256')!=candidate['model_sha256'] or
                    after['processed_frames']<=status['processed_frames']):
                    raise RuntimeError('Candidate HTTP inference identity/progress failed')
                save(folder/'HTTP_INFERENCE_CHECK.json',dict(prediction=prediction,after=after,
                    input='960x540 zero-valued diagnostic frame; not a training capture or accuracy test'))
                print('CANDIDATE_HTTP_INFERENCE_CHECK_PASS',flush=True);result=0
            else:
                supervisor.live_guard=live_guard
                supervisor.load_guard=load_guard
                supervisor.scene_settings=lambda config,selected:candidate_settings(original_settings,config,selected)
                sys.argv=[str(Path(__file__)),release['configuration'],'--kit',str(args.kit),'--case',str(args.case),
                    '--seconds',str(args.seconds),'--representative',str(args.representative),'--attended']
                result=supervisor.main(phase='candidate22',record_terrain=False)
                if result:failure='Scene supervisor stopped; inspect its preserved result.json for the cause'
        except BaseException as error:
            failure=type(error).__name__+': '+str(error);raise
        finally:
            supervisor.live_guard=original_live_guard;supervisor.scene_settings=original_settings
            supervisor.load_guard=original_load_guard
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGTERM)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:pass
            cleanup=training.cleanup_owned(token,os.getuid())
            if child is not None and child.poll() is None:child.wait(timeout=10)
            save(folder/'RESULT.json',dict(utc=trial.utc(),failure=failure,supervisor_exit=result,
                candidate_process_exit=child.returncode if child else None,cleanup=cleanup,
                case=str(args.case),manual_assessment='OPERATOR_REVIEW_REQUIRED',
                baseline_default_changed=False,model_promoted=False))
    return result


if __name__=='__main__':raise SystemExit(main())
