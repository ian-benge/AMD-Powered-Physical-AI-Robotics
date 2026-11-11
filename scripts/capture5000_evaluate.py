"""Freeze the validation-selected checkpoint and compare it with the deployed source model."""
import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from capture_training_dataset import NAMES,copy_checked,sha,save,verify_export
from capture_training_worker import initialize_gpu
import start_capture5000_training as training

ROOT=Path(__file__).resolve().parent


def summarize(metrics):
    indices=[int(i) for i in metrics.box.ap_class_index]
    classes={}
    for index,class_id in enumerate(indices):
        if class_id not in range(len(NAMES)):raise RuntimeError('Unexpected evaluated class')
        classes[NAMES[class_id]]=dict(precision=float(metrics.box.p[index]),recall=float(metrics.box.r[index]),
            map50=float(metrics.box.ap50[index]),map50_95=float(metrics.box.ap[index]))
    overall={key:float(value) for key,value in metrics.results_dict.items()}
    if set(classes)!=set(NAMES) or not all(math.isfinite(v) for row in [overall,*classes.values()] for v in row.values()):
        raise RuntimeError('Incomplete or non-finite evaluation metrics')
    return dict(overall=overall,classes=classes,speed_ms={key:float(v) for key,v in metrics.speed.items()},
        precision_recall_definition='Ultralytics mean-F1 operating point; AP confidence floor 0.001')


def evaluate(config):
    root=Path(config['output']);release=training.verify_release()
    verify_export(Path(release['dataset']))
    selection=json.loads((root/'MODEL_SELECTION.json').read_text())
    torch,_,runtime=initialize_gpu();save(root/'GPU_RUNTIME.json',runtime)
    from ultralytics import YOLO,settings
    settings.update({key:False for key in ['sync','wandb','mlflow','comet','clearml','neptune','dvc','hub','tensorboard'] if key in settings})
    checkpoint=torch.load(selection['candidate'],map_location='cpu',weights_only=False)
    save(root/'CHECKPOINT_METADATA.json',dict(epoch=checkpoint.get('epoch'),
        completed_epochs=int(checkpoint['epoch'])+1,training_metrics=checkpoint.get('train_metrics'),
        selection='best.pt selected on validation before any test evaluation',
        interrupted_training=True,training_completed_epochs=28))
    reports={}
    for label,path,expected in [('baseline',release['base_model'],release['base_model_sha256']),
                                ('candidate',selection['candidate'],selection['candidate_sha256'])]:
        if sha(Path(path))!=expected:raise RuntimeError('Selected checkpoint changed')
        reports[label]={}
        for split in ['val','test']:
            model=YOLO(path)
            if tuple(model.names[i] for i in range(4))!=NAMES:raise RuntimeError('Class order changed')
            def capture_matrix(validator):
                save(root/(label+'-'+split+'-confusion.json'),dict(
                    class_names=list(NAMES)+['background'],matrix=validator.confusion_matrix.matrix.tolist(),
                    matrix_orientation='rows=predicted, columns=actual'))
            model.add_callback('on_val_end',capture_matrix)
            metrics=model.val(data=str(Path(release['dataset'])/'data.yaml'),split=split,
                imgsz=960,batch=4,device=0,half=False,rect=False,workers=2,
                conf=.001,iou=.7,max_det=300,plots=True,save_json=False,
                project=str(root/'plots'),name=label+'-'+split,verbose=False)
            report=dict(model=label,checkpoint=path,checkpoint_sha256=expected,split=split,
                frames=100,dataset_manifest_sha256=release['dataset_manifest_sha256'],
                settings=dict(imgsz=960,batch=4,device='qualified ROCm GPU',half=False,rect=False,
                    conf=.001,nms_iou=.7,max_det=300),**summarize(metrics))
            save(root/(label+'-'+split+'.json'),report);reports[label][split]=report
            print('EVALUATION_COMPLETE',label,split,json.dumps(report['overall']),flush=True)
            if sha(Path(path))!=expected:raise RuntimeError('Evaluation changed source weights')
            del model
    save(root/'EVALUATION_COMPLETE.json',dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(),
        reports={label:{split:str(root/(label+'-'+split+'.json')) for split in values} for label,values in reports.items()},
        candidate_sha256=selection['candidate_sha256'],deployment_approved=False,
        limitation='100 legacy val and 100 legacy test images; no new-environment blind-test claim'))
    lines=['# Frozen detector evaluation','',
        f"Selected checkpoint: epoch {int(checkpoint['epoch'])+1}; candidate SHA-256 `{selection['candidate_sha256']}`.",'',
        '| Model | Split | Precision | Recall | mAP50 | mAP50-95 |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for label in reports:
        for split,report in reports[label].items():
            m=report['overall']
            values=[m[key] for key in ['metrics/precision(B)','metrics/recall(B)','metrics/mAP50(B)','metrics/mAP50-95(B)']]
            lines.append('| '+label+' | '+split+' | '+' | '.join(f'{v*100:.2f}%' for v in values)+' |')
    lines+=['','Precision/recall use the mean-F1 operating point. Both models used identical 960-square FP32 evaluation settings.',
        'Each split contains 100 legacy images. These results do not establish unseen-environment or physical-camera performance.',
        'Per-class JSON, confusion matrices and plots are retained beside this report. NPU quantization accuracy, runtime proof and browser behavior require separate checks.','']
    (root/'REPORT.md').write_text('\n'.join(lines))


def run():
    import fcntl,pwd
    source=json.loads((ROOT/'EVALUATION_SOURCE_V1.json').read_text())
    for name,expected in source['deployed_files'].items():
        if sha(ROOT/name)!=expected:raise RuntimeError('Evaluation source changed: '+name)
    release=training.verify_release();native=training.load_native()
    sys.path.insert(0,str(training.CONTROL))
    import start_bulk10000_collection as starter
    view=native.NativeEngine.__new__(native.NativeEngine);view.account=pwd.getpwnam('ian')
    environment=view.desktop_environment()
    kit,case,trial,live_guard=starter.session()
    output=Path(tempfile.mkdtemp(prefix='evaluation-',dir=ROOT))
    copy_checked(ROOT/'runs/capture5000_yolov8s_960/weights/best.pt',output/'candidate-best.pt',
        sha(ROOT/'runs/capture5000_yolov8s_960/weights/best.pt'))
    selection=dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(),candidate=str(output/'candidate-best.pt'),
        candidate_sha256=sha(output/'candidate-best.pt'),baseline=release['base_model'],
        baseline_sha256=release['base_model_sha256'],dataset_manifest_sha256=release['dataset_manifest_sha256'],
        selection_basis='best validation checkpoint from the stopped 28-epoch run; frozen before test evaluation',
        source_script_sha256=sha(Path(__file__)))
    save(output/'MODEL_SELECTION.json',selection)
    save(output/'WORKER_CONFIGURATION.json',dict(output=str(output)))
    token=str(uuid.uuid4());environment.update(ROVER_TRAINING_TOKEN=token,PYTHONUNBUFFERED='1',
        PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',
        YOLO_OFFLINE='true',YOLO_CONFIG_DIR=str(ROOT/'ultralytics-settings'))
    child=None;failure=None
    def check(identity):
        if sha(Path(__file__))!=selection['source_script_sha256']:raise RuntimeError('Running evaluation source changed')
        native.healthy_state(case,trial.policy.EXPECTED_BOOT)
        training.training_live_check(starter,trial,live_guard,case,identity,output/'guard.jsonl')
        subprocess.run(['/usr/bin/python3',str(training.CONTROL/'desktop_render_guard.py'),'--probe'],
            env=environment,check=True,capture_output=True,timeout=7)
    with (ROOT/'training.lock').open('r') as lock,(output/'evaluation.log').open('xb') as log:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            check(None)
            child=subprocess.Popen([release['training_python'],'-B',str(Path(__file__)),
                '--worker',str(output/'WORKER_CONFIGURATION.json')],env=environment,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            identity=training.process_identity(child.pid)
            if identity['uid']!=os.getuid() or identity['token']!=token:raise RuntimeError('Evaluation owner mismatch')
            deadline=time.monotonic()+1200
            while child.poll() is None:
                check(identity)
                if time.monotonic()>deadline:raise RuntimeError('Evaluation exceeded its monitored budget')
                time.sleep(2)
            if child.returncode or not (output/'EVALUATION_COMPLETE.json').is_file():
                raise RuntimeError('Evaluation failed; preserve '+str(output/'evaluation.log'))
        except BaseException as error:
            failure=type(error).__name__+': '+str(error);raise
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:pass
            clean=training.cleanup_owned(token,os.getuid())
            if child is not None and child.poll() is None:child.wait(timeout=10)
            save(output/'SUPERVISOR_RESULT.json',dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                failure=failure,cleanup=clean,case=str(case),kit=str(kit),
                source_release_sha256=sha(ROOT/'TRAINING_RELEASE.json')))
    (ROOT/'CURRENT_EVALUATION.path').write_text(str(output)+'\n')
    print('EVALUATION_OUTPUT='+str(output),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--run',action='store_true');g.add_argument('--worker',type=Path)
    a=p.parse_args()
    if a.worker:evaluate(json.loads(a.worker.read_text()))
    else:run()
