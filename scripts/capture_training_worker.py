"""One guarded ROCm training window; save resumable epochs before renewal."""
import ctypes
import datetime as dt
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time

from capture_training_dataset import NAMES, sha, save, verify_export

QUALIFIED_SITE = Path('/home/ian/capstone-training/venvs/terrain-rocm7141/lib/python3.12/site-packages')
LIBRARIES = {
    '_rocm_sdk_core/lib/libamdhip64.so.7': 'db0f21d07e81a22311906cd7c45d3854c1d07ddd2e9fa84a1fdc052b4426fca9',
    '_rocm_sdk_core/lib/libhsa-runtime64.so.1': 'bb24bcdee28c4eea895eb453669f08ca123051aa8b16d826cfa04a1e7c807a64',
}


def atomic(path, value):
    path = Path(path); temporary = path.with_name(path.name+'.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')
        stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)


class MonitoringPause(Exception):
    pass


def pause_due(now, deadline, epoch_seconds, finished):
    if finished: return False
    return now+max(120., epoch_seconds*1.5+60.) >= deadline


def finite_losses(losses):
    """Read the installed trainer's named scalar loss contract without dropping names."""
    if not isinstance(losses,dict) or not losses:
        raise RuntimeError('Trainer did not provide named loss values')
    result={name:float(value.detach() if hasattr(value,'detach') else value)
            for name,value in losses.items()}
    if not all(isinstance(name,str) and math.isfinite(value) for name,value in result.items()):
        raise RuntimeError('Non-finite or invalid training loss; no automatic recovery authorized')
    return result


def initialize_gpu():
    for relative, expected in LIBRARIES.items():
        if sha((QUALIFIED_SITE/relative).resolve()) != expected:
            raise RuntimeError('Qualified HIP/HSA library changed')
    hip = ctypes.CDLL(str(QUALIFIED_SITE/'_rocm_sdk_core/lib/libamdhip64.so.7'))
    hip.hipSetDeviceFlags.argtypes = [ctypes.c_uint]; hip.hipSetDeviceFlags.restype = ctypes.c_int
    hip.hipGetDeviceFlags.argtypes = [ctypes.POINTER(ctypes.c_uint)]; hip.hipGetDeviceFlags.restype = ctypes.c_int
    if hip.hipSetDeviceFlags(4) != 0:
        raise RuntimeError('Cannot select HIP blocking synchronization before GPU initialization')
    import torch
    import torchvision
    if torch.__version__ != '2.12.0+rocm7.14.1' or torch.version.hip != '7.14.60850':
        raise RuntimeError('Training Torch/HIP differs from the qualified stack')
    if torchvision.__version__ != '0.27.0+rocm7.14.1' or not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise RuntimeError('Matching ROCm Torchvision and one GPU are required')
    if '8060S' not in torch.cuda.get_device_name(0):
        raise RuntimeError('Unexpected training device')
    torch.set_num_threads(2)
    flags = ctypes.c_uint()
    torch.ones(1, device='cuda').sum().item()
    if hip.hipGetDeviceFlags(ctypes.byref(flags)) != 0 or flags.value & 7 != 4:
        raise RuntimeError('HIP flags 4 did not survive warmup')
    return torch, torchvision, dict(torch=torch.__version__, hip=torch.version.hip,
        torchvision=torchvision.__version__, device=torch.cuda.get_device_name(0),
        requested_hip_flags=4, observed_hip_flags=flags.value)


def check_resume(checkpoint, config, torch):
    path = Path(checkpoint['path'])
    if sha(path) != checkpoint['sha256']:
        raise RuntimeError('Resume checkpoint changed')
    value = torch.load(path, map_location='cpu', weights_only=False)
    epoch = value.get('epoch')
    if (type(epoch) is not int or epoch+1 != checkpoint['completed_epochs'] or
            not value.get('optimizer') or not 0 < epoch+1 < config['recipe']['epochs'] or
            Path(value['train_args']['data']) != Path(config['dataset'])/'data.yaml'):
        raise RuntimeError('Checkpoint lacks matching epoch, optimizer or dataset identity')
    return path


def run(config):
    root, work = Path(config['training_root']), Path(config['work'])
    dataset = Path(config['dataset']); base = Path(config['base_model'])
    if sha(dataset/'DATASET_MANIFEST.json') != config['dataset_manifest_sha256'] or sha(base) != config['base_model_sha256']:
        raise RuntimeError('Dataset or original model changed')
    verify_export(dataset)
    torch, vision, runtime = initialize_gpu()
    save(work/'GPU_RUNTIME.json', runtime)
    from ultralytics import YOLO, settings
    disable = {key:False for key in ['sync','wandb','mlflow','comet','clearml','neptune','dvc','hub','tensorboard'] if key in settings}
    settings.update(disable)
    if config.get('probe'):
        from ultralytics.cfg import get_cfg
        model = YOLO(str(base)).model.to('cuda').train()
        # Deployment checkpoints retain only a small argument subset and freeze
        # gradients. Restore the same defaults/trainability used by the trainer.
        model.args = get_cfg(overrides=model.args)
        for name, parameter in model.named_parameters():
            parameter.requires_grad_('.dfl' not in name)
        model.criterion = None
        if tuple(model.names[i] for i in range(4)) != NAMES:
            raise RuntimeError('Baseline checkpoint does not use the four deployment classes')
        batch = dict(img=torch.rand(4,3,960,960,device='cuda'),
            batch_idx=torch.arange(4,device='cuda',dtype=torch.float32),
            cls=torch.arange(4,device='cuda',dtype=torch.float32).reshape(-1,1),
            bboxes=torch.tensor([[.5,.5,.2,.2]]*4,device='cuda'))
        loss, parts = model(batch)
        if not torch.isfinite(loss).all(): raise RuntimeError('Non-finite probe loss')
        loss.sum().backward()
        if any(p.grad is not None and not torch.isfinite(p.grad).all() for p in model.parameters()):
            raise RuntimeError('Non-finite probe gradient')
        torch.optim.SGD(model.parameters(),lr=.0001).step()
        indices = vision.ops.nms(torch.tensor([[0.,0.,10.,10.],[0.,0.,10.,10.]],device='cuda'),
                                 torch.tensor([.9,.8],device='cuda'), .5)
        if indices.device.type != 'cuda' or indices.tolist() != [0]:
            raise RuntimeError('ROCm Torchvision NMS probe failed')
        torch.cuda.synchronize()
        save(work/'WORKER_RESULT.json',dict(stage='GPU_TRAINING_PROBE_PASS',loss=float(loss.sum()),runtime=runtime,
            original_model_unchanged=sha(base)==config['base_model_sha256']))
        return
    resume = config.get('resume')
    model = YOLO(str(check_resume(resume, config, torch) if resume else base))
    completed = resume['completed_epochs'] if resume else 0
    epoch_start = time.monotonic()
    deadline = config['deadline_monotonic']
    newest = None
    batches = 0

    def epoch_begin(trainer):
        nonlocal epoch_start, batches
        epoch_start = time.monotonic()
        batches = 0
        if trainer.device.type != 'cuda' or trainer.batch_size != 4 or trainer.amp:
            raise RuntimeError('Training device, batch or FP32 recipe changed')
        if any(p.device.type!='cuda' or p.dtype!=torch.float32 for p in trainer.model.parameters()):
            raise RuntimeError('Every learned parameter must train on the GPU in FP32')
        if tuple(trainer.model.names[i] for i in range(4))!=NAMES:
            raise RuntimeError('Training class order differs from the deployment schema')

    def batch_end(trainer):
        nonlocal batches
        batches += 1
        losses=finite_losses(trainer.tloss)
        if time.monotonic() >= deadline:
            raise RuntimeError('An epoch exceeded its monitored budget; last completed checkpoint preserved')
        if batches == 1 or batches % 25 == 0 or batches == len(trainer.train_loader):
            atomic(root/'BATCH_PROGRESS.json',dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                epoch=trainer.epoch+1,planned_epochs=config['recipe']['epochs'],completed_batches=batches,
                batches_in_epoch=len(trainer.train_loader),loss=losses,
                dataset_manifest_sha256=config['dataset_manifest_sha256']))

    def epoch_end(trainer):
        finite_losses(trainer.tloss)

    def checkpoint_saved(trainer):
        nonlocal completed, newest
        epoch = trainer.epoch+1
        if not epoch > completed:
            raise RuntimeError('Training epoch did not advance')
        target = root/'checkpoints'/f'epoch-{epoch:03d}.pt'
        target.parent.mkdir(exist_ok=True)
        # Preserve the optimizer-bearing file before Ultralytics final evaluation
        # strips optimizer state from last.pt and best.pt.
        with Path(trainer.last).open('rb') as src, target.open('xb') as dest:
            shutil.copyfileobj(src,dest); dest.flush(); os.fsync(dest.fileno())
        newest = dict(path=str(target),sha256=sha(target),completed_epochs=epoch)
        check_resume(newest,config,torch) if epoch < config['recipe']['epochs'] else None
        metrics = {k:float(v) for k,v in trainer.metrics.items()}
        if not all(math.isfinite(v) for v in metrics.values()):
            raise RuntimeError('Non-finite validation metrics')
        completed = epoch
        atomic(root/'TRAINING_PROGRESS.json',dict(stage='EPOCH_COMPLETE',utc=dt.datetime.now(dt.timezone.utc).isoformat(),
            completed_epochs=epoch,planned_epochs=config['recipe']['epochs'],checkpoint=newest,
            validation_metrics=metrics,epoch_seconds=time.monotonic()-epoch_start,
            dataset_manifest_sha256=config['dataset_manifest_sha256'],deployment_approved=False))
        print('EPOCH_COMPLETE',epoch,json.dumps(metrics),flush=True)

    def fit_end(trainer):
        if pause_due(time.monotonic(),deadline,time.monotonic()-epoch_start,bool(trainer.stop)):
            if newest is None: raise RuntimeError('No resumable checkpoint was saved')
            raise MonitoringPause()

    model.add_callback('on_train_epoch_start',epoch_begin)
    model.add_callback('on_train_batch_end',batch_end)
    model.add_callback('on_train_epoch_end',epoch_end)
    model.add_callback('on_model_save',checkpoint_saved)
    model.add_callback('on_fit_epoch_end',fit_end)
    try:
        if resume:
            model.train(resume=True)
        else:
            model.train(data=str(dataset/'data.yaml'),project=str(root/'runs'),name='capture5000_yolov8s_960',
                        **config['recipe'])
    except MonitoringPause:
        # A callback exception bypasses Ultralytics' normal loader shutdown.
        # Join these owned workers before publishing a successful pause record.
        for loader in (model.trainer.train_loader,model.trainer.test_loader):
            if hasattr(loader,'close'):loader.close()
        save(work/'WORKER_RESULT.json',dict(stage='PAUSED_FOR_MONITORING',checkpoint=newest,completed_epochs=completed))
        return
    best = root/'runs/capture5000_yolov8s_960/weights/best.pt'
    if not best.is_file() or completed < 1:
        raise RuntimeError('Training completed without a saved model')
    save(work/'WORKER_RESULT.json',dict(stage='TRAINING_COMPLETE_PENDING_EVALUATION',completed_epochs=completed,
        best_model=str(best),best_model_sha256=sha(best),deployment_approved=False))


if __name__ == '__main__':
    config = json.loads(Path(sys.argv[1]).read_text())
    run(config)
