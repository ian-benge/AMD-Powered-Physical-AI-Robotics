"""Immutable v2 capture attempts and content-verified resume/commit."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import time

from .common import write_json

QUALIFIED_PLUGIN=Path('/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre.so.8.2.3')
QUALIFIED_PLUGIN_SHA256='f6e8842247198a2b34b25cfdb40a8f4bc4f156c2e90a54135a68698dc239a3e6'


class FatalRendererError(RuntimeError):
    """A deterministic renderer failure must not be retried unchanged."""


def renderer_failure(log):
    text = Path(log).read_text(errors='replace')
    for line in text.splitlines():
        if ('camera not supported by:' in line.lower() or
                'segmentation fault' in line.lower() or
                re.search(r'exit code\s*[:=]?\s*(?:139|-11)\b', line, re.I)):
            return line.strip()
    return None


def require_desktop_renderer(env):
    if not env.get('DISPLAY') or env.get('LIBGL_ALWAYS_SOFTWARE','0').lower() not in ('0','false'):
        raise RuntimeError('Qualified Ogre capture requires the attended desktop and hardware rendering')
    if not QUALIFIED_PLUGIN.is_file() or file_hash(QUALIFIED_PLUGIN)!=QUALIFIED_PLUGIN_SHA256:
        raise RuntimeError('Qualified opaque-material Ogre plugin is missing or changed')


def file_hash(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def checked_path(root, relative):
    p=Path(relative)
    if p.is_absolute() or '..' in p.parts or not (root/p).resolve().is_relative_to(root.resolve()):
        raise ValueError('Capture evidence path escapes its output')
    return root/p


def evidence_hashes(root, report, entry):
    if report.get('scene_sha256')!=entry['content_sha256'] or report.get('frames')!=entry['views']:
        raise ValueError('Capture report scene/frame identity mismatch')
    records=report.get('records',[])
    if len(records)!=entry['views'] or len(set(records))!=len(records):
        raise ValueError('Capture report record identities are incomplete')
    hashes={}
    for name in records:
        meta=checked_path(root,name);r=json.loads(meta.read_text())
        if r['scene_sha256']!=entry['content_sha256']:
            raise ValueError('Capture metadata scene changed')
        for relative in (name,r['image'],r['label']):
            hashes[relative]=file_hash(checked_path(root,relative))
        for folder in ('previews','depth'):
            for path in (root/folder).glob('*/'+Path(name).stem+'.*'):
                hashes[path.relative_to(root).as_posix()]=file_hash(path)
    return hashes


def verify_completed(root, report, entry):
    actual=evidence_hashes(root,report,entry)
    if not report.get('evidence_sha256') or actual!=report['evidence_sha256']:
        raise ValueError('Completed capture evidence hash mismatch')
    return actual


def commit_attempt(attempt, output, report, entry):
    hashes=verify_completed(attempt,report,entry)
    for name,sha in hashes.items():
        target=checked_path(output,name);target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():
            if file_hash(target)!=sha:raise ValueError('Existing canonical capture differs: '+name)
        else:
            with target.open('xb') as dest,checked_path(attempt,name).open('rb') as src:
                shutil.copyfileobj(src,dest)
    write_json(output/'reports'/(entry['id']+'.json'),report)


def capture_scene(bundle, output, entry, scene, resume, process_context):
    canonical=output/'reports'/(entry['id']+'.json')
    if canonical.exists():
        if not resume:raise FileExistsError('Scene already captured; use verified resume or a new output')
        verify_completed(output,json.loads(canonical.read_text()),entry)
        return False
    attempts=output/'attempts'/entry['id'];attempts.mkdir(parents=True,exist_ok=True)
    if resume:
        for prior in sorted(attempts.glob('attempt-*')):
            f=prior/'reports'/(entry['id']+'.json')
            if f.exists():
                report=json.loads(f.read_text())
                if report.get('evidence_sha256'):
                    commit_attempt(prior,output,report,entry)
                    return True
    elif any(attempts.iterdir()):
        raise FileExistsError('Preserved attempts exist; use resume or another output')
    start=time.monotonic()
    number=max([int(p.name.split('-')[-1]) for p in attempts.glob('attempt-*')]+[0])
    for offset in range(1,4):
        attempt=attempts/f'attempt-{number+offset:04d}';attempt.mkdir()
        for name in ('plan.json','splits.json','classes.txt','data.yaml'):
            if (output/name).exists():shutil.copy2(output/name,attempt/name)
        cmd=['ros2','launch','mars_training_environment','mars_world.launch.py',
             'scene:='+str(bundle/'scenes'/entry['id']/'scene.json'),'output:='+str(attempt),'capture:=true','gui:=false']
        code=None;error=None
        try:
            with (attempt/'launch.log').open('xb') as log,process_context(cmd,log,qualified_desktop=True) as process:
                code=process.wait(timeout=scene['config']['capture']['timeout_wall_sec']*(entry['views']+2))
            fatal = renderer_failure(attempt/'launch.log')
            if fatal:
                raise FatalRendererError('Renderer failure: '+fatal)
            report_path=attempt/'reports'/(entry['id']+'.json')
            if code!=0:raise RuntimeError('Scene launch exited '+str(code))
            if not report_path.is_file():
                raise RuntimeError('Scene launch produced no completed capture report; see '+str(attempt/'launch.log'))
            report=json.loads(report_path.read_text())
            report['evidence_sha256']=evidence_hashes(attempt,report,entry)
            report['launch_wall_seconds']=time.monotonic()-start
            report['attempt']=attempt.relative_to(output).as_posix()
            write_json(report_path,report)
            commit_attempt(attempt,output,report,entry)
            write_json(attempt/'attempt-result.json',dict(result='COMMITTED',exit_code=code))
            return True
        except Exception as exc:
            error=str(exc)
            write_json(attempt/'attempt-result.json',dict(result='FAILED_PRESERVED',error=error,exit_code=code))
            if isinstance(exc, FatalRendererError) or renderer_failure(attempt/'launch.log'):
                raise RuntimeError('Fatal renderer failure; all evidence retained: '+str(attempt)+'; '+error) from exc
    raise RuntimeError('Three capture attempts failed; all evidence retained: '+str(attempts)+'; '+error)
