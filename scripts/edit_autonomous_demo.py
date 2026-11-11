#!/usr/bin/env python3
"""Create a captioned 3–5 minute edit from one accepted Windows original."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def srt_time(value):
    value=round(value*1000);hours,value=divmod(value,3600000);minutes,value=divmod(value,60000);seconds,millis=divmod(value,1000)
    return f'{hours:02}:{minutes:02}:{seconds:02},{millis:03}'


def validate(plan,review,duration):
    if (review.get('passed') is not True or review.get('mission_id')!=plan.get('mission_id') or
            review.get('original_sha256')!=plan.get('original_sha256')):
        raise ValueError('Independent acceptance must identify this mission and continuous original')
    clips=plan['clips'];total=0.;last=-1.;collection=False
    if not 1<=len(clips)<=30:raise ValueError('Use a bounded edit of one continuous original')
    for clip in clips:
        start,end,speed=(clip[k] for k in ('start_seconds','end_seconds','speed'))
        if (not all(type(v) in (float,int) and math.isfinite(v) for v in (start,end,speed)) or
                not 0<=start<end<=duration or start<last or not 1<=speed<=8):
            raise ValueError('Clips must be chronological, within the original and 1–8x playback')
        if not isinstance(clip.get('caption'),str) or not clip['caption'].strip():raise ValueError('Every clip needs a caption')
        if clip.get('simulated_collection'):
            if speed!=1 or end-start<10:raise ValueError('Show at least ten real seconds of simulated collection at 1x')
            collection=True
        last=end;total+=(end-start)/speed
    if not collection or not 180<=total<=300:raise ValueError('Require a simulated collection clip and a 180–300 second edit')
    return total


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('plan','review','ffmpeg','output'):p.add_argument('--'+name,required=True,type=Path)
    a=p.parse_args()
    if os.name!='nt':raise ValueError('Encode the presentation video on Windows')
    plan=json.loads(a.plan.read_text());review=json.loads(a.review.read_text());original=Path(plan['original']).resolve()
    if sha(original)!=plan['original_sha256']:raise ValueError('Original recording changed')
    probe=a.ffmpeg.with_name('ffprobe.exe')
    duration=float(json.loads(subprocess.check_output([str(probe),'-v','error','-show_entries','format=duration','-of','json',str(original)],text=True))['format']['duration'])
    total=validate(plan,review,duration);a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'edit-plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    offset=0.;captions=[];names=[]
    try:
        for i,clip in enumerate(plan['clips'],1):
            start,end,speed=(clip[k] for k in ('start_seconds','end_seconds','speed'));length=(end-start)/speed
            name=f'part-{i:03}.mp4';names.append(name)
            args=[str(a.ffmpeg.resolve()),'-nostdin','-v','warning','-n','-ss',str(start),'-t',str(end-start),'-i',str(original),
                '-an','-vf',f'setpts=(PTS-STARTPTS)/{speed},scale=-2:1080,fps=30','-c:v','libx264','-crf','18','-preset','medium',name]
            subprocess.run(args,cwd=a.output,check=True)
            label=clip['caption'].replace('\n',' ').strip()+f' | {speed:g}x playback | edited excerpt'
            if clip.get('simulated_collection'):label+=' | Simulated collection: no physical arm or material transfer'
            captions.append(f'{i}\n{srt_time(offset)} --> {srt_time(offset+length)}\n{label}\n')
            offset+=length
        (a.output/'segments.ffconcat').write_text('ffconcat version 1.0\n'+''.join('file '+n+'\n' for n in names))
        (a.output/'captions.srt').write_text('\n'.join(captions),encoding='utf-8')
        subprocess.run([str(a.ffmpeg.resolve()),'-nostdin','-v','warning','-n','-f','concat','-safe','1','-i','segments.ffconcat','-c','copy','edited-source.mp4'],cwd=a.output,check=True)
        subprocess.run([str(a.ffmpeg.resolve()),'-nostdin','-v','warning','-n','-i','edited-source.mp4','-vf','subtitles=captions.srt',
            '-c:v','libx264','-crf','18','-preset','medium','-movflags','+faststart','autonomous-demo-captioned.mp4'],cwd=a.output,check=True)
        result=dict(original=str(original),original_sha256=sha(original),mission_id=plan['mission_id'],duration_seconds=total,
            source_acceptance_review_sha256=sha(a.review),edits_explicitly_labeled=True,
            video_sha256=sha(a.output/'autonomous-demo-captioned.mp4'))
        (a.output/'EDIT_RECEIPT.json').write_text(json.dumps(result,indent=2)+'\n')
    except Exception as error:
        (a.output/'FAILED_PRESERVED.json').write_text(json.dumps(dict(error=str(error),original_unchanged=sha(original)==plan['original_sha256']),indent=2)+'\n')
        raise


if __name__=='__main__':main()
