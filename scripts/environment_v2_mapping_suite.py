#!/usr/bin/env python3
"""Actual ROS 40 m replay at outer coordinates; optional exact ROCm parity.

Uses declared translations of the retained analytic sensor fixtures. No live
simulator, physical command topic, detector or mode selection is involved.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np


CASES = [('flat',12.,12.),('gentle',-14.,-12.),('step',12.,-12.),
         ('rock',-14.,12.),('ridge',12.,12.),('weak-none',-14.,-12.),
         ('weak-missing_depth',12.,-12.),('weak-localization',-14.,12.),
         ('weak-map_correction',12.,12.),('weak-stationary',-14.,-12.),
         ('flat',17.8,0.),('flat',-19.5,0.),('flat',0.,19.4),('flat',0.,-19.4)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('input','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--backend',choices=('cpu','rocm_http'),required=True)
    p.add_argument('--cpu-reference',type=Path)
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='219' or os.environ.get('PHASE3_MAPPING_BENCH')!='1':
        p.error('Isolated bench environment required')
    if a.backend=='rocm_http' and not a.cpu_reference:p.error('ROCm requires saved CPU reference')
    a.output.mkdir(parents=True,exist_ok=False)
    report=dict(result='RUNNING',backend=a.backend,map_size_m=40.,cases=[],motion_requested=False,
                guard_output='/phase3/bench_cmd_vel',controlled_poses=True)
    try:
        for index,(name,x,y) in enumerate(CASES):
            folder=a.output/f'case-{index:02d}';folder.mkdir()
            out=folder/name
            command=[sys.executable,str(Path(__file__).with_name('phase3_ros_replay.py')),
                '--input',str(a.input/name),'--output',str(out),'--backend',a.backend,
                '--map-size','40','--pose-shift',str(x),str(y)]
            with (folder/'log.txt').open('xb') as log:
                result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            record=dict(index=index,case=name,translation_xy=[x,y],exit_code=result.returncode,command=command)
            report['cases'].append(record)
            if result.returncode:raise RuntimeError('ROS case failed; inspect '+str(folder))
            result_record=json.loads((out/'result.json').read_text())
            if result_record['result']!='CONTROLLED_ROS_REPLAY_PASS':raise RuntimeError('Case did not pass')
            record.update(result=result_record['result'],publications=result_record['publications'])
            if a.cpu_reference:
                prior=a.cpu_reference/f'case-{index:02d}'/name
                cpu=json.loads((prior/'result.json').read_text())
                if cpu['result']!='CONTROLLED_ROS_REPLAY_PASS':raise RuntimeError('Unqualified CPU reference')
                files=sorted(p.name for p in out.glob('published-*.npz'))
                if files!=sorted(p.name for p in prior.glob('published-*.npz')):raise RuntimeError('Publication set differs')
                for file in files:
                    with np.load(out/file) as gpu,np.load(prior/file) as ref:
                        if set(gpu.files)!=set(ref.files) or any(not np.array_equal(gpu[k],ref[k]) for k in ref.files):
                            raise RuntimeError('CPU/ROCm publication mismatch: '+file)
                gpu_rows=json.loads((out/'publications.json').read_text())
                cpu_rows=json.loads((prior/'publications.json').read_text())
                for gpu,ref in zip(gpu_rows,cpu_rows):
                    for key in ('cleared_cells','protected_cells','pending_cells'):
                        if gpu['evidence'].get(key)!=ref['evidence'].get(key):raise RuntimeError('Clearing evidence differs')
                record['exact_cpu_publication_and_clearing_parity']=True
            (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(record),flush=True)
        report['result']='OUTER_40M_ROS_AND_PARITY_PASS' if a.cpu_reference else 'OUTER_40M_CPU_ROS_PASS'
    except Exception as e:
        report.update(result='FAIL',failure=str(e))
        raise
    finally:
        (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
