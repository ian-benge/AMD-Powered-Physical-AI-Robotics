#!/usr/bin/env python3
"""CPU-only seeded terrain qualification. Generates no camera dataset or motion."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import time

from mars_training_environment.common import digest, write_json
from mars_training_environment.generate import make_scene
from mars_training_environment.reachability import mixed_reachable_config
from mars_training_environment.reachability_audit import audit_layout


def copy_library(source, target):
    def link(a,b):
        try: os.link(a,b)
        except OSError: shutil.copy2(a,b)
        return b
    shutil.copytree(source,target,copy_function=link)


def geometry_config(source, seed):
    config=mixed_reachable_config(source)
    config['dataset'].update(seeds=[seed],empty_seeds=[],rocks_only_seeds=[])
    config['capture']['views_per_scene']=0
    config['capture'].pop('target_class_views',None)
    return config


def plot_scene(root, scene, audit):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    import numpy as np
    fig,ax=plt.subplots(figsize=(9,9),layout='constrained')
    h=np.asarray(scene['terrain']['heights']);coords=np.linspace(-10,10,len(h))
    image=ax.pcolormesh(coords,coords,h,shading='gouraud',cmap='terrain',rasterized=True)
    fig.colorbar(image,ax=ax,label='Surface height (m)',shrink=.7)
    ax.contour(coords,coords,h,levels=12,colors='black',linewidths=.3,alpha=.4)
    for region in scene['regions']:
        ax.add_patch(Polygon(region['polygon_xy'],facecolor='white',edgecolor='gray',alpha=.8))
    for obj in scene['objects']:
        ax.add_patch(Circle(obj['xyz'][:2],obj['radius_m'],color='#48423c'))
    ax.add_patch(Circle((0,0),1.3,fill=False,edgecolor='black',linestyle='--'))
    for route,color in zip(scene['reachability']['routes'],['#c42d36','#315bce','#702888']):
        path=np.asarray(route['path_xy'])
        ax.plot(*path.T,color=color,linewidth=13,alpha=.13,solid_capstyle='round')
        ax.plot(*path.T,color=color,linewidth=2,label=route['category'].replace('_',' '))
        ax.scatter(*route['staging_xy'],color=color,marker='s',s=45)
        ax.scatter(*route['target_xy'],color=color,marker='x',s=55)
    ax.scatter(.13,0,color='black',marker='^',s=70)
    ax.set(xlim=(-10,10),ylim=(-10,10),aspect='equal',xlabel='World x (m)',ylabel='World y (m)',
           title=f"{scene['id']} · mixed_reachable v1\nEvaluator routes only · 1 m corridors · maximum surface slope {audit['maximum_slope_deg']:.1f}°")
    ax.legend(loc='upper left');ax.grid(alpha=.2)
    fig.savefig(root/(scene['id']+'-routes.png'),dpi=150)
    fig.savefig(root/(scene['id']+'-routes.svg'))
    plt.close(fig)


def qualify(template, library, output, seeds, materialize=3):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    source=json.loads(Path(template).read_text())
    copy_library(Path(library),output/'assets')
    shutil.copy2(template,output/'template.json')
    assets=json.loads((output/'assets/catalog.json').read_text())
    ground_path=output/'assets/ground/catalog.json'
    ground=json.loads(ground_path.read_text()) if ground_path.exists() else []
    report=dict(schema_version=1,utc=datetime.now(timezone.utc).isoformat(),result='RUNNING',
                profile='mixed_reachable',profile_version=1,seeds=list(seeds),layouts=[],
                materialized=[],capture_requested=False,truth_to_navigation=False,
                asset_catalog_sha256=digest(assets),source_template_sha256=hashlib.sha256(Path(template).read_bytes()).hexdigest())
    try:
        for seed in seeds:
            config=geometry_config(source['config'],seed)
            started=time.monotonic()
            first=make_scene(output,config,seed,assets,ground,validation_only=True)
            audit=audit_layout(output,first['geometry'],first['reachability'],assets)
            # Re-execute placement, retries and terrain from scratch in another
            # directory; no cached manifest or saved output can satisfy replay.
            replay=output/'replay'
            if not (replay/'assets').exists():copy_library(output/'assets',replay/'assets')
            second=make_scene(replay,config,seed,assets,ground,validation_only=True)
            if digest(first)!=digest(second):raise ValueError('Seed/config replay changed: '+str(seed))
            write_json(output/'geometry'/f'{seed}.json',first)
            row=dict(seed=seed,geometry_sha256=first['geometry_sha256'],
                     layout_attempt=first['geometry']['layout_attempt'],layout_failures=first['layout_failures'],
                     reproducible=True,seconds=time.monotonic()-started,audit=audit)
            report['layouts'].append(row)
            write_json(output/'qualification.json',report)
            print(json.dumps(dict(seed=seed,attempt=row['layout_attempt'],result='PASS',seconds=row['seconds'])),flush=True)
        ordered=sorted(report['layouts'],key=lambda r:r['audit']['maximum_slope_deg'])
        indices=[round(i*(len(ordered)-1)/max(1,materialize-1)) for i in range(materialize)]
        for index in indices:
            row=ordered[index];seed=row['seed'];config=geometry_config(source['config'],seed)
            destination=output/'representative-scenes'/str(seed)
            copy_library(output/'assets',destination/'assets')
            scene=make_scene(destination,config,seed,assets,ground)
            if scene['geometry_sha256']!=row['geometry_sha256']:
                raise ValueError('Launchable scene differs from qualified geometry')
            geometry=json.loads((output/'geometry'/f'{seed}.json').read_text())['geometry']
            audit=audit_layout(destination,geometry,scene['reachability'],assets)
            plot_scene(output,scene,audit)
            report['materialized'].append(dict(seed=seed,bundle=str(destination.resolve()),
                                              scene=scene['id'],content_sha256=scene['content_sha256'],
                                              geometry_sha256=scene['geometry_sha256'],audit=audit))
        report['result']='CPU_GEOMETRY_PASS_MANUAL_TRAVERSAL_PENDING'
    except Exception as error:
        report.update(result='FAIL',failure=str(error))
        raise
    finally:write_json(output/'qualification.json',report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template',type=Path,required=True)
    parser.add_argument('--assets',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed-start',type=int,default=2601100)
    parser.add_argument('--count',type=int,default=50)
    parser.add_argument('--materialize',type=int,default=3)
    args=parser.parse_args()
    if not (0<=args.seed_start<1_000_000_000 and 1<=args.count<=1000 and
            args.seed_start+args.count<=1_000_000_000 and 0<=args.materialize<=args.count):
        parser.error('Invalid bounded seed count or representative count')
    report=qualify(args.template,args.assets,args.output,
                   range(args.seed_start,args.seed_start+args.count),args.materialize)
    print(json.dumps(dict(result=report['result'],layouts=len(report['layouts']),materialized=len(report['materialized']))))


if __name__=='__main__':main()
