#!/usr/bin/env python3
"""Predeclared CPU geometry qualification; creates no training images or motion."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import time

from mars_training_environment.common import digest,write_json
from mars_training_environment.environment_v2 import configuration,qualification_recipe,QUALIFICATION_SEEDS
from mars_training_environment.generate import make_scene
from mars_training_environment.reachability_audit import audit_layout
from phase2_validate_layouts import copy_library


def qualify(template,library,output,count=50,materialize=True):
    output.mkdir(parents=True,exist_ok=False)
    source=json.loads(template.read_text());copy_library(library,output/'assets')
    assets=json.loads((output/'assets/catalog.json').read_text())
    gp=output/'assets/ground/catalog.json';ground=json.loads(gp.read_text()) if gp.exists() else []
    recipes=[qualification_recipe(i) for i in range(count)]
    write_json(output/'PREDECLARED_RECIPES.json',recipes)
    report=dict(result='RUNNING',capture_requested=False,truth_to_navigation=False,recipes=recipes,layouts=[],materialized=[])
    write_json(output/'qualification.json',report)
    try:
        for recipe in recipes:
            seed=recipe['seed'];config=configuration(source['config'],recipe['density'],recipe['sunlight'],manual=True)
            config['dataset'].update(seeds=[seed],empty_seeds=[],rocks_only_seeds=[])
            start=time.monotonic();first=make_scene(output,config,seed,assets,ground,validation_only=True)
            audit=audit_layout(output,first['geometry'],first['reachability'],assets)
            replay=output/'replay'
            if not (replay/'assets').exists():copy_library(output/'assets',replay/'assets')
            second=make_scene(replay,config,seed,assets,ground,validation_only=True)
            if digest(first)!=digest(second):raise ValueError('Independent seeded replay differs')
            write_json(output/'geometry'/f'{seed}.json',first)
            row=dict(**recipe,geometry_sha256=first['geometry_sha256'],layout_attempt=first['geometry']['layout_attempt'],
                failures=first['layout_failures'],audit=audit,reproducible=True,seconds=time.monotonic()-start)
            report['layouts'].append(row);write_json(output/'qualification.json',report)
            print(json.dumps(dict(seed=seed,density=recipe['density'],result='PASS',seconds=row['seconds'])),flush=True)
        if materialize:
            for density in ('sparse','standard','cluttered'):
                recipe=next(r for r in recipes if r['density']==density);seed=recipe['seed']
                config=configuration(source['config'],density,recipe['sunlight'],manual=True)
                config['dataset'].update(seeds=[seed],empty_seeds=[],rocks_only_seeds=[])
                dest=output/'representative-scenes'/str(seed);copy_library(output/'assets',dest/'assets')
                scene=make_scene(dest,config,seed,assets,ground)
                expected=next(r for r in report['layouts'] if r['seed']==seed)
                if scene['geometry_sha256']!=expected['geometry_sha256']:raise ValueError('Materialized geometry changed')
                report['materialized'].append(dict(**recipe,bundle=str(dest),scene=scene['id'],
                    content_sha256=scene['content_sha256'],geometry_sha256=scene['geometry_sha256']))
                write_json(output/'qualification.json',report)
        report['result']='CPU_GEOMETRY_PASS_LIVE_QUALIFICATION_PENDING'
    except Exception as error:
        report.update(result='FAIL',failure=str(error));raise
    finally:write_json(output/'qualification.json',report)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('template','assets','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--count',type=int,default=50);p.add_argument('--no-materialize',action='store_true')
    a=p.parse_args()
    if not 1<=a.count<=50:p.error('Count must be in 1..50; full acceptance requires 50')
    qualify(a.template,a.assets,a.output,a.count,not a.no_materialize)
