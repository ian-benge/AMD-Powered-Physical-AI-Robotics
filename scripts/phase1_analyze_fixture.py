#!/usr/bin/env python3
"""Compare saved fixture depth with independent geometry and visible RGB ROIs."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from mars_training_environment.annotation import rasterize
from rover_training.geometry import nominal_intrinsics, quaternion_to_matrix
from phase1_capture_fixture import camera_intrinsics


BOX_FACES=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                    [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])


def rgb_support(samples, color):
    # The two known texture stripes differ in chromaticity by <0.025 for
    # these fixture colors. 0.05 allows quantization/shading while separating
    # the buried yellow target from brown foreground ground.
    expected=np.asarray(color)*np.array([210.,120.,70.])
    expected=expected/max(1.,expected.sum())
    samples=np.asarray(samples,dtype=float)
    observed=samples/np.maximum(1.,samples.sum(axis=-1,keepdims=True))
    return np.max(np.abs(observed-expected),axis=-1)<=.05


def near_background_metrics(rgb, near_mask, background):
    # Match the other RGB/depth gates: exclude two silhouette pixels, whose
    # coverage changes with rasterization and bounded measured-pose timing.
    interior=cv2.erode(near_mask.astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)
    support=np.abs(rgb.astype(float)-np.asarray(background)*255).max(axis=-1)<3
    return dict(near_occluded_pixels=int(near_mask.sum()),
                near_comparison_pixels=int(interior.sum()),
                near_raw_background_support_fraction=float(support[near_mask].mean()) if near_mask.any() else None,
                near_background_support_fraction=float(support[interior].mean()) if interior.any() else None)


def box_depth(item, xyz, optical, intrinsics, near, far, radial, rays=None, world_rays=None):
    """Independent analytic ray / axis-aligned-box intersection (optical Z)."""
    if rays is None:
        yy,xx=np.indices((intrinsics['height'],intrinsics['width']))
        rays=np.stack(((xx+.5-intrinsics['cx'])/intrinsics['fx'],
                       (yy+.5-intrinsics['cy'])/intrinsics['fy'],np.ones_like(xx)),axis=-1)
    if world_rays is None: world_rays=rays@optical
    center,size=np.asarray(item['xyz']),np.asarray(item['size'])
    low,high=center-size/2,center+size/2
    with np.errstate(divide='ignore',invalid='ignore'):
        a=(low-xyz)/world_rays; b=(high-xyz)/world_rays
    parallel=np.abs(world_rays)<1e-12
    outside=parallel & ((np.asarray(xyz)<low)|(np.asarray(xyz)>high))
    entry=np.max(np.where(parallel,-np.inf,np.minimum(a,b)),axis=-1)
    exit=np.min(np.where(parallel,np.inf,np.maximum(a,b)),axis=-1)
    depth=np.where(entry>=near,entry,exit)
    distance=depth*np.linalg.norm(rays,axis=-1) if radial else depth
    valid=(exit>=entry)&(depth>=near)&(distance<=far)&~outside.any(axis=-1)
    return np.where(valid,depth,np.inf).astype(np.float32)


def expected(fixture, pose, far, radial=True, include_transparent=False, cache=None):
    camera=fixture['camera']
    k=camera.get('intrinsics') or nominal_intrinsics(camera['width'],camera['height'],camera['horizontal_fov_rad'])
    xyz,q=pose[1:]
    optical=np.array([[0,-1,0],[0,0,-1],[1,0,0]])@np.array(quaternion_to_matrix(*q)).T
    if cache is None: cache={}
    key=tuple(xyz)+tuple(q)
    if cache.get('pose')!=key:
        cache.clear(); cache['pose']=key; cache['layers']={}
        yy,xx=np.indices((camera['height'],camera['width']))
        cache['rays']=np.stack(((xx+.5-k['cx'])/k['fx'],(yy+.5-k['cy'])/k['fy'],np.ones_like(xx)),axis=-1)
        cache['world_rays']=cache['rays']@optical
        cache['radial_scale']=np.linalg.norm(cache['rays'],axis=-1)
    zbuffer=np.full((camera['height'],camera['width']),np.inf,np.float32)
    owners=np.zeros(zbuffer.shape,np.uint8)
    layers={}
    for index,item in enumerate(fixture['objects'],1):
        if item['texture']=='translucent' and not include_transparent: continue
        if item['name'] not in cache['layers']:
            render_near=1e-4 if camera.get('near_clip_mode')=='occlude' else camera['near_m']
            if item['shape']=='box':
                depth=box_depth(item,np.asarray(xyz),optical,k,render_near,1000,False,
                                cache['rays'],cache['world_rays'])
            else:
                vertices=np.array(fixture['rock_vertices'])*item['size']+item['xyz']
                transformed=(vertices-np.array(xyz))@optical.T
                depth=rasterize(transformed,fixture['rock_faces'],k,render_near,1000,far_clip_mode='axial')
            cache['layers'][item['name']]=depth
        depth=cache['layers'][item['name']].copy()
        depth[depth*cache['radial_scale']>far if radial else depth>far]=np.inf
        layers[item['name']]=depth
        nearer=depth<zbuffer
        zbuffer[nearer]=depth[nearer];owners[nearer]=index
    near_occluded=np.isfinite(zbuffer)&(zbuffer<=camera['near_m'])
    owners[near_occluded]=0; zbuffer[near_occluded]=np.inf
    cache['last_near_mask']=near_occluded
    for depth in layers.values(): depth[depth<=camera['near_m']]=np.inf
    return zbuffer,owners,layers


def analyze(fixture, capture, stride=1, limit=None, output_path=None, calibration_path=None):
    if stride<1 or (limit is not None and limit<1):
        raise ValueError('Stride and limit must be positive')
    fixture_path=Path(fixture)
    fixture=json.loads(fixture_path.read_text())
    capture=Path(capture)
    calibration=Path(calibration_path) if calibration_path else capture/'camera-info.json'
    if calibration.exists():
        intrinsics=camera_intrinsics(json.loads(calibration.read_text()))
        if (intrinsics['width'],intrinsics['height'])!=(fixture['camera']['width'],fixture['camera']['height']):
            raise ValueError('Calibration does not match fixture dimensions')
        fixture['camera']['intrinsics']=intrinsics
    records=[json.loads(line) for line in (capture/'frames.jsonl').read_text().splitlines()][::stride]
    if limit is not None: records=records[:limit]
    output=[]
    geometry_cache={}
    for frame in records:
        if not frame['synchronized_pose']:
            output.append(dict(index=frame['index'],error='missing synchronized measured pose'));continue
        index=frame['index']
        depth_file=capture/f'depth-{index:05d}.npz'
        measured=(np.load(depth_file)['depth'] if depth_file.exists() else np.load(capture/f'depth-{index:05d}.npy'))
        rgb=cv2.imread(str(capture/f'rgb-{index:05d}.png'))[:,:,::-1]
        modes={}
        for radial in (True,False):
            rgb_predicted,owners,layers=expected(fixture,frame['pose'],fixture['camera']['rgb_far_m'],radial,cache=geometry_cache)
            near_mask=geometry_cache['last_near_mask']
            # Compute depth clipping independently: a 20 m RGB mask is never
            # used as evidence of 20 m usable depth, including off-axis pixels.
            predicted,depth_owners,_=expected(fixture,frame['pose'],fixture['camera']['far_m'],radial,cache=geometry_cache)
            panel_predicted,panel_owners,_=expected(fixture,frame['pose'],fixture['camera']['rgb_far_m'],radial,True,geometry_cache)
            panel_id=next(i for i,o in enumerate(fixture['objects'],1) if o['texture']=='translucent')
            panel=panel_owners==panel_id
            sky_panel=cv2.erode((panel&(owners==0)&~near_mask).astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)
            sky=np.array(fixture.get('background_rgb',[.25,.4,.65]))*255
            opaque=.8*np.array([210.,120.,70.])  # known fixture ambient and white panel material
            alpha=((rgb[sky_panel]-sky)@(opaque-sky))/np.dot(opaque-sky,opaque-sky) if sky_panel.any() else np.array([])
            usable=np.isfinite(measured)&(measured>fixture['camera']['near_m'])&(measured<=fixture['camera']['far_m'])
            valid=np.isfinite(predicted)&usable
            # Ignore silhouettes and depth discontinuities; they receive
            # subpixel rasterization/antialiasing contributions in Ogre.
            edge=cv2.dilate(depth_owners,np.ones((5,5),np.uint8))!=cv2.erode(depth_owners,np.ones((5,5),np.uint8))
            valid &= ~edge & ~panel
            errors=np.abs(predicted[valid]-measured[valid])
            visibility=[]
            for ident,item in enumerate(fixture['objects'],1):
                mask=cv2.erode(((owners==ident)&~panel).astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)
                if not mask.any():
                    visibility.append(dict(name=item['name'],predicted_pixels=0,
                        projected_pixels=int(np.isfinite(layers.get(item['name'],np.inf)).sum()),
                        explanation='occluded, clipped or outside image; inspect projection layers'))
                    continue
                samples=rgb[mask].astype(float)
                support=rgb_support(samples,item['color'])
                visibility.append(dict(name=item['name'],predicted_pixels=int(mask.sum()),
                    projected_pixels=int(np.isfinite(layers[item['name']]).sum()),
                    rgb_mean=samples.mean(axis=0).tolist(),rgb_chroma_support_fraction=float(support.mean()),
                    finite_depth_fraction=float(np.isfinite(measured[mask]).mean()),
                    usable_depth_fraction=float(usable[mask].mean()),
                    rgb_only_pixels=int((mask&~np.isfinite(predicted)).sum())))
            modes['radial' if radial else 'axial']=dict(compared_pixels=int(valid.sum()),
                median_error_m=float(np.median(errors)) if len(errors) else None,
                p95_error_m=float(np.percentile(errors,95)) if len(errors) else None,
                fraction_over_3cm=float((errors>.03).mean()) if len(errors) else None,
                expected_finite_missing_fraction=float((np.isfinite(predicted)&~usable&~edge&~panel).sum()/max(1,(np.isfinite(predicted)&~edge&~panel).sum())),
                unexpected_usable_fraction=float((~np.isfinite(predicted)&usable&~edge&~panel).mean()),
                **near_background_metrics(rgb,near_mask,fixture.get('background_rgb',[.25,.4,.65])),
                transparent_panel=dict(pixels=int(panel.sum()),
                    rgb_mean=rgb[panel].mean(axis=0).tolist() if panel.any() else None,
                    sky_pixels=int(sky_panel.sum()),
                    apparent_alpha_median=float(np.median(alpha)) if len(alpha) else None,
                    intermediate_alpha_fraction=float(((alpha>.05)&(alpha<.95)).mean()) if len(alpha) else None,
                    note='Known ambient/texture estimate, front/back faces may both blend; review with material probe'),
                visibility=visibility)
        output.append(dict(index=index,stamp_ns=frame['stamp_ns'],modes=modes))
    result=dict(version=3,silhouette_exclusion_pixels=2,fixture_source_sha256=fixture.get('source_sha256'),
                fixture_sha256=hashlib.sha256(fixture_path.read_bytes()).hexdigest(),
                analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                calibration_source=str(calibration) if calibration.exists() else 'NOMINAL_UNQUALIFIED',
                calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest() if calibration.exists() else None,
                intrinsics=fixture['camera'].get('intrinsics'),
                capture=str(capture),sampled=bool(stride!=1 or limit is not None),
                note='Analytic box geometry; mesh rasterization; RGB chroma tolerance 0.05 is a diagnostic, not detector accuracy',frames=output)
    with (Path(output_path) if output_path else capture/'geometry-comparison.json').open('x') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps(output[:2],indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fixture',type=Path);parser.add_argument('capture',type=Path)
    parser.add_argument('--stride',type=int,default=1);parser.add_argument('--limit',type=int)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--calibration',type=Path)
    args=parser.parse_args();analyze(args.fixture,args.capture,args.stride,args.limit,args.output,args.calibration)
