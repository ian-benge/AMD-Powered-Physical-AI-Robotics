"""Independent mesh/continuous-route checks; evaluator only, never navigation."""
import math
from pathlib import Path

import numpy as np

from .common import rotation
from .meshes import read_collada
from .terrain import Terrain


def distance_to_path(points, path):
    points=np.asarray(points,dtype=float);path=np.asarray(path,dtype=float)
    distance=np.full(len(points),np.inf)
    for a,b in zip(path,path[1:]):
        delta=b-a
        t=np.clip((points-a)@delta/max(float(delta@delta),1e-20),0,1)
        distance=np.minimum(distance,np.linalg.norm(points-(a+t[:,None]*delta),axis=1))
    return distance


def audit_layout(root, geometry, certificate, assets):
    """Check saved meshes and every complete corridor without the planner mask.

    Each terrain triangle is enclosed by a circle about its centroid. Testing
    every enclosing circle that intersects the corridor is conservative, and
    includes triangle interiors, segment interiors and the complete turn sweep.
    """
    root=Path(root);terrain=geometry['terrain'];objects=geometry['objects']
    folder=root/'scenes'/f"scene_{geometry['seed']:06d}"
    vertices,faces=read_collada(folder/'terrain.dae',normalize=False)
    cv,cf=read_collada(folder/'terrain_collision.dae',normalize=False)
    if not np.array_equal(vertices,cv) or not np.array_equal(faces,cf):
        raise ValueError('Visible and physical terrain triangles disagree')
    field=Terrain(terrain['heights'],terrain['size_m'])
    exact,expected_faces=field.mesh()
    if (not np.allclose(vertices,exact,atol=1e-7,rtol=0) or
            not np.array_equal(faces,expected_faces) or
            not np.array_equal(field.h,np.asarray(terrain['base_heights']))):
        raise ValueError('Placement/annotation and saved terrain meshes disagree')
    triangles=vertices[faces]
    normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    slopes=np.degrees(np.arctan2(np.linalg.norm(normal[:,:2],axis=1),normal[:,2]))
    centroids=triangles[:,:,:2].mean(axis=1)
    bounds=np.linalg.norm(triangles[:,:,:2]-centroids[:,None,:],axis=2).max(axis=1)
    touches_pad=np.linalg.norm(centroids,axis=1)<=1.3+bounds
    if abs(triangles[touches_pad,:,2]).max()>1e-7:
        raise ValueError('The full 1.3 m launch disk is not flat')
    if np.ptp(vertices[:,2])>.8000001 or not np.isfinite(vertices).all():
        raise ValueError('Invalid relief/vertices')
    by_id={a['id']:a for a in assets}
    radii=[]
    for obj in objects:
        asset=by_id[obj['asset_id']]
        radius=0.
        for kind in ('mesh','collision'):
            v,_=read_collada(root/obj.get(kind,asset[kind]),normalize=False)
            p=v@rotation(obj['rpy']).T*obj['scale']
            radius=max(radius,float(np.linalg.norm(p[:,:2],axis=1).max()))
        if radius>obj['radius_m']+1e-6:
            raise ValueError('Object clearance radius excludes mesh: '+obj['name'])
        if np.linalg.norm(obj['xyz'][:2])-radius<1.3-1e-6:
            raise ValueError('An object intrudes into the launch disk')
        radii.append(radius)
    if certificate.get('truth_to_navigation') is not False or not certificate.get('evaluator_only'):
        raise ValueError('Route truth must remain evaluator-only')
    version=terrain.get('profile_version',1)
    required={'ordinary_rock','pale_soil','crystal_rock'}
    if version==2:
        required={name+':'+band for name in ('rock','pale_soil_patch','yellow_crystal_rock','mudstone_slab') for band in ('near','outer')}
        if not objects and not geometry['regions']:required=set()
        context=terrain['context_geometry']
        av,af=read_collada(folder/context['mesh'],normalize=False)
        at=av[af]
        normals=np.cross(at[:,1]-at[:,0],at[:,2]-at[:,0])
        area=float(normals[:,2].sum()/2)
        if abs(area-4800)>1e-4 or (normals[:,2]<=0).any() or (np.max(abs(at.mean(axis=1)[:,:2]),axis=1)<=20).any():
            raise ValueError('Context ring does not cover the joined outer area')
        inner=np.max(abs(av[:,:2]),axis=1)==20
        if inner.sum()!=1024 or not np.allclose(av[inner,2],field.height(av[inner,0],av[inner,1]),atol=1e-7,rtol=0):
            raise ValueError('Context seam is discontinuous')
    if {r['category'] for r in certificate['routes']}!=required:
        raise ValueError('Missing required staging class')
    routes=[]
    targets={t['name']:t for t in objects+geometry['regions']}
    for route in certificate['routes']:
        path=np.asarray(route['path_xy'])
        if (path.ndim!=2 or path.shape[1]!=2 or not np.isfinite(path).all() or
                not np.allclose(path[0],[.13,0],atol=1e-10) or
                not np.allclose(path[-1],route['staging_xy'],atol=1e-10)):
            raise ValueError('Incomplete route')
        clearance=float(np.min(distance_to_path([o['xyz'][:2] for o in objects],path)-radii))
        edge=terrain['size_m']/2-float(abs(path).max())
        if version==1:swept=distance_to_path(centroids,path)<=.5+bounds+1e-7
        else:
            swept=np.zeros(len(centroids),dtype=bool)
            for a,b in zip(path,path[1:]):
                margin=.5+bounds.max()+1e-7
                candidates=np.flatnonzero((centroids>=np.minimum(a,b)-margin).all(axis=1)&(centroids<=np.maximum(a,b)+margin).all(axis=1))
                swept[candidates]|=distance_to_path(centroids[candidates],[a,b])<=.5+bounds[candidates]+1e-7
        worst=float(slopes[swept].max())
        if min(clearance,edge)<.5-1e-7 or worst>=8:
            raise ValueError('Continuous corridor fails width/terrain-slope check')
        target=targets[route['target']]
        radius=target['radius_m']
        if 'polygon_xy' in target:
            radius=float(np.linalg.norm(np.asarray(target['polygon_xy'])-target['xyz'][:2],axis=1).max())
        gap=float(np.linalg.norm(path[-1]-target['xyz'][:2])-radius)
        lo,hi=(12,17) if version==2 and route['category'].endswith(':outer') else (3,7)
        if not (lo-1e-7<=np.linalg.norm(path[-1])<=hi+1e-7 and gap>=.75-1e-7 and gap<=1.5+1e-7):
            raise ValueError('Staging is outside the required range/target clearance')
        if version==2:
            name=route['category'].split(':')[0]
            appearance=target.get('class_name','rock')
            actual=('mudstone_slab' if appearance in ('cheyava_rock','veined_lookalike') or target.get('asset_id')=='cand_slab' else
                    'yellow_crystal_rock' if appearance=='sulfur_rock' else 'pale_soil_patch' if appearance in ('silica_soil','pale_lookalike') else 'rock')
            if actual!=name or (lo==12 and np.max(abs(path[-1]))<12):raise ValueError('Wrong export target or outer area')
        routes.append(dict(category=route['category'],maximum_corridor_slope_deg=worst,
                           minimum_obstacle_clearance_m=clearance,minimum_edge_clearance_m=edge,
                           checked_triangles=int(swept.sum()),target_boundary_gap_m=gap,
                           path_length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())))
    return dict(result='PASS',method='saved mesh normals and conservative continuous swept-corridor checks',
                shared_mesh=True,clear_launch_radius_m=1.3,relief_m=float(np.ptp(vertices[:,2])),
                maximum_slope_deg=float(slopes.max()),routes=routes)
