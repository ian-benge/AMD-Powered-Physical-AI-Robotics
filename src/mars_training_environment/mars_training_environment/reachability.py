"""Generation/evaluation truth only. Never consumed by navigation controllers."""
from copy import deepcopy
import heapq
import json
import math

import numpy as np

PROFILE = 'mixed_reachable'
VERSION = 1
# The retained rover envelope about its front-axle drive center. A 0.5 m
# corridor radius also contains its complete padded in-place turning sweep.
FOOTPRINT = (.07, -.33, .17)
TURN_RADIUS = math.hypot(.33, .17)+.025
CORRIDOR_RADIUS = max(.5, TURN_RADIUS)
START = (.13, 0.)


class LayoutError(ValueError):
    """A deterministic layout is infeasible; a bounded retry is permitted."""


def mixed_reachable_config(source):
    if source.get('terrain', {}).get('profile_version') == 2:
        from .environment_v2 import configuration
        return configuration(source)
    from .common import config_path
    config=deepcopy(source)
    updates=json.loads(config_path().with_name('mixed_reachable.json').read_text())
    for section,values in updates.items():
        config.setdefault(section,{}).update(values)
    validate_profile(config['terrain'])
    return config


def validate_profile(terrain):
    profile=terrain.get('profile')
    if profile is None:
        return
    version=terrain.get('profile_version')
    if profile!=PROFILE or version not in (1,2):
        raise ValueError('Unknown terrain profile/version')
    if version==2:
        from .environment_v2 import DENSITIES,SUNLIGHT
        if (terrain['size_m']!=40 or terrain['grid_size']!=257 or terrain['visual_grid_size']!=257 or
                terrain.get('apron_m')!=20 or terrain.get('composite_px')!=8192 or
                terrain.get('density') not in DENSITIES or terrain.get('sunlight') not in SUNLIGHT):
            raise ValueError('mixed_reachable v2 requires 40 m, shared 257 grids, 20 m apron, 8192 textures and named recipes')
    if (terrain['size_m']!=(40 if version==2 else 20) or terrain['spawn_clearance_m']!=1.3 or
            terrain['grid_size']!=terrain['visual_grid_size'] or terrain['grid_size']<129 or
            not 0<terrain['relief_m']<=.8 or not 8<terrain['max_slope_deg']<25):
        raise ValueError('mixed_reachable v1 requires shared >=129 grids, 20 m world, 1.3 m pad and <=0.8 m relief')
    if type(terrain.get('layout_attempts')) is not int or not 1<=terrain['layout_attempts']<=64:
        raise ValueError('layout_attempts must be an integer in 1..64')


def triangle_slopes(field):
    """Worst of the two exact collision-triangle gradients in each cell."""
    h,s=field.h,field.step
    a=np.hypot(h[:-1,1:]-h[:-1,:-1],h[1:,:-1]-h[:-1,:-1])/s
    b=np.hypot(h[1:,1:]-h[1:,:-1],h[1:,1:]-h[:-1,1:])/s
    return np.degrees(np.arctan(np.maximum(a,b)))


def _inflate(mask, radius, step):
    result=np.zeros_like(mask)
    count=math.ceil(radius/step)
    n=len(mask)
    for dy in range(-count,count+1):
        for dx in range(-count,count+1):
            if math.hypot(dx,dy)*step>radius+1e-12:
                continue
            y0,y1=max(0,dy),min(n,n+dy);x0,x1=max(0,dx),min(n,n+dx)
            result[y0:y1,x0:x1]|=mask[y0-dy:y1-dy,x0-dx:x1-dx]
    return result


def free_centers(field, objects):
    """Conservative continuous 1 m corridors, including turns and grid error."""
    slopes=triangle_slopes(field)
    steep=slopes>=8.0-1e-9
    bad=np.zeros_like(field.h,dtype=bool)
    for dy,dx in [(0,0),(0,1),(1,0),(1,1)]:
        bad[dy:dy+len(steep),dx:dx+len(steep)]|=steep
    # Expand steep cells to cover their area and continuous path segments,
    # rather than certifying only the height at a center point.
    margin=CORRIDOR_RADIUS+math.sqrt(2)*field.step
    bad=_inflate(bad,margin,field.step)
    coords=np.linspace(-field.size/2,field.size/2,field.n)
    x,y=np.meshgrid(coords,coords)
    free=~bad & (np.maximum(abs(x),abs(y))<=field.size/2-margin)
    for obj in objects:
        radius=float(obj['radius_m'])
        if not math.isfinite(radius) or radius<=0:
            raise LayoutError('Invalid object footprint')
        free &= np.hypot(x-obj['xyz'][0],y-obj['xyz'][1])>radius+margin
    return free,x,y,slopes


def _connected(free, start, step):
    n=len(free);distance=np.full(free.shape,np.inf);parent=np.full((*free.shape,2),-1,dtype=np.int16)
    if not free[start]:
        raise LayoutError('Launch drive center has insufficient turning/corridor clearance')
    distance[start]=0;queue=[(0.,*start)]
    while queue:
        cost,r,c=heapq.heappop(queue)
        if cost!=distance[r,c]:continue
        for dr,dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr,nc=r+dr,c+dc
            if not (0<=nr<n and 0<=nc<n and free[nr,nc]):continue
            if dr and dc and not (free[r,nc] and free[nr,c]):continue
            new=cost+step*math.hypot(dr,dc)
            if new<distance[nr,nc]:
                distance[nr,nc]=new;parent[nr,nc]=[r,c];heapq.heappush(queue,(new,nr,nc))
    return distance,parent


def validate_layout(field, objects, regions, version=1, background=False):
    """Find three post-placement routes, retaining truth only for evaluation."""
    if not np.isfinite(field.h).all() or np.ptp(field.h)>.80000001:
        raise LayoutError('Non-finite surface or relief above 0.8 m')
    free,x,y,slopes=free_centers(field,objects)
    start=(round((START[1]+field.size/2)/field.step),round((START[0]+field.size/2)/field.step))
    distance,parent=_connected(free,start,field.step)
    radial=np.hypot(x,y)
    available=np.isfinite(distance)&(radial>=3)&(radial<=7)
    groups=[('crystal_rock',[o for o in objects if o.get('class_name')=='sulfur_rock']),
            ('pale_soil',[o for o in regions if o.get('class_name')=='silica_soil']),
            ('ordinary_rock',[o for o in objects if o.get('class_name','rock')=='rock'])]
    if version==2:
        from .environment_v2 import export_class_id
        from rover_training.geometry import FOUR_CLASS_NAMES
        groups=[(name,[o for o in objects+regions if export_class_id(o)==cid]) for cid,name in enumerate(FOUR_CLASS_NAMES)]
        groups=[] if background else [(name+':'+band,targets) for band in ('near','outer') for name,targets in groups]
    routes=[]
    for category,targets in groups:
        if version==2:
            band=category.split(':')[1]
            lo,hi=(3,7) if band=='near' else (12,17)
            available=np.isfinite(distance)&(radial>=lo)&(radial<=hi)
            if band=='outer':available &= np.maximum(abs(x),abs(y))>=12
        best=None
        for target in targets:
            center=np.asarray(target['xyz'][:2]);radius=target['radius_m']
            if 'polygon_xy' in target:
                radius=float(np.linalg.norm(np.asarray(target['polygon_xy'])-center,axis=1).max())
            gap=np.hypot(x-center[0],y-center[1])-radius
            eligible=available&(gap>=.75)&(gap<=1.5)
            for route in routes:
                eligible &= np.hypot(x-route['staging_xy'][0],y-route['staging_xy'][1])>=1.25
            score=np.where(eligible,distance+.2*abs(gap-1.2),np.inf)
            index=np.unravel_index(np.argmin(score),score.shape)
            if not np.isfinite(score[index]):continue
            if best is None or score[index]<best[0]:best=(score[index],index,target,float(gap[index]))
        if best is None:
            raise LayoutError(f'No reachable {category} staging area after object placement')
        _,index,target,gap=best;path=[];current=index
        while current!=start:
            path.append([float(x[current]),float(y[current])]);current=tuple(parent[current])
            if current[0]<0:raise LayoutError('Broken connectivity certificate')
        path.append([float(x[start]),float(y[start])]);path.reverse()
        # Every segment lies inside the conservatively inflated free mask.
        routes.append(dict(category=category,target=target['name'],target_xy=target['xyz'][:2],
                           staging_xy=[float(x[index]),float(y[index])],target_boundary_gap_m=gap,
                           launch_distance_m=float(radial[index]),path_length_m=float(distance[index]),
                           path_xy=[list(START)]+path))
    if version==2 and background and not (np.isfinite(distance)&(np.maximum(abs(x),abs(y))>=12)).any():
        raise LayoutError('Background has no connected outer operating area')
    return dict(schema_version=version,policy='mixed_reachable_v'+str(version),evaluator_only=True,
                truth_to_navigation=False,start_xy=list(START),corridor_width_m=2*CORRIDOR_RADIUS,
                padded_turn_radius_m=TURN_RADIUS,max_passage_slope_deg=8.,
                slope_method='exact mesh triangles, conservatively inflated over complete swept corridor',
                discretization_margin_m=math.sqrt(2)*field.step,
                connected_center_fraction=float(np.isfinite(distance).mean()),
                blocked_center_fraction=float((~free).mean()),maximum_surface_slope_deg=float(slopes.max()),routes=routes)
