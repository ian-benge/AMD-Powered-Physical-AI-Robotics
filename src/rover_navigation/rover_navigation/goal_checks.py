"""Full-footprint path checks, shared by preview and mission acceptance."""
from itertools import islice
import math

import numpy as np

from .core import FOOTPRINT, footprint_avoids_disk, samples


def pose_values(message):
    if message.header.frame_id != 'map':
        raise ValueError('Select a goal in the map frame')
    p, q = message.pose.position, message.pose.orientation
    if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
            abs(q.x) > 1e-4 or abs(q.y) > 1e-4 or
            abs(q.z*q.z+q.w*q.w-1.) > .01):
        raise ValueError('Goal must be a finite planar pose with a unit quaternion')
    return p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


def interpolated_poses(path, resolution):
    for a, b in zip(path, path[1:]):
        turn = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        steps = max(1, math.ceil(math.dist(a[:2], b[:2])/(resolution/2)),
                    math.ceil(abs(turn)/math.radians(2)))
        for i in range(steps+1):
            t = i/steps
            yield a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), a[2]+t*turn


def check_path(grid, poses, start, target, *, keepout=None, avoid_center=None):
    if not poses:
        raise ValueError('Planner returned an empty path')
    if math.dist(poses[0][:2], start[:2]) > .2 or math.dist(poses[-1][:2], target[:2]) > .15:
        raise ValueError('Planner path does not connect the requested endpoints')
    # Batch the original half-cell and two-degree samples. Long return
    # checks can use the latest grid without blocking input callbacks per pose.
    dx, dy = (delta.ravel() for delta in samples(grid, FOOTPRINT, .025))
    path = interpolated_poses([start, *poses, target], grid.resolution)
    for block in iter(lambda: list(islice(path, 128)), []):
        xy = np.asarray(block)[:, :2]
        c = np.array([math.cos(pose[2]) for pose in block])[:, None]
        s = np.array([math.sin(pose[2]) for pose in block])[:, None]
        cols = np.floor((xy[:, 0, None]+dx*c-dy*s-grid.x)/grid.resolution).astype(int)
        rows = np.floor((xy[:, 1, None]+dx*s+dy*c-grid.y)/grid.resolution).astype(int)
        inside = (rows>=0)&(rows<grid.data.shape[0])&(cols>=0)&(cols<grid.data.shape[1])
        free = np.zeros_like(inside)
        values = grid.data[rows[inside], cols[inside]]
        free[inside] = (values>=0)&(values<90)
        for pose, clear in zip(block, free.all(axis=1)):
            if avoid_center is not None and math.dist(pose[:2],avoid_center[:2])<avoid_center[2]:
                raise ValueError('Route repeats the recent contact approach')
            if keepout is not None and not footprint_avoids_disk(pose, keepout):
                raise ValueError('Path footprint crosses the selected sample site')
            if not clear:
                raise ValueError('Path footprint crosses hazardous or unknown ground at '
                                 f'({pose[0]:.3f}, {pose[1]:.3f}), heading {math.degrees(pose[2]):.1f} degrees')


def overlay_hazards(grid, points, padding=.04):
    """Only add lethal cells; never promote unknown terrain to free space."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    cells = np.floor((points[:, :2]-[grid.x, grid.y])/grid.resolution).astype(int)
    pad = math.ceil(padding/grid.resolution)
    for dy in range(-pad, pad+1):
        for dx in range(-pad, pad+1):
            c, r = cells[:, 0]+dx, cells[:, 1]+dy
            ok = (r >= 0) & (c >= 0) & (r < grid.data.shape[0]) & (c < grid.data.shape[1])
            grid.data[r[ok], c[ok]] = 100
    return grid
