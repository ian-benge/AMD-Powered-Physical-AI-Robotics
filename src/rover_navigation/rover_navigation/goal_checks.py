"""Full-footprint path checks, shared by preview and mission acceptance."""
import math

from .core import footprint_clear


def pose_values(message):
    if message.header.frame_id != 'map':
        raise ValueError('Select a goal in the map frame')
    p, q = message.pose.position, message.pose.orientation
    if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
            abs(q.x) > 1e-4 or abs(q.y) > 1e-4 or
            abs(q.z*q.z+q.w*q.w-1.) > .01):
        raise ValueError('Goal must be a finite planar pose with a unit quaternion')
    return p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


def check_path(grid, poses, start, target):
    if not poses:
        raise ValueError('Planner returned an empty path')
    if math.dist(poses[0][:2], start[:2]) > .2 or math.dist(poses[-1][:2], target[:2]) > .15:
        raise ValueError('Planner path does not connect the requested endpoints')
    samples = [start, *poses, target]
    for a, b in zip(samples, samples[1:]):
        turn = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        steps = max(1, math.ceil(math.dist(a[:2], b[:2])/(grid.resolution/2)),
                    math.ceil(abs(turn)/math.radians(2)))
        for i in range(steps+1):
            t = i/steps
            if not footprint_clear(grid, a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), a[2]+t*turn):
                raise ValueError('Path footprint crosses hazardous or unknown ground at '
                                 f'({a[0]+t*(b[0]-a[0]):.3f}, {a[1]+t*(b[1]-a[1]):.3f}), '
                                 f'heading {math.degrees(a[2]+t*turn):.1f} degrees')


def overlay_hazards(grid, points, padding=.04):
    """Only add lethal cells; never promote unknown terrain to free space."""
    import numpy as np
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    cells = np.floor((points[:, :2]-[grid.x, grid.y])/grid.resolution).astype(int)
    pad = math.ceil(padding/grid.resolution)
    for dy in range(-pad, pad+1):
        for dx in range(-pad, pad+1):
            c, r = cells[:, 0]+dx, cells[:, 1]+dy
            ok = (r >= 0) & (c >= 0) & (r < grid.data.shape[0]) & (c < grid.data.shape[1])
            grid.data[r[ok], c[ok]] = 100
    return grid
