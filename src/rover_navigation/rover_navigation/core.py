"""Deterministic geometry, traversability and exploration, independent of ROS."""
from dataclasses import dataclass
import heapq
import math

import numpy as np


# Measured in Harmonic (halo_navigation_turn_check.py): the rover turns and
# drives as a differential drive about its front axle, 0.13 m ahead of
# base_link. Navigation plans, checks and controls this point (drive_center).
DRIVE_CENTER_X = .13
# Chassis and wheel envelope about base_link: front, rear, half width.
BODY = (.20, -.20, .17)


def outline(drive_x=DRIVE_CENTER_X):
    """Envelope about the drive center: front, rear, half width."""
    return BODY[0]-drive_x, BODY[1]-drive_x, BODY[2]


FOOTPRINT = outline()


def drive_pose(x, y, yaw, drive_x=DRIVE_CENTER_X):
    """Drive-center pose from a base_link pose."""
    return x+drive_x*math.cos(yaw), y+drive_x*math.sin(yaw), yaw


def turn_radius(footprint=FOOTPRINT, margin=.025):
    """Radius swept by a full in-place turn of the padded envelope."""
    return math.hypot(max(abs(footprint[0]), abs(footprint[1])), footprint[2])+margin


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform(points, translation, quaternion):
    return np.asarray(points) @ rotation(quaternion).T + np.asarray(translation)


def depth_points(data, encoding, width, height, step, bigendian, k, stride=3):
    """Read ROS depth with row padding/endianness and back-project optical XYZ."""
    types = {'32FC1': ('f4', 1.), '16UC1': ('u2', .001)}
    if encoding not in types:
        raise ValueError('Depth must be 32FC1 metres or 16UC1 millimetres')
    kind, scale = types[encoding]
    dtype = np.dtype(('>' if bigendian else '<') + kind)
    if (min(width, height, stride) <= 0 or not np.isfinite(k).all() or
            step < width*dtype.itemsize or len(data) != step*height or min(k[0], k[4]) <= 0):
        raise ValueError('Invalid depth stride, buffer or calibration')
    depth = np.ndarray((height, width), dtype=dtype, buffer=bytes(data), strides=(step, dtype.itemsize))
    v, u = np.mgrid[0:height:stride, 0:width:stride]
    z = depth[::stride, ::stride].astype(float)*scale
    valid = np.isfinite(z) & (z >= .3) & (z <= 8.)
    return np.column_stack(((u[valid]-k[2])*z[valid]/k[0],
                            (v[valid]-k[5])*z[valid]/k[4], z[valid]))


@dataclass
class Grid:
    data: np.ndarray
    resolution: float
    x: float
    y: float

    def cell(self, x, y):
        return int(math.floor((y-self.y)/self.resolution)), int(math.floor((x-self.x)/self.resolution))

    def point(self, row, col):
        return self.x+(col+.5)*self.resolution, self.y+(row+.5)*self.resolution

    def at(self, x, y):
        r, c = self.cell(x, y)
        return int(self.data[r, c]) if 0 <= r < self.data.shape[0] and 0 <= c < self.data.shape[1] else -1


def traversability(points, resolution=.05, size=20., center=(0., 0.), max_slope=12., max_step=.03):
    """Conservative elevation/height-span grid. No ray or hole implies free ground.

    Slopes use opposed observed neighbours across the footprint scale, or the one
    observed neighbour at the edge of a measurement (a single frame's near edge).
    A point cluster without support remains unknown. Vertical spans are obstacles.
    """
    if not all(math.isfinite(v) and v > 0 for v in (resolution, size, max_slope, max_step)):
        raise ValueError('Terrain dimensions and hazard thresholds must be positive and finite')
    n = int(round(size/resolution))
    origin = np.floor((np.asarray(center)-size/2)/resolution)*resolution
    low, high = np.full((n, n), np.inf), np.full((n, n), -np.inf)
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    cells = np.floor((p[:, :2]-origin)/resolution).astype(int)
    valid = (cells >= 0).all(axis=1) & (cells < n).all(axis=1)
    c, z = cells[valid], p[valid, 2]
    index = c[:, 1]*n+c[:, 0]
    np.minimum.at(low.ravel(), index, z)
    np.maximum.at(high.ravel(), index, z)
    # Repeated depth history contains millions of duplicate cell indices.
    # Bincount accumulates the same sufficient statistics in compiled loops.
    counts = np.bincount(index, minlength=n*n).reshape(n, n)
    sums = np.bincount(index, weights=z, minlength=n*n).reshape(n, n)
    squares = np.bincount(index, weights=z*z, minlength=n*n).reshape(n, n)
    roughness = np.sqrt(np.maximum(0., squares/np.maximum(1, counts)-(sums/np.maximum(1, counts))**2))
    roughness[counts == 0] = np.nan
    observed = np.isfinite(low)
    elevation = np.where(observed, low, np.nan)
    radius = max(1, round(.10/resolution))
    padded = np.pad(elevation, radius, constant_values=np.nan)

    def gradient(dr, dc):
        ahead = padded[radius+dr:radius+dr+n, radius+dc:radius+dc+n]
        behind = padded[radius-dr:radius-dr+n, radius-dc:radius-dc+n]
        central = (ahead-behind)/(2*radius*resolution)
        one_sided = np.where(np.isfinite(ahead), ahead-elevation, elevation-behind)/(radius*resolution)
        return np.where(np.isfinite(central), central, one_sided)

    slope = np.degrees(np.arctan(np.hypot(gradient(0, radius), gradient(radius, 0))))
    span = np.where(observed, high-low, np.nan)
    safe = observed & np.isfinite(slope) & (slope <= max_slope) & (span <= max_step)
    costs = np.full((n, n), -1, dtype=np.int8)
    costs[safe] = np.minimum(80, np.round(60*slope[safe]/max_slope+20*roughness[safe]/max_step)).astype(np.int8)
    # A measured vertical column is hazardous even when nearby ground is missing.
    costs[observed & ((span > max_step) | (slope > max_slope))] = 100
    return Grid(costs, resolution, *origin), {'elevation': elevation, 'slope': slope, 'height_span': span,
                                             'roughness': roughness, 'observation_count': counts}


def aligned_copy(source, like):
    """Values of `source` on `like`'s cells; both grids share resolution and cell boundaries."""
    if abs(source.resolution-like.resolution) > 1e-9:
        raise ValueError('Aligned copies require equal resolution')
    out = Grid(np.full(like.data.shape, -1, np.int8), like.resolution, like.x, like.y)
    r0 = int(round((like.y-source.y)/source.resolution))
    c0 = int(round((like.x-source.x)/source.resolution))
    rows, cols = like.data.shape
    rs, cs = max(0, -r0), max(0, -c0)
    re, ce = min(rows, source.data.shape[0]-r0), min(cols, source.data.shape[1]-c0)
    if rs < re and cs < ce:
        out.data[rs:re, cs:ce] = source.data[r0+rs:r0+re, c0+cs:c0+ce]
    return out


class TerrainMemory:
    """Per-frame terrain classifications fused in one fixed frame.

    Height span and slope are judged within a single depth frame, so pose
    inconsistency between frames cannot create steps. A hazard is retained
    once distinct frames confirm it, and free readings never clear it.
    """

    def __init__(self, size=20., resolution=.025, confirmations=2):
        n = int(round(size/resolution))
        origin = math.floor(-size/2/resolution)*resolution
        self.grid = Grid(np.full((n, n), -1, np.int8), resolution, origin, origin)
        self.hits = np.zeros((n, n), np.uint8)
        self.causes = np.zeros((n, n), np.int8)  # diagnostic evidence only; never a safety input
        self.confirmations = max(1, int(confirmations))

    def remember(self, frame, reasons=None):
        known = aligned_copy(frame, self.grid).data
        hazard = known >= 90
        reason = aligned_copy(reasons, self.grid).data if reasons is not None else np.full(known.shape, 8, np.int8)
        self.causes[hazard] |= np.where(reason[hazard] > 0, reason[hazard], 8).astype(np.int8)
        self.hits[hazard] = np.minimum(self.hits[hazard], 254)+1
        safe = (known >= 0) & ~hazard & (self.grid.data < 90)
        self.grid.data[safe] = known[safe]
        self.grid.data[self.hits >= self.confirmations] = 100

    def reasons(self):
        return Grid(np.where(self.grid.data >= 90, self.causes,
                             np.where(self.grid.data >= 0, 0, -1)).astype(np.int8),
                    self.grid.resolution, self.grid.x, self.grid.y)

    def values(self, x, y, grid=None):
        g = self.grid if grid is None else grid
        cols = np.floor((x-g.x)/g.resolution).astype(int)
        rows = np.floor((y-g.y)/g.resolution).astype(int)
        inside = (rows >= 0) & (rows < g.data.shape[0]) & (cols >= 0) & (cols < g.data.shape[1])
        values = np.full(x.shape, -1, dtype=int)
        values[inside] = g.data[rows[inside], cols[inside]]
        return values

    def project(self, to_memory, size, resolution, reasons=False):
        """Resample through a rigid transform: hazard if any sub-sample is, known only if all are."""
        n = int(round(size/resolution))
        origin = math.floor(-size/2/resolution)*resolution
        centres = origin+(np.arange(n)+.5)*resolution
        x, y = np.meshgrid(centres, centres)
        samples, causes = [], []
        reason_grid = self.reasons() if reasons else None
        for dx in (-.25, .25):
            for dy in (-.25, .25):
                points = np.column_stack([(x+dx*resolution).ravel(), (y+dy*resolution).ravel(), np.zeros(n*n)])
                moved = to_memory(points)
                samples.append(self.values(moved[:, 0], moved[:, 1]))
                if reasons:
                    causes.append(self.values(moved[:, 0], moved[:, 1], reason_grid))
        samples = np.array(samples)
        data = np.where((samples >= 90).any(axis=0), 100,
                        np.where((samples >= 0).all(axis=0), samples.max(axis=0), -1))
        grid = Grid(data.reshape(n, n).astype(np.int8), resolution, origin, origin)
        if reasons:
            flags = np.bitwise_or.reduce(np.where(samples >= 90, causes, 0).astype(np.int8), axis=0)
            data = np.where(data >= 90, flags, np.where(data >= 0, 0, -1))
            return grid, Grid(data.reshape(n, n).astype(np.int8), resolution, origin, origin)
        return grid


def terrain_reasons(grid, layers, max_slope=12., max_step=.03):
    """Explain the actual classification: -1 unknown, 0 clear, 2 slope, 4 span, 8 unspecified hazard."""
    flags = (2*(layers['slope'] > max_slope) | 4*(layers['height_span'] > max_step)).astype(np.int8)
    return Grid(np.where(grid.data >= 90, np.where(flags > 0, flags, 8),
                         np.where(grid.data >= 0, 0, -1)).astype(np.int8), grid.resolution, grid.x, grid.y)


def cause_names(code):
    if code < 0:
        return ['unknown_terrain']
    names = [name for bit, name in ((2, 'excessive_slope'), (4, 'excessive_height_change'),
                                    (8, 'hazard_evidence_unavailable')) if code & bit]
    return names or ['traversable']


def footprint_polygon(pose, margin=.025):
    x, y, yaw = pose
    front, rear, half = FOOTPRINT
    return [[x+dx*math.cos(yaw)-dy*math.sin(yaw), y+dx*math.sin(yaw)+dy*math.cos(yaw)]
            for dx, dy in ((front+margin, half+margin), (rear-margin, half+margin),
                           (rear-margin, -half-margin), (front+margin, -half-margin))]


def samples(grid, footprint, margin):
    front, rear, half = footprint
    return np.meshgrid(np.arange(rear-margin, front+margin+grid.resolution/2, grid.resolution/2),
                       np.arange(-half-margin, half+margin+grid.resolution/2, grid.resolution/2))


def footprint_clear(grid, x, y, yaw, margin=.025, footprint=FOOTPRINT):
    """Sample the full chassis/wheel envelope about the drive center, including its interior."""
    dx, dy = samples(grid, footprint, margin)
    cols = np.floor((x+dx*math.cos(yaw)-dy*math.sin(yaw)-grid.x)/grid.resolution).astype(int)
    rows = np.floor((y+dx*math.sin(yaw)+dy*math.cos(yaw)-grid.y)/grid.resolution).astype(int)
    if not ((rows >= 0) & (rows < grid.data.shape[0]) & (cols >= 0) & (cols < grid.data.shape[1])).all():
        return False
    values = grid.data[rows, cols]
    return bool(((values >= 0) & (values < 90)).all())


def footprint_conflict(grid, x, y, yaw, margin=.025, footprint=FOOTPRINT):
    """First footprint sample that footprint_clear rejects, as JSON-safe evidence."""
    dx, dy = samples(grid, footprint, margin)
    px = (x+dx*math.cos(yaw)-dy*math.sin(yaw)).ravel()
    py = (y+dx*math.sin(yaw)+dy*math.cos(yaw)).ravel()
    cols = np.floor((px-grid.x)/grid.resolution).astype(int)
    rows = np.floor((py-grid.y)/grid.resolution).astype(int)
    inside = (rows >= 0) & (rows < grid.data.shape[0]) & (cols >= 0) & (cols < grid.data.shape[1])
    values = np.full(px.shape, -1, dtype=int)
    values[inside] = grid.data[rows[inside], cols[inside]]
    bad = np.flatnonzero(~inside | (values < 0) | (values >= 90))
    if not len(bad):
        return None
    i = bad[0]
    return dict(x=float(px[i]), y=float(py[i]), cell=int(values[i]) if inside[i] else None)


def swept_poses(pose, v, w, horizon=1.):
    x, y, yaw = pose
    for _ in range(11):
        yield x, y, yaw
        x += v*math.cos(yaw)*horizon/10
        y += v*math.sin(yaw)*horizon/10
        yaw += w*horizon/10


def swept_clear(grid, pose, v, w, horizon=1.):
    return all(footprint_clear(grid, *p) for p in swept_poses(pose, v, w, horizon))


def swept_conflict(grid, pose, v, w, horizon=1.):
    for step, p in enumerate(swept_poses(pose, v, w, horizon)):
        conflict = footprint_conflict(grid, *p)
        if conflict is not None:
            return dict(step=step, pose=[float(value) for value in p], **conflict)
    return None


def reachable(grid, start, radius=None):
    """Dijkstra over known ground with full-turn clearance and no corner cutting."""
    radius = turn_radius() if radius is None else radius
    free = (grid.data >= 0) & (grid.data < 90)
    pad = int(math.ceil(radius/grid.resolution))
    blocked = np.pad(~free, pad, constant_values=True)
    allowed = free.copy()
    for dy in range(-pad, pad+1):
        for dx in range(-pad, pad+1):
            if math.hypot(dx, dy)*grid.resolution <= radius+grid.resolution:
                allowed &= ~blocked[pad+dy:pad+dy+free.shape[0], pad+dx:pad+dx+free.shape[1]]
    r, c = grid.cell(*start)
    distances = np.full(free.shape, np.inf)
    if not (0 <= r < free.shape[0] and 0 <= c < free.shape[1] and allowed[r, c]):
        return distances, allowed
    distances[r, c] = 0.
    queue = [(0., r, c)]
    while queue:
        cost, r, c = heapq.heappop(queue)
        if cost > distances[r, c]:
            continue
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < free.shape[0] and 0 <= nc < free.shape[1] and allowed[nr, nc]:
                value = cost+grid.resolution*(1+grid.data[nr, nc]/100)
                if value < distances[nr, nc]:
                    distances[nr, nc] = value
                    heapq.heappush(queue, (value, nr, nc))
    return distances, allowed


def frontiers(grid, start, excluded=()):
    """Reachable observation goals set back from unknown ground by the footprint."""
    distances, allowed = reachable(grid, start)
    unknown = grid.data < 0
    near_unknown = np.zeros_like(unknown)
    radius = int(math.ceil(.45/grid.resolution))
    # Outside the configured mapping area is a boundary, not an exploration goal.
    padded = np.pad(unknown, radius, constant_values=False)
    for dr, dc in ((radius, 0), (-radius, 0), (0, radius), (0, -radius)):
        near_unknown |= padded[radius+dr:radius+dr+unknown.shape[0], radius+dc:radius+dc+unknown.shape[1]]
    candidates = np.argwhere(allowed & near_unknown & np.isfinite(distances) & (distances > .5))
    goals = []
    for r, c in sorted(candidates, key=lambda rc: distances[tuple(rc)]):
        x, y = grid.point(r, c)
        if any(math.hypot(x-a, y-b) < .65 for a, b in (*excluded, *[(g[0], g[1]) for g in goals])):
            continue
        directions = [(dc, dr) for dr, dc in ((radius, 0), (-radius, 0), (0, radius), (0, -radius))
                      if padded[radius+r+dr, radius+c+dc]]
        vx, vy = np.sum(directions, axis=0) if directions else (0., 0.)
        yaw = math.atan2(vy, vx) if vx or vy else math.atan2(y-start[1], x-start[0])
        goals.append((x, y, yaw, float(distances[r, c])))
        if len(goals) >= 16:
            break
    return goals


def return_required(elapsed, budget, path_length, speed=.2):
    return not math.isfinite(path_length) or budget-elapsed <= 1.5*path_length/speed+10.
