"""ROS-independent CPU reference and AMD ROCm terrain kernels.

Outputs retain the reference mapper's float64 geometry, unknown cells and
conservative four-sample map projection. Torch is imported only for ROCm.
"""
import math
import time
import numpy as np

from .core import Grid, TerrainMemory, depth_points, rotation, transform, traversability

MAX_GRID_SIDE = 1600


def depth_array(data, encoding, width, height, step, bigendian, k, stride=3):
    kinds = {'32FC1': ('f4', 1.), '16UC1': ('u2', .001)}
    if encoding not in kinds:
        raise ValueError('Depth must be 32FC1 metres or 16UC1 millimetres')
    kind, scale = kinds[encoding]
    dtype = np.dtype(('>' if bigendian else '<')+kind)
    if (min(width, height, stride) <= 0 or len(k) != 9 or not np.isfinite(k).all()
            or step < width*dtype.itemsize or len(data) != step*height or min(k[0], k[4]) <= 0):
        raise ValueError('Invalid depth stride, buffer or calibration')
    image = np.ndarray((height, width), dtype=dtype, buffer=bytes(data), strides=(step, dtype.itemsize))
    return np.ascontiguousarray(image[::stride, ::stride], dtype=np.float64)*scale


def dimensions(resolution, size, center, max_slope, max_step):
    if (not all(math.isfinite(v) and v > 0 for v in (resolution, size, max_slope, max_step))
            or len(center) != 2 or not np.isfinite(center).all()):
        raise ValueError('Invalid terrain dimensions, center or thresholds')
    n = int(round(size/resolution))
    if not 1 <= n <= MAX_GRID_SIDE:
        raise ValueError('Terrain grid must have 1..1600 cells per dimension')
    return n, np.floor((np.asarray(center)-size/2)/resolution)*resolution


class CpuTerrainCompute:
    metadata = dict(backend='cpu', device='cpu')

    def classify(self, points, **kw):
        return traversability(points, **kw)

    def depth(self, data, encoding, width, height, step, bigendian, k,
              translation, quaternion, stride=3, **kw):
        points = depth_points(data, encoding, width, height, step, bigendian, k, stride)
        grid, layers = self.classify(transform(points, translation, quaternion), **kw)
        return grid, layers, len(points)

    def project(self, memory, translation, quaternion, size, resolution):
        return memory.project(lambda p: transform(p, translation, quaternion), size, resolution, reasons=True)


class RocmTerrainCompute:
    def __init__(self):
        import torch
        if not torch.version.hip or not torch.cuda.is_available():
            raise RuntimeError('AMD ROCm GPU is required; CPU fallback is disabled')
        torch.set_num_threads(1)
        self.torch, self.device = torch, torch.device('cuda:0')
        self.metadata = dict(backend='rocm', device=torch.cuda.get_device_name(0),
                             torch=torch.__version__, hip=torch.version.hip, geometry='float64')
        self.last_ms = None

    def tensor(self, array, dtype=None):
        array = np.ascontiguousarray(array)
        if not array.flags.writeable:
            array = array.copy()
        return self.torch.as_tensor(array, dtype=dtype or self.torch.float64,
                                    device=self.device)

    def moved(self, points, translation, quaternion):
        if len(translation) != 3 or not np.isfinite(translation).all():
            raise ValueError('Invalid translation')
        r = self.tensor(rotation(quaternion))
        # Explicit arithmetic avoids different BLAS precision/TF32 policies.
        return (points[:, 0:1]*r[:, 0]+points[:, 1:2]*r[:, 1]+
                points[:, 2:3]*r[:, 2]+self.tensor(translation))

    def classify(self, points, **kw):
        with self.torch.inference_mode():
            return self._classify(self.tensor(np.asarray(points, dtype=float).reshape(-1, 3)), **kw)

    def _classify(self, p, resolution=.05, size=20., center=(0., 0.), max_slope=12., max_step=.03):
        t = self.torch
        n, origin = dimensions(resolution, size, center, max_slope, max_step)
        p = p[t.isfinite(p).all(dim=1)]
        cells = t.floor((p[:, :2]-self.tensor(origin))/resolution).to(t.int64)
        valid = (cells >= 0).all(dim=1) & (cells < n).all(dim=1)
        cells, z = cells[valid], p[valid, 2]
        index = cells[:, 1]*n+cells[:, 0]
        def full(value):
            return t.full((n*n,), value, dtype=t.float64, device=self.device)
        low, high = full(float('inf')), full(-float('inf'))
        low.scatter_reduce_(0, index, z, reduce='amin', include_self=True)
        high.scatter_reduce_(0, index, z, reduce='amax', include_self=True)
        counts = t.zeros(n*n, dtype=t.int64, device=self.device)
        counts.scatter_add_(0, index, t.ones_like(index))
        sums, squares = full(0.), full(0.)
        sums.scatter_add_(0, index, z)
        squares.scatter_add_(0, index, z*z)
        low, high, counts, sums, squares = [v.reshape(n, n) for v in (low, high, counts, sums, squares)]
        denominator = counts.clamp_min(1)
        roughness = t.sqrt((squares/denominator-(sums/denominator)**2).clamp_min(0.))
        roughness[counts == 0] = float('nan')
        observed = t.isfinite(low)
        elevation = t.where(observed, low, float('nan'))
        radius = max(1, round(.10/resolution))
        padded = t.nn.functional.pad(elevation, (radius,)*4, value=float('nan'))
        def gradient(dr, dc):
            ahead = padded[radius+dr:radius+dr+n, radius+dc:radius+dc+n]
            behind = padded[radius-dr:radius-dr+n, radius-dc:radius-dc+n]
            central = (ahead-behind)/(2*radius*resolution)
            one = t.where(t.isfinite(ahead), ahead-elevation, elevation-behind)/(radius*resolution)
            return t.where(t.isfinite(central), central, one)
        slope = t.rad2deg(t.atan(t.hypot(gradient(0, radius), gradient(radius, 0))))
        span = t.where(observed, high-low, float('nan'))
        safe = observed & t.isfinite(slope) & (slope <= max_slope) & (span <= max_step)
        costs = t.full((n, n), -1, dtype=t.int8, device=self.device)
        costs[safe] = t.round(60*slope[safe]/max_slope+20*roughness[safe]/max_step).clamp_max(80).to(t.int8)
        costs[observed & ((span > max_step) | (slope > max_slope))] = 100
        layers = dict(elevation=elevation, slope=slope, height_span=span,
                      roughness=roughness, observation_count=counts)
        return Grid(costs.cpu().numpy(), resolution, *origin), {k:v.cpu().numpy() for k,v in layers.items()}

    def depth(self, data, encoding, width, height, step, bigendian, k,
              translation, quaternion, stride=3, **kw):
        t = self.torch
        with t.inference_mode():
            z = self.tensor(depth_array(data, encoding, width, height, step, bigendian, k, stride))
            v, u = t.meshgrid(t.arange(0, height, stride, device=self.device, dtype=t.float64),
                              t.arange(0, width, stride, device=self.device, dtype=t.float64), indexing='ij')
            valid = t.isfinite(z) & (z >= .3) & (z <= 8.)
            z, u, v = z[valid], u[valid], v[valid]
            p = t.stack(((u-k[2])*z/k[0], (v-k[5])*z/k[4], z), dim=1)
            grid, layers = self._classify(self.moved(p, translation, quaternion), **kw)
            return grid, layers, p.shape[0]

    def project(self, memory, translation, quaternion, size, resolution):
        t = self.torch
        n, _ = dimensions(resolution, size, (0., 0.), 12., .03)
        origin = math.floor(-size/2/resolution)*resolution
        with t.inference_mode():
            centres = origin+(t.arange(n, dtype=t.float64, device=self.device)+.5)*resolution
            y, x = t.meshgrid(centres, centres, indexing='ij')
            source = self.tensor(memory.grid.data, t.int8)
            reasons = self.tensor(memory.reasons().data, t.int8)
            samples, causes = [], []
            for dx in (-.25, .25):
                for dy in (-.25, .25):
                    p = t.stack(((x+dx*resolution).ravel(), (y+dy*resolution).ravel(),
                                 t.zeros(n*n, dtype=t.float64, device=self.device)), dim=1)
                    moved = self.moved(p, translation, quaternion)
                    cols = t.floor((moved[:, 0]-memory.grid.x)/memory.grid.resolution).to(t.int64)
                    rows = t.floor((moved[:, 1]-memory.grid.y)/memory.grid.resolution).to(t.int64)
                    inside = (rows >= 0) & (cols >= 0) & (rows < source.shape[0]) & (cols < source.shape[1])
                    values = t.full((n*n,), -1, dtype=t.int8, device=self.device)
                    flags = values.clone()
                    values[inside], flags[inside] = source[rows[inside], cols[inside]], reasons[rows[inside], cols[inside]]
                    samples.append(values); causes.append(flags)
            samples = t.stack(samples)
            hazard = (samples >= 90).any(dim=0)
            data = t.where(hazard, 100, t.where((samples >= 0).all(dim=0), samples.max(dim=0).values, -1)).to(t.int8)
            flags = t.zeros(n*n, dtype=t.int8, device=self.device)
            for i in range(4):
                flags |= t.where(samples[i] >= 90, causes[i], 0)
            flags = t.where(hazard, flags, t.where(data >= 0, 0, -1)).to(t.int8)
            return (Grid(data.reshape(n,n).cpu().numpy(), resolution, origin, origin),
                    Grid(flags.reshape(n,n).cpu().numpy(), resolution, origin, origin))
