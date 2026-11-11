"""Measured provenance, independent of seeded clearance and navigation costs."""
import math
import numpy as np
from .core import Grid


class MeasuredCoverage:
    def __init__(self, size=40., resolution=.025):
        n = round(size/resolution)
        self.size, self.resolution, self.origin = size, resolution, -size/2
        self.seen = np.zeros((n, n), bool)
        self.last_stamp = -math.inf

    def remember(self, frame, stamp):
        if stamp <= self.last_stamp:
            return
        if abs(frame.resolution-self.resolution) > 1e-9:
            raise ValueError('Coverage must use the measured frame resolution')
        self.last_stamp = stamp
        r = round((frame.y-self.origin)/self.resolution)
        c = round((frame.x-self.origin)/self.resolution)
        r0, c0 = max(0, r), max(0, c)
        r1, c1 = min(self.seen.shape[0], r+frame.data.shape[0]), min(self.seen.shape[1], c+frame.data.shape[1])
        if r1 > r0 and c1 > c0:
            self.seen[r0:r1, c0:c1] |= frame.data[r0-r:r1-r, c0-c:c1-c] >= 0

    def project(self, like, to_odom_translation, to_odom_rotation):
        yy = like.y+(np.arange(like.data.shape[0])+.5)*like.resolution
        xx = like.x+(np.arange(like.data.shape[1])+.5)*like.resolution
        r = np.asarray(to_odom_rotation)
        t = np.asarray(to_odom_translation)
        seen = np.ones(like.data.shape, bool)
        for dx in (-.25, .25):
            for dy in (-.25, .25):
                x = r[0, 0]*(xx[None, :]+dx*like.resolution)+r[0, 1]*(yy[:, None]+dy*like.resolution)+t[0]
                y = r[1, 0]*(xx[None, :]+dx*like.resolution)+r[1, 1]*(yy[:, None]+dy*like.resolution)+t[1]
                c = np.floor((x-self.origin)/self.resolution).astype(int)
                row = np.floor((y-self.origin)/self.resolution).astype(int)
                inside = (c >= 0) & (row >= 0) & (c < self.seen.shape[1]) & (row < self.seen.shape[0])
                supported = np.zeros_like(seen)
                supported[inside] = self.seen[row[inside], c[inside]]
                seen &= supported
        measured = Grid(np.where(seen, like.data, -1).astype(np.int8), like.resolution, like.x, like.y)
        area = like.resolution**2
        total = like.data.size*area
        count = int((measured.data >= 0).sum())
        return measured, dict(measured_area_m2=count*area, operating_area_m2=total,
            measured_percent=100*count*area/total, measured_safe_m2=int(((measured.data >= 0) & (measured.data < 90)).sum())*area,
            measured_hazard_m2=int((measured.data >= 90).sum())*area,
            seeded_only_m2=int(((like.data >= 0) & ~seen).sum())*area,
            unknown_m2=int((like.data < 0).sum())*area, provenance='accepted_depth_supported_cells_only')
