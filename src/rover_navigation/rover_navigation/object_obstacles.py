"""Project confirmed raised objects into navigation grids without clearing terrain."""
import math

import numpy as np


SOLID_CLASSES = frozenset(('rock', 'yellow_crystal_rock', 'mudstone_slab'))
OBJECT_REASON = 16
OBJECT_CLEARANCE_M = .10


def object_radius(track):
    extent = np.asarray(track['extent'], float)
    if extent.shape != (3,) or not np.isfinite(extent).all() or np.any(extent <= 0):
        raise ValueError('Invalid object extent')
    return max(.08, .5 * math.hypot(*extent[:2])) + OBJECT_CLEARANCE_M


class ObjectObstacles:
    """Retain the last valid obstacles through missing or ambiguous observations."""

    def __init__(self):
        self.epoch = None
        self.stamp = -math.inf
        self.disks = {}

    def update(self, report):
        if report.get('schema_version') != 2 or report.get('frame') != 'odom' or not report.get('ready'):
            return
        epoch, stamp = report['epoch'], report['accepted_stamp']
        if (type(epoch) is not int or epoch < 0 or not math.isfinite(stamp)
                or self.epoch is not None and epoch < self.epoch
                or epoch == self.epoch and stamp <= self.stamp):
            return
        disks = {}
        for track in report['objects']:
            if track['state'] != 'confirmed' or track['class_name'] not in SOLID_CLASSES:
                continue
            ident = track['track_id']
            point = np.asarray(track['odom_position'], float)
            cov = np.asarray(track['covariance'], float)
            if (track['epoch'] != epoch or not isinstance(ident, str)
                    or point.shape != (3,) or not np.isfinite(point).all()
                    or cov.shape != (3, 3) or not np.isfinite(cov).all()):
                raise ValueError('Invalid tracked obstacle')
            planar = cov[:2, :2]
            if not np.allclose(planar, planar.T) or np.linalg.eigvalsh(planar)[0] < 0.:
                raise ValueError('Invalid obstacle covariance')
            radius = object_radius(track) + 2. * math.sqrt(np.linalg.eigvalsh(planar)[-1])
            disks[ident] = (point, radius)
        if epoch == self.epoch:
            for track in report['tentative_objects']:
                ident = track.get('track_id')
                if ident in self.disks and ident not in disks:
                    disks[ident] = self.disks[ident]
        self.epoch, self.stamp, self.disks = epoch, stamp, disks

    def apply(self, grid, reasons, transform=None):
        if not self.disks:
            return
        entries = list(self.disks.values())
        points = [p for p, _ in entries]
        if transform is not None:
            points = transform(points)
        width, height = grid.data.shape[1], grid.data.shape[0]
        for point, (_, radius) in zip(points, entries):
            x, y = point[:2]
            radius += grid.resolution / math.sqrt(2.)  # cells touching the disk
            top, left = grid.cell(x - radius, y - radius)
            bottom, right = grid.cell(x + radius, y + radius)
            x0, x1 = max(0, left), min(width, right + 1)
            y0, y1 = max(0, top), min(height, bottom + 1)
            if x0 >= x1 or y0 >= y1:
                continue
            xx = grid.x + (np.arange(x0, x1) + .5) * grid.resolution
            yy = grid.y + (np.arange(y0, y1) + .5) * grid.resolution
            cells = grid.data[y0:y1, x0:x1]
            blocked = ((xx[None, :] - x)**2 + (yy[:, None] - y)**2 <= radius**2) & (cells >= 0)
            cells[blocked] = 100
            why = reasons.data[y0:y1, x0:x1]
            why[blocked] = np.maximum(0, why[blocked]) | OBJECT_REASON
