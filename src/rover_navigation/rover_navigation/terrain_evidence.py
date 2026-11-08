"""Conservative reconsideration of weak depth hazards; no command ownership."""
from collections import deque
from dataclasses import asdict, dataclass
import math

import numpy as np

from .site_geometry import pose_covariance


@dataclass(frozen=True)
class ClearingPolicy:
    views: int = 3
    baseline_m: float = .20
    span_seconds: float = 2.
    window_seconds: float = 30.
    max_range_m: float = 3.
    range_improvement_m: float = .25
    range_fraction: float = .80
    max_slope_deg: float = 8.
    max_span_m: float = .015
    min_samples: int = 3
    support_radius_m: float = .10
    elevation_tolerance_m: float = .015
    protected_hazard_views: int = 3


class ClearanceLocalization:
    """Image-time covariance and discontinuity checks used only to permit clearing."""
    def __init__(self):
        self.samples = deque(maxlen=120)
        self.received = -math.inf
        self.fault = ''
        self.valid = False

    def add(self, stamp, position, quaternion, covariance, now):
        if not math.isfinite(stamp) or (self.samples and stamp <= self.samples[-1][0]):
            return  # duplicated/out-of-order receipts cannot refresh evidence health
        self.received = now
        try:
            covariance = pose_covariance(covariance)
            p, q = np.asarray(position, float), np.asarray(quaternion, float)
            if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
                raise ValueError('invalid localization pose')
            if not .9 <= float(q @ q) <= 1.1:
                raise ValueError('invalid localization orientation')
            if self.samples:
                previous = self.samples[-1]
                dt = stamp-previous[0]
                if dt <= 0:
                    self.fault = 'localization clock/reset discontinuity; restart required'
                    raise ValueError(self.fault)
                angle = 2*math.acos(float(np.clip(abs(q @ previous[2])/(np.linalg.norm(q)*np.linalg.norm(previous[2])), 0, 1)))
                if np.linalg.norm(p-previous[1]) > .05+.5*min(dt, 1.) or angle > .10+.6*min(dt, 1.):
                    self.fault = 'localization pose discontinuity; restart required'
                    raise ValueError(self.fault)
            self.samples.append((stamp, p, q, covariance))
            self.valid = True
        except ValueError:
            self.valid = False

    def check(self, stamp, now):
        if self.fault:
            return self.fault
        if not self.valid or now-self.received > .5:
            return 'localization invalid, uncertain or stale'
        before = [sample for sample in self.samples if sample[0] <= stamp]
        after = [sample for sample in self.samples if sample[0] >= stamp]
        if not before or not after or stamp-before[-1][0] > .05 or after[0][0]-stamp > .05:
            return 'image-time localization covariance unavailable'
        return ''


def supported_ground(frame, layers, policy):
    """Require measured dense ground around a cell, never empty-space ray clearing."""
    shape = frame.data.shape
    names = ('elevation', 'slope', 'height_span', 'observation_count')
    if any(name not in layers or np.shape(layers[name]) != shape for name in names):
        return np.zeros(shape, bool)
    elevation, slope, span, count = [np.asarray(layers[name]) for name in names]
    safe = ((frame.data >= 0) & (frame.data < 90) & np.isfinite(elevation) &
            np.isfinite(slope) & (slope >= 0) & (slope <= policy.max_slope_deg) &
            np.isfinite(span) & (span >= 0) & (span <= policy.max_span_m) &
            np.isfinite(count) & (count >= policy.min_samples))
    radius = max(1, math.ceil(policy.support_radius_m/frame.resolution))
    width = 2*radius+1
    # Integral image: every cell in this square must have positive ground support.
    padded = np.pad(safe.astype(np.int32), radius)
    sums = np.pad(padded.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    counts = sums[width:, width:]-sums[:-width, width:]-sums[width:, :-width]+sums[:-width, :-width]
    return counts == width*width


class TerrainEvidence:
    """Three closer, translated views may clear weak hazards. Established ones stay blocked."""
    def __init__(self, memory, policy=None):
        self.policy = policy or ClearingPolicy()
        shape = memory.grid.data.shape
        self.hazard_views = np.zeros(shape, np.uint8)
        self.clear_views = np.zeros(shape, np.uint8)
        self.hazard_positions = np.full((3, *shape, 2), np.nan, np.float32)
        self.clear_positions = np.full((3, *shape, 2), np.nan, np.float32)
        self.closest_hazard = np.full(shape, np.inf, np.float32)
        self.ground = np.full(shape, np.nan, np.float32)
        self.first_clear = np.full(shape, np.nan, np.float64)
        self.protected = memory.grid.data >= 90  # unknown provenance cannot be cleared
        self.last_stamp = -math.inf
        self.total_cleared = 0
        self.events = []
        self.grid = memory.grid  # diagnostic counts only; this is the existing memory
        self.status = dict(enabled=True, suspended='waiting for qualified depth', cleared_cells=0, pending_cells=0)

    def update_counts(self):
        retained = self.grid.data >= 90
        weak = retained & ~self.protected
        self.status.update(cleared_cells=self.total_cleared,
            pending_cells=int((weak & (self.clear_views > 0)).sum()),
            weak_hazard_cells=int(weak.sum()),
            protected_cells=int((retained & self.protected).sum()),
            required_views=self.policy.views,
            max_supporting_views=int(self.clear_views[weak].max()) if weak.any() else 0,
            unknown_cells=int((self.grid.data < 0).sum()))

    @staticmethod
    def independent(positions, camera, baseline):
        distances = np.linalg.norm(positions-np.asarray(camera[:2]), axis=-1)
        return ((distances >= baseline-1e-6) | ~np.isfinite(distances)).all(axis=0)

    def reset_pending(self):
        self.clear_views.fill(0)
        self.clear_positions.fill(np.nan)
        self.first_clear.fill(np.nan)

    def suspend(self, reason, stamp):
        if self.status['suspended'] != reason:
            self.events.append(dict(kind='clearing_suspended', reason=reason, stamp=stamp))
        self.reset_pending()
        self.status.update(suspended=reason, pending_cells=0)
        self.update_counts()

    def observe(self, memory, frame, layers, camera, stamp, suspension=''):
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            return  # replayed/stale frames never add evidence or clear cells
        self.last_stamp = stamp
        if suspension:
            self.suspend(suspension, stamp)
        if not np.isfinite(camera).all():
            suspension = 'invalid camera pose'
            self.reset_pending()
        grid = memory.grid
        if abs(frame.resolution-grid.resolution) > 1e-9:
            raise ValueError('Terrain evidence requires aligned resolutions')
        r0, c0 = round((frame.y-grid.y)/grid.resolution), round((frame.x-grid.x)/grid.resolution)
        rows, cols = frame.data.shape
        fr, fc = max(0, -r0), max(0, -c0)
        re, ce = min(rows, grid.data.shape[0]-r0), min(cols, grid.data.shape[1]-c0)
        if re <= fr or ce <= fc:
            return
        fs = (slice(fr, re), slice(fc, ce))
        ms = (slice(r0+fr, r0+re), slice(c0+fc, c0+ce))
        costs = frame.data[fs]
        hazard = costs >= 90
        hv, cv = self.hazard_views[ms], self.clear_views[ms]
        hp, cp = self.hazard_positions[(slice(None),)+ms], self.clear_positions[(slice(None),)+ms]
        protected, first = self.protected[ms], self.first_clear[ms]
        old_ground, closest = self.ground[ms], self.closest_hazard[ms]
        # Any fresh measured hazard immediately invalidates previous clearing evidence.
        cv[hazard] = 0
        cp[:, hazard] = np.nan
        first[hazard] = np.nan
        if suspension:
            protected[hazard] = True
            self.status.update(suspended=suspension, pending_cells=0)
            self.update_counts()
            return
        rr, cc = np.indices(costs.shape)
        x = frame.x+(cc+fc+.5)*frame.resolution
        y = frame.y+(rr+fr+.5)*frame.resolution
        distance = np.hypot(x-camera[0], y-camera[1])
        elevation = np.asarray(layers.get('elevation', np.full(frame.data.shape, np.nan)))[fs]
        valid_hazard = hazard & np.isfinite(elevation)
        protected[hazard & ~valid_hazard] = True
        closest[valid_hazard] = np.minimum(closest[valid_hazard], distance[valid_hazard])
        old_ground[valid_hazard] = np.fmin(old_ground[valid_hazard], elevation[valid_hazard])
        new_hazard_view = valid_hazard & (hv < 3) & self.independent(hp, camera, self.policy.baseline_m)
        for index in range(3):
            mask = new_hazard_view & (hv == index)
            hp[index, mask] = camera[:2]
        hv[new_hazard_view] += 1
        protected[hv >= self.policy.protected_hazard_views] = True
        expired = np.isfinite(first) & (stamp-first > self.policy.window_seconds)
        cv[expired] = 0
        cp[:, expired] = np.nan
        first[expired] = np.nan
        closer = distance <= np.minimum(closest*self.policy.range_fraction, closest-self.policy.range_improvement_m)
        candidate = ((grid.data[ms] >= 90) & ~protected & (hv > 0) & closer &
                     (distance <= self.policy.max_range_m) & supported_ground(frame, layers, self.policy)[fs] &
                     (np.abs(elevation-old_ground) <= self.policy.elevation_tolerance_m))
        add = candidate & (cv < self.policy.views) & self.independent(cp, camera, self.policy.baseline_m)
        # Repeated frames after three rapid views cannot manufacture a two-second span.
        add &= (cv < self.policy.views-1) | (stamp-first >= self.policy.span_seconds)
        if add.any():
            ar, ac = np.nonzero(add)
            self.events.append(dict(kind='clearing_view', stamp=stamp, camera=list(camera),
                cells=np.column_stack((ar+r0+fr, ac+c0+fc)).tolist(), views=(cv[add]+1).tolist(),
                range_m=distance[add].tolist(), hazard_range_m=closest[add].tolist(),
                elevation_m=elevation[add].tolist(), hazard_ground_m=old_ground[add].tolist(),
                dense_safe_neighborhood=True))
        first[add & (cv == 0)] = stamp
        for index in range(self.policy.views):
            mask = add & (cv == index)
            cp[index, mask] = camera[:2]
        cv[add] += 1
        cleared = candidate & (cv >= self.policy.views) & (stamp-first >= self.policy.span_seconds)
        if cleared.any():
            cell_rows, cell_cols = np.nonzero(cleared)
            indices = np.column_stack((cell_rows+r0+fr, cell_cols+c0+fc))
            self.events.append(dict(kind='hazards_cleared', stamp=stamp, camera=list(camera),
                                    cells=indices.tolist(), independent_views=self.policy.views,
                                    view_positions=cp[:, cleared].transpose(1, 0, 2).tolist()))
            self.total_cleared += len(indices)
            grid.data[ms][cleared] = costs[cleared]
            memory.hits[ms][cleared] = 0
            memory.causes[ms][cleared] = 0
            hv[cleared] = cv[cleared] = 0
            hp[:, cleared] = cp[:, cleared] = np.nan
            closest[cleared] = np.inf
            old_ground[cleared] = first[cleared] = np.nan
        self.status.update(suspended='')
        self.update_counts()

    def configuration(self):
        return asdict(self.policy)
