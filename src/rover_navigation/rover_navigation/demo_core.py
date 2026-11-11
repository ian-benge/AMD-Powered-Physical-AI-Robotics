"""ROS-free, monotonic-time mission policy. No truth inputs or motor ownership."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import secrets

import numpy as np

from .core import Grid, reachable, footprint_clear, turn_radius
from .goal_checks import check_path


@dataclass(frozen=True)
class DemoPolicy:
    total_seconds: float = 1800.
    exploration_seconds: float = 900.
    selection_seconds: float = 300.
    sampling_seconds: float = 480.
    collection_seconds: float = 10.
    speed: float = .10
    angular_speed: float = .30
    heartbeat_seconds: float = 1.
    preview_seconds: float = 30.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('Demo limits must be finite and positive')
        if (self.total_seconds > 1800 or self.speed > .1 or self.angular_speed > .3 or
                self.heartbeat_seconds > 1 or self.preview_seconds > 30 or
                self.collection_seconds != 10 or
                self.exploration_seconds+self.selection_seconds+self.sampling_seconds > self.total_seconds):
            raise ValueError('Demo limits exceed the approved envelope')

    @classmethod
    def rehearsal(cls, speed=.05):
        return cls(total_seconds=720., exploration_seconds=360., selection_seconds=60.,
                   sampling_seconds=240., speed=speed)


ACTIVE = frozenset(('sweeping', 'exploring', 'exploration_return', 'sample_outbound',
                    'collecting', 'sample_return'))
TRAIL_PHASES = ('exploring', 'exploration_return', 'sample_outbound', 'sample_return')


def path_length(path):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))


def return_reserve(path, speed, rtf=1.):
    if not path or not math.isfinite(rtf) or rtf <= 0:
        return math.inf
    turns = sum(abs(math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2])))
                for a, b in zip(path, path[1:]))
    return 1.5*(path_length(path)/speed+turns/.3)/min(1., rtf)+30.


def observation_frontiers(grid, pose, attempted=(), max_goals=16):
    """Cluster reachable observation poses; estimate gain, never mark unknown free.

    Uses known/hazard occupancy for conservative 2-D ray occlusion. Gain is a
    heuristic (not a promise of future visible terrain or a coverage denominator).
    """
    distances, allowed = reachable(grid, pose[:2])
    unknown = grid.data < 0
    near = np.zeros_like(unknown)
    step = max(1, round(.45/grid.resolution))
    padded = np.pad(unknown, step, constant_values=False)
    for dr, dc in ((step, 0), (-step, 0), (0, step), (0, -step)):
        near |= padded[step+dr:step+dr+unknown.shape[0], step+dc:step+dc+unknown.shape[1]]
    cells = set(map(tuple, np.argwhere(allowed & near & np.isfinite(distances) & (distances > .5))))
    clusters = []
    while cells:
        first = min(cells)
        cells.remove(first)
        component, pending = [first], [first]
        while pending:
            r, c = pending.pop()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                point = (r+dr, c+dc)
                if point in cells:
                    cells.remove(point); pending.append(point); component.append(point)
        clusters.append(component)
    goals = []
    # Bounded candidates keep a large map's frontier search away from the guard.
    for component in sorted(clusters, key=lambda v: min(distances[p] for p in v))[:64]:
        representatives = sorted(component, key=lambda p: distances[p])
        for r, c in representatives:
            x, y = grid.point(r, c)
            if any(math.hypot(x-a, y-b) < .65 for a, b in attempted):
                continue
            neighborhood = np.argwhere(unknown[max(0, r-step):r+step+1, max(0, c-step):c+step+1])
            if not len(neighborhood):
                continue
            center = neighborhood.mean(axis=0)+[max(0, r-step), max(0, c-step)]
            yaw = math.atan2(center[0]-r, center[1]-c)
            observed = set()
            for angle in np.linspace(yaw-.52, yaw+.52, 17):
                for distance in np.arange(.3, 8.01, max(.1, grid.resolution)):
                    nr, nc = grid.cell(x+distance*math.cos(angle), y+distance*math.sin(angle))
                    if not (0 <= nr < unknown.shape[0] and 0 <= nc < unknown.shape[1]):
                        break
                    if grid.data[nr, nc] >= 90:
                        break
                    if unknown[nr, nc]:
                        observed.add((nr//4, nc//4))
            gain = len(observed)*(4*grid.resolution)**2
            turn = abs(math.atan2(math.sin(yaw-pose[2]), math.cos(yaw-pose[2])))
            goals.append(dict(pose=[x, y, yaw], expected_gain_m2=gain,
                              distance_m=float(distances[r, c]),
                              score=gain/(1.+float(distances[r, c])/.1+turn/.3)))
            break
    return sorted(goals, key=lambda g: (-g['score'], g['distance_m'], g['pose']))[:max_goals]


def collection_poses(grid, point, envelope_radius=0., covariance=None):
    """Eight poses outside an uncertainty-expanded site and full turning circle."""
    point = np.asarray(point, float)
    if point.shape != (2,) or not np.isfinite(point).all() or grid.at(*point) < 0 or grid.at(*point) >= 90:
        raise ValueError('Select measured, traversable target ground')
    uncertainty = 0.
    if covariance is not None:
        c = np.asarray(covariance, float)
        if c.shape != (3, 3) or not np.isfinite(c).all():
            raise ValueError('Invalid target covariance')
        uncertainty = 2*math.sqrt(max(0., np.linalg.eigvalsh(c[:2, :2]).max()))
    if not math.isfinite(envelope_radius) or envelope_radius < 0:
        raise ValueError('Invalid target envelope')
    radius = max(1.5, envelope_radius+uncertainty+turn_radius()+.30)
    poses = []
    for angle in np.linspace(0., 2*math.pi, 8, endpoint=False):
        x, y = point+radius*np.array([math.cos(angle), math.sin(angle)])
        yaw = math.atan2(point[1]-y, point[0]-x)
        if all(footprint_clear(grid, x, y, a) for a in np.linspace(-math.pi, math.pi, 37)):
            poses.append([float(x), float(y), yaw])
    return poses


class DemoRun:
    """Authoritative phase/approval/collection fencing; external adapter plans/drives."""
    def __init__(self, session, policy=None):
        self.session, self.policy = session, policy or DemoPolicy()
        self.mission_id = secrets.token_hex(16)
        self.phase, self.reason = 'ready', ''
        self.home = self.target = self.preview = self.sample = None
        self.epoch = 0
        self.generation = 0
        self.started = self.phase_started = None
        self.owner = None
        self.heartbeat_wall = -math.inf
        self.heartbeat_sequence = -1
        self.preview_wall = -math.inf
        self.collection_started = None
        self.low_gain_visits = 0
        self.frontier_visits = []
        self.baseline_area = self.visit_area = 0.
        self.events = []

    def event(self, now, kind, **extra):
        row=dict(kind=kind, wall_monotonic=now, mission_id=self.mission_id, phase=self.phase)
        row.update(extra)
        self.events.append(row)

    def transition(self, phase, now, reason=''):
        self.phase, self.phase_started, self.reason = phase, now, reason
        self.event(now, 'transition', reason=reason)

    def verify(self, session, mission, owner, epoch):
        if (session != self.session or mission != self.mission_id or epoch != self.epoch or
                self.owner is not None and owner != self.owner):
            raise ValueError('Stale session, epoch, mission or operator token')
        if not isinstance(owner, str) or not 16 <= len(owner) <= 128:
            raise ValueError('An operator-owned token is required')

    def heartbeat(self, now, sequence):
        if sequence <= self.heartbeat_sequence:
            raise ValueError('Stale heartbeat sequence')
        self.heartbeat_sequence, self.heartbeat_wall = sequence, now

    def start(self, now, owner, home, epoch, measured_area, manual_used=False):
        if self.phase != 'ready' or manual_used:
            raise ValueError('Start requires a fresh scene without manual pre-mapping')
        if len(home) != 3 or not np.isfinite(home).all():
            raise ValueError('Home must be a finite estimated drive-center pose')
        self.owner, self.home, self.epoch = owner, list(home), epoch
        self.started = now
        self.baseline_area = self.visit_area = measured_area
        self.heartbeat_wall = now
        self.transition('sweeping', now)

    def stop(self, now, reason):
        self.generation += 1
        self.preview = None
        self.collection_started = None
        self.transition('stopped', now, reason)

    def deadlines(self, now, healthy=True):
        if self.started is None or self.phase in ('ready', 'complete', 'stopped'):
            return
        if now-self.started >= self.policy.total_seconds:
            self.stop(now, 'Overall mission wall-time limit')
        elif self.phase in ('sweeping','exploring','exploration_return') and now-self.started >= self.policy.exploration_seconds:
            self.stop(now, 'Exploration and first return wall-time limit')
        elif self.phase in ACTIVE and (not healthy or now-self.heartbeat_wall > self.policy.heartbeat_seconds):
            self.stop(now, 'Unhealthy navigation or operator heartbeat lost')
        elif self.phase in ('awaiting_target', 'preview_ready') and now-self.selection_started >= self.policy.selection_seconds:
            self.stop(now, 'Target selection timed out; no departure authorized')
        elif self.phase in ('sample_outbound', 'collecting', 'sample_return') and now-self.sample_started >= self.policy.sampling_seconds:
            self.stop(now, 'Sampling round-trip wall-time limit')

    def exploration_reason(self, now, return_path, rtf, candidates):
        reserve = return_reserve(return_path, self.policy.speed, rtf)
        if not math.isfinite(reserve):
            raise ValueError('No verified safe return route')
        if now-self.started+reserve >= self.policy.exploration_seconds:
            return 'Return reserve reached'
        if self.low_gain_visits >= 3:
            return 'Three distinct low-gain frontier visits'
        if not candidates:
            return 'No safe useful frontier remains'
        return ''

    def visited(self, now, goal_odom, area):
        gain = max(0., area-self.visit_area)
        distinct = all(math.dist(goal_odom[:2], v['pose'][:2]) >= .65 for v in self.frontier_visits)
        if distinct:
            self.low_gain_visits = self.low_gain_visits+1 if gain < 1. else 0
            self.frontier_visits.append(dict(pose=list(goal_odom), gain_m2=gain))
        self.visit_area = area
        self.event(now, 'frontier_visit', gain_m2=gain, distinct=distinct, pose=list(goal_odom))

    def home_arrived(self, now):
        if self.phase == 'exploration_return':
            self.selection_started = now
            self.transition('awaiting_target', now, 'Exploration complete—select a sample location')
        elif self.phase == 'sample_return' and self.sample is not None:
            self.sample['delivered'] = True
            self.sample['delivered_wall'] = now
            self.transition('complete', now, 'Simulated sample delivered home')
        else:
            raise ValueError('Unexpected home arrival')

    def select(self, now, target):
        if self.phase not in ('awaiting_target', 'preview_ready'):
            raise ValueError('Select a target only after the first home return')
        if target.get('source') not in ('operator_selected', 'detected_soil'):
            raise ValueError('Invalid target source')
        if len(target.get('odom_position', [])) != 3 or not np.isfinite(target['odom_position']).all():
            raise ValueError('Invalid target position')
        self.generation += 1
        self.target, self.preview = dict(target), None
        self.transition('awaiting_target', now)

    def set_preview(self, now, outward, back, target_pose, start, grid, target_signature, rtf=1.):
        if self.phase != 'awaiting_target' or self.target is None:
            raise ValueError('No selected target')
        check_path(grid, outward, start, target_pose)
        check_path(grid, back, target_pose, start)
        estimate = return_reserve(outward, self.policy.speed,rtf)+return_reserve(back, self.policy.speed,rtf)+10.
        if estimate > self.policy.sampling_seconds or now-self.started+estimate >= self.policy.total_seconds:
            raise ValueError('Sample round trip does not fit the remaining budget')
        self.preview = dict(id=secrets.token_hex(16), outward=outward, back=back,
                            stopping_pose=list(target_pose), start=list(start),
                            target_signature=target_signature, estimated_seconds=estimate)
        self.preview_wall = now
        self.transition('preview_ready', now)

    def approve(self, now, preview_id, start, grid, target_signature):
        if (self.phase != 'preview_ready' or self.preview is None or self.preview['id'] != preview_id or
                now-self.preview_wall >= self.policy.preview_seconds):
            raise ValueError('Preview expired or changed; request a new preview')
        p = self.preview
        if (math.dist(start[:2], p['start'][:2]) > .05 or
                abs(math.atan2(math.sin(start[2]-p['start'][2]), math.cos(start[2]-p['start'][2]))) > math.radians(5) or
                target_signature != p['target_signature']):
            raise ValueError('Pose or target changed; request a new preview')
        check_path(grid, p['outward'], start, p['stopping_pose'])
        check_path(grid, p['back'], p['stopping_pose'], p['start'])
        if (now-self.started+p['estimated_seconds'] >= self.policy.total_seconds or
                p['estimated_seconds'] > self.policy.sampling_seconds):
            raise ValueError('Insufficient remaining mission budget')
        self.sample_started = now
        self.transition('sample_outbound', now)

    def collect(self, now, stationary, fresh_target, healthy):
        if self.phase not in ('sample_outbound', 'collecting'):
            raise ValueError('Collection is not authorized')
        if not stationary or not healthy or not fresh_target:
            self.stop(now, 'Collection interrupted: movement, health or target evidence changed')
            return False
        if self.phase == 'sample_outbound':
            self.collection_started = now
            self.transition('collecting', now)
        if now-self.collection_started < self.policy.collection_seconds:
            return False
        if self.sample is None:
            self.sample = dict(id=self.mission_id+':sample-1', simulated=True, delivered=False,
                               collected_wall=now, target=dict(self.target), physical_collection=False)
            self.event(now, 'simulated_collection_complete', sample=self.sample)
        self.transition('sample_return', now)
        return True

    def status(self, now):
        return dict(schema_version=1, session_id=self.session, mission_id=self.mission_id,
                    phase=self.phase, reason=self.reason, epoch=self.epoch, home_odom=self.home,
                    target=self.target, preview=self.preview, sample=self.sample,
                    elapsed_seconds=0. if self.started is None else now-self.started,
                    remaining_seconds=self.policy.total_seconds if self.started is None else max(0., self.policy.total_seconds-(now-self.started)),
                    preview_remaining_seconds=max(0., self.policy.preview_seconds-(now-self.preview_wall)),
                    frontier_visits=self.frontier_visits, policy=asdict(self.policy))


def target_signature(target):
    return hashlib.sha256(json.dumps(target, sort_keys=True, allow_nan=False).encode()).hexdigest()
