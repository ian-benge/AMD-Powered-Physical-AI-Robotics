"""ROS-free, monotonic-time mission policy. No truth inputs or motor ownership."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import secrets

import cv2
import numpy as np

from .core import Grid, footprint_avoids_disk, footprint_clear, reachable, turn_radius
from .goal_checks import check_path


DEMO_MAX_SPEED = .15


@dataclass(frozen=True)
class DemoPolicy:
    total_seconds: float = 1800.
    exploration_seconds: float = 1200.
    selection_seconds: float = 120.
    sampling_seconds: float = 480.
    collection_seconds: float = 10.
    speed: float = .10
    angular_speed: float = .30
    heartbeat_seconds: float = 1.
    preview_seconds: float = 30.
    mapping_seconds: float | None = None

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for name, v in asdict(self).items() if name != 'mapping_seconds'):
            raise ValueError('Demo limits must be finite and positive')
        if self.mapping_seconds is not None and not 0. < self.mapping_seconds <= 1800.:
            raise ValueError('Survey mapping time must be in (0, 1800] seconds')
        if self.mapping_seconds is not None and self.mapping_seconds >= self.exploration_seconds:
            raise ValueError('Survey mapping must leave time for the first return')
        total_limit = 2520. if self.mapping_seconds is not None else 1800.
        if (self.total_seconds > total_limit or self.speed > DEMO_MAX_SPEED or self.angular_speed > .3 or
                self.heartbeat_seconds > 1 or self.preview_seconds > 30 or
                self.collection_seconds != 10 or
                self.exploration_seconds+self.selection_seconds+self.sampling_seconds > self.total_seconds):
            raise ValueError('Demo limits exceed the approved envelope')

    @classmethod
    def rehearsal(cls, speed=.05):
        return cls(total_seconds=720., exploration_seconds=360., selection_seconds=60.,
                   sampling_seconds=240., speed=speed)

    @classmethod
    def survey(cls, speed=.15):
        return cls(total_seconds=2520., exploration_seconds=1920., mapping_seconds=1500., speed=speed)


ACTIVE = frozenset(('sweeping', 'exploring', 'exploration_reposition', 'exploration_return', 'sample_outbound',
                    'collecting', 'awaiting_sample_return', 'sample_backing', 'sample_return',
                    'contact_reversing', 'contact_rejoin'))
SAMPLE_AT_SITE = frozenset(('collecting','awaiting_sample_return'))
EXPLORATION_ROUTE_PHASES = frozenset(('exploring','exploration_reposition','exploration_return'))
CONTACT_RECOVERABLE = EXPLORATION_ROUTE_PHASES | {'sample_outbound','sample_return'}
TRAIL_PHASES = ('exploring', 'exploration_return', 'sample_outbound', 'sample_backing', 'sample_return')
FRONTIER_ARRIVAL_RADIUS = .40
FRONTIER_ARRIVAL_YAW = .28
REPOSITION_ARRIVAL_RADIUS = .22
REPOSITION_ARRIVAL_YAW = .21
WAYPOINT_CLEARANCE_PAD = .03
REPOSITION_ARRIVAL_MARGIN = REPOSITION_ARRIVAL_RADIUS+WAYPOINT_CLEARANCE_PAD
EXPLORATION_TRAIL_DISTANCE = 3.
FRONTIER_SHORTLIST_SIZE = 24
FRONTIER_CANDIDATE_LIMIT = 80
SAMPLE_REVERSE_DISTANCE = .50
SAMPLE_REVERSE_MAX_SPEED = .05
SAMPLE_HEADING_TOLERANCE = math.radians(5)
CONTACT_RETREAT_DISTANCE = .50
CONTACT_RETRY_CLEARANCE = .18

class SampleBudgetError(ValueError):
    def __init__(self, required, available):
        self.required = required
        super().__init__(f'Clear round trip needs about {math.ceil(required)} s. '
                         f'{math.floor(available)} s available. Select a closer sample site.')


class SampleGeometryChanged(ValueError):
    """The same confirmed target requires a newly checked approach."""


def path_length(path):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))


def distance_to_route(point, route):
    """Minimum plan-view distance to the continuous route, between samples too."""
    distance=min(math.dist(point[:2],p[:2]) for p in route)
    for a,b in zip(route,route[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length2=dx*dx+dy*dy
        if length2>0.:
            fraction=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2))
            distance=min(distance,math.hypot(point[0]-a[0]-fraction*dx,point[1]-a[1]-fraction*dy))
    return distance


def return_reserve(path, speed, rtf=1., *, contingency_seconds=30.):
    if not path or not math.isfinite(rtf) or rtf <= 0:
        return math.inf
    turns = sum(abs(math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2])))
                for a, b in zip(path, path[1:]))
    return 1.5*(path_length(path)/speed+turns/.3)/min(1., rtf)+contingency_seconds


def frontier_stop_clear(grid, pose, arrival_margin=FRONTIER_ARRIVAL_RADIUS+WAYPOINT_CLEARANCE_PAD):
    """Allow a full turn at arrival with margin for goal and map uncertainty."""
    x,y=pose[:2]
    radius=turn_radius()+arrival_margin+grid.resolution/math.sqrt(2)
    r0,c0=grid.cell(x-radius,y-radius)
    r1,c1=grid.cell(x+radius,y+radius)
    if r0<0 or c0<0 or r1>=grid.data.shape[0] or c1>=grid.data.shape[1]:
        return False
    rr,cc=np.ogrid[r0:r1+1,c0:c1+1]
    disk=(grid.x+(cc+.5)*grid.resolution-x)**2+(grid.y+(rr+.5)*grid.resolution-y)**2<=radius**2
    cells=grid.data[r0:r1+1,c0:c1+1][disk]
    return bool(np.all((cells>=0)&(cells<90)))


def exploration_overlap(path, trail):
    """Weight spacing from earlier travel, including nearby parallel passes."""
    if not len(trail) or len(path)<2:
        return 0.
    xy=np.asarray(path)[:,:2]
    earlier=np.asarray(trail)
    # All routes necessarily share the departure area with the previous leg.
    earlier=earlier[np.linalg.norm(earlier-xy[0],axis=1)>1.]
    if not len(earlier):
        return 0.
    distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    if distance[-1]<=.75:
        return 0.
    # Nav2 path point spacing varies around turns. Sample uniformly in distance.
    stations=np.linspace(.75,distance[-1],max(2,math.ceil((distance[-1]-.75)/.20)+1))
    points=np.column_stack([np.interp(stations,distance,xy[:,axis]) for axis in (0,1)])
    separation=np.sqrt(np.min(np.sum((points[:,None,:]-earlier[None,:,:])**2,axis=2),axis=1))
    proximity=np.clip((EXPLORATION_TRAIL_DISTANCE-separation)/(EXPLORATION_TRAIL_DISTANCE-.85),0.,1.)
    return float(np.mean(proximity))


def observation_frontiers(grid, pose, attempted=(), max_goals=FRONTIER_SHORTLIST_SIZE,
                          speed=.1, angular_speed=.3, *, home=None, trail=()):
    """Cluster reachable observation poses; estimate gain, never mark unknown free.

    Uses known/hazard occupancy for conservative 2-D ray occlusion. Gain is a
    heuristic (not a promise of future visible terrain or a coverage denominator).
    """
    if (not math.isfinite(speed) or not 0 < speed <= DEMO_MAX_SPEED or
            not math.isfinite(angular_speed) or not 0 < angular_speed <= .3):
        raise ValueError('Frontier speeds must fit the demo movement limits')
    unknown = grid.data < 0
    if not unknown.any():
        return []
    distances, allowed = reachable(grid, pose[:2])
    home_distances = reachable(grid, home[:2])[0] if home is not None else None
    near = np.zeros_like(unknown)
    step = max(1, round(1.75/grid.resolution))
    # Small unseen holes along a mapped route should not compete with a real
    # opening. Count a cell only when it belongs to a patch of unseen ground.
    bulk = np.pad(unknown, 1, constant_values=False)
    unknown_patch = (unknown & bulk[:-2,1:-1] & bulk[2:,1:-1] &
                     bulk[1:-1,:-2] & bulk[1:-1,2:])
    padded = np.pad(unknown_patch, step, constant_values=False)
    novelty = np.ones(unknown.shape, dtype=np.float32)
    if len(trail):
        cells = np.floor((np.asarray(trail)-[grid.x,grid.y])/grid.resolution).astype(int)
        cc,rr=cells.T
        inside=(rr>=0)&(cc>=0)&(rr<unknown.shape[0])&(cc<unknown.shape[1])
        visited=np.ones(unknown.shape,dtype=np.uint8)
        visited[rr[inside],cc[inside]]=0
        clearance=cv2.distanceTransform(visited,cv2.DIST_L2,cv2.DIST_MASK_5)*grid.resolution
        novelty=.25+.75*np.clip((clearance-.50)/(EXPLORATION_TRAIL_DISTANCE-.50),0.,1.)
    for setback in (.75, 1., 1.25, 1.5, 1.75):
        gap = min(step, max(1, round(setback/grid.resolution)))
        for dr, dc in ((gap, 0), (-gap, 0), (0, gap), (0, -gap)):
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
    ray_distances = np.arange(.3, 8.01, max(.1, grid.resolution))
    ray_angles = np.linspace(-.52, .52, 17)
    def round_trip_distance(cell):
        outbound = float(distances[cell])
        if home_distances is None:
            return outbound
        back = float(home_distances[cell])
        if not math.isfinite(back):
            # Leave narrow routes for the lattice planner to check.
            back = math.dist(grid.point(*cell), home[:2])
        return outbound+back

    for cluster_id, component in enumerate(sorted(clusters, key=lambda v: min(map(round_trip_distance,v)))):
        if len(goals)>=FRONTIER_CANDIDATE_LIMIT:
            break
        # Give each sector of a connected frontier a useful new viewpoint before
        # filling the rest of its shortlist with nearby alternatives.
        def viewpoint_cost(cell):
            return round_trip_distance(cell)/novelty[cell]
        representatives = sorted(component,key=lambda p:(viewpoint_cost(p),distances[p],p))
        sectors = {}
        for cell in representatives:
            x,y = grid.point(*cell)
            bearing = math.atan2(y-pose[1],x-pose[0])
            sector = int((bearing+math.pi)/(math.tau/12)) % 12
            sectors.setdefault(sector,cell)
        preferred = set(sectors.values())
        representatives = (sorted(preferred, key=lambda p: (viewpoint_cost(p),p))+
                           [cell for cell in representatives if cell not in preferred])
        selected = []
        for r, c in representatives:
            x, y = grid.point(r, c)
            # Grid distance can exceed the wider radius even when the rover
            # is already inside it diagonally. Require a useful departure.
            if math.hypot(x-pose[0],y-pose[1])<=FRONTIER_ARRIVAL_RADIUS+.15:
                continue
            if any(math.hypot(x-a, y-b) < 2*FRONTIER_ARRIVAL_RADIUS for a, b in (*attempted, *selected)):
                continue
            if not frontier_stop_clear(grid,(x,y)):
                continue
            neighborhood = np.argwhere(unknown_patch[max(0, r-step):r+step+1,max(0, c-step):c+step+1])
            if not len(neighborhood):
                continue
            center = neighborhood.mean(axis=0)+[max(0, r-step), max(0, c-step)]
            yaw = math.atan2(center[0]-r, center[1]-c)
            angles = yaw+ray_angles
            rows = np.floor((y+np.sin(angles)[:,None]*ray_distances-grid.y)/grid.resolution).astype(int)
            cols = np.floor((x+np.cos(angles)[:,None]*ray_distances-grid.x)/grid.resolution).astype(int)
            in_bounds=(rows>=0)&(rows<unknown.shape[0])&(cols>=0)&(cols<unknown.shape[1])
            nr=np.clip(rows,0,unknown.shape[0]-1)
            nc=np.clip(cols,0,unknown.shape[1]-1)
            visible=np.logical_and.accumulate(in_bounds&(grid.data[nr,nc]<90),axis=1)&unknown_patch[nr,nc]
            observed=np.unique((nr[visible]//4)*((unknown.shape[1]+3)//4)+nc[visible]//4)
            gain = len(observed)*(4*grid.resolution)**2
            if gain <= 0:
                continue
            turn = abs(math.atan2(math.sin(yaw-pose[2]), math.cos(yaw-pose[2])))
            travel_heading=math.atan2(y-pose[1],x-pose[0])
            outward_preference=.85+.15*math.cos(travel_heading-pose[2])
            travel_seconds = round_trip_distance((r,c))/speed+turn/angular_speed
            novelty_weight = float(novelty[r,c])
            goals.append(dict(pose=[x, y, yaw], expected_gain_m2=gain, trail_novelty=novelty_weight,
                              distance_m=float(distances[r, c]),
                              cluster_id=cluster_id,
                              travel_seconds=travel_seconds, score=gain*outward_preference*novelty_weight**2/(1.+travel_seconds)))
            # A long connected boundary needs several spatially distinct views.
            # Keep ray casting bounded while leaving alternatives if Nav2 rejects
            # the nearest observation pose. All retain full-turn clearance.
            selected.append((x, y))
            if len(selected)>=16 or len(goals)>=FRONTIER_CANDIDATE_LIMIT:
                break
    by_gain = sorted(goals, key=lambda g: (-g['score'], g['distance_m'], g['pose']))
    by_cost = sorted(goals, key=lambda g: (g['travel_seconds'], -g['expected_gain_m2'], g['pose']))
    shortlist = []
    seen = set()
    for ranked in zip(by_gain,by_cost):
        for goal in ranked:
            point = tuple(goal['pose'][:2])
            if point not in seen:
                shortlist.append(goal)
                seen.add(point)
            if len(shortlist)>=max_goals:
                return shortlist
    return shortlist


def collection_poses(grid, point, envelope_radius=0., covariance=None, *,
                     raised_object=False, detected_soil=False):
    """Prefer close, clear sample poses with wider alternatives around the site."""
    point = np.asarray(point, float)
    if point.shape != (2,) or not np.isfinite(point).all():
        raise ValueError('Invalid sample point')
    if not (raised_object or detected_soil) and not 0 <= grid.at(*point) < 90:
        raise ValueError('Choose measured traversable ground or a confirmed sample marker')
    uncertainty = 0.
    if covariance is not None:
        c = np.asarray(covariance, float)
        if c.shape != (3, 3) or not np.isfinite(c).all():
            raise ValueError('Invalid target covariance')
        uncertainty = 2*math.sqrt(max(0., np.linalg.eigvalsh(c[:2, :2]).max()))
    if not math.isfinite(envelope_radius) or envelope_radius < 0:
        raise ValueError('Invalid target envelope')
    if raised_object:
        radius = max(1.5, envelope_radius+uncertainty+turn_radius()+.30)
        # The visible object can sit beyond the measured ground under it.
        wider = radius+.8
    else:
        # A soil patch is measured ground, so its area is not a collision
        # envelope. Leave room for the camera to see the uncertain target.
        radius = max(1.2, uncertainty+.50)
        wider = max(radius, 1.5, envelope_radius+uncertainty+turn_radius()+.30)
    poses = []
    for distance in sorted({radius, (radius+wider)/2, wider}):
        for angle in np.linspace(0., 2*math.pi, 8, endpoint=False):
            x, y = point+distance*np.array([math.cos(angle), math.sin(angle)])
            yaw = math.atan2(point[1]-y, point[0]-x)
            if all(footprint_clear(grid, x, y, a) for a in np.linspace(-math.pi, math.pi, 37)):
                poses.append([float(x), float(y), yaw])
    return poses


def heading_error(yaw, desired):
    return math.atan2(math.sin(desired-yaw), math.cos(desired-yaw))


def recorded_retreat(grid, start, breadcrumbs, keepout=None):
    """Cut a reverse route exactly 0.50 m along the recent driven track."""
    route=[list(start)]
    remaining=CONTACT_RETREAT_DISTANCE
    for point in reversed(breadcrumbs):
        last=route[-1]
        distance=math.dist(last[:2],point[:2])
        if distance<.006:
            continue
        if distance>.14:
            raise ValueError('The recorded approach has a gap')
        bearing=math.atan2(last[1]-point[1],last[0]-point[0])
        if abs(heading_error(last[2],bearing))>.55:
            raise ValueError('The recorded approach cannot be retraced in reverse')
        if distance>=remaining:
            fraction=remaining/distance
            end=[a+(b-a)*fraction for a,b in zip(last[:2],point[:2])]
            route.append([*end,last[2]+fraction*heading_error(last[2],point[2])])
            check_path(grid,route,start,route[-1],keepout=keepout)
            return route
        route.append(list(point))
        remaining-=distance
    raise ValueError('At least 0.50 m of continuous approach history is required')


def follow_recorded_retreat(route, pose, previous_progress):
    """Track the breadcrumb arc without switching to a nearby earlier segment."""
    walked=0.
    match=None
    vectors=[]
    for start,end in zip(route,route[1:]):
        dx,dy=end[0]-start[0],end[1]-start[1]
        length=math.hypot(dx,dy)
        fraction=max(0.,min(1.,((pose[0]-start[0])*dx+(pose[1]-start[1])*dy)/length**2))
        station=walked+fraction*length
        position=(start[0]+fraction*dx,start[1]+fraction*dy)
        error=math.dist(pose[:2],position)
        if previous_progress-.06<=station<=previous_progress+.18 and (match is None or error<match[0]):
            match=error,station
        vectors.append((start,dx,dy,walked,length))
        walked+=length
    if match is None or match[0]>.06:
        raise ValueError('Rover left the checked reverse corridor')
    progress=max(previous_progress,match[1])
    if progress>=CONTACT_RETREAT_DISTANCE-.02 and math.dist(pose[:2],route[-1][:2])<=.03:
        return progress,None
    lookahead=min(walked,progress+.10)
    start,dx,dy,walk,length=next(v for v in vectors if v[3]+v[4]>=lookahead)
    f=(lookahead-walk)/length
    x,y=start[0]+f*dx,start[1]+f*dy
    yaw=math.atan2(pose[1]-y,pose[0]-x)
    return progress,[x,y,yaw]


def sample_reverse_pose(grid, pose, keepout):
    """Check a straight reverse and room for the turn at its far end."""
    x, y, yaw = pose
    end = [x-SAMPLE_REVERSE_DISTANCE*math.cos(yaw), y-SAMPLE_REVERSE_DISTANCE*math.sin(yaw), yaw]
    check_path(grid, [pose, end], pose, end, keepout=keepout)
    check_sample_turn(grid, end, keepout)
    return end


def check_sample_turn(grid, pose, keepout):
    for angle in np.linspace(-math.pi, math.pi, 181):
        turn_pose = (*pose[:2], angle)
        if not footprint_clear(grid, *turn_pose) or not footprint_avoids_disk(turn_pose, keepout):
            raise ValueError('No clear space to turn after backing away from the sample')


def sample_reverse_reserve(speed, rtf):
    if not math.isfinite(rtf) or rtf <= 0.:
        return math.inf
    return (1.5*SAMPLE_REVERSE_DISTANCE/min(speed, SAMPLE_REVERSE_MAX_SPEED)+8.)/min(1., rtf)


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
        self.collection_elapsed = 0.
        self.collection_last_check = self.collection_last_good = None
        self.collection_fresh = False
        self.contact_from = None
        self.selection_started = self.sample_started = None
        self.sample_review_until = None
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

    def deadlines(self, now, healthy=True, missing_inputs=()):
        if self.started is None or self.phase in ('ready', 'complete', 'stopped'):
            return
        phase=self.contact_from if self.phase in ('contact_paused','contact_reversing','contact_rejoin') else self.phase
        if now-self.started >= self.policy.total_seconds:
            self.stop(now, 'Overall mission wall-time limit')
        elif phase in EXPLORATION_ROUTE_PHASES | {'sweeping','exploration_paused'} and now-self.started >= self.policy.exploration_seconds:
            self.stop(now, 'Exploration and first return wall-time limit')
        elif self.phase in ACTIVE and not healthy:
            detail = ' ('+', '.join(missing_inputs)+')' if missing_inputs else ''
            self.stop(now, 'Navigation inputs became unhealthy'+detail)
        elif self.phase in ACTIVE and now-self.heartbeat_wall > self.policy.heartbeat_seconds:
            self.stop(now, 'Operator heartbeat lost')
        elif (self.phase in ('awaiting_target', 'preview_ready') and
              now >= (self.sample_review_until if self.sample_review_until is not None
                      else self.selection_started+self.policy.selection_seconds)):
            self.stop(now, 'Target selection timed out; no departure authorized')
        elif phase in SAMPLE_AT_SITE | {'sample_outbound','sample_backing','sample_return'} and self.sample_time_left(now) <= 0.:
            self.stop(now, 'Sampling round-trip wall-time limit')

    def exploration_reason(self, now, return_path, rtf, candidates):
        reserve = return_reserve(return_path, self.policy.speed, rtf)
        if not math.isfinite(reserve):
            raise ValueError('No verified safe return route')
        if now-self.started+reserve >= self.policy.exploration_seconds:
            return 'Return reserve reached'
        if self.policy.mapping_seconds is not None and now-self.started >= self.policy.mapping_seconds:
            return 'Survey mapping time reached'
        if self.policy.mapping_seconds is None and self.low_gain_visits >= 3:
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
        elif self.phase == 'sample_return':
            if self.sample is not None:
                self.sample['delivered'] = True
                self.sample['delivered_wall'] = now
            reason='Simulated sample delivered home' if self.sample else 'Returned home without a confirmed sample'
            self.transition('complete',now,reason)
        else:
            raise ValueError('Unexpected home arrival')

    def select(self, now, target):
        if self.phase not in ('awaiting_target', 'preview_ready'):
            raise ValueError('Select a target only after the first home return')
        if target.get('source') not in ('operator_selected', 'detected_soil', 'detected_object'):
            raise ValueError('Invalid target source')
        if len(target.get('odom_position', [])) != 3 or not np.isfinite(target['odom_position']).all():
            raise ValueError('Invalid target position')
        self.generation += 1
        self.target, self.preview = dict(target), None
        self.transition('awaiting_target', now)

    def sample_time_left(self, now):
        if self.selection_started is None:
            return 0.
        unused_exploration = max(0., self.started+self.policy.exploration_seconds-self.selection_started)
        sample_start = now if self.sample_started is None else self.sample_started
        end = min(self.started+self.policy.total_seconds,
                  sample_start+self.policy.sampling_seconds+unused_exploration)
        return max(0., end-now)

    def set_preview(self, now, outward, back, target_pose, start, grid, target_signature, *,
                    home, target_map, keepout, rtf=1.):
        if self.phase != 'awaiting_target' or self.target is None:
            raise ValueError('No selected target')
        backup = sample_reverse_pose(grid, target_pose, keepout)
        check_path(grid, outward, start, target_pose, keepout=keepout)
        check_path(grid, back, backup, home, keepout=keepout)
        estimate = (return_reserve([start, *outward, target_pose], self.policy.speed, rtf) +
                    return_reserve([backup, *back, home], self.policy.speed, rtf) +
                    sample_reverse_reserve(self.policy.speed, rtf) + self.policy.collection_seconds)
        if not math.isfinite(estimate):
            raise ValueError('Sample travel time is unavailable')
        available = self.sample_time_left(now)
        if estimate >= available:
            raise SampleBudgetError(estimate, available)
        self.preview = dict(id=secrets.token_hex(16), outward=outward, back=back,
                            stopping_pose=list(target_pose), start=list(start),
                            backup_pose=backup, home=list(home), target_map=list(target_map),
                            keepout=list(keepout),
                            target_signature=target_signature, estimated_seconds=estimate)
        self.preview_wall = now
        self.transition('preview_ready', now)

    def validate_approval(self, now, preview_id, start, grid, target_signature):
        if (self.phase != 'preview_ready' or self.preview is None or self.preview['id'] != preview_id or
                now-self.preview_wall >= self.policy.preview_seconds):
            raise ValueError('Preview expired or changed; request a new preview')
        p = self.preview
        if (math.dist(start[:2], p['start'][:2]) > .05 or
                abs(math.atan2(math.sin(start[2]-p['start'][2]), math.cos(start[2]-p['start'][2]))) > math.radians(5) or
                target_signature != p['target_signature']):
            raise ValueError('Pose or target changed; request a new preview')
        sample_reverse_pose(grid, p['stopping_pose'], p['keepout'])
        check_path(grid, p['outward'], start, p['stopping_pose'], keepout=p['keepout'])
        check_path(grid, p['back'], p['backup_pose'], p['home'], keepout=p['keepout'])
        available = self.sample_time_left(now)
        if p['estimated_seconds'] >= available:
            raise SampleBudgetError(p['estimated_seconds'], available)

    def approve(self, now, preview_id, start, grid, target_signature):
        self.validate_approval(now, preview_id, start, grid, target_signature)
        if self.sample_started is None:
            self.sample_started = now
        self.transition('sample_outbound', now)

    def collect(self, now, stationary, fresh_target, healthy):
        if self.phase not in ('sample_outbound', 'collecting'):
            raise ValueError('Collection is not authorized')
        if not stationary or not healthy:
            self.stop(now, 'Collection stopped because movement or navigation health changed')
            return False
        if self.phase == 'sample_outbound':
            if not fresh_target:return False
            self.collection_started = now
            self.collection_elapsed = 0.
            self.collection_last_check = self.collection_last_good = now
            self.collection_fresh = True
            self.transition('collecting', now)
        elif not fresh_target:
            if self.collection_fresh:self.event(now,'sample_evidence_gap')
            self.collection_fresh = False
            self.collection_last_check = now
            if now-self.collection_last_good>=1.5:
                self.transition('awaiting_sample_return',now,'Sample unconfirmed. Return home when ready.')
            return False
        else:
            if self.collection_fresh:
                self.collection_elapsed+=now-self.collection_last_check
            self.collection_fresh=True
            self.collection_last_check=self.collection_last_good=now
        if self.collection_elapsed < self.policy.collection_seconds:return False
        if self.sample is None:
            self.sample = dict(id=self.mission_id+':sample-1', simulated=True, delivered=False,
                               collected_wall=now, target=dict(self.target), physical_collection=False)
            self.event(now, 'simulated_collection_complete', sample=self.sample)
        self.transition('awaiting_sample_return',now,'Simulated sample collected. Return home when ready.')
        return True

    def request_sample_return(self,now):
        if self.phase != 'awaiting_sample_return' or self.preview is None:
            raise ValueError('Wait for the collection result before returning home')
        self.transition('sample_backing',now,'Operator requested return from the sample site')

    def status(self, now):
        elapsed = 0. if self.started is None else now-self.started
        mapping_remaining = None
        if self.policy.mapping_seconds is not None and self.phase in (
                'ready', 'sweeping', 'exploring', 'exploration_reposition', 'exploration_paused'):
            mapping_remaining = max(0., self.policy.mapping_seconds-elapsed)
        return dict(schema_version=1, session_id=self.session, mission_id=self.mission_id,
                    phase=self.phase, reason=self.reason, epoch=self.epoch, home_odom=self.home,
                    target=self.target, preview=self.preview, sample=self.sample,
                    elapsed_seconds=elapsed, remaining_seconds=max(0., self.policy.total_seconds-elapsed),
                    mapping_remaining_seconds=mapping_remaining,
                    sample_time_left_seconds=self.sample_time_left(now),
                    preview_remaining_seconds=max(0., self.policy.preview_seconds-(now-self.preview_wall)),
                    frontier_visits=self.frontier_visits, policy=asdict(self.policy))


def target_signature(target):
    return hashlib.sha256(json.dumps(target, sort_keys=True, allow_nan=False).encode()).hexdigest()
