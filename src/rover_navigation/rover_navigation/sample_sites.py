"""Depth projection and session-local object tracks, independent of ROS."""
import math
from collections import deque
import numpy as np
from rover_training.geometry import FOUR_CLASS_NAMES
from .site_geometry import covariance_floor


DEFAULTS = dict(min_views=3, min_baseline_m=.20, min_view_span_s=2., min_confidence=.50,
                min_class_support=.70, confirmation_score=.90, mahalanobis_squared=11.34,
                maximum_distance_m=.50, position_std_floor_m=.05, tentative_timeout_s=30.,
                prior=.20, hit_log_odds=math.log(4), miss_log_odds=math.log(3),
                score_bounds=[.02, .98], automatic_classes=['rock', 'yellow_crystal_rock'])


def depth_location(data, encoding, width, height, step, bigendian, k, box):
    types = {'32FC1': ('f4', 1.), '16UC1': ('u2', .001)}
    if encoding not in types:
        raise ValueError('Unsupported depth encoding')
    kind, scale = types[encoding]
    dtype = np.dtype(('>' if bigendian else '<')+kind)
    if (min(width, height) <= 0 or step < width*dtype.itemsize or len(data) != step*height or
            len(k) != 9 or not np.isfinite(k).all() or min(k[0], k[4]) <= 0 or
            len(box) != 4 or not np.isfinite(box).all()):
        raise ValueError('Invalid depth buffer, calibration or box')
    x1, y1, x2, y2 = box
    if not 0 <= x1 < x2 <= width or not 0 <= y1 < y2 <= height:
        raise ValueError('Box outside aligned image')
    # The central half reduces ground/background leaking around the object.
    u, v = (x1+x2)/2, (y1+y2)/2
    left, right = max(0, math.floor(u-(x2-x1)/4)), min(width, math.ceil(u+(x2-x1)/4))
    top, bottom = max(0, math.floor(v-(y2-y1)/4)), min(height, math.ceil(v+(y2-y1)/4))
    depth = np.ndarray((height, width), dtype=dtype, buffer=bytes(data), strides=(step, dtype.itemsize))
    stride = max(1, math.ceil(math.sqrt((right-left)*(bottom-top)/4096)))
    values = depth[top:bottom:stride, left:right:stride].astype(float)*scale
    valid = values[np.isfinite(values) & (values >= .3) & (values <= 8.)]
    if len(valid) < 9 or len(valid) < .2*values.size:
        raise ValueError('Insufficient valid depth inside detection')
    z = float(np.median(valid))
    return np.array([(u-k[2])*z/k[0], (v-k[5])*z/k[4], z])


def _independent(views, camera, observer):
    # Requiring observer translation additionally excludes a camera moving on
    # its mount's arc during a pure rover rotation.
    return all(np.linalg.norm(np.asarray(v['camera'])-camera) >= .20-1e-8 and
               np.linalg.norm(np.asarray(v['observer'])-observer[:3]) >= .20-1e-8 for v in views)


def _view(stamp, camera, observer):
    return dict(stamp=stamp, camera=list(camera), observer=list(observer[:3]))


def _iou(a, b):
    if a is None or b is None:
        return 0.
    x1, y1 = np.maximum(a[:2], b[:2])
    x2, y2 = np.minimum(a[2:], b[2:])
    area = max(0., x2-x1)*max(0., y2-y1)
    return area/max(1e-12, (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area)


class SiteStore:
    """Conservative, epoch-local static tracks; contains no robot control."""
    def __init__(self, epoch=0):
        self.epoch, self.objects, self.next_id = epoch, [], 1
        self.ambiguous = []
        self.last_stamp = -math.inf
        self.events = deque(maxlen=4096)

    def event(self, kind, stamp, item=None, **detail):
        self.events.append(dict(kind=kind, stamp=stamp, epoch=self.epoch,
                                track_id=item['track_id'] if item else None, **detail))

    def export(self, item):
        return {k: v for k, v in item.items() if not k.startswith('_')}

    def expire(self, stamp):
        for item in self.objects:
            if item['state'] == 'tentative' and stamp-item['last_stamp'] > 30.:
                self.transition(item, 'retired', stamp, 'tentative observation expired')

    def transition(self, item, state, stamp, reason):
        old = item['state']
        item['state'], item['reason'] = state, reason
        if old != state:
            if state == 'unverified':
                item['_confirmation_views'] = []
                item['confirmation_view_count'] = 0
            self.event('transition', stamp, item, previous=old, state=state, reason=reason)

    def _score(self, item, amount):
        bound = math.log(49.)
        item['_log_odds'] = max(-bound, min(bound, item['_log_odds']+amount))
        item['evidence_score'] = 1/(1+math.exp(-item['_log_odds']))

    def _state(self, item, stamp):
        eligible = item['class_name'] in DEFAULTS['automatic_classes'] and item['class_support'] >= .70
        if item['state'] == 'confirmed' and (item['evidence_score'] < .50 or not eligible):
            self.transition(item, 'unverified', stamp, 'contradictory existence or class evidence')
        if item['evidence_score'] <= .20+1e-10 and item['contradiction_count'] >= 3:
            self.transition(item, 'retired', stamp, 'independent visible contradictions')
            return
        views = item['_confirmation_views']
        if (eligible and len(views) >= 3 and views[-1]['stamp']-views[0]['stamp'] >= 2.-1e-8 and
                item['evidence_score'] >= .90):
            self.transition(item, 'confirmed', stamp, 'multi-view rock and raised geometry supported')
        elif item['state'] not in ('confirmed', 'retired'):
            if item['class_name'] not in DEFAULTS['automatic_classes']:
                item['reason'] = 'class requires later validation; automatic confirmation disabled'
            elif item['class_support'] < .70:
                item['reason'] = 'inconsistent class evidence'
            elif item.get('geometry', {}).get('supported'):
                item['reason'] = 'needs another viewpoint or longer observation span'
            else:
                item['reason'] = item.get('geometry', {}).get('reason', 'insufficient raised geometry')

    def _new(self, obs, stamp, observer):
        if len(self.objects) >= 1000:
            self.event('capacity_rejected', stamp)
            return None
        item = dict(id=self.next_id, track_id=f'{self.epoch}:{self.next_id}', epoch=self.epoch,
                    state='tentative', reason='needs another viewpoint', class_name=obs['class_name'],
                    odom_position=list(obs['odom_position']), covariance=obs['covariance'],
                    extent=obs['extent'], appearance=obs.get('appearance'),
                    detection_count=0, max_confidence=0., first_stamp=stamp, last_stamp=stamp,
                    observation_pose=list(observer), independent_view_count=0, confirmation_view_count=0,
                    contradiction_count=0, evidence_score=.20, class_support=0., class_evidence={},
                    _log_odds=math.log(.20/.80), _positive_views=[], _negative_views=[],
                    _class_views=[], _class_samples=[], _confirmation_views=[])
        self.objects.append(item)
        self.next_id += 1
        self.event('birth', stamp, item, observation=obs)
        return item

    def _hit(self, item, obs, stamp, camera, observer):
        item['last_stamp'], item['observation_pose'] = stamp, list(observer)
        item['detection_count'] += 1
        item['max_confidence'] = max(item['max_confidence'], obs['confidence'])
        item['geometry'] = obs['geometry']
        if _independent(item['_class_views'], camera, observer) and len(item['_class_views']) < 128:
            item['_class_views'].append(_view(stamp, camera, observer))
            item['_class_samples'] = (item['_class_samples']+[(obs['class_name'], obs['confidence'])])[-32:]
            evidence = {name: sum(q for label, q in item['_class_samples'] if label == name) for name in FOUR_CLASS_NAMES}
            item['class_evidence'] = evidence
            item['class_name'] = max(evidence, key=evidence.get)
            item['class_support'] = evidence[item['class_name']]/max(1e-12, sum(evidence.values()))
        supported = obs['geometry'].get('supported') and obs['confidence'] >= .50
        independent = _independent(item['_positive_views'], camera, observer)
        if supported and independent and len(item['_positive_views']) < 128:
            view = _view(stamp, camera, observer)
            item['_positive_views'].append(view)
            item['_confirmation_views'].append(view)
            item['independent_view_count'] = len(item['_positive_views'])
            item['confirmation_view_count'] = len(item['_confirmation_views'])
            self._score(item, math.log(4))
            # Fixed-weight covariance intersection avoids assuming independent
            # measurements when both contain the same visual odometry errors.
            a, b = np.linalg.inv(item['covariance']), np.linalg.inv(obs['covariance'])
            covariance = np.linalg.inv(.5*a+.5*b)
            point = covariance @ (.5*a @ item['odom_position']+.5*b @ obs['odom_position'])
            item['covariance'] = covariance_floor(covariance).tolist()
            item['odom_position'] = point.tolist()
            item['extent'], item['appearance'] = obs['extent'], obs.get('appearance')
            self.event('independent_support', stamp, item, observation=obs, view=view)
        elif obs['geometry'].get('contradiction'):
            self._miss(item, stamp, camera, observer, 'visible depth contradicts raised object')
        self._state(item, stamp)

    def _miss(self, item, stamp, camera, observer, reason):
        if item['state'] == 'retired' or not _independent(item['_negative_views'], camera, observer):
            return
        if len(item['_negative_views']) >= 128:
            return
        view = _view(stamp, camera, observer)
        item['_negative_views'].append(view)
        item['contradiction_count'] += 1
        self._score(item, -math.log(3))
        self.event('independent_contradiction', stamp, item, reason=reason, view=view)
        self._state(item, stamp)

    def update(self, observations, stamp, observer, camera, visible_misses=(), suspension=''):
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            return []
        self.last_stamp = stamp
        if suspension:
            self.event('suspended', stamp, reason=suspension)
            return []
        self.ambiguous = []
        self.expire(stamp)
        rows = []
        for obs in sorted(observations, key=lambda o: -o['confidence']):
            try:
                if (obs['class_name'] not in FOUR_CLASS_NAMES or not math.isfinite(obs['confidence']) or
                        not 0 <= obs['confidence'] <= 1 or len(obs['odom_position']) != 3 or
                        not np.isfinite(obs['odom_position']).all()):
                    raise ValueError('Invalid object observation')
                obs = dict(obs, covariance=covariance_floor(obs['covariance']).tolist())
                if any(_iou(obs.get('box'), old.get('box')) >= .70 and
                       np.linalg.norm(np.asarray(obs['odom_position'])-old['odom_position']) < .10 for old in rows):
                    self.event('duplicate_box', stamp)
                    continue
                rows.append(obs)
            except (ValueError, KeyError, TypeError) as error:
                self.event('observation_rejected', stamp, reason=str(error))
        tracks = [item for item in self.objects if item['state'] != 'retired']
        distances = np.full((len(rows), len(tracks)), np.inf)
        for i, obs in enumerate(rows):
            for j, item in enumerate(tracks):
                delta = np.asarray(obs['odom_position'])-item['odom_position']
                if np.linalg.norm(delta) <= .50:
                    d = float(delta @ np.linalg.solve(np.asarray(item['covariance'])+obs['covariance'], delta))
                    if d <= 11.34:
                        distances[i, j] = d
        decisions, matched, protected = [], set(), set()
        for i, obs in enumerate(rows):
            order = np.argsort(distances[i])
            finite = [int(j) for j in order if np.isfinite(distances[i, j])]
            if not finite:
                item = self._new(obs, stamp, observer)
                if item is not None:
                    self._hit(item, obs, stamp, camera, observer)
                    matched.add(item['track_id'])
                    decisions.append(dict(outcome='new', id=item['track_id']))
                continue
            j = finite[0]
            best = distances[i, j]
            margin = len(finite) == 1 or (distances[i, finite[1]] > 1e-12 and best <= .75*distances[i, finite[1]])
            mutual = int(np.argmin(distances[:, j])) == i
            item = tracks[j]
            # Appearance/extent can veto an inconsistent match, but cannot
            # override an ambiguous spatial gate or authorize a distant match.
            appearance_ok = True
            if item.get('appearance') is not None and obs.get('appearance') is not None:
                similarity = float(np.sqrt(np.asarray(item['appearance'])*obs['appearance']).sum())
                appearance_ok = similarity >= .30
            extent_ratio = np.linalg.norm(obs['extent'])/max(.01, np.linalg.norm(item['extent']))
            if not margin or not mutual or not appearance_ok or not .25 <= extent_ratio <= 4.:
                protected.update(tracks[n]['track_id'] for n in finite)
                self.event('ambiguous', stamp, item, observation=obs, candidates=[tracks[n]['track_id'] for n in finite])
                if len(self.ambiguous) < 20:
                    self.ambiguous.append(dict(obs, id=900000+len(self.ambiguous),
                        track_id=f'{self.epoch}:ambiguous:{len(self.ambiguous)}', epoch=self.epoch,
                        state='unverified', reason='ambiguous match; no track evidence updated',
                        confirmation_view_count=0, independent_view_count=0, evidence_score=.20,
                        last_stamp=stamp, max_confidence=obs['confidence']))
                decisions.append(dict(outcome='ambiguous', id=item['track_id']))
                continue
            self._hit(item, obs, stamp, camera, observer)
            matched.add(item['track_id'])
            self.event('matched', stamp, item, position=obs['odom_position'])
            decisions.append(dict(outcome='matched', id=item['track_id']))
        for item in tracks:
            if item['track_id'] in visible_misses and item['track_id'] not in matched | protected:
                self._miss(item, stamp, camera, observer, 'healthy fully visible region without detection')
        return decisions
