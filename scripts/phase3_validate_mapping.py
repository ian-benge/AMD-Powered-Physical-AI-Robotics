#!/usr/bin/env python3
"""Controlled depth replay through production mapping; no ROS, GPU or motion.

Analytic geometry supplies sensor rays and evaluator labels only. Production
mapping receives encoded optical depth, calibration and camera transforms.
The explicitly marked flat-ground depth artifact is a test input, not a claim
that a natural sensor defect has been reproduced. No detector labels are used.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from rover_navigation.core import Grid, TerrainMemory, aligned_copy, rotation, terrain_reasons
from rover_navigation.safety import SafetyGate
from rover_navigation.terrain_compute import CpuTerrainCompute
from rover_navigation.terrain_evidence import ClearanceLocalization, ClearingPolicy, TerrainEvidence


WIDTH, HEIGHT = 960, 540
FOCAL = WIDTH / (2 * math.tan(1.22 / 2))
K = [FOCAL, 0., (WIDTH-1)/2, 0., FOCAL, (HEIGHT-1)/2, 0., 0., 1.]
C, S = math.cos(.4/2), math.sin(.4/2)
CAMERA_Q = [-.5*(C+S), .5*(C+S), .5*(S-C), .5*(C-S)]


def render_depth(surface, camera, artifact=False):
    """Exact plane/box/sphere intersections, expressed as optical axial metres."""
    v, u = np.mgrid[:HEIGHT, :WIDTH]
    rays = np.stack(((u-K[2])/K[0], (v-K[5])/K[4], np.ones_like(u)), -1) @ rotation(CAMERA_Q).T
    camera = np.asarray(camera, float)
    depth = np.full((HEIGHT, WIDTH), np.inf)
    sections = [(0., 0., -10., 10.)]
    if surface == 'gentle':
        sections = [(math.tan(math.radians(6)), 0., -10., 10.)]
    if surface == 'ridge':
        sections = [(0., 0., -10., 1.8), (.35, -.63, 1.8, 2.4),
                    (-.35, 1.05, 2.4, 3.), (0., 0., 3., 10.)]
    for slope, intercept, left, right in sections:
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (slope*camera[0]+intercept-camera[2])/(rays[..., 2]-slope*rays[..., 0])
        x = camera[0]+t*rays[..., 0]
        valid = (t > 0) & (x >= left) & (x <= right)
        depth = np.minimum(depth, np.where(valid, t, np.inf))
    if surface == 'step':
        with np.errstate(divide='ignore', invalid='ignore'):
            ta = (np.array([2., -.55, 0.])-camera)/rays
            tb = (np.array([2.8, .55, .06])-camera)/rays
        near, far = np.minimum(ta, tb).max(-1), np.maximum(ta, tb).min(-1)
        depth = np.minimum(depth, np.where((near > 0) & (far >= near), near, np.inf))
    if surface == 'rock':
        offset = camera-np.array([2.4, 0., .22])
        a = (rays*rays).sum(-1)
        b = (rays*offset).sum(-1)
        discriminant = b*b-a*(offset@offset-.22**2)
        t = (-b-np.sqrt(np.maximum(0., discriminant)))/a
        depth = np.minimum(depth, np.where((discriminant >= 0) & (t > 0), t, np.inf))
    if artifact:
        # A documented 6% short-return patch on physically flat ground. Some
        # rays remain correct, creating an erroneous within-cell height span.
        with np.errstate(invalid='ignore'):
            ground = camera+depth[..., None]*rays
        selected = ((ground[..., 0] > 2.2) & (ground[..., 0] < 2.3) &
                    (np.abs(ground[..., 1]) < .08) & ((u//3+v//3) % 2 == 0))
        depth[selected] *= .94
    depth[(depth < .3) | (depth > 8.)] = np.inf
    return depth.astype('<f4')


def encoded_frame(depth, camera):
    return dict(data=depth.tobytes(), encoding='32FC1', width=WIDTH, height=HEIGHT,
                step=WIDTH*4, bigendian=False, k=K, translation=list(camera), quaternion=CAMERA_Q,
                resolution=.025, size=10., center=list(camera[:2]), max_slope=12., max_step=.03)


def continuous_views(anchors):
    """Supply fresh 5 Hz observations between the independent key viewpoints."""
    yield anchors[0]
    for (x0, t0), (x1, t1) in zip(anchors, anchors[1:]):
        steps = math.ceil((t1-t0)/.2-1e-9)
        for index in range(1, steps+1):
            fraction = index/steps
            yield x0+(x1-x0)*fraction, round(t0+(t1-t0)*fraction, 9)


class Replay:
    def __init__(self, folder):
        self.folder = folder
        folder.mkdir()
        self.compute = CpuTerrainCompute()
        self.memory = TerrainMemory(20., .025)
        self.evidence = TerrainEvidence(self.memory)
        self.localization = ClearanceLocalization()
        self.guard = SafetyGate()
        self.guard.stop('unsafe_or_unobserved_ground')
        self.frames = []
        self.events = []

    def observe(self, surface, x, stamp, artifact=False, loss=''):
        camera = [x, 0., .762+(math.tan(math.radians(6))*x if surface == 'gentle' else 0.)]
        depth = render_depth(surface, camera, artifact)
        if loss == 'missing_depth':
            depth.fill(np.inf)
        args = encoded_frame(depth, camera)
        frame, layers, count = self.compute.depth(**args)
        covariance = np.eye(6)*.0001
        if loss == 'localization':
            covariance[0, 0] = 1.
        self.localization.add(stamp, camera, [0., 0., 0., 1.], covariance.ravel(), stamp)
        suspension = self.localization.check(stamp, stamp)
        if loss == 'map_correction':
            suspension = 'map localization unavailable or stale'
        if count < 100:
            self.evidence.suspend('accepted depth unavailable or stale', stamp)
        else:
            self.memory.remember(frame, terrain_reasons(frame, layers))
            self.evidence.observe(self.memory, frame, layers, camera[:2], stamp, suspension)
        self.events.extend(self.evidence.events)
        self.evidence.events.clear()
        # Same conservative resampling and current-frame overlay as Terrain.
        origin = np.floor((np.array(camera[:2])-3.)/.025)*.025
        local = aligned_copy(self.memory.grid, Grid(np.empty((240, 240), np.int8), .025, *origin))
        local.data[aligned_copy(frame, local).data >= 90] = 100
        global_map, global_reasons = self.compute.project(self.memory, [.25, -.2, 0.],
            [0., 0., math.sin(.1), math.cos(.1)], 20., .05)
        record = dict(stamp=stamp, camera=camera, surface=surface, controlled_depth_artifact=artifact,
            loss=loss, input_sha256=hashlib.sha256(args['data']).hexdigest(),
            points=count, frame_hazards=int((frame.data >= 90).sum()),
            retained_hazards=int((self.memory.grid.data >= 90).sum()),
            evidence=dict(self.evidence.status), guard_mode=self.guard.mode)
        number = len(self.frames)
        np.savez_compressed(self.folder/f'frame-{number:03}.npz', depth=depth, frame=frame.data,
            frame_origin=[frame.x, frame.y], local=local.data,
            local_origin=[local.x, local.y], global_map=global_map.data, global_reasons=global_reasons.data,
            memory=self.memory.grid.data, **layers)
        self.frames.append(record)
        self.local, self.global_map = local, global_map
        self.save()  # retain inputs and the last decision even if an assertion fails
        return record

    def save(self):
        (self.folder/'frames.json').write_text(json.dumps(self.frames, indent=2)+'\n')
        (self.folder/'events.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in self.events))


def validate(output):
    output.mkdir(parents=True, exist_ok=False)
    results = []
    for surface in ('flat', 'gentle', 'step', 'rock', 'ridge'):
        replay = Replay(output/surface)
        for x, stamp in continuous_views([(0., 1.), (0., 1.2), (.25, 2.), (.5, 3.), (.8, 4.), (1.05, 5.)]):
            row = replay.observe(surface, x, stamp)
        if surface in ('flat', 'gentle'):
            assert all(x['frame_hazards'] == 0 for x in replay.frames), surface+' unexpected hazard'
        else:
            assert row['retained_hazards'] > 0, surface+' obstacle disappeared'
            assert row['evidence']['protected_cells'] > 0, surface+' has no protected evidence'
        replay.save()
        results.append(dict(case=surface, result='PASS', final=row))
    for loss in ('none', 'missing_depth', 'localization', 'map_correction', 'stationary'):
        replay = Replay(output/('weak-'+loss))
        replay.observe('flat', 0., 1., artifact=True)
        before = replay.observe('flat', 0., 1.2, artifact=True)
        assert before['retained_hazards'] > 0, 'Controlled artifact did not establish a weak hazard'
        initial_global = replay.global_map.data.copy()
        anchors=[(0.,1.2),(.4,2.),(.65,3.),(.65 if loss=='stationary' else .9,4.),
                 (.65 if loss=='stationary' else 1.1,5.)]
        for x, stamp in list(continuous_views(anchors))[1:]:
            row = replay.observe('flat', x, stamp,
                loss=loss if stamp >= 4. and loss not in ('none', 'stationary') else '')
        if loss == 'none':
            assert row['evidence']['cleared_cells'] > 0, 'No weak false hazard cleared'
            assert ((initial_global >= 90) & (replay.global_map.data >= 0) &
                    (replay.global_map.data < 90)).any(), 'Global planning map did not clear'
            for event in replay.events:
                if event['kind'] != 'hazards_cleared':
                    continue
                for r, c in event['cells']:
                    x, y = replay.memory.grid.point(r, c)
                    assert 0 <= replay.local.at(x, y) < 90, 'Local safety map did not clear'
                for positions in event['view_positions']:
                    assert min(math.dist(a, b) for i, a in enumerate(positions) for b in positions[i+1:]) >= .2-1e-6
        else:
            assert row['evidence']['cleared_cells'] == 0 and row['retained_hazards'] == before['retained_hazards'], loss
            if loss != 'stationary':
                assert row['evidence']['pending_cells'] == 0 and row['evidence']['suspended'], loss
        assert replay.guard.mode == 'stopped', 'Clearing rearmed the safety gate'
        replay.save()
        results.append(dict(case='weak-'+loss, result='PASS', initial=before, final=row))
    report = dict(result='CONTROLLED_ENCODED_DEPTH_REPLAY_PASS', cases=results,
        calibration=dict(width=WIDTH, height=HEIGHT, k=K, pitch_rad=.4, optical_center_height_m=.762),
        policy=asdict(ClearingPolicy()), thresholds=dict(slope_deg=12., height_change_m=.03),
        limitations=['Analytic optical-depth inputs, including an explicitly injected short-return artifact; not a natural sensor failure or live driving acceptance.',
                    'Production CPU mapping and clearing exercised. Actual ROS publication, ROCm parity and rendered-depth validation are separate checks.',
                    'No ROS graph, detector, GPU workload, simulator pose or motion command was used.'])
    (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(result=report['result'], cases=len(results), output=str(output))))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    validate(parser.parse_args().output)
