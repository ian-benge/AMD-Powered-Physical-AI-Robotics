#!/usr/bin/env python3
"""Read-only terrain audit; never publishes, selects modes or commands motion.

Saves the navigation terrain map and classifies the newest depth frame on its
own, recording which limit (height span or slope) marks each hazard. For
generated scenes it compares hazards with the scene's rocks and with its visual
and collision height grids. Scene truth is used only for this evaluation.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String

from rover_navigation.core import depth_points, traversability
from rover_navigation.ros_utils import apply_tf, from_grid, stamp_seconds
from rover_perception.live_core import image_to_bgr


class Audit(Node):
    def __init__(self):
        super().__init__('navigation_terrain_audit')
        self.map, self.depth, self.infos = None, None, {}
        self.buffers = {name: {} for name in ('map', 'depth', 'rgb', 'info', 'local', 'reasons')}
        self.arrivals = {name: deque(maxlen=5000) for name in self.buffers}
        self.status = {}
        self.last_captured = -math.inf
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/navigation/terrain_map', lambda m: self.cache('map', m), durable)
        self.create_subscription(OccupancyGrid, '/navigation/terrain_reasons', lambda m: self.cache('reasons', m), durable)
        self.create_subscription(OccupancyGrid, '/navigation/local_ground', lambda m: self.cache('local', m), 1)
        self.create_subscription(Image, '/oak/depth/image_raw', lambda m: self.cache('depth', m), qos_profile_sensor_data)
        self.create_subscription(Image, '/oak/image_raw', lambda m: self.cache('rgb', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/oak/camera_info',
                                 lambda m: self.cache('info', m), qos_profile_sensor_data)
        self.create_subscription(String, '/navigation/status', lambda m: self.status.update(json.loads(m.data)), 1)

    def cache(self, name, msg):
        cache = self.buffers[name]
        stamp = stamp_seconds(msg.header.stamp)
        self.arrivals[name].append((time.monotonic(), stamp))
        cache[stamp] = msg
        while len(cache) > (40 if name in ('depth', 'rgb', 'info') else 10):
            cache.pop(next(iter(cache)))

    def frame(self):
        for stamp in sorted(self.buffers['map'], reverse=True):
            if stamp <= self.last_captured or not all(stamp in cache for cache in self.buffers.values()):
                continue
            self.matched = {name: cache[stamp] for name, cache in self.buffers.items()}
            depth, info, rgb = (self.matched[n] for n in ('depth', 'info', 'rgb'))
            if (depth.width, depth.height, depth.header.frame_id) != (info.width, info.height, info.header.frame_id):
                continue
            if (rgb.width, rgb.height, rgb.header.frame_id) != (depth.width, depth.height, depth.header.frame_id):
                continue
            when = Time.from_msg(depth.header.stamp)
            if not all(self.tf.can_transform(target, source, when) for target, source in
                       (('map', depth.header.frame_id), ('map', 'drive_center'), ('odom', depth.header.frame_id))):
                continue
            self.map, self.depth = self.matched['map'], depth
            return depth, info, self.tf.lookup_transform('map', depth.header.frame_id, when), \
                self.tf.lookup_transform('map', 'drive_center', when)
        return None


def capture(prefix, node, ready):
    """Raw bytes preserve depth encoding, row stride and endian order without reinterpretation."""
    depth, info, camera, pose = ready
    stamp = stamp_seconds(depth.header.stamp)
    arrays = {name: from_grid(node.matched[name]).data for name in ('map', 'local', 'reasons')}
    arrays.update(depth_bytes=np.frombuffer(bytes(depth.data), np.uint8),
                  rgb_bgr=image_to_bgr(node.matched['rgb']))
    np.savez_compressed(prefix.with_suffix('.npz'), **arrays)
    if not cv2.imwrite(str(prefix.with_suffix('.jpg')), arrays['rgb_bgr']):
        raise RuntimeError('Could not save synchronized RGB evidence')

    def tf_record(tf):
        t, q = tf.transform.translation, tf.transform.rotation
        return dict(parent=tf.header.frame_id, child=tf.child_frame_id, stamp=stamp_seconds(tf.header.stamp),
                    translation=[t.x, t.y, t.z], rotation_xyzw=[q.x, q.y, q.z, q.w])

    grids = {name: dict(frame=msg.header.frame_id, stamp=stamp_seconds(msg.header.stamp),
                       origin=[msg.info.origin.position.x, msg.info.origin.position.y],
                       resolution=msg.info.resolution, width=msg.info.width, height=msg.info.height)
             for name, msg in node.matched.items() if name in ('map', 'local', 'reasons')}
    row = dict(stamp=stamp, wall_received=time.monotonic(), exact_rgb_depth_info_terrain_stamp=True,
               depth=dict(encoding=depth.encoding, width=depth.width, height=depth.height, step=depth.step,
                          is_bigendian=bool(depth.is_bigendian), frame=depth.header.frame_id),
               calibration=dict(k=list(info.k), d=list(info.d), distortion_model=info.distortion_model),
               transforms=dict(camera_to_map=tf_record(camera), drive_center_to_map=tf_record(pose),
                 camera_to_odom=tf_record(node.tf.lookup_transform('odom', depth.header.frame_id, Time.from_msg(depth.header.stamp)))),
               grids=grids, guard_status=dict(node.status),
               guard_status_alignment='latest receipt; guard sim_time recorded separately',
               map_projection_alignment='map is the published processing-time projection; image-time transforms are separate',
               known_cells=int((arrays['map'] >= 0).sum()), hazard_cells=int((arrays['map'] >= 90).sum()),
               cause_cells=dict(slope=int(((arrays['reasons'] > 0) & ((arrays['reasons'] & 2) != 0)).sum()),
                                height_change=int(((arrays['reasons'] > 0) & ((arrays['reasons'] & 4) != 0)).sum())))
    prefix.with_suffix('.json').write_text(json.dumps(row, indent=2, allow_nan=False)+'\n')
    node.last_captured = stamp
    return dict(stamp=stamp, known_cells=row['known_cells'], hazard_cells=row['hazard_cells'],
                cause_cells=row['cause_cells'], evidence=prefix.name)


def capture_unmapped_sensor(prefix, node):
    """Preserve inputs when localization/terrain fails; never call this a mapped capture."""
    common = set(node.buffers['depth']) & set(node.buffers['rgb']) & set(node.buffers['info'])
    if not common:
        return None
    stamp = max(common)
    depth, rgb, info = (node.buffers[name][stamp] for name in ('depth', 'rgb', 'info'))
    if (depth.width, depth.height, depth.header.frame_id) != (info.width, info.height, info.header.frame_id):
        return None
    if (rgb.width, rgb.height, rgb.header.frame_id) != (depth.width, depth.height, depth.header.frame_id):
        return None
    image = image_to_bgr(rgb)
    np.savez_compressed(prefix.with_suffix('.npz'), depth_bytes=np.frombuffer(bytes(depth.data), np.uint8), rgb_bgr=image)
    if not cv2.imwrite(str(prefix.with_suffix('.jpg')), image):
        raise RuntimeError('Could not preserve failure camera evidence')
    metadata = dict(partial=True, mapped_capture=False, stamp=stamp, encoding=depth.encoding,
                    width=depth.width, height=depth.height, step=depth.step, is_bigendian=bool(depth.is_bigendian),
                    frame=depth.header.frame_id, k=list(info.k), guard_status=dict(node.status),
                    missing='No complete matching terrain/localization evidence; no pose substitution')
    prefix.with_suffix('.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
    return dict(evidence=prefix.name, stamp=stamp, partial=True)


def timing_summary(samples):
    result = dict(count=len(samples))
    if len(samples) > 1:
        wall, stamps = np.asarray(samples).T
        gaps = np.diff(wall)
        result.update(rate_wall_hz=float((len(wall)-1)/(wall[-1]-wall[0])),
                      max_gap_wall_seconds=float(gaps.max()), p95_gap_wall_seconds=float(np.percentile(gaps, 95)),
                      max_stamp_gap_sim_seconds=float(np.diff(stamps).max()))
    return result


def heightfield(values, size):
    """Bilinear sampler for a square grid spanning [-size/2, size/2] in both axes."""
    grid = np.asarray(values, dtype=float)
    n = grid.shape[0]-1

    def sample(x, y, transpose=False, flip=False):
        g = grid.T if transpose else grid
        g = g[::-1] if flip else g
        u, v = (np.asarray(x)+size/2)/size*n, (np.asarray(y)+size/2)/size*n
        inside = (u >= 0) & (u < n) & (v >= 0) & (v < n)
        u, v = np.clip(u, 0, n-1e-9), np.clip(v, 0, n-1e-9)
        i, j = v.astype(int), u.astype(int)
        a, b = v-i, u-j
        z = (g[i, j]*(1-a)*(1-b)+g[i, j+1]*(1-a)*b+g[i+1, j]*a*(1-b)+g[i+1, j+1]*a*b)
        return np.where(inside, z, np.nan)
    return sample


def run(args):
    session, output = args.session.resolve(), args.output.resolve()
    manifest = json.loads((session/'session.json').read_text())
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='terrain_audit', motion_commanded=False, session=str(session), fixture=manifest.get('fixture'))
    with output.open('x') as summary:
        rclpy.init()
        node = Audit()
        try:
            end, ready = time.monotonic()+args.wait, None
            while time.monotonic() < end and ready is None:
                rclpy.spin_once(node, timeout_sec=.05)
                ready = node.frame()
            if ready is None:
                raise RuntimeError('No exact-stamp RGB/depth/calibration/terrain/reasons/local grid and image-time transforms')
            report['captures'] = [capture(output.with_name(output.stem+'-frame-0001'), node, ready)]
            until, next_capture = time.monotonic()+args.duration, time.monotonic()+args.interval
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
                if time.monotonic() < next_capture:
                    continue
                candidate = node.frame()
                if candidate is not None:
                    ready = candidate
                    prefix = output.with_name(output.stem+f'-frame-{len(report["captures"])+1:04}')
                    report['captures'].append(capture(prefix, node, ready))
                    next_capture = time.monotonic()+args.interval
            report['capture_span_sim_seconds'] = report['captures'][-1]['stamp']-report['captures'][0]['stamp']
            report['synchronization'] = 'Exact sensor and published terrain stamps; image-time TF; no truth used by navigation'
            depth, info, camera, base = ready
            grid = from_grid(node.map)
            points = apply_tf(depth_points(depth.data, depth.encoding, depth.width, depth.height, depth.step,
                                           depth.is_bigendian, info.k), camera)
            pose = (base.transform.translation.x, base.transform.translation.y)
            frame, layers = traversability(points, .025, 10., pose)
            hazard = frame.data >= 90
            span, slope = np.nan_to_num(layers['height_span']), np.nan_to_num(layers['slope'])
            finite_slopes = layers['slope'][np.isfinite(layers['slope'])]
            report['map'] = dict(known_cells=int((grid.data >= 0).sum()), hazard_cells=int((grid.data >= 90).sum()),
                                 resolution_m=grid.resolution)
            report['frame'] = dict(stamp=stamp_seconds(depth.header.stamp), known_cells=int((frame.data >= 0).sum()),
                                   hazard_cells=int(hazard.sum()), by_step=int((hazard & (span > .03)).sum()),
                                   by_slope=int((hazard & (slope > 12.)).sum()),
                                   slope_p95_deg=float(np.percentile(finite_slopes, 95)) if finite_slopes.size else None)
            yy, xx = np.indices(frame.data.shape)
            fx, fy = frame.x+(xx+.5)*frame.resolution, frame.y+(yy+.5)*frame.resolution
            np.savez_compressed(output.with_suffix('.npz'), map=grid.data, map_origin=[grid.x, grid.y],
                                frame=frame.data, frame_origin=[frame.x, frame.y], **layers)
            rocks = []
            if not manifest.get('fixture'):
                source = Path(manifest['scene'])
                scene = json.loads(source.read_text())
                terrain = json.loads((source.parent/'terrain.json').read_text())
                rocks = [(o['xyz'][0], o['xyz'][1], o['radius_m']) for o in scene['objects']]
                edge = np.full(frame.data.shape, np.inf)
                for rx, ry, rr in rocks:
                    edge = np.minimum(edge, np.hypot(fx-rx, fy-ry)-rr)
                report['frame'].update(hazards_near_rocks=int((hazard & (edge <= .10)).sum()),
                                       hazards_away_from_rocks=int((hazard & (edge > .10)).sum()))
                # Measured ground is relative to the spawn pose; compare shapes, not absolute height.
                measured = layers['elevation']
                ok = np.isfinite(measured) & (edge > .15)
                fits = {}
                for name, key in (('visual', 'heights'), ('collision', 'base_heights')):
                    sample = heightfield(terrain[key], terrain['size_m'])
                    best = None
                    for transpose in (False, True):
                        for flip in (False, True):
                            truth = sample(fx[ok], fy[ok], transpose, flip)
                            good = np.isfinite(truth)
                            residual = measured[ok][good]-truth[good]
                            rms = float(np.sqrt(np.mean((residual-np.median(residual))**2)))
                            if best is None or rms < best[0]:
                                best = (rms, transpose, flip)
                    fits[name] = dict(rms_m=best[0], transpose=best[1], flip=best[2])
                report['measured_vs_heightfield'] = fits
            report['passed'] = True  # capture integrity, NOT terrain accuracy or missing-ground acceptance
            report['acceptance_scope'] = 'Read-only synchronized evidence capture; interpret fixture results separately'
        except Exception as error:
            report['error'] = str(error)
            report['passed'] = False
            try:
                report['partial_sensor_evidence'] = capture_unmapped_sensor(output.with_name(output.stem+'-unmapped'), node)
            except Exception as capture_error:
                report['partial_capture_error'] = str(capture_error)
        finally:
            report['timing'] = {name: timing_summary(samples) for name, samples in node.arrivals.items()}
            if node.map is not None and 'frame' in report:
                image = np.full((*frame.data.shape, 3), 80, np.uint8)
                image[(frame.data >= 0) & (frame.data < 90)] = (200, 240, 200)
                image[hazard & (span > .03)] = (0, 0, 220)
                image[hazard & (span <= .03)] = (0, 160, 255)
                image = cv2.resize(image[::-1], None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)

                def pixel(x, y):
                    return int((x-frame.x)/frame.resolution*2), int((frame.data.shape[0]-(y-frame.y)/frame.resolution)*2)
                for rx, ry, rr in rocks:
                    cv2.circle(image, pixel(rx, ry), max(3, int(rr/frame.resolution*2)), (255, 0, 255), 1)
                cv2.circle(image, pixel(*pose), 6, (255, 0, 0), -1)
                cv2.putText(image, 'single depth frame: green known, red step hazard, orange slope hazard, magenta true rocks',
                            (6, 16), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
                cv2.imwrite(str(output.with_suffix('.png')), image)
            node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2), flush=True)
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--wait', type=float, default=10.)
    parser.add_argument('--duration', type=float, default=0., help='Bounded wall seconds of synchronized evidence, max 120')
    parser.add_argument('--interval', type=float, default=1., help='Seconds between saved bundles, minimum 0.2')
    args = parser.parse_args()
    if not 0 <= args.duration <= 120 or not .2 <= args.interval <= 30 or not 0 < args.wait <= 120:
        parser.error('Require duration 0..120 s, interval 0.2..30 s, wait 0..120 s')
    raise SystemExit(run(args))
