"""Confirm timestamp-aligned rock tracks; never changes terrain or motion state."""
from collections import Counter, OrderedDict, deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray

from rover_perception.live_core import image_to_bgr
from .core import rotation
from .ros_utils import apply_tf, stamp_seconds
from .sample_sites import DEFAULTS, SiteStore
from .site_geometry import color_histogram, depth_image, measure, pose_covariance, visible_region


def key(header):
    return header.stamp.sec, header.stamp.nanosec


def pose(stamped):
    t, q = stamped.transform.translation, stamped.transform.rotation
    return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]


class SampleSites(Node):
    def __init__(self):
        super().__init__('navigation_sample_sites')
        self.declare_parameter('output', '')
        self.output = Path(self.get_parameter('output').value)
        if not self.get_parameter('output').value or self.output.exists():
            raise ValueError('Specify a new sample-site output path; historical tracks are never restored')
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.event_stream = self.output.with_name(self.output.stem+'-events.jsonl').open('x')
        self.event_stream.write(json.dumps(dict(kind='configuration', schema_version=2, defaults=DEFAULTS,
            geometry=dict(plane_samples=30, plane_inlier_fraction=.5, plane_tolerance_m=.015,
                          max_plane_tilt_deg=20, min_relief_m=.03, raised_fraction=.2),
            score_description='engineering evidence score; not calibrated detector accuracy'))+'\n')
        self.store, self.archived = SiteStore(), []
        self.depths, self.infos, self.images = OrderedDict(), OrderedDict(), OrderedDict()
        self.pending, self.seen, self.odometry = deque(maxlen=1), deque(maxlen=256), deque(maxlen=120)
        self.counts, self.guard, self.detector = Counter(), {}, {}
        self.guard_wall = self.detector_wall = self.last_save = 0.
        self.odometry_resets, self.last_processed = None, -math.inf
        self.fault, self.suspension, self.localization_fault = '', 'waiting for synchronized sensor evidence', ''
        self.processing_ms = 0.
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation/sample_site_markers', durable)
        self.tentative_pub = self.create_publisher(MarkerArray, '/navigation/tentative_site_markers', durable)
        self.pub = self.create_publisher(String, '/navigation/sample_sites', durable)
        for topic, cache in (('/oak/depth/image_raw', self.depths), ('/oak/image_raw', self.images),
                             ('/oak/camera_info', self.infos)):
            self.create_subscription(CameraInfo if cache is self.infos else Image, topic,
                                     lambda msg, c=cache: self.cache(c, msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_received, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detections', self.detected, 1)
        self.create_subscription(String, '/navigation/status', self.guard_received, 1)
        self.create_subscription(String, '/rock_detector/status', self.detector_received, 1)
        self.create_timer(.2, self.process)
        self.create_timer(.5, self.publish)

    def cache(self, cache, msg):
        cache[key(msg.header)] = msg
        while len(cache) > 45:
            cache.popitem(last=False)

    def odom_received(self, msg):
        if msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link':
            stamp = stamp_seconds(msg.header.stamp)
            if not self.odometry or stamp > self.odometry[-1][0]:
                if self.odometry:
                    previous_stamp, previous = self.odometry[-1]
                    a, b = previous.pose.pose.position, msg.pose.pose.position
                    dt = stamp-previous_stamp
                    if dt < .5 and np.linalg.norm([a.x-b.x, a.y-b.y, a.z-b.z]) > .05+.2*dt:
                        self.localization_fault = 'localization discontinuity; restart session or await explicit odometry reset'
                    qa, qb = previous.pose.pose.orientation, msg.pose.pose.orientation
                    va, vb = np.array([qa.x, qa.y, qa.z, qa.w]), np.array([qb.x, qb.y, qb.z, qb.w])
                    norm = np.linalg.norm(va)*np.linalg.norm(vb)
                    angle = 2*math.acos(float(np.clip(abs(va @ vb)/norm, 0., 1.))) if norm > 1e-8 else math.inf
                    if dt < .5 and angle > .10+.6*dt:
                        self.localization_fault = 'localization orientation discontinuity; restart session or await explicit odometry reset'
                self.odometry.append((stamp, msg))

    def detector_received(self, msg):
        self.detector, self.detector_wall = json.loads(msg.data), time.monotonic()

    def detected(self, msg):
        if key(msg.header) not in self.seen:
            self.pending.append((msg, time.monotonic()))

    def guard_received(self, msg):
        self.guard, self.guard_wall = json.loads(msg.data), time.monotonic()
        resets = self.guard.get('odometry_resets', 0)
        if self.odometry_resets is not None and resets != self.odometry_resets:
            self.flush_events()
            self.archived.append(dict(epoch=self.store.epoch, reason='odometry_reset',
                                      objects=[self.store.export(x) for x in self.store.objects]))
            self.store = SiteStore(epoch=self.store.epoch+1)
            for collection in (self.pending, self.depths, self.infos, self.images, self.seen, self.odometry):
                collection.clear()
            self.last_processed = -math.inf
            self.localization_fault = ''
            self.store.event('epoch_started', self.guard.get('sim_time', 0.), reason='odometry_reset')
        self.odometry_resets = resets
        if self.guard.get('reason', '').startswith('simulation_clock_reset'):
            self.fault = 'Clock reset; restart session before mapping objects'

    def covariance_at(self, stamp):
        before = [(t, m) for t, m in self.odometry if t <= stamp]
        after = [(t, m) for t, m in self.odometry if t >= stamp]
        if not before or not after or stamp-before[-1][0] > .05 or after[0][0]-stamp > .05:
            raise ValueError('localization covariance missing at image time')
        ta, a = before[-1]
        tb, b = after[0]
        ca, cb = pose_covariance(a.pose.covariance), pose_covariance(b.pose.covariance)
        pa, pb = a.pose.pose.position, b.pose.pose.position
        if np.linalg.norm(np.array([pa.x-pb.x, pa.y-pb.y, pa.z-pb.z])) > .05+.2*(tb-ta):
            raise ValueError('localization discontinuity at image time')
        if tb == ta:
            return ca
        weight = (stamp-ta)/(tb-ta)
        return pose_covariance(((1-weight)*ca+weight*cb).ravel())

    def health(self):
        if self.fault:
            return self.fault
        if self.localization_fault:
            return self.localization_fault
        if not self.guard.get('ready') or time.monotonic()-self.guard_wall > .6:
            return 'localization or guard sensor readiness unavailable'
        if not self.detector.get('ready') or time.monotonic()-self.detector_wall > .6:
            return 'detector unavailable; evidence updates suspended'
        return ''

    def sim_time(self):
        # This node uses use_sim_time. Guard status is sampled more slowly than
        # TF and is not the clock against which a current transform is aged.
        return self.get_clock().now().nanoseconds/1e9

    def flush_events(self):
        while self.store.events:
            self.event_stream.write(json.dumps(self.store.events.popleft(), allow_nan=False)+'\n')
        self.event_stream.flush()

    def process(self):
        self.suspension = self.health()
        if self.suspension or not self.pending:
            return
        msg, received = self.pending.pop()
        stamp_key, stamp = key(msg.header), stamp_seconds(msg.header.stamp)
        if stamp_key in self.seen or stamp <= self.last_processed:
            return
        if time.monotonic()-received > 1. or not -.05 <= self.sim_time()-stamp <= .6:
            self.counts['expired'] += 1
            self.suspension = 'synchronized detection frame is stale'
            return
        depth, info, rgb = self.depths.get(stamp_key), self.infos.get(stamp_key), self.images.get(stamp_key)
        if depth is None or info is None or rgb is None:
            self.pending.append((msg, received))
            self.suspension = 'waiting for exact RGB/depth/calibration alignment'
            return
        if any((m.width, m.height, m.header.frame_id) != (info.width, info.height, info.header.frame_id)
               for m in (depth, rgb)) or msg.header.frame_id != info.header.frame_id:
            self.counts['alignment_rejected'] += 1
            self.seen.append(stamp_key)
            self.suspension = 'RGB/depth/calibration alignment rejected'
            return
        try:
            when = Time.from_msg(msg.header.stamp)
            sensor_tf = self.tf.lookup_transform('odom', info.header.frame_id, when)
            observer_tf = self.tf.lookup_transform('odom', 'drive_center', when)
            covariance = self.covariance_at(stamp)
        except (TransformException, ValueError) as error:
            self.pending.append((msg, received))
            self.suspension = str(error)
            return
        started = time.monotonic()
        self.seen.append(stamp_key)
        self.last_processed = stamp
        sensor, observer = pose(sensor_tf), pose(observer_tf)
        try:
            distances = depth_image(depth.data, depth.encoding, depth.width, depth.height,
                                    depth.step, depth.is_bigendian)
            bgr = image_to_bgr(rgb)
            rows = []
            for detection in msg.detections:
                if not detection.results:
                    continue
                hypothesis = max(detection.results, key=lambda r: r.hypothesis.score).hypothesis
                box = detection.bbox
                center = box.center.position if hasattr(box.center, 'position') else box.center
                xyxy = [center.x-box.size_x/2, center.y-box.size_y/2, center.x+box.size_x/2, center.y+box.size_y/2]
                try:
                    measurement = measure(distances, info.k, xyxy, sensor[:3], sensor[3:], covariance)
                    rows.append(dict(measurement, class_name=hypothesis.class_id, confidence=hypothesis.score,
                                     appearance=color_histogram(bgr, xyxy), box=xyxy))
                except (ValueError, np.linalg.LinAlgError) as error:
                    self.counts['depth_or_class_rejected'] += 1
                    self.store.event('geometry_rejected', stamp, reason=str(error), box=xyxy)
            visible = []
            # A box that cannot be projected could be a real detection of this
            # object. Processing failure is not evidence of object absence.
            if len(rows) == sum(bool(d.results) for d in msg.detections):
                for item in self.store.objects:
                    if item['state'] != 'retired' and visible_region(item, distances, info.k, sensor[:3], sensor[3:])[0]:
                        visible.append(item['track_id'])
            decisions = self.store.update(rows, stamp, observer, sensor[:3], visible)
            self.counts['observations'] += len(rows)
            self.counts['processed_frames'] += 1
            self.store.event('frame', stamp, observations=rows, decisions=decisions,
                             observer=observer, camera=sensor, eligible_visible_misses=visible)
            self.suspension = ''
        except (ValueError, np.linalg.LinAlgError) as error:
            self.suspension = str(error)
            self.store.event('frame_rejected', stamp, reason=str(error))
        self.processing_ms = 1000*(time.monotonic()-started)
        self.flush_events()

    def marker_array(self, objects, tentative=False):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for item in objects:
            for text in (False, True):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'tentative_sites' if tentative else 'sample_sites'
                marker.id = item['id']+(1000000 if text else 0)
                marker.type = Marker.TEXT_VIEW_FACING if text else Marker.LINE_STRIP if tentative else Marker.SPHERE
                marker.action = Marker.ADD
                point = item['map_position']
                marker.pose.position.x, marker.pose.position.y = float(point[0]), float(point[1])
                marker.pose.position.z = float(point[2])+(.25 if text else .08)
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = .12
                marker.scale.z = .14 if text else .12
                if tentative and not text:
                    marker.scale.x = .015
                    for angle in np.linspace(0., 2*math.pi, 33):
                        vertex = Point()
                        vertex.x, vertex.y, vertex.z = float(.06*math.cos(angle)), float(.06*math.sin(angle)), 0.
                        marker.points.append(vertex)
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .75, .1, .4 if tentative else 1.
                marker.text = f"{item['class_name']} #{item['track_id']} {item['state']}"
                markers.markers.append(marker)
        return markers

    def publish(self, emit=True):
        objects, tentative = [], []
        detail = self.fault or self.health() or self.suspension
        if not detail and self.sim_time()-self.last_processed > .6:
            detail = 'no fresh synchronized tracking frame'
        transform_age = None
        try:
            if self.fault:
                raise ValueError(self.fault)
            # RTAB-Map broadcasts map->odom slightly into the future. Request
            # the correction at NOW, rather than interpreting its latest future
            # publication stamp as evidence of a clock fault. TF must bracket
            # the requested time; unavailable/stale transforms still fail.
            projection_time = self.get_clock().now()
            correction = self.tf.lookup_transform('map', 'odom', projection_time)
            transform_age = projection_time.nanoseconds/1e9-stamp_seconds(correction.header.stamp)
            if not -.05 <= transform_age <= .6:
                raise ValueError('map localization is stale; object markers hidden')
            q = correction.transform.rotation
            r = rotation([q.x, q.y, q.z, q.w])
            for item in self.store.objects+self.store.ambiguous:
                if item['state'] == 'retired':
                    continue
                exported = dict(self.store.export(item),
                    map_position=apply_tf([item['odom_position']], correction)[0].tolist(),
                    map_covariance=(r @ np.asarray(item['covariance']) @ r.T).tolist())
                (objects if item['state'] == 'confirmed' else tentative).append(exported)
        except (TransformException, ValueError) as error:
            detail = str(error)
        payload = dict(schema_version=2, frame='odom', map_projection='current map->odom',
                       ready=not bool(detail), error=detail, suspended_reason=detail,
                       evidence_score_description='engineering evidence score; not calibrated detector accuracy',
                       objects=objects, tentative_objects=tentative, stored_count=len(self.store.objects),
                       retired_count=sum(x['state'] == 'retired' for x in self.store.objects),
                       counts=dict(self.counts), archived_epochs=self.archived, epoch=self.store.epoch,
                       odometry_resets=self.odometry_resets, accepted_stamp=self.last_processed if math.isfinite(self.last_processed) else None,
                       map_transform_age_seconds=transform_age,
                       processing_ms=self.processing_ms, defaults=DEFAULTS)
        if emit:
            self.marker_pub.publish(self.marker_array(objects))
            self.tentative_pub.publish(self.marker_array(tentative, True))
            message = String()
            message.data = json.dumps(payload, allow_nan=False)
            self.pub.publish(message)
        if time.monotonic()-self.last_save >= 1.:
            saved = dict(payload, tracks=[self.store.export(x) for x in self.store.objects])
            temporary = self.output.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(saved, indent=2, allow_nan=False)+'\n')
            temporary.replace(self.output)
            self.last_save = time.monotonic()
        self.flush_events()


def main():
    rclpy.init()
    node = SampleSites()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.last_save = 0.
        node.publish(emit=rclpy.ok())
        node.event_stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
