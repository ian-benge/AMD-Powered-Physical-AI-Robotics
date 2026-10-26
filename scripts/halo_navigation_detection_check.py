#!/usr/bin/env python3
"""Read-only capture of image alignment, detector output and persistent markers."""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray


class Capture(Node):
    def __init__(self, stream):
        super().__init__('navigation_detection_evidence')
        self.stream, self.latest, self.counts = stream, {}, Counter()
        self.stamps = {name: deque(maxlen=600) for name in ('rgb', 'depth', 'info', 'detections', 'annotated')}
        self.classes, self.jpeg = Counter(), None
        self.first_sites = None
        for name, topic, kind in (('rgb', '/oak/image_raw', Image), ('depth', '/oak/depth/image_raw', Image),
                                  ('info', '/oak/camera_info', CameraInfo),
                                  ('annotated', '/navigation/annotated_image/compressed', CompressedImage)):
            self.create_subscription(kind, topic, lambda m, n=name: self.image(n, m), qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detections', self.detection, 1)
        for name, topic in (('guard', '/navigation/status'), ('detector', '/rock_detector/status'),
                             ('sites', '/navigation/sample_sites')):
            self.create_subscription(String, topic, lambda m, n=name: self.json(n, m), 1)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(MarkerArray, '/navigation/sample_site_markers', self.markers, durable)

    def record(self, name, data):
        self.stream.write(json.dumps(dict(kind=name, wall=time.monotonic(), **data), allow_nan=False)+'\n')

    def image(self, name, msg):
        self.stamps[name].append((msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id))
        self.counts[name] += 1
        if name == 'annotated':
            self.jpeg = bytes(msg.data)

    def detection(self, msg):
        self.stamps['detections'].append((msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id))
        self.counts['detections'] += 1
        rows = []
        for item in msg.detections:
            for result in item.results:
                h = result.hypothesis
                self.classes[h.class_id] += 1
                rows.append(dict(class_name=h.class_id, confidence=h.score))
        self.record('detections', dict(stamp=list(self.stamps['detections'][-1]), detections=rows))

    def json(self, name, msg):
        self.latest[name] = json.loads(msg.data)
        if name == 'sites' and self.first_sites is None:
            self.first_sites = self.latest[name]
        self.record(name, self.latest[name])

    def markers(self, msg):
        self.counts['marker_arrays'] += 1
        self.latest['marker_count'] = sum(m.action == m.ADD for m in msg.markers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=5.)
    args = parser.parse_args()
    if not 1 <= args.duration <= 30:
        parser.error('Duration must be 1–30 wall seconds')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as summary, args.output.with_suffix('.jsonl').open('x') as stream:
        rclpy.init()
        node = Capture(stream)
        started = time.monotonic()
        try:
            while time.monotonic()-started < args.duration:
                rclpy.spin_once(node, timeout_sec=.01)
            report = dict(duration_wall_s=time.monotonic()-started, counts=dict(node.counts),
                          detected_classes=dict(node.classes), first_sites=node.first_sites, **node.latest)
            report['aligned_stamps'] = len(set.intersection(*(set(v) for v in node.stamps.values())))
            report['command_owners'] = [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
            report['passed'] = bool(report['aligned_stamps'] and node.latest.get('detector', {}).get('ready') and
                                    node.latest.get('sites', {}).get('ready') and
                                    node.latest.get('sites', {}).get('stored_count', 0) and
                                    node.latest.get('marker_count', 0) and
                                    report['command_owners'] == ['navigation_motion_guard'])
            summary.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
            if node.jpeg:
                args.output.with_suffix('.jpg').write_bytes(node.jpeg)
            print(json.dumps({k: v for k, v in report.items() if k not in ('sites', 'first_sites')}, indent=2))
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
