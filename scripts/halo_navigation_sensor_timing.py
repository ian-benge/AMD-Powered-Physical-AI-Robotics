#!/usr/bin/env python3
"""Read-only camera and visual-odometry timing; never publishes or selects motion.

Classifies each visual-odometry output gap: camera frames that never arrived
here (upstream rendering/bridge/transport), or RGB/depth pairs that arrived but
produced no odometry. Messages stay serialized; only header stamps are decoded,
so this probe drops far fewer frames than a deserializing Python subscriber.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

STREAMS = (('rgb', Image, '/oak/image_raw'), ('depth', Image, '/oak/depth/image_raw'),
           ('info', CameraInfo, '/oak/camera_info'), ('odom', Odometry, '/odom'))


def stamp(serialized):
    """Header stamp from a CDR message: every stream here begins with std_msgs/Header."""
    order = '<' if serialized[1] & 1 else '>'
    sec, nanosec = struct.unpack_from(order+'iI', serialized, 4)
    return sec+nanosec/1e9


class Timing(Node):
    def __init__(self, events):
        super().__init__('navigation_sensor_timing')
        self.events = events
        self.samples = {name: [] for name, _, _ in STREAMS}
        self.clock = []
        for name, kind, topic in STREAMS:
            self.create_subscription(kind, topic, lambda data, n=name: self.record(n, stamp(data)),
                                     qos_profile_sensor_data, raw=True)
        self.create_subscription(Clock, '/clock', self.tick,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def record(self, name, sim):
        wall = time.monotonic()
        self.samples[name].append((wall, sim))
        self.events.write(json.dumps(dict(stream=name, wall=wall, stamp=sim))+'\n')

    def tick(self, msg):
        wall = time.monotonic()
        if not self.clock or wall-self.clock[-1][0] >= .5:
            self.clock.append((wall, msg.clock.sec+msg.clock.nanosec/1e9))


def gaps(values):
    return [b-a for a, b in zip(values, values[1:])]


def stream_summary(samples):
    walls, stamps = [s[0] for s in samples], sorted(s[1] for s in samples)
    arrival, stamped = gaps(walls), gaps(stamps)
    result = dict(count=len(samples))
    if len(samples) > 1:
        result.update(rate_wall_hz=(len(samples)-1)/(walls[-1]-walls[0]),
                      max_arrival_gap_wall_s=max(arrival), max_stamp_gap_sim_s=max(stamped),
                      arrival_gaps_over_s={str(limit): sum(g > limit for g in arrival) for limit in (.4, .6, 1.)},
                      stamp_gaps_over_s={str(limit): sum(g > limit for g in stamped) for limit in (.1, .4, .6)})
    return result


def classify(samples, limit=.4):
    """Explain odometry stamp gaps using the camera stamps received in between."""
    rgb = {round(s[1], 3) for s in samples['rgb']}
    depth = {round(s[1], 3) for s in samples['depth']}
    odom = sorted(s[1] for s in samples['odom'])
    report = []
    for a, b in zip(odom, odom[1:]):
        if b-a <= limit:
            continue
        inside = lambda values: sorted(v for v in values if a+1e-3 < v < b-1e-3)
        pairs = sorted(set(inside(rgb)) & set(inside(depth)))
        report.append(dict(start_sim=a, gap_sim_s=b-a, rgb_frames=len(inside(rgb)), depth_frames=len(inside(depth)),
                           matched_pairs=len(pairs),
                           cause='pairs_received_without_odometry' if pairs else 'camera_frames_missing_upstream'))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=120.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 5. <= args.duration <= 600.:
        parser.error('Duration must be in [5,600] wall seconds')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as summary, output.with_suffix('.jsonl').open('x') as events:
        rclpy.init()
        node = Timing(events)
        try:
            end = time.monotonic()+args.duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
        finally:
            report = dict(test='sensor_timing', duration_wall_s=args.duration, motion_commanded=False,
                          streams={name: stream_summary(node.samples[name]) for name in node.samples},
                          odometry_gaps=classify(node.samples))
            if len(node.clock) > 1:
                (w0, s0), (w1, s1) = node.clock[0], node.clock[-1]
                report['real_time_factor'] = (s1-s0)/(w1-w0)
            node.destroy_node()
            rclpy.shutdown()
            summary.write(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
