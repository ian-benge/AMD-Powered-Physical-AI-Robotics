#!/usr/bin/env python3
"""Replay controlled optical-depth inputs through actual ROS mapper/guard nodes.

Only ROS_DOMAIN_ID=219 is allowed. The guard output is remapped to a bench-only
topic and remains latched stopped. No Gazebo, drive request, mode-selection
request or detector is used. Synthetic poses/covariance are declared test
inputs, not evidence of real visual odometry accuracy.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from phase3_validate_mapping import CAMERA_Q, HEIGHT, K, WIDTH
from rover_navigation.guard_node import Guard
from rover_navigation.ros_utils import from_grid, stamp_seconds
from rover_navigation.terrain_node import Terrain


def stamp(value):
    from builtin_interfaces.msg import Time
    nanoseconds = round(value*1e9)
    return Time(sec=nanoseconds//1_000_000_000, nanosec=nanoseconds%1_000_000_000)


class Bus(Node):
    def __init__(self):
        super().__init__('phase3_controlled_replay')
        self.lock = threading.Lock()
        self.grids = {'local': {}, 'global': {}}
        self.mapping = {}
        self.commands = []
        self.guard_status = {}
        self.clock_pub = self.create_publisher(Clock, '/clock', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/oak/depth/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/oak/camera_info', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/navigation/local_ground', lambda m: self.grid('local', m), 1)
        self.create_subscription(OccupancyGrid, '/navigation/terrain_map', lambda m: self.grid('global', m), durable)
        self.create_subscription(String, '/navigation/mapping_status', self.status, durable)
        self.create_subscription(String, '/navigation/status', self.guard, 1)
        self.create_subscription(Twist, '/phase3/bench_cmd_vel', self.command, 1)

    def grid(self, name, msg):
        with self.lock:
            self.grids[name][stamp_seconds(msg.header.stamp)] = msg

    def status(self, msg):
        with self.lock:
            self.mapping = json.loads(msg.data)

    def guard(self, msg):
        with self.lock:
            self.guard_status = json.loads(msg.data)

    def command(self, msg):
        with self.lock:
            self.commands.append([msg.linear.x, msg.angular.z])

    def pulse(self, row, elapsed, depth, covariance_phase, clock_elapsed):
        # Deliver the inputs before advancing across their stamp. This gives
        # the real simulation-time timer a tick after delayed TF delivery,
        # without rewinding across the closely spaced first two observations.
        now = row['stamp']-.08+(min(.16, clock_elapsed*.2) if clock_elapsed is not None else 0.)
        self.clock_pub.publish(Clock(clock=stamp(now)))
        camera = row['camera']
        covariance = (np.eye(6)*.0001).ravel().tolist()
        if row['loss'] == 'localization':
            covariance[0] = 1.
        when=(row['stamp']-.01 if covariance_phase=='before' else row['stamp']+.01
              if covariance_phase=='after' else row['stamp']+.011+elapsed*.001)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp(when), 'odom', 'base_link'
        odom.pose.pose.position.x = camera[0]-.2
        odom.pose.pose.position.y = camera[1]
        odom.pose.pose.position.z = camera[2]-.762
        odom.pose.pose.orientation.w = 1.
        odom.pose.covariance = covariance
        self.odom_pub.publish(odom)
        for index in (0, 1):
            # These controlled poses are held constant around the input stamp.
            # Preload their bracket before the mapper's timer reaches the image;
            # a missing bracket must make production mapping wait, not extrapolate.
            tf_stamp = row['stamp']+(-.05 if index == 0 else .05)
            transforms = []
            for parent, child, xyz, q in [('odom', 'base_link', [camera[0]-.2, camera[1], camera[2]-.762], [0., 0., 0., 1.]),
                    ('odom', 'camera_optical_frame', camera, CAMERA_Q)]:
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp(tf_stamp), parent, child
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
                transforms.append(tf)
            if row['loss'] != 'map_correction':
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp(tf_stamp), 'map', 'odom'
                tf.transform.rotation.w = 1.
                transforms.append(tf)
            self.tf.sendTransform(transforms)
        image = Image()
        image.header.stamp, image.header.frame_id = stamp(row['stamp']), 'camera_optical_frame'
        image.height, image.width, image.encoding, image.step = HEIGHT, WIDTH, '32FC1', WIDTH*4
        image.data = depth.tobytes()
        info = CameraInfo()
        info.header, info.height, info.width, info.k = image.header, HEIGHT, WIDTH, K
        self.info_pub.publish(info)
        self.depth_pub.publish(image)


def run(args):
    if os.environ.get('ROS_DOMAIN_ID') != '219' or os.environ.get('PHASE3_MAPPING_BENCH') != '1':
        raise RuntimeError('Requires isolated ROS_DOMAIN_ID=219 and PHASE3_MAPPING_BENCH=1')
    args.output.mkdir(parents=True, exist_ok=False)
    rows = json.loads((args.input/'frames.json').read_text())
    shift = getattr(args, 'pose_shift', (0., 0.))
    for row in rows:
        row['camera'] = [row['camera'][0]+shift[0], row['camera'][1]+shift[1], row['camera'][2]]
    rclpy.init(args=['--ros-args', '-r', '/cmd_vel:=/phase3/bench_cmd_vel',
                    '-p', 'use_sim_time:=true', '-p', 'terrain_backend:='+args.backend,
                    '-p', 'map_size_m:='+str(getattr(args,'map_size',20.)),
                    '-p', 'evidence_output:='+str(args.output/'terrain-evidence-events.jsonl')])
    bus, terrain, guard = Bus(), Terrain(), Guard()
    guard.gate.stop('phase3_bench_latched_stop')
    assert guard.pub.topic_name == '/phase3/bench_cmd_vel'
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (bus, terrain, guard):
        executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    records, error, initial_global, graph_topics = [], None, None, []
    try:
        for number, row in enumerate(rows):
            depth = np.load(args.input/f'frame-{number:03}.npz')['depth']
            assert hashlib.sha256(depth.tobytes()).hexdigest() == row['input_sha256']
            started = time.monotonic()
            clock_started = None
            complete = False
            while time.monotonic()-started < 8.:
                elapsed = time.monotonic()-started
                samples=list(terrain.localization.samples)
                before=any(0 <= row['stamp']-v[0] <= .05 for v in samples)
                after=any(0 <= v[0]-row['stamp'] <= .05 for v in samples)
                # Confirm receipt of each covariance bracket before advancing
                # the controlled clock. A best-effort queue may otherwise drop
                # the single before-sample while newer samples are in flight.
                phase=('refresh' if row['loss']=='localization' or (before and after)
                       else 'before' if not before else 'after')
                if phase=='refresh' and clock_started is None:
                    clock_started=time.monotonic()
                clock_elapsed=time.monotonic()-clock_started if clock_started is not None else None
                bus.pulse(row, elapsed, depth, phase, clock_elapsed)
                time.sleep(.025)
                with bus.lock:
                    mapped = all(row['stamp'] in v for v in bus.grids.values())
                    missing = row['loss'] == 'missing_depth'
                if (elapsed >= .15 and mapped) or (clock_elapsed is not None and clock_elapsed >= .45 and missing):
                    complete = True
                    break
            if not complete:
                raise RuntimeError('No qualified ROS publication/suspension for input '+str(number))
            if row['loss'] == 'missing_depth':
                assert not mapped and terrain.last_stamp != row['stamp'], 'Empty depth refreshed planning maps'
            with bus.lock:
                grids = {name: cache.get(row['stamp']) for name, cache in bus.grids.items()}
                mapping = dict(bus.mapping)
            arrays = {name: from_grid(msg).data for name, msg in grids.items() if msg is not None}
            if args.map_size == 40. and arrays:
                assert arrays['global'].shape == (800,800) and arrays['local'].shape == (240,240)
                local_grid=from_grid(grids['local'])
                x=local_grid.x+(np.arange(240)+.5)*.025
                y=local_grid.y+(np.arange(240)+.5)*.025
                outside=(abs(y)>=20.)[:,None] | (abs(x)>=20.)[None,:]
                assert (arrays['local'][outside]==-1).all(), 'Context terrain expanded the operating area'
            np.savez_compressed(args.output/f'published-{number:03}.npz', **arrays)
            record = dict(input=row, current_stamp_map_publication=mapped,
                          evidence=dict(terrain.evidence.status), mapping_status=mapping)
            records.append(record)
            (args.output/'publications.json').write_text(json.dumps(records, indent=2)+'\n')
            if number == 1 and 'global' in arrays:
                initial_global = arrays['global'].copy()
        evidence = dict(terrain.evidence.status)
        if args.input.name == 'weak-none':
            assert evidence['cleared_cells'] > 0, 'No weak false hazard cleared in ROS'
            latest_global = from_grid(grids['global'])
            assert ((initial_global >= 90) & (latest_global.data >= 0) & (latest_global.data < 90)).any()
            events = [json.loads(x) for x in (args.output/'terrain-evidence-events.jsonl').read_text().splitlines()]
            local = from_grid(grids['local'])
            for event in events:
                if event['kind'] == 'hazards_cleared':
                    for r, c in event['cells']:
                        x, y = terrain.memory.grid.point(r, c)
                        assert 0 <= local.at(x, y) < 90, 'ROS local map did not clear'
        elif args.input.name.startswith('weak-'):
            assert evidence['cleared_cells'] == 0, 'Unsafe weak-hazard clearing'
            if args.input.name not in ('weak-stationary',):
                assert evidence['pending_cells'] == 0 and evidence['suspended'], 'Loss did not suspend clearing'
        elif args.input.name in ('step', 'rock', 'ridge'):
            assert evidence['protected_cells'] > 0, 'Genuine hazard was not retained'
        else:
            assert not (terrain.memory.grid.data >= 90).any(), 'Flat/gentle ground acquired a hazard'
        with bus.lock:
            assert bus.commands and all(v == [0., 0.] for v in bus.commands), 'Nonzero bench guard output'
        assert guard.gate.mode == 'stopped', 'Clearing rearmed the ROS guard'
        # A name-based publisher query itself follows ROS remapping. Inspect
        # actual discovered graph names so the bench publisher is not mistaken
        # for a publisher on its original, explicitly remapped name.
        graph_topics = [name for name, _ in bus.get_topic_names_and_types()]
        assert '/cmd_vel' not in graph_topics, 'Unexpected physical command topic'
    except Exception as exception:
        error = str(exception)
    finally:
        executor.shutdown(timeout_sec=5.)
        worker.join(timeout=5.)
        for node in (terrain, guard, bus):
            node.destroy_node()
        rclpy.shutdown()
        report = dict(result='CONTROLLED_ROS_REPLAY_PASS' if error is None else 'FAIL', error=error,
            case=args.input.name, backend=args.backend, backend_metadata=terrain.compute.metadata,
            map_size_m=args.map_size, controlled_scene_translation_xy=list(shift),
            publications=len(records), guard_mode=guard.gate.mode, zero_guard_outputs=len(bus.commands),
            isolation=dict(ros_domain_id=219, guard_output='/phase3/bench_cmd_vel',
                           mode_selection_requested=False, motion_requested=False, simulator_used=False),
            graph_topics=graph_topics,
            limitation='Controlled encoded-depth and pose/covariance replay; not live rendered-depth or VIO accuracy acceptance.')
        (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report), flush=True)
    return int(error is not None)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--backend', choices=('cpu', 'rocm_http'), default='cpu')
    parser.add_argument('--map-size',type=float,choices=(20.,40.),default=20.)
    parser.add_argument('--pose-shift', type=float, nargs=2, default=(0.,0.), metavar=('X','Y'),
                        help='Translate the complete controlled scene and camera poses together; depth bytes stay identical.')
    raise SystemExit(run(parser.parse_args()))
