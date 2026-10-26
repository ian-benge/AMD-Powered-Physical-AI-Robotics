#!/usr/bin/env python3
"""Inspect terrain/TF/lifecycle and request a path; never publish motion/goals.

The only action used is ComputePathToPose. This is not a NavigateToPose test.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from rover_navigation.core import footprint_clear
from rover_navigation.goal_checks import check_path
from rover_navigation.ros_utils import from_grid


def wait(node, future, seconds=10.):
    end = time.monotonic()+seconds
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
    if not future.done():
        raise RuntimeError('ROS request timed out')
    return future.result()


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Inspector(Node):
    def __init__(self):
        super().__init__('navigation_map_inspection')
        self.map = self.status = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/navigation/terrain_map', self.receive_map, durable)
        self.create_subscription(String, '/navigation/status', self.receive_status, 1)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    def receive_map(self, msg):
        self.map = msg

    def receive_status(self, msg):
        self.status = json.loads(msg.data)


def render(grid, pose, target, path, filename):
    row, col = grid.cell(*pose[:2])
    r0, r1 = max(0, row-40), min(grid.data.shape[0], row+41)
    c0, c1 = max(0, col-40), min(grid.data.shape[1], col+41)
    data = grid.data[r0:r1, c0:c1]
    image = np.full((*data.shape, 3), 80, dtype=np.uint8)
    image[(data >= 0) & (data < 90)] = [200, 240, 200]
    image[data >= 90] = [60, 60, 220]
    image = cv2.resize(image[::-1], None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST)

    def pixel(x, y):
        r, c = grid.cell(x, y)
        return int((c-c0)*8+4), int((r1-1-r)*8+4)

    if path:
        points = np.array([pixel(p.pose.position.x, p.pose.position.y) for p in path], np.int32)
        cv2.polylines(image, [points], False, (255, 180, 0), 2)
    cv2.circle(image, pixel(*pose[:2]), 6, (255, 0, 0), -1)
    tip = pixel(pose[0]+.25*math.cos(pose[2]), pose[1]+.25*math.sin(pose[2]))
    cv2.arrowedLine(image, pixel(*pose[:2]), tip, (255, 0, 0), 2)
    cv2.drawMarker(image, pixel(*target), (0, 140, 255), cv2.MARKER_CROSS, 16, 2)
    cv2.putText(image, 'Terrain: green known, gray unknown, red hazard', (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, 'Blue rover; orange goal; cyan planned path; no drive', (8, 38),
                cv2.FONT_HERSHEY_SIMPLEX, .40, (255, 255, 255), 1, cv2.LINE_AA)
    if not cv2.imwrite(str(filename), image):
        raise RuntimeError('Could not write map preview')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--distance', type=float, default=.35)
    parser.add_argument('--return-path', action='store_true', help='Plan from the proposed goal back to current home')
    args = parser.parse_args()
    if not math.isfinite(args.distance) or not 0 < args.distance <= 3.:
        parser.error('Distance must be in (0,3] metres')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    report = dict(test='terrain_and_compute_path', passed=False, motion_commanded=False)
    with output.open('x', encoding='utf-8') as file:
        rclpy.init()
        node = Inspector()
        path = []
        try:
            until = time.monotonic()+5.
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            if node.map is None or node.status is None or not node.status['ready']:
                raise RuntimeError('Fresh mapping/status not available')
            if node.status['mode'] != 'stopped':
                raise RuntimeError('Map inspection requires stopped motion')
            if node.map.header.frame_id != 'map':
                raise RuntimeError('Unexpected terrain map frame')
            grid = from_grid(node.map)
            t = node.tf.lookup_transform('map', 'drive_center', Time())
            pose = [t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)]
            target = [pose[0]+args.distance*math.cos(pose[2]), pose[1]+args.distance*math.sin(pose[2])]
            report.update(map_pose=pose, proposed_target=target, resolution_m=grid.resolution,
                          known_cells=int(np.sum(grid.data >= 0)), hazardous_cells=int(np.sum(grid.data >= 90)),
                          footprint_known_clear=footprint_clear(grid, *pose),
                          target_footprint_known_clear=footprint_clear(grid, *target, pose[2]))
            report['lifecycle'] = {}
            for name in ('planner_server', 'controller_server', 'bt_navigator'):
                client = node.create_client(GetState, '/'+name+'/get_state')
                until = time.monotonic()+4.
                while not client.service_is_ready() and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.02)
                if not client.service_is_ready():
                    raise RuntimeError('Lifecycle service unavailable: '+name)
                state = wait(node, client.call_async(GetState.Request())).current_state
                report['lifecycle'][name] = dict(id=state.id, label=state.label)
                if state.id != 3:
                    raise RuntimeError('Inactive Nav2 component: '+name)
            if not report['footprint_known_clear'] or not report['target_footprint_known_clear']:
                raise RuntimeError('Current/proposed footprint is not verified clear')
            if not node.planner.wait_for_server(timeout_sec=4.):
                raise RuntimeError('Planner action unavailable')
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.header.stamp = node.map.header.stamp
            goal.goal.pose.position.x, goal.goal.pose.position.y = target
            goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
            goal.planner_id = 'GridBased'
            goal.use_start = False
            if args.return_path:
                goal.start.header.frame_id = 'map'
                goal.start.header.stamp = node.map.header.stamp
                goal.start.pose.position.x, goal.start.pose.position.y = target
                goal.start.pose.orientation = goal.goal.pose.orientation
                goal.goal.pose.position.x, goal.goal.pose.position.y = pose[:2]
                goal.use_start = True
            requested = time.monotonic()
            handle = wait(node, node.planner.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Planner rejected path request')
            try:
                result = wait(node, handle.get_result_async(), 12.)
            except Exception:
                handle.cancel_goal_async()
                raise
            report.update(action_status=result.status, planning_wall_s=time.monotonic()-requested,
                          error_code=getattr(result.result, 'error_code', None),
                          error_message=getattr(result.result, 'error_msg', ''))
            path = list(result.result.path.poses)
            report['path_poses'] = len(path)
            report['path_length_m'] = sum(math.hypot(b.pose.position.x-a.pose.position.x,
                                                     b.pose.position.y-a.pose.position.y)
                                          for a, b in zip(path, path[1:]))
            report['all_path_footprints_known_clear'] = bool(path) and all(
                footprint_clear(grid, p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)) for p in path)
            report['poses'] = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)] for p in path]
            report['in_place_rotation_poses'] = sum(math.dist(a[:2], b[:2]) < 1e-3 and abs(a[2]-b[2]) > 1e-3
                                                    for a, b in zip(report['poses'], report['poses'][1:]))
            report['unsafe_poses'] = [p for p in report['poses'] if not footprint_clear(grid, *p)]
            report['return_path'] = args.return_path
            if result.status == 4 and path:
                check_path(grid, report['poses'], [*target, pose[2]] if args.return_path else pose,
                           pose if args.return_path else [*target, pose[2]])
            if result.status != 4 or not path or not report['all_path_footprints_known_clear']:
                raise RuntimeError('No successful path confined to verified ground')
            report['passed'] = True
        except Exception as error:
            report['error'] = str(error)
        finally:
            if node.map is not None and 'map_pose' in report:
                render(from_grid(node.map), report['map_pose'], report['proposed_target'], path, output.with_suffix('.png'))
            file.write(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report, indent=2), flush=True)
            node.destroy_node()
            rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
