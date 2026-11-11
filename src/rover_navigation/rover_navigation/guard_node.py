"""The only navigation command publisher to the rover's existing Twist stamper."""
import json
import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time

from .core import cause_names, drive_pose, footprint_polygon, swept_clear, swept_conflict, swept_poses
from .ros_utils import from_grid, stamp_seconds
from .safety import SafetyGate


class Guard(Node):
    def __init__(self):
        super().__init__('navigation_motion_guard')
        self.gate = SafetyGate()
        self.declare_parameter('max_linear_speed', .5)
        self.declare_parameter('max_angular_speed', .6)
        linear,angular=(self.get_parameter(k).value for k in ('max_linear_speed','max_angular_speed'))
        if not (math.isfinite(linear) and math.isfinite(angular) and 0<linear<=.5 and 0<angular<=.6):
            raise ValueError('Guard speed limits may only restrict the existing envelope')
        self.gate.MAX_LINEAR_SPEED=linear
        self.gate.MAX_ANGULAR_SPEED=angular
        self.grid = None
        self.reason_grids = {}
        self.in_contact = False
        self.stop_generation = 0
        self.pose = None
        self.last_odom = self.last_grid = -math.inf
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status = self.create_publisher(String, '/navigation/status', 1)
        self.cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.reset_odometry = self.create_client(Empty, '/rgbd_odometry/reset_odom')
        self.armed_once = False
        self.lost_since, self.last_reset, self.odometry_resets = None, -math.inf, 0
        self.create_subscription(ClockMessage, '/clock', self.clock_received,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/navigation/local_ground', self.ground, 1)
        self.create_subscription(OccupancyGrid, '/navigation/local_ground_reasons', self.ground_reasons, 1)
        self.create_subscription(Contacts, '/navigation/chassis_contacts', self.contact, qos_profile_sensor_data)
        for source in ('manual', 'autonomous'):
            self.create_subscription(Twist, '/navigation/'+source+'_cmd_vel',
                                     lambda msg, s=source: self.gate.command(s, msg.linear.x, msg.angular.z, time.monotonic()), 1)
        self.create_service(Trigger, '/navigation/manual', lambda req, res: self.select('manual', res))
        self.create_service(Trigger, '/navigation/enable_autonomy', lambda req, res: self.select('autonomous', res))
        self.create_service(Trigger, '/navigation/stop', self.stop)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.count = 0
        self.last_mode = 'stopped'
        self.published = None
        self.status_sequence = 0

    def clock_received(self, msg):
        stamp = stamp_seconds(msg.clock)
        if self.gate.sim_stamp is not None and stamp < self.gate.sim_stamp:
            self.last_odom = self.last_grid = -math.inf
            self.grid, self.pose = None, None
            self.reason_grids = {}
        self.gate.clock(stamp, time.monotonic())

    def odometry(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        now_sim = self.gate.sim_stamp
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if now_sim is None or not -.05 <= now_sim-stamp <= .5 or stamp <= self.last_odom:
            return
        if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or
                not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                not .9 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.1 or
                not all(math.isfinite(msg.pose.covariance[i]) and 0 <= msg.pose.covariance[i] <= 1.
                        for i in (0, 7, 14, 21, 28, 35))):
            self.gate.stop('invalid_odometry')
            self.gate.sensors.pop('pose', None)
            self.annotate(time.monotonic(), odometry=dict(stamp=stamp,
                frame=msg.header.frame_id, child_frame=msg.child_frame_id,
                covariance_diagonal=[float(msg.pose.covariance[i]) if math.isfinite(msg.pose.covariance[i]) else None
                                     for i in (0, 7, 14, 21, 28, 35)]))
            if not self.armed_once:
                self.recover_startup_odometry(time.monotonic())
            return
        self.lost_since = None
        self.last_odom = stamp
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        # Swept checks project motion about the point the rover actually turns on.
        self.pose = drive_pose(p.x, p.y, yaw)
        self.gate.health('pose', time.monotonic())

    def ground(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        now_sim = self.gate.sim_stamp
        if now_sim is None or not -.05 <= now_sim-stamp <= .6 or stamp <= self.last_grid or msg.header.frame_id != 'odom':
            return
        try:
            self.grid = from_grid(msg)
            self.last_grid = stamp
            self.gate.health('depth', time.monotonic())
        except ValueError:
            self.gate.stop('invalid_ground_grid')
            self.gate.sensors.pop('depth', None)

    def recover_startup_odometry(self, now):
        """Reset visual odometry lost before any mode was ever selected.

        Its first registration can fail while the simulator is still loading
        textures, and it never resets itself. The rover cannot have moved, so
        restarting odometry at its current pose is consistent. After motion has
        been enabled once, lost odometry stays a latched fault.
        """
        self.lost_since = now if self.lost_since is None else self.lost_since
        if now-self.lost_since >= 2. and now-self.last_reset >= 5. and self.reset_odometry.service_is_ready():
            self.reset_odometry.call_async(Empty.Request())
            self.last_reset = now
            self.odometry_resets += 1
            self.get_logger().warning('Visual odometry lost before any motion; requested a reset')

    def ground_reasons(self, msg):
        """Optional explanations must never refresh watchdogs or authorize motion."""
        if msg.header.frame_id != 'odom':
            return
        try:
            grid = from_grid(msg)
        except ValueError:
            return
        stamp = stamp_seconds(msg.header.stamp)
        self.reason_grids[stamp] = grid
        while len(self.reason_grids) > 6:
            self.reason_grids.pop(next(iter(self.reason_grids)))
        # Topics can arrive in either order. Enrich only the matching frozen fault.
        fault = self.gate.fault
        if fault and fault.get('detail', {}).get('grid_stamp') == stamp:
            fault['detail'].update(self.conflict_cause(fault['detail']))

    def conflict_cause(self, detail):
        if 'cell' not in detail:
            return {}
        if detail['cell'] is None:
            return dict(causes=['outside_local_grid'])
        if detail['cell'] < 0:
            return dict(causes=['unknown_terrain'])
        grid = getattr(self, 'reason_grids', {}).get(detail.get('grid_stamp'))
        code = grid.at(detail['x'], detail['y']) if grid else 8
        return dict(causes=cause_names(code if code > 0 else 8))

    def cancel_goal(self):
        if self.cancel.service_is_ready():
            self.cancel.call_async(CancelGoal.Request())

    def contact(self, msg):
        was_in_contact = self.in_contact
        self.in_contact = bool(msg.contacts)
        if self.in_contact and not was_in_contact:
            self.stop_generation += 1
            self.gate.stop('chassis_contact')
            self.pub.publish(Twist())
            self.cancel_goal()
            self.annotate(time.monotonic(), contacts=len(msg.contacts))
            self.publish_status()

    def select(self, mode, response):
        self.cancel_goal()
        if mode == 'manual':
            self.stop_generation += 1
        if self.in_contact:
            response.message = 'Chassis contact is active'
            return response
        if mode == 'autonomous' and not self.tf.can_transform('map', 'base_link', Time()):
            response.message = 'Map localization is not ready'
            return response
        response.success = self.gate.select(mode, time.monotonic())
        self.armed_once = self.armed_once or response.success
        self.last_mode = self.gate.mode
        response.message = self.gate.mode if response.success else self.gate.reason
        self.publish_status()
        if response.success and mode == 'autonomous':
            # Status published before this one may arrive after the response.
            response.message = json.dumps(dict(mode=mode, stop_generation=self.stop_generation,
                                               status_sequence=self.status_sequence))
        return response

    def stop(self, request, response):
        self.stop_generation += 1
        self.gate.stop('operator_stop')
        self.last_mode = self.gate.mode
        self.cancel_goal()
        self.pub.publish(Twist())
        self.publish_status()
        response.success, response.message = True, 'Stopped; explicit mode selection required to resume'
        return response

    def tick(self):
        now = time.monotonic()
        detail = {}
        if self.gate.mode == 'autonomous':
            try:
                tf = self.tf.lookup_transform('map', 'base_link', Time())
                age = self.gate.sim_stamp-stamp_seconds(tf.header.stamp)
                if age > .6:
                    self.gate.stop('map_localization_stale')
                    detail['map_transform_age_seconds'] = age
            except TransformException as error:
                self.gate.stop('map_localization_lost')
                detail['map_error'] = str(error)
        rejected = []

        def clear(v, w):
            if self.grid is not None and self.pose is not None and swept_clear(self.grid, self.pose, v, w):
                return True
            rejected.append((v, w))
            return False

        v, w = self.gate.output(now, clear)
        if rejected and self.gate.reason == 'unsafe_or_unobserved_ground':
            detail.update(self.rejection(*rejected[-1]))
        self.annotate(now, **detail)
        if self.last_mode != self.gate.mode:
            self.cancel_goal()
            self.last_mode = self.gate.mode
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.pub.publish(msg)
        self.count += 1
        if self.count % 4 == 0 or (self.gate.mode, self.gate.reason) != self.published:
            self.publish_status([v, w])

    def rejection(self, v, w):
        detail = dict(command=[v, w], frame='odom', grid_stamp=self.last_grid if math.isfinite(self.last_grid) else None,
                      sim_time=self.gate.sim_stamp)
        if self.grid is None or self.pose is None:
            detail['missing'] = 'ground' if self.grid is None else 'pose'
        else:
            detail.update(swept_conflict(self.grid, self.pose, v, w) or {})
            detail['start_pose'] = list(self.pose)
            detail['sweep'] = [footprint_polygon(p) for p in swept_poses(self.pose, v, w)]
            if 'x' in detail:
                row, col = self.grid.cell(detail['x'], detail['y'])
                cx, cy = self.grid.point(row, col)
                detail.update(row=row, column=col, cell_center=[cx, cy], resolution=self.grid.resolution)
            detail.update(self.conflict_cause(detail))
        return detail

    def ages(self, now):
        return dict(depth_age_seconds=now-self.gate.sensors['depth'] if 'depth' in self.gate.sensors else None,
                    pose_age_seconds=now-self.gate.sensors['pose'] if 'pose' in self.gate.sensors else None,
                    clock_age_seconds=now-self.gate.clock_received if self.gate.sim_stamp is not None else None)

    def annotate(self, now, **detail):
        if self.gate.fault is not None and 'detail' not in self.gate.fault:
            self.gate.fault['detail'] = dict(self.ages(now), last_valid_pose=list(self.pose) if self.pose else None,
                                            pose_stamp=self.last_odom if math.isfinite(self.last_odom) else None,
                                            **detail)

    def publish_status(self, command=None):
        now = time.monotonic()
        self.published = self.gate.mode, self.gate.reason
        self.status_sequence += 1
        status = String()
        command = command or [0., 0.]
        geometry = dict(frame='odom', stamp=self.last_odom, horizon_seconds=1., padding_m=.025,
                        footprint=footprint_polygon(self.pose),
                        sweep=[footprint_polygon(p) for p in swept_poses(self.pose, *command)]) if self.pose else None
        status.data = json.dumps(dict(mode=self.gate.mode, reason=self.gate.reason, fault=self.gate.fault,
                                     stop_generation=self.stop_generation, sequence=self.status_sequence,
                                     odometry_resets=self.odometry_resets,
                                     ready=self.gate.ready(now), command=command, geometry=geometry,
                                     sim_time=self.gate.sim_stamp, **self.ages(now)))
        self.status.publish(status)


def main():
    rclpy.init()
    node = Guard()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
