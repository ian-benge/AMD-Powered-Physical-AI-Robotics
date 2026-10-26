"""Independent ROS recorder: simulator truth is consumed ONLY by this evaluator."""
import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry, Path as PathMessage
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Contacts
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .ros_utils import stamp_seconds


class Recorder(Node):
    def __init__(self):
        super().__init__('navigation_evaluator')
        self.declare_parameter('output', '')
        path = Path(self.get_parameter('output').value).expanduser()
        if not self.get_parameter('output').value:
            raise ValueError('Specify a new output JSONL path')
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open('x', encoding='utf-8')
        self.truth_home = None
        self.latest = {}
        self.write('session', dict(wall_time=time.time(), acceptance_position_m=.15, acceptance_yaw_rad=math.radians(10)))
        self.create_subscription(Odometry, '/odom_gt', self.truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda msg: self.odom('estimate', msg), qos_profile_sensor_data)
        self.create_subscription(Contacts, '/navigation/chassis_contacts',
                                 lambda msg: self.write('chassis_contacts', dict(sim_time=stamp_seconds(msg.header.stamp),
                                                                                count=len(msg.contacts))), qos_profile_sensor_data)
        for topic in ('status', 'mission_status'):
            self.create_subscription(String, '/navigation/'+topic,
                                     lambda msg, t=topic: self.event(t, msg), 10)
        self.create_subscription(PoseStamped, '/navigation/selected_goal',
                                 lambda msg: self.write('selected_goal', self.pose_event(msg)), 1)
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        for topic in ('/navigation/preview_outward', '/navigation/preview_return', '/plan'):
            self.create_subscription(PathMessage, topic, lambda msg, name=topic: self.write('path', dict(
                topic=name, sim_time=stamp_seconds(msg.header.stamp), frame=msg.header.frame_id,
                poses=[self.pose_event(p)['pose'] for p in msg.poses])), durable if topic != '/plan' else 1)
        self.create_timer(1., self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def write(self, kind, data):
        self.file.write(json.dumps(dict(kind=kind, wall_monotonic=time.monotonic(), **data), allow_nan=False)+'\n')

    @staticmethod
    def pose_event(msg):
        p, q = msg.pose.position, msg.pose.orientation
        return dict(sim_time=stamp_seconds(msg.header.stamp), frame=msg.header.frame_id,
                    pose=[p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])

    def odom(self, kind, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
            self.write('invalid_'+kind, dict(sim_time=stamp_seconds(msg.header.stamp)))
            return
        self.latest[kind] = [p.x, p.y, yaw]
        self.write(kind, dict(sim_time=stamp_seconds(msg.header.stamp), pose=self.latest[kind]))

    def truth(self, msg):
        self.odom('truth', msg)

    def event(self, kind, msg):
        data = json.loads(msg.data)
        self.write(kind, data)
        if (kind == 'mission_status' and data.get('mission') != 'idle' and
                self.truth_home is None and 'truth' in self.latest):
            self.truth_home = list(self.latest['truth'])

    def flush(self):
        if self.truth_home is not None:
            pose, home = self.latest['truth'], self.truth_home
            distance = math.hypot(pose[0]-home[0], pose[1]-home[1])
            yaw_error = abs(math.atan2(math.sin(pose[2]-home[2]), math.cos(pose[2]-home[2])))
            self.write('home_error', dict(position_m=distance, yaw_rad=yaw_error,
                                          within_tolerance=distance <= .15 and yaw_error <= math.radians(10)))
        self.file.flush()


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    finally:
        node.flush()
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()
