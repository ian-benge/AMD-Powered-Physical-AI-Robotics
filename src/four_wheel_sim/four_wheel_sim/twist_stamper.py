import rclpy
import math
import time
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.frame = self.declare_parameter('frame_id', 'base_link').value
        self.timeout = self.declare_parameter('watchdog_timeout_wall_sec', 0.0).value
        if not math.isfinite(self.timeout) or self.timeout < 0:
            raise ValueError('watchdog_timeout_wall_sec must be finite and nonnegative')
        self.received = -math.inf
        depth = 1 if self.timeout else 10
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', depth)
        self.create_subscription(Twist, '/cmd_vel', self.cb, depth)
        if self.timeout:
            self.create_timer(.05, self.expire, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def cb(self, msg):
        self.received = time.monotonic()
        self.publish(msg)

    def expire(self):
        if self.timeout and time.monotonic()-self.received > self.timeout:
            self.publish(Twist())

    def publish(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.timeout:
        node.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
