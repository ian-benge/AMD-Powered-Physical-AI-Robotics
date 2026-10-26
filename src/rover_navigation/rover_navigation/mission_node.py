"""Explicit waypoint/exploration missions using Nav2 actions and safe return budgets."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .core import footprint_clear, frontiers, reachable, return_required
from .goal_checks import check_path, pose_values
from .ros_utils import from_grid


class Mission(Node):
    def __init__(self):
        super().__init__('navigation_mission')
        self.declare_parameter('budget_seconds', 600.)
        self.declare_parameter('waypoints', '')  # JSON [[x,y,yaw], ...], in map metres/radians
        self.declare_parameter('navigation_speed', .10)
        self.declare_parameter('mission_timeout_wall_sec', 300.)
        self.declare_parameter('home_yaw_tolerance_deg', 5.)
        self.declare_parameter('home_position_tolerance_m', .08)
        self.declare_parameter('home_reapproaches', 2)
        self.home_attempts = 0
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.guard_wall = -math.inf
        self.guard_sequence = 0
        self.selection_generation = 0
        self.selected = self.preview_home = None
        self.selection_state, self.selection_reason = 'none', ''
        self.preview_paths = []
        self.starting = False
        self.start_epoch = None
        self.started_wall = 0.
        self.last_route_check = -math.inf
        self.trial = 0
        self.outbound_reached = False
        self.grid = None
        self.guard = {}
        self.mode, self.home, self.started = 'idle', None, 0.
        self.goal = None
        self.pending = False
        self.generation = 0
        self.visited = []
        self.route = []
        self.result = ''
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.stop_client = self.create_client(Trigger, '/navigation/stop')
        self.enable_client = self.create_client(Trigger, '/navigation/enable_autonomy')
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/navigation/terrain_map', self.map_received, durable)
        self.create_subscription(String, '/navigation/status', self.guard_received, 1)
        self.pub = self.create_publisher(String, '/navigation/mission_status', 1)
        self.outward_pub = self.create_publisher(Path, '/navigation/preview_outward', durable)
        self.return_pub = self.create_publisher(Path, '/navigation/preview_return', durable)
        self.markers = self.create_publisher(MarkerArray, '/navigation/mission_markers', durable)
        self.create_subscription(PoseStamped, '/navigation/selected_goal', self.select_goal, 1)
        self.create_service(Trigger, '/navigation/start_selected_goal', self.start_selected,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, '/navigation/start_waypoints', lambda req, res: self.start('waypoints', res))
        self.create_service(Trigger, '/navigation/start_exploration', lambda req, res: self.start('exploring', res))
        self.create_service(Trigger, '/navigation/return_home', self.return_home)
        self.create_timer(.2, self.tick, clock=self.wall_clock)

    def map_received(self, msg):
        if msg.header.frame_id == 'map':
            self.grid = from_grid(msg)

    def guard_received(self, msg):
        status = json.loads(msg.data)
        if status.get('sequence', math.inf) < self.guard_sequence:
            return  # published before the current arming; not a takeover signal
        self.guard = status
        self.guard_wall = time.monotonic()
        if self.starting and self.start_epoch is not None and (self.guard.get('stop_generation') != self.start_epoch or not self.guard.get('ready')):
            self.abort('Start cancelled by Stop, takeover or safety fault ('+self.guard_cause()+')')
        if self.mode != 'idle' and self.guard.get('mode') != 'autonomous':
            self.abort('Motion guard '+str(self.guard.get('mode'))+' ('+self.guard_cause()+'); mission cancelled')

    def guard_cause(self):
        fault = self.guard.get('fault') or {}
        return fault.get('reason') or self.guard.get('reason') or ('not ready' if not self.guard.get('ready') else 'takeover')

    def selected_pose(self, values):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose.position.x, result.pose.position.y = float(values[0]), float(values[1])
        result.pose.orientation.z = math.sin(values[2]/2)
        result.pose.orientation.w = math.cos(values[2]/2)
        return result

    def select_goal(self, msg):
        if self.mode != 'idle' or self.starting:
            self.selection_reason = 'Stop the current mission before selecting another destination'
            self.publish_status()
            return
        self.selection_generation += 1
        token = self.selection_generation
        self.preview_paths = []
        try:
            self.selected = pose_values(msg)
            self.selection_state, self.selection_reason = 'checking', 'Checking outward and return paths'
            self.executor.create_task(self.preview, token)
        except ValueError as error:
            self.selected = None
            self.selection_state, self.selection_reason = 'rejected', str(error)
        self.publish_status()

    def valid_token(self, token):
        if token != self.selection_generation:
            raise ValueError('Selection or Start was cancelled')

    async def wait(self, future, token, timeout=8.):
        end = time.monotonic()+timeout
        while not future.done():
            self.valid_token(token)
            if time.monotonic() >= end:
                raise ValueError('Navigation service/action timed out')
            from rclpy.task import Future
            wake = Future()
            timer = self.create_timer(.02, lambda: wake.set_result(None) if not wake.done() else None,
                                      clock=self.wall_clock)
            try:
                await wake
            finally:
                self.destroy_timer(timer)
        self.valid_token(token)
        return future.result()

    async def planned_path(self, start, target, token):
        if not self.planner.server_is_ready():
            raise ValueError('Nav2 planner is not active')
        request = ComputePathToPose.Goal()
        request.start, request.goal = self.selected_pose(start), self.selected_pose(target)
        request.use_start, request.planner_id = True, 'GridBased'
        pending = self.planner.send_goal_async(request)
        handle = None
        try:
            handle = await self.wait(pending, token)
            if not handle.accepted:
                raise ValueError('Planner rejected the path request')
            answer = await self.wait(handle.get_result_async(), token)
            if answer.status != GoalStatus.STATUS_SUCCEEDED or answer.result.error_code:
                raise ValueError('No known clear path: '+answer.result.error_msg)
            return answer.result.path
        finally:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            elif handle is None:
                pending.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result().accepted else None)

    def check_preview(self, start, home):
        if self.grid is None or self.selected is None:
            raise ValueError('A terrain map and selected destination are required')
        for pose in (start, self.selected, home):
            if not footprint_clear(self.grid, *pose):
                raise ValueError('Rover, destination or home footprint is hazardous or unknown')
        if len(self.preview_paths) == 2:
            for path, a, b in ((self.preview_paths[0], start, self.selected),
                               (self.preview_paths[1], self.selected, home)):
                check_path(self.grid, [pose_values(p) for p in path.poses], a, b)

    async def validate(self, token):
        start = self.pose()
        home = self.home or start
        self.preview_paths = []
        self.check_preview(start, home)
        outward = await self.planned_path(start, self.selected, token)
        back = await self.planned_path(self.selected, home, token)
        self.preview_paths = [outward, back]
        # Recheck the current map after both asynchronous planner responses.
        self.check_preview(self.pose(), home)
        self.preview_home = home
        self.outward_pub.publish(outward)
        self.return_pub.publish(back)
        self.selection_state, self.selection_reason = 'ready', 'Outward and return paths verified'

    async def preview(self, token):
        try:
            await self.validate(token)
        except Exception as error:
            if token == self.selection_generation:
                self.preview_paths = []
                self.selection_state, self.selection_reason = 'rejected', str(error)
        self.publish_status()

    async def start_selected(self, request, response):
        if (self.mode != 'idle' or self.starting or self.selected is None or
                self.selection_state != 'ready' or self.guard.get('mode') != 'stopped' or
                not self.guard.get('ready') or time.monotonic()-self.guard_wall > .6):
            response.message = 'Require a verified destination, stopped ready guard and idle mission'
            return response
        self.selection_generation += 1
        token = self.selection_generation
        self.start_epoch = self.guard.get('stop_generation')
        self.starting = True
        armed = False
        enable_pending = None
        try:
            self.selection_state = 'checking'
            await self.validate(token)
            if not self.enable_client.service_is_ready() or not self.nav.server_is_ready():
                raise ValueError('Motion guard or Nav2 is unavailable')
            if time.monotonic()-self.guard_wall > .6 or not self.guard.get('ready'):
                raise ValueError('Guard readiness is stale')
            enable_pending = self.enable_client.call_async(Trigger.Request())
            enabled = await self.wait(enable_pending, token)
            armed = enabled.success
            if not enabled.success:
                raise ValueError(enabled.message)
            reply = json.loads(enabled.message)
            if reply['stop_generation'] != self.start_epoch:
                raise ValueError('Stop or takeover occurred while enabling autonomy')
            self.valid_token(token)
            self.guard_sequence = reply.get('status_sequence', 0)
            self.home = self.home or self.preview_home
            self.begin('waypoints', [self.selected])
            response.success, response.message = True, 'Waypoint and automatic return started'
        except Exception as error:
            if token == self.selection_generation:
                self.selection_state, self.selection_reason = 'rejected', str(error)
            response.message = str(error)
            if armed and self.stop_client.service_is_ready():
                self.stop_client.call_async(Trigger.Request())
        finally:
            if enable_pending is not None and not response.success:
                # If Stop cancelled the await before the arm response arrived,
                # also stop AFTER that late response. Never leave a late arm live.
                def stop_late(_):
                    if self.stop_client.service_is_ready():
                        self.stop_client.call_async(Trigger.Request())
                enable_pending.add_done_callback(stop_late)
            self.starting = False
            self.publish_status()
        return response

    def begin(self, mode, route):
        self.mode, self.route = mode, list(route)
        self.started = self.get_clock().now().nanoseconds/1e9
        self.started_wall = time.monotonic()
        self.last_route_check = -math.inf
        self.visited, self.result = [], ''
        self.outbound_reached = False
        self.home_attempts = 0
        self.trial += 1
        self.generation += 1
        self.publish_status()

    def pose(self):
        tf = self.tf.lookup_transform('map', 'drive_center', Time())
        p, q = tf.transform.translation, tf.transform.rotation
        return p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def start(self, mode, response):
        if self.mode != 'idle' or self.grid is None or self.guard.get('mode') != 'autonomous' or not self.nav.server_is_ready():
            response.message = 'Require idle mission, terrain map, enabled autonomy and active Nav2'
            return response
        try:
            route = json.loads(self.get_parameter('waypoints').value or '[]') if mode == 'waypoints' else []
            if mode == 'waypoints' and (not route or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in route)):
                raise ValueError('Configure waypoints as a nonempty JSON array of [x,y,yaw]')
            pose = self.pose()
        except (ValueError, TypeError, TransformException) as error:
            response.message = str(error)
            return response
        self.home = self.home or pose
        self.begin(mode, route)
        response.success, response.message = True, mode
        return response

    def return_home(self, request, response):
        if self.home is None or self.guard.get('mode') != 'autonomous':
            response.message = 'Home and enabled autonomy are required'
            return response
        self.cancel()
        self.mode = 'returning'
        self.started_wall = time.monotonic()
        response.success, response.message = True, 'Returning home'
        return response

    def cancel(self):
        self.generation += 1
        if self.goal is not None:
            self.goal.cancel_goal_async()
        self.goal, self.pending = None, False

    def abort(self, reason):
        self.selection_generation += 1
        self.start_epoch = None
        self.guard_sequence = 0
        self.cancel()
        self.mode = 'idle'
        self.result = reason
        self.publish_status()
        self.get_logger().warning(reason)
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())

    def send(self, target):
        if self.grid.at(target[0], target[1]) < 0 or self.grid.at(target[0], target[1]) >= 90:
            self.abort('Goal is hazardous or unobserved')
            return
        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = float(target[0]), float(target[1])
        request.pose.pose.orientation.z = math.sin(target[2]/2)
        request.pose.pose.orientation.w = math.cos(target[2]/2)
        self.pending = True
        generation = self.generation

        def accepted(future):
            handle = future.result()
            if generation != self.generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.pending = False
            if not handle.accepted:
                self.abort('Nav2 rejected the goal')
                return
            self.goal = handle
            handle.get_result_async().add_done_callback(completed)

        def completed(future):
            if generation != self.generation:
                return
            self.goal = None
            answer = future.result()
            if answer.status != GoalStatus.STATUS_SUCCEEDED:
                if self.mode == 'exploring':
                    self.visited.append(tuple(target[:2]))
                else:
                    self.abort('Navigation failed ('+self.nav2_failure(answer)+'); no automatic motion recovery')
            elif self.mode == 'returning':
                self.finish_return()
            elif self.mode == 'aligning':
                self.abort('Home reached')
            elif self.mode == 'waypoints':
                self.outbound_reached = True
                self.route.pop(0)
            else:
                self.visited.append(tuple(target[:2]))

        self.nav.send_goal_async(request).add_done_callback(accepted)

    def nav2_failure(self, answer):
        name = {5: 'cancelled', 6: 'aborted'}.get(answer.status, 'status '+str(answer.status))
        detail = 'Nav2 '+name+', error '+str(getattr(answer.result, 'error_code', 0))
        message = getattr(answer.result, 'error_msg', '')
        if message:
            detail += ': '+message
        if self.guard.get('fault'):
            detail += '; guard fault '+self.guard['fault'].get('reason', '')
        return detail

    def finish_return(self):
        try:
            pose = self.pose()
        except TransformException:
            self.abort('Localization unavailable')
            return
        # Nav2's goal check latches position before its final turn; bounded
        # re-approaches catch any position lost during that turn.
        if (math.dist(pose[:2], self.home[:2]) > self.get_parameter('home_position_tolerance_m').value and
                self.home_attempts < self.get_parameter('home_reapproaches').value):
            self.home_attempts += 1
            return
        turn = math.atan2(math.sin(self.home[2]-pose[2]), math.cos(self.home[2]-pose[2]))
        if abs(turn) <= math.radians(self.get_parameter('home_yaw_tolerance_deg').value):
            self.abort('Home reached')
            return
        try:
            target = (*pose[:2], pose[2]+turn)
            check_path(self.grid, [pose, target], pose, target)
        except ValueError as error:
            self.abort('Home heading alignment is not verified clear: '+str(error))
            return
        # Smac lattice paths end on its nearest heading bin, up to 13 degrees
        # from home. A goal in the rover's own planner cell keeps the exact
        # orientation, so the controller rotates in place without translating.
        x, y = self.grid.point(*self.grid.cell(*pose[:2]))
        self.mode, self.route = 'aligning', [(x, y, self.home[2])]

    def publish_status(self):
        message = String()
        message.data = json.dumps(dict(mission=self.mode, home=self.home, goal_active=self.goal is not None,
                                       result=self.result, trial=self.trial, outbound_reached=self.outbound_reached,
                                       selected_goal=self.selected, selection_state=self.selection_state,
                                       selection_reason=self.selection_reason, starting=self.starting,
                                       home_attempts=self.home_attempts,
                                       sim_time=self.get_clock().now().nanoseconds/1e9))
        self.pub.publish(message)
        markers = MarkerArray()
        for index, (pose, color) in enumerate(((self.home or self.preview_home, (0., 1., .3)),
                                               (self.selected, (1., .5, 0.)))):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id = 'mission', index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD if pose is not None else Marker.DELETE
            if pose is not None:
                marker.pose = self.selected_pose(pose).pose
                marker.pose.position.z = .15
            marker.scale.x, marker.scale.y, marker.scale.z = .35, .08, .08
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.
            markers.markers.append(marker)
        self.markers.publish(markers)
        if not self.preview_paths:
            empty = Path()
            empty.header.frame_id = 'map'
            self.outward_pub.publish(empty)
            self.return_pub.publish(empty)

    def tick(self):
        self.publish_status()
        if self.mode == 'idle' or self.grid is None:
            return
        if time.monotonic()-self.guard_wall > .6:
            self.abort('Motion guard status unavailable')
            return
        if time.monotonic()-self.started_wall > self.get_parameter('mission_timeout_wall_sec').value:
            self.abort('Mission wall-time limit exceeded')
            return
        now = self.get_clock().now().nanoseconds/1e9
        if now-self.last_route_check < 1.:
            return
        self.last_route_check = now
        try:
            pose = self.pose()
            distances, _ = reachable(self.grid, pose[:2])
            r, c = self.grid.cell(*self.home[:2])
            distance = distances[r, c] if 0 <= r < distances.shape[0] and 0 <= c < distances.shape[1] else math.inf
            if not math.isfinite(distance):
                self.abort('No verified safe return route remains')
                return
            elapsed = self.get_clock().now().nanoseconds/1e9-self.started
            if self.mode not in ('returning', 'aligning') and return_required(
                    elapsed, self.get_parameter('budget_seconds').value, distance,
                    self.get_parameter('navigation_speed').value):
                self.cancel()
                self.mode = 'returning'
            if self.goal is not None or self.pending:
                return
            if self.mode == 'returning':
                self.send(self.home)
            elif self.mode == 'aligning':
                self.send(self.route[0])
            elif self.mode == 'waypoints':
                if self.route:
                    self.send(self.route[0])
                else:
                    self.mode = 'returning'
            else:
                choices = frontiers(self.grid, pose[:2], self.visited)
                if choices:
                    self.send(choices[0][:3])
                else:
                    self.mode = 'returning'
        except TransformException:
            self.abort('Localization unavailable')


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    finally:
        node.abort('Mission shutdown')
        node.destroy_node()
        rclpy.shutdown()
