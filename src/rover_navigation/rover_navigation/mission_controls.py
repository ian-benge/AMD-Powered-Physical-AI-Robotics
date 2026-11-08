"""Loopback manual/mission controls. Only the guard owns the rover command."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import threading
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from rover_perception.live_core import image_to_bgr
from .manual_input import ManualInput, SPEED_LIMITS
from .diagnostics import localization_health, map_overlay, reason_colors
from .ros_utils import apply_tf, stamp_seconds
from .coverage import image_detail


class Controls(Node):
    def __init__(self):
        super().__init__('navigation_mission_controls')
        self.declare_parameter('session', '')
        self.session = Path(self.get_parameter('session').value).name
        self.lock = threading.Lock()
        self.latest = {}
        self.sim_clock = None
        self.declare_parameter('max_linear_speed', SPEED_LIMITS['linear']['max'])
        self.declare_parameter('max_angular_speed', SPEED_LIMITS['angular']['max'])
        self.declare_parameter('initial_linear_speed', -1.)
        initial=self.get_parameter('initial_linear_speed').value
        self.manual_input = ManualInput(max_linear=self.get_parameter('max_linear_speed').value,
                                        max_angular=self.get_parameter('max_angular_speed').value,
                                        initial_linear=None if initial==-1. else initial)
        self.manual_pub = None
        self.camera = self.annotated = self.map_image = None
        self.reason_image = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(ClockMessage, '/clock', self.clock_received,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.service_clients = {name: self.create_client(Trigger, '/navigation/'+service)
                        for name, service in (('start', 'start_selected_goal'), ('stop', 'stop'),
                                              ('manual', 'manual'), ('confirm', 'confirm_clear_start'))}
        for topic in ('status', 'mission_status'):
            self.create_subscription(String, '/navigation/'+topic,
                                     lambda msg, name=topic: self.receive(name, msg), 1)
        for topic, name in (('/rock_detector/status', 'detector'), ('/navigation/sample_sites', 'sample_sites')):
            self.create_subscription(String, topic, lambda msg, key=name: self.receive(key, msg), 1)
        self.create_subscription(String, '/navigation/mapping_status', lambda msg: self.receive('mapping', msg),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Image, '/oak/image_raw', self.raw_image, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/navigation/annotated_image/compressed', self.annotated_image, qos_profile_sensor_data)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/navigation/terrain_map', self.terrain, durable)
        self.create_subscription(OccupancyGrid, '/navigation/terrain_reasons', self.terrain_reasons, durable)

    def clock_received(self, msg):
        with self.lock:
            self.sim_clock = stamp_seconds(msg.clock), time.monotonic()

    def terrain_reasons(self, msg):
        if msg.header.frame_id != 'map':
            return
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)[::-1]
        ok, encoded = cv2.imencode('.png', reason_colors(data), [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if ok:
            with self.lock:
                self.reason_image = encoded.tobytes()
                self.latest['diagnostics'] = dict(data=dict(stamp=stamp_seconds(msg.header.stamp),
                    slope_cells=int(((data > 0) & ((data & 2) != 0)).sum()),
                    height_change_cells=int(((data > 0) & ((data & 4) != 0)).sum())), received=time.monotonic())

    def raw_image(self, msg):
        now = time.monotonic()
        if now-getattr(self, 'last_detail_at', -float('inf')) >= .5:
            try:
                detail = dict(available=True, stamp=stamp_seconds(msg.header.stamp), **image_detail(image_to_bgr(msg)))
            except (ValueError, TypeError, cv2.error):
                detail = dict(available=False)
            with self.lock:
                self.latest['camera_health'] = dict(data=detail, received=now)
            self.last_detail_at = now
        with self.lock:
            self.camera = msg, now

    def annotated_image(self, msg):
        with self.lock:
            self.annotated = msg, time.monotonic()

    def terrain(self, msg):
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)[::-1]
        rgb = np.full((*data.shape, 3), (99, 85, 74), dtype=np.uint8)  # OpenCV BGR: gray-blue unknown.
        rgb[(data >= 0) & (data < 90)] = (200, 223, 203)
        rgb[data >= 90] = (70, 75, 225)
        ok, encoded = cv2.imencode('.png', rgb, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if ok:
            with self.lock:
                self.map_image = encoded.tobytes()
                self.latest['map'] = dict(data=dict(width=msg.info.width, height=msg.info.height,
                    resolution=msg.info.resolution, origin=[msg.info.origin.position.x, msg.info.origin.position.y],
                    stamp=stamp_seconds(msg.header.stamp),
                    known_cells=int(np.count_nonzero(data >= 0)), hazard_cells=int(np.count_nonzero(data >= 90))),
                    received=time.monotonic())
                rows, cols = np.nonzero(data >= 0)
                if len(rows):
                    self.latest['map']['data']['view_box'] = [max(0, int(cols.min())-12), max(0, int(rows.min())-12),
                        min(msg.info.width, int(cols.max())-int(cols.min())+25),
                        min(msg.info.height, int(rows.max())-int(rows.min())+25)]

    def camera_frame(self):
        with self.lock:
            source, annotated = self.camera, self.annotated
        now = time.monotonic()
        if annotated and now-annotated[1] < 1.:
            msg = annotated[0]
            return bytes(msg.data), msg.header, True
        if source and now-source[1] < 1.:
            ok, encoded = cv2.imencode('.jpg', image_to_bgr(source[0]), [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                return encoded.tobytes(), source[0].header, False
        return None

    def receive(self, name, message):
        with self.lock:
            self.latest[name] = dict(data=json.loads(message.data), received=time.monotonic())
        if name == 'status' and (self.latest[name]['data'].get('mode') != 'manual' or not self.latest[name]['data'].get('ready')):
            self.manual_input.release()

    def snapshot(self):
        with self.lock:
            result = {name: dict(**value['data'], age_seconds=time.monotonic()-value['received'])
                      for name, value in self.latest.items()}
            clock = self.sim_clock
        result['session'] = self.session
        result['manual_speed_limits'] = self.manual_input.limits
        try:
            with urlopen('http://127.0.0.1:8776/status', timeout=.2) as response:
                result['scene'] = dict(available=True, **json.load(response))
        except (OSError, ValueError):
            result['scene'] = dict(available=False, phase='unavailable', error='Scene restart service is unavailable')
        try:
            tf = self.tf.lookup_transform('map', 'drive_center', Time())
            p, q = tf.transform.translation, tf.transform.rotation
            import math
            result['rover_pose'] = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            result['localization'] = localization_health(clock[0] if clock else None,
                                                         stamp_seconds(tf.header.stamp))
            result['localization']['clock_age_seconds'] = time.monotonic()-clock[1] if clock else None
            if not clock or time.monotonic()-clock[1] > .5:
                result['localization']['healthy'] = False
            correction = self.tf.lookup_transform('map', 'odom', Time())
            mapping = result.get('mapping', {})
            coverage = mapping.get('coverage')
            if not (coverage and clock and mapping.get('age_seconds', 99) < 1.
                    and -.05 <= clock[0]-coverage['stamp'] <= .6):
                coverage = None
            result['motion_overlay'] = map_overlay(result.get('status', {}), lambda p: apply_tf(p, correction), coverage)
        except TransformException:
            result['localization'] = dict(available=False, healthy=False)
        return result

    def drive(self, data):
        # The 20 Hz command path needs only the cached guard receipt. Rendering
        # overlays, TF lookups and the supervisor HTTP request belong to /status.
        with self.lock:
            latest = self.latest.get('status')
            guard = dict(**latest['data'], age_seconds=time.monotonic()-latest['received']) if latest else {}
        if guard.get('mode') != 'manual' or not guard.get('ready') or guard.get('age_seconds', 99) > .6:
            self.manual_input.release()
            reason = guard.get('reason') or 'manual mode has not been selected'
            detail = ('The full predicted motion sweep reaches unknown or unsafe ground. Inspect the rejected cell and fresh coverage before selecting a mode again.'
                      if reason == 'unsafe_or_unobserved_ground' else
                      'Check fresh sensors and explicitly enable manual drive again.')
            raise ValueError(f'Manual control stopped: {reason}. {detail}')
        v, w = self.manual_input.command(data)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.manual_pub.publish(msg)
        return dict(success=True, command=[v, w])

    def call(self, name):
        if name in ('restart', 'new-scene'):
            stopped = self.call('stop')
            if not stopped['success']:
                return stopped
            try:
                with urlopen(Request('http://127.0.0.1:8776/'+name, data=b'', method='POST'), timeout=2.) as response:
                    return json.load(response)
            except (OSError, ValueError) as error:
                return dict(success=False, message='Could not restart scene: '+str(error))
        if name in ('stop', 'start', 'manual'):
            self.manual_input.release()
        client = self.service_clients[name]
        if not client.service_is_ready():
            return dict(success=False, message='Mission service unavailable')
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(28. if name == 'start' else 3.):
            # A timed-out Start is uncertain: explicitly stop, even if its
            # validation/arming callback has not completed yet.
            if self.service_clients['stop'].service_is_ready():
                self.service_clients['stop'].call_async(Trigger.Request())
            return dict(success=False, message='Request timed out; Stop requested')
        try:
            reply = future.result()
            result = dict(success=reply.success, message=reply.message)
            if name == 'manual' and reply.success:
                end = time.monotonic()+.5
                while self.snapshot().get('status', {}).get('mode') != 'manual' and time.monotonic() < end:
                    time.sleep(.01)
                if self.snapshot().get('status', {}).get('mode') != 'manual':
                    return dict(success=False, message='Manual selection was interrupted; inspect rover status')
                if self.manual_pub is None:
                    self.manual_pub = self.create_publisher(Twist, '/navigation/manual_cmd_vel', 1)
                result['session'] = self.manual_input.arm()
            return result
        except Exception as error:
            return dict(success=False, message=str(error))


def handler(node):
    page = files('rover_navigation').joinpath('mission_controls.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, body, content='application/json', headers=None):
            encoded = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', content)
            self.send_header('Content-Length', str(len(encoded)))
            self.send_header('Cache-Control', 'no-store')
            for key, value in (headers or {}).items():
                self.send_header(key, str(value))
            self.end_headers()
            try:
                self.wfile.write(encoded)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def local_request(self):
            host = self.headers.get('Host', '')
            origin = self.headers.get('Origin')
            try:
                address = urlsplit('http://'+host)
                return (address.hostname in ('127.0.0.1', 'localhost') and address.username is None and
                        0 < (address.port or 80) <= 65535 and (not origin or urlsplit(origin).netloc == host))
            except ValueError:
                return False

        def do_GET(self):
            if not self.local_request():
                return self.reply(403, dict(message='Use the local mission page'))
            route = urlsplit(self.path).path
            if route == '/':
                return self.reply(200, page, 'text/html; charset=utf-8')
            if route == '/status':
                return self.reply(200, node.snapshot())
            if route == '/camera.jpg':
                frame = node.camera_frame()
                if frame:
                    body, header, annotated = frame
                    return self.reply(200, body, 'image/jpeg', {'X-Image-Stamp': f'{header.stamp.sec}:{header.stamp.nanosec}',
                                                               'X-Annotated': str(annotated).lower()})
                return self.reply(503, dict(message='Waiting for camera'))
            if route in ('/map.png', '/diagnostics.png'):
                with node.lock:
                    body = node.map_image if route == '/map.png' else node.reason_image
                if body:
                    return self.reply(200, body, 'image/png')
                return self.reply(503, dict(message='Waiting for terrain map'))
            self.reply(404, dict(message='Not found'))

        def do_POST(self):
            if not self.local_request():
                return self.reply(403, dict(message='Use the local mission page'))
            if self.path in ('/drive', '/release'):
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 1024 or self.headers.get('Content-Type') != 'application/json':
                        raise ValueError('Expected bounded JSON control input')
                    data = json.loads(self.rfile.read(size))
                    if self.path == '/release':
                        node.manual_input.release(data.get('session', ''))
                        return self.reply(200, dict(success=True, message='Input released; command lease will stop the rover'))
                    return self.reply(200, node.drive(data))
                except (ValueError, TypeError, KeyError) as error:
                    self.close_connection = True
                    return self.reply(400, dict(success=False, message=str(error)))
            if self.path not in ('/start', '/stop', '/manual', '/confirm', '/restart', '/new-scene'):
                return self.reply(404, dict(message='Not found'))
            if self.headers.get('Content-Length', '0') != '0':
                return self.reply(400, dict(message='This action takes no request body'))
            self.reply(200, node.call(self.path[1:]))

    return Handler


def main():
    rclpy.init()
    node = Controls()
    port = node.declare_parameter('port', 8774).value
    server = ThreadingHTTPServer(('127.0.0.1', port), handler(node))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok() and node.service_clients['stop'].service_is_ready():
            node.service_clients['stop'].call_async(Trigger.Request())
            rclpy.spin_once(node, timeout_sec=.1)
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
