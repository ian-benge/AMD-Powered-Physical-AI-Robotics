"""Depth-derived local safety grid and SLAM-corrected global traversability."""
from collections import deque
import math
import json
import hashlib
import time
from pathlib import Path
from dataclasses import asdict

import message_filters
import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .core import Grid, TerrainMemory, aligned_copy, depth_points, mask_outside_memory, terrain_reasons, traversability
from .ros_utils import apply_tf, grid_message, stamp_seconds
from .coverage import coverage_profile
from .terrain_compute import CpuTerrainCompute
from .terrain_evidence import ClearanceLocalization, ClearingPolicy, TerrainEvidence


class Terrain(Node):
    def __init__(self):
        super().__init__('navigation_terrain')
        for name, value in dict(max_slope_deg=12., max_step_m=.03, map_size_m=20.,
                                clear_start_radius_m=1.1, hazard_confirmations=2,
                                mapping_on_confirm=False, terrain_backend='cpu',
                                terrain_url='http://127.0.0.1:8782', adaptive_terrain=True,
                                evidence_output='').items():
            self.declare_parameter(name, value)
        self.mapping_on_confirm = self.get_parameter('mapping_on_confirm').value
        backend = self.get_parameter('terrain_backend').value
        if backend == 'rocm_http':
            from .terrain_service import RemoteTerrainCompute
            self.compute = RemoteTerrainCompute(self.get_parameter('terrain_url').value)
            if round(self.get_parameter('map_size_m').value/.025)>self.compute.metadata.get('max_memory_side',1200):
                raise ValueError('Terrain service cannot hold this map; start the checked environment-v2 terrain candidate')
        elif backend == 'cpu':
            self.compute = CpuTerrainCompute()
        else:
            raise ValueError('terrain_backend must be cpu or rocm_http')
        self.compute_error = ''
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.latest = None
        self.pending = deque(maxlen=20)
        self.last_stamp = None
        self.global_cloud = None
        self.cached_cloud = None
        self.memory = None  # per-frame classifications in odom coordinates
        self.start = None
        self.start_map = None
        self.last_global = -math.inf
        self.coverage = None
        self.adaptive_enabled = self.get_parameter('adaptive_terrain').value
        self.localization = ClearanceLocalization()
        self.evidence = None
        output = self.get_parameter('evidence_output').value
        self.event_stream = Path(output).open('x') if output else None
        if self.event_stream:
            self.event_stream.write(json.dumps(dict(kind='configuration', adaptive_terrain=self.adaptive_enabled,
                policy=asdict(ClearingPolicy())))+'\n')
            self.event_stream.flush()
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image, '/oak/depth/image_raw', qos_profile=qos_profile_sensor_data)
        self.info = message_filters.Subscriber(self, CameraInfo, '/oak/camera_info', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.depth, self.info], 5, .015)
        self.sync.registerCallback(self.camera)
        self.create_subscription(PointCloud2, '/rtabmap/cloud_map', self.cloud, qos_profile_sensor_data)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.local_pub = self.create_publisher(OccupancyGrid, '/navigation/local_ground', 1)
        self.local_reasons_pub = self.create_publisher(OccupancyGrid, '/navigation/local_ground_reasons', 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/navigation/terrain_map', durable)
        self.map_reasons_pub = self.create_publisher(OccupancyGrid, '/navigation/terrain_reasons', durable)
        self.layers_pub = self.create_publisher(GridMap, '/navigation/terrain_layers', durable)
        self.mapping_pub = self.create_publisher(String, '/navigation/mapping_status', durable)
        self.create_service(Trigger, '/navigation/confirm_clear_start', self.confirm)
        self.create_timer(.2, self.publish_mapping_status)
        # A 40 m projection can spend most of a 200 ms timer period computing.
        # Poll ready image-time transforms sooner; unchanged stamp checks still
        # admit each depth frame once and the sensor bounds the processing rate.
        self.update_period_seconds = .05 if self.get_parameter('map_size_m').value == 40. else .2
        self.create_timer(self.update_period_seconds, self.update)
        if self.mapping_on_confirm:
            size = self.get_parameter('map_size_m').value
            blank = Grid(np.full((round(size/.05), round(size/.05)), -1, np.int8), .05, -size/2, -size/2)
            self.map_pub.publish(grid_message(blank, 'map', self.get_clock().now().to_msg()))
            self.map_reasons_pub.publish(grid_message(blank, 'map', self.get_clock().now().to_msg()))

    def publish_mapping_status(self):
        if self.evidence is not None:
            now_sim = self.get_clock().now().nanoseconds/1e9
            reason = self.localization.check(self.last_stamp or now_sim, time.monotonic())
            if self.last_stamp is None or now_sim-self.last_stamp > .6:
                reason = 'accepted depth unavailable or stale'
            if reason:
                self.evidence.suspend(reason, now_sim)
            self.flush_evidence()
        msg = String()
        msg.data = json.dumps(dict(waiting_for_confirmation=self.mapping_on_confirm and self.start is None,
                                   clearance_confirmed=self.start is not None,
                                   depth_received_age_seconds=time.monotonic()-self.latest[2] if self.latest else None,
                                   accepted_depth_stamp=self.last_stamp,
                                   update_seconds=getattr(self, 'update_seconds', None),
                                   compute=self.compute.metadata, compute_error=self.compute_error,
                                   compute_timings=getattr(self.compute, 'timings', {}),
                                   coverage=getattr(self, 'coverage', None),
                                   terrain_evidence=(self.evidence.status if self.evidence else
                                       dict(enabled=self.adaptive_enabled, suspended='waiting for measured terrain',
                                            cleared_cells=0, pending_cells=0)),
                                   thresholds=dict(slope_deg=self.get_parameter('max_slope_deg').value,
                                                   height_change_m=self.get_parameter('max_step_m').value)))
        self.mapping_pub.publish(msg)

    def flush_evidence(self):
        if self.evidence is not None:
            if getattr(self, 'event_stream', None):
                for event in self.evidence.events:
                    self.event_stream.write(json.dumps(event)+'\n')
                self.event_stream.flush()
            self.evidence.events.clear()

    def record_frame(self, depth, info, camera, inverse, frame, layers, local, global_grid):
        """Read-only provenance for replaying the actual successful publication."""
        if not getattr(self, 'event_stream', None):
            return
        def grid_record(grid):
            if grid is None:
                return None
            return dict(origin=[grid.x, grid.y], resolution=grid.resolution,
                        shape=list(grid.data.shape), sha256=hashlib.sha256(grid.data.tobytes()).hexdigest())
        translation, quaternion = self.rigid(camera)
        row = dict(kind='terrain_frame', stamp=stamp_seconds(depth.header.stamp),
            input_sha256=hashlib.sha256(bytes(depth.data)).hexdigest(),
            calibration_k=list(info.k), camera_to_odom=dict(translation=translation, quaternion=quaternion),
            projection_to_odom=(dict(zip(('translation', 'quaternion'), self.rigid(inverse))) if inverse else None),
            thresholds=dict(slope_deg=self.get_parameter('max_slope_deg').value,
                            height_change_m=self.get_parameter('max_step_m').value),
            current_frame=grid_record(frame), local=grid_record(local), global_map=grid_record(global_grid),
            observed_cells=int((layers['observation_count'] > 0).sum()),
            hazard_cells=int((frame.data >= 90).sum()),
            terrain_evidence=(dict(self.evidence.status) if self.evidence else dict(enabled=False)))
        received=getattr(self,'frame_received_monotonic',None)
        if received is not None:
            row['depth_to_local_publication_seconds']=getattr(self,'local_publication_monotonic',time.monotonic())-received
            row['local_publication_monotonic']=self.local_publication_monotonic
            started = getattr(self, 'frame_update_started_monotonic', None)
            if started is not None:
                row['depth_wait_seconds'] = started-received
                row['mapping_processing_seconds'] = self.local_publication_monotonic-started
                row['update_poll_period_seconds'] = self.update_period_seconds
        self.event_stream.write(json.dumps(row)+'\n')
        self.event_stream.flush()

    def odometry(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.localization.valid = False
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.localization.add(stamp_seconds(msg.header.stamp), [p.x, p.y, p.z],
                              [q.x, q.y, q.z, q.w], msg.pose.covariance, time.monotonic())

    def camera(self, depth, info):
        if (depth.width, depth.height, depth.header.frame_id) != (info.width, info.height, info.header.frame_id):
            self.get_logger().warning('Depth and CameraInfo must be aligned and share dimensions/frame', throttle_duration_sec=5.)
            return
        self.latest = depth, info, time.monotonic()
        self.pending.append(self.latest)

    def cloud(self, msg):
        if msg.header.frame_id != 'map':
            return
        self.global_cloud = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)

    def confirm(self, request, response):
        # This is explicit operator confirmation of the clear launch area, not a
        # privileged simulator map. It is accepted once per process/session.
        if self.start is not None:
            response.message = 'Launch clearance already confirmed; restart to establish another launch area'
            return response
        try:
            if self.latest is None or time.monotonic()-self.latest[2] > 1.:
                raise ValueError('Fresh depth is required')
            t = self.tf.lookup_transform('odom', 'base_link', Time())
            start = np.array([[t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]])
            map_tf = self.tf.lookup_transform('map', 'odom', Time())
            # Anchor operator-confirmed clearance in the persistent map. Applying
            # a NEW map->odom correction to an OLD odom point would move it.
            start_map = apply_tf(start, map_tf)[0]
            self.start, self.start_map = start, start_map
            response.success, response.message = True, 'Confirmed clear launch area; manual control can now be selected'
        except (TransformException, ValueError) as error:
            response.message = str(error)
        return response

    def seeded(self, grid, position):
        if position is not None:
            yy, xx = np.indices(grid.data.shape)
            disk = ((grid.x+(xx+.5)*grid.resolution-position[0])**2+
                    (grid.y+(yy+.5)*grid.resolution-position[1])**2 <= self.get_parameter('clear_start_radius_m').value**2)
            grid.data[disk & (grid.data < 0)] = 0
        return grid

    def build(self, points, resolution, size, center):
        return self.compute.classify(points, resolution=resolution, size=size, center=center,
                                     max_slope=self.get_parameter('max_slope_deg').value,
                                     max_step=self.get_parameter('max_step_m').value)

    @staticmethod
    def rigid(stamped):
        t, q = stamped.transform.translation, stamped.transform.rotation
        return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]

    def ready_depth(self, now_sim):
        """Choose fresh depth at its own timestamp without blocking TF delivery."""
        now_wall = time.monotonic()
        self.pending = deque((sample for sample in self.pending
                              if 0 <= now_wall-sample[2] <= 1.
                              and now_sim-stamp_seconds(sample[0].header.stamp) <= .6
                              and (self.last_stamp is None
                                   or stamp_seconds(sample[0].header.stamp) > self.last_stamp)),
                             maxlen=self.pending.maxlen)
        error = None
        for depth, info, _ in reversed(self.pending):
            if stamp_seconds(depth.header.stamp) > now_sim:
                continue
            when = Time.from_msg(depth.header.stamp)
            try:
                # The executor must remain free to receive delayed odometry TF.
                # Keep the frame for another tick instead of waiting here or
                # substituting an unrelated latest transform for its timestamp.
                camera = self.tf.lookup_transform('odom', depth.header.frame_id, when)
                pose = self.tf.lookup_transform('odom', 'base_link', when)
                return depth, info, camera, pose
            except TransformException as missing:
                error = missing
        if error is not None:
            self.get_logger().warning('Waiting for time-aligned depth transforms: '+str(error),
                                      throttle_duration_sec=5.)
        return None

    def update(self):
        started = time.monotonic()
        if getattr(self, 'mapping_on_confirm', False) and self.start is None:
            return
        if self.latest is None:
            return
        latest_stamp = stamp_seconds(self.latest[0].header.stamp)
        now_sim = self.get_clock().now().nanoseconds/1e9
        if self.last_stamp is not None and latest_stamp < self.last_stamp:
            self.global_cloud, self.start, self.start_map = None, None, None
            self.cached_cloud = self.memory = None
            self.evidence = None
            self.localization = ClearanceLocalization()
            self.coverage = None
            self.last_stamp, self.last_global = None, -math.inf
            self.pending.clear()
            self.pending.append(self.latest)
        ready = self.ready_depth(now_sim)
        if ready is None:
            return
        depth, info, tf, pose = ready
        stamp = stamp_seconds(depth.header.stamp)
        self.frame_update_started_monotonic = started
        self.frame_received_monotonic=next((v[2] for v in self.pending if stamp_seconds(v[0].header.stamp)==stamp),started)
        try:
            center = (pose.transform.translation.x, pose.transform.translation.y)
            # Classify this frame alone: spans and slopes across frames would
            # turn pose inconsistency into phantom steps.
            translation, quaternion = self.rigid(tf)
            frame, frame_layers, count = self.compute.depth(
                depth.data, encoding=depth.encoding, width=depth.width, height=depth.height,
                step=depth.step, bigendian=bool(depth.is_bigendian), k=list(info.k),
                translation=translation, quaternion=quaternion, resolution=.025, size=10.,
                center=center, max_slope=self.get_parameter('max_slope_deg').value,
                max_step=self.get_parameter('max_step_m').value)
            if count < 100:
                return
            q = pose.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.coverage = dict(stamp=stamp, **coverage_profile(frame, (*center, yaw)))
            reasons = terrain_reasons(frame, frame_layers, self.get_parameter('max_slope_deg').value,
                                       self.get_parameter('max_step_m').value)
            if self.memory is None:
                self.memory = TerrainMemory(self.get_parameter('map_size_m').value, .025,
                                            self.get_parameter('hazard_confirmations').value)
                if getattr(self, 'adaptive_enabled', False):
                    self.evidence = TerrainEvidence(self.memory)
            self.memory.remember(frame, reasons)
            map_tf = inverse = None
            if self.tf.can_transform('map', 'odom', Time()):
                map_tf = self.tf.lookup_transform('map', 'odom', Time())
                inverse = self.tf.lookup_transform('odom', 'map', Time())
            if getattr(self, 'evidence', None) is not None:
                suspension = self.localization.check(stamp, time.monotonic())
                try:
                    # RTAB-Map broadcasts ahead by tf_tolerance. Require TF to
                    # bracket NOW instead of rejecting that future publication.
                    correction_time = self.get_clock().now()
                    correction = self.tf.lookup_transform('map', 'odom', correction_time)
                    age = correction_time.nanoseconds/1e9-stamp_seconds(correction.header.stamp)
                    if map_tf is None or not -.05 <= age <= .6:
                        raise ValueError('map localization unavailable or stale')
                except (TransformException, ValueError):
                    suspension = 'map localization unavailable or stale'
                self.evidence.observe(self.memory, frame, frame_layers, translation[:2], stamp, suspension)
                self.flush_evidence()
            n = round(6./.025)
            origin = np.floor((np.asarray(center)-3.)/.025)*.025
            local = aligned_copy(self.memory.grid, Grid(np.empty((n, n), np.int8), .025, *origin))
            # Unconfirmed hazards are not retained, but still stop this update.
            local.data[aligned_copy(frame, local).data >= 90] = 100
            local_reasons = aligned_copy(self.memory.reasons(), local)
            fresh_reasons = aligned_copy(reasons, local).data
            fresh_hazards = fresh_reasons > 0
            local_reasons.data[fresh_hazards] = np.maximum(0, local_reasons.data[fresh_hazards]) | fresh_reasons[fresh_hazards]
            seed_local = apply_tf([self.start_map], inverse)[0] if self.start_map is not None and inverse is not None else None
            local = self.seeded(local, seed_local)
            local_reasons.data[(local.data >= 0) & (local.data < 90)] = 0
            if self.get_parameter('map_size_m').value == 40.:
                # Context terrain is rendered and collidable, but observations
                # there cannot expand the declared navigation operating area.
                mask_outside_memory(local, self.memory.grid)
                mask_outside_memory(local_reasons, self.memory.grid)
            self.last_stamp = stamp
            self.pending = deque((sample for sample in self.pending
                                  if stamp_seconds(sample[0].header.stamp) > stamp),
                                 maxlen=self.pending.maxlen)
            if map_tf is not None:
                # Layers from RTAB-Map's merged cloud are diagnostic only: its
                # cross-keyframe height spans are not used for navigation.
                if (self.global_cloud is not None and self.cached_cloud is not self.global_cloud
                        and stamp-self.last_global >= 1.):
                    grid, layers = self.build(self.global_cloud, .05, self.get_parameter('map_size_m').value, (0., 0.))
                    self.cached_cloud = self.global_cloud
                    self.publish_layers(grid, layers, depth.header.stamp)
                    self.last_global = stamp
                # Odom-frame memory moves with the latest map correction as a whole.
                translation, quaternion = self.rigid(inverse)
                global_grid, global_reasons = self.compute.project(self.memory, translation, quaternion,
                                            self.get_parameter('map_size_m').value, .05)
                self.seeded(global_grid, self.start_map)
                global_reasons.data[(global_grid.data >= 0) & (global_grid.data < 90)] = 0
                self.map_reasons_pub.publish(grid_message(global_reasons, 'map', depth.header.stamp))
                self.map_pub.publish(grid_message(global_grid, 'map', depth.header.stamp))
            # Publish safety input only after all requested GPU operations succeed.
            self.local_reasons_pub.publish(grid_message(local_reasons, 'odom', depth.header.stamp))
            self.local_pub.publish(grid_message(local, 'odom', depth.header.stamp))
            self.local_publication_monotonic=time.monotonic()
            self.record_frame(depth, info, tf, inverse, frame, frame_layers, local,
                              global_grid if map_tf is not None else None)
            self.update_seconds = time.monotonic()-started
            self.compute_error = ''
        except (TransformException, ValueError, OSError, RuntimeError) as error:
            self.compute_error = str(error)
            self.get_logger().warning(str(error), throttle_duration_sec=5.)

    def publish_layers(self, grid, layers, stamp):
        msg = GridMap()
        msg.header.frame_id, msg.header.stamp = 'map', stamp
        msg.info.resolution = grid.resolution
        msg.info.length_x = grid.data.shape[1]*grid.resolution
        msg.info.length_y = grid.data.shape[0]*grid.resolution
        msg.info.pose.position.x = grid.x+msg.info.length_x/2
        msg.info.pose.position.y = grid.y+msg.info.length_y/2
        msg.info.pose.orientation.w = 1.
        for name, values in layers.items():
            msg.layers.append(name)
            array = Float32MultiArray()
            matrix = values[::-1, ::-1].T
            array.layout.dim = [MultiArrayDimension(label='column_index', size=matrix.shape[1], stride=matrix.size),
                                MultiArrayDimension(label='row_index', size=matrix.shape[0], stride=matrix.shape[0])]
            array.data = matrix.astype(np.float32).ravel(order='F').tolist()
            msg.data.append(array)
        self.layers_pub.publish(msg)


def main():
    rclpy.init()
    node = Terrain()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
