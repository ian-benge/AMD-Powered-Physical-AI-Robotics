"""Opt-in two-mission ROS adapter. Only the existing guard publishes cmd_vel."""
from concurrent.futures import ProcessPoolExecutor
import json
import math
from multiprocessing import get_context
from pathlib import Path
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from rover_navigation_interfaces.srv import DemoCommand

from .core import Grid, drive_pose, footprint_avoids_disk, footprint_clear, turn_radius
from .demo_core import (ACTIVE, CONTACT_RECOVERABLE, CONTACT_RETRY_CLEARANCE, CONTACT_RETREAT_DISTANCE,
                        EXPLORATION_ROUTE_PHASES, FRONTIER_ARRIVAL_RADIUS, FRONTIER_ARRIVAL_YAW,
                        FRONTIER_CANDIDATE_LIMIT, FRONTIER_SHORTLIST_SIZE,
                        REPOSITION_ARRIVAL_MARGIN, REPOSITION_ARRIVAL_RADIUS, REPOSITION_ARRIVAL_YAW,
                        SAMPLE_AT_SITE, SAMPLE_HEADING_TOLERANCE, SAMPLE_REVERSE_DISTANCE, SAMPLE_REVERSE_MAX_SPEED,
                        WAYPOINT_CLEARANCE_PAD,
                        DemoPolicy, DemoRun, SampleBudgetError, SampleGeometryChanged, check_sample_turn, collection_poses,
                        distance_to_route, exploration_overlap, heading_error,
                        follow_recorded_retreat, frontier_stop_clear, observation_frontiers, path_length,
                        recorded_retreat, return_reserve, sample_reverse_pose,
                        sample_reverse_reserve, target_signature)
from .goal_checks import check_path, pose_values
from .mission_node import Mission
from .object_obstacles import SOLID_CLASSES, object_radius
from .ros_utils import apply_tf, stamp_seconds
from .site_geometry import pose_covariance


MAX_AUTOMATIC_REPLANS = 2
ROUTE_COMPARISON_LIMIT = 3
MAX_FRONTIER_RETRACE_SECONDS = 60.
SOIL_REFINEMENT_ALLOWANCE_M = .30
STATIONARY_SOIL_EVIDENCE_SECONDS = .85
NAV2_FAILED_TO_MAKE_PROGRESS = 105
WAYPOINT_PURPOSE = {'exploring':'frontier','exploration_reposition':'clearance','contact_rejoin':'clearance'}


class DemoMission(Mission):
    def __init__(self):
        super().__init__()
        for key,value in dict(session='',demo_output='',demo_profile='full',frontier_bt='',clearance_bt='').items():
            self.declare_parameter(key,value)
        session=Path(self.get_parameter('session').value).name
        output=Path(self.get_parameter('demo_output').value)
        if not session or not str(output) or output.exists():
            raise ValueError('Demo needs a new exclusive output and session')
        profile=self.get_parameter('demo_profile').value
        speed=self.get_parameter('navigation_speed').value
        if profile=='rehearsal':
            policy=DemoPolicy.rehearsal(speed)
        elif profile=='survey':
            policy=DemoPolicy.survey(speed)
        elif profile=='full':
            policy=DemoPolicy(speed=speed)
        else:
            raise ValueError('Unknown demo profile')
        self.run=DemoRun(session,policy)
        self.frontier_bt=self.get_parameter('frontier_bt').value
        self.clearance_bt=self.get_parameter('clearance_bt').value
        output.parent.mkdir(parents=True,exist_ok=True)
        self.stream=output.open('x')
        self.output=output
        self.stream.write(json.dumps(dict(kind='configuration',session_id=session,policy=self.run.status(time.monotonic())['policy']))+'\n')
        self.working=False
        self.expected_stop=False
        self.nav_done=None
        self.nav_arrived_at=None
        self.waypoint_stop_deadline=None
        self.current_target=None
        self.current_purpose=''
        self.goal_started=0.
        self.odometry_pose=None
        self.odometry_wall=-math.inf
        self.odometry_error='Waiting for estimated odometry'
        self.velocity=[0.,0.]
        self.mapping={};self.mapping_wall=-math.inf
        self.soils={};self.soils_wall=-math.inf
        self.objects={};self.objects_wall=-math.inf
        self.detector={};self.detector_wall=-math.inf
        self.manual_used=False
        self.last_stop_generation=None
        self.sweep=[]
        self.visited=[]
        self.trails=[]
        self.collection_pose=None
        self.sample_arrival_wall=None
        self.backup_start=None
        self.return_path=[]
        self.active_path=[]
        self.frontier=None
        self.return_seconds=None
        self.return_requested=False
        self.pause_stop_epoch=None
        self.sample_view_angles=[]
        self.failed_route=[]
        self.frontier_arrival_blocked=False
        self.alternate_route=[]
        self.auto_retry_pending=False
        self.auto_retry_count=0
        self.auto_retry_until=0.
        self.resume_return=False
        self.contact_point=None
        self.retreat_route=None
        self.retreat_pub=self.create_publisher(Twist,'/navigation/retreat_cmd_vel',1)
        self.retreat_client=self.create_client(Trigger,'/navigation/enable_retreat')
        self.actual_distance=0.
        self.rtf_samples=[]
        self.last_snapshot=-math.inf
        # The Dijkstra scan holds the Python interpreter on large maps.
        # Use a fresh process that never inherits any ROS handles.
        self.worker=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn'))
        self.frontier_future=None
        self.frontier_trail=()
        self.create_service(DemoCommand,'/navigation/demo_command',self.command,callback_group=ReentrantCallbackGroup())
        self.demo_pub=self.create_publisher(String,'/navigation/demo_status',1)
        self.sample_keepout_pub=self.create_publisher(Float64MultiArray,'/navigation/sample_keepout_odom',1)
        self.sample_motion_pub=self.create_publisher(Twist,'/navigation/autonomous_cmd_vel',1)
        durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String,'/navigation/mapping_status',self.mapping_received,durable)
        self.create_subscription(String,'/navigation/soil_sites',self.soils_received,durable)
        self.create_subscription(String,'/navigation/sample_sites',self.objects_received,durable)
        self.create_subscription(String,'/rock_detector/status',self.detector_received,1)
        self.create_subscription(Odometry,'/odom',self.odom_received,qos_profile_sensor_data)
        self.create_subscription(PathMessage,'/plan',self.actual_plan,1)

    # Legacy mission entrypoints must not bypass the demo state machine.
    def start(self,mode,response):
        response.message='Use the session-fenced demo command service';return response

    async def start_selected(self,request,response):
        response.message='Use the sample trip preview and approval';return response

    def return_home(self,request,response):
        response.message='Use the session-fenced demo Return now command';return response

    def select_goal(self,msg):
        pass

    def odom_received(self,msg):
        try:
            if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':
                raise ValueError('Invalid odometry frame')
            pose_covariance(msg.pose.covariance)
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            pose=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            if not np.isfinite(pose).all():raise ValueError('Invalid pose')
            self.odometry_pose=pose;self.odometry_wall=time.monotonic()
            self.odometry_error=''
            self.velocity=[msg.twist.twist.linear.x,msg.twist.twist.angular.z]
            if hasattr(self,'run') and self.run.phase in SAMPLE_AT_SITE and (
                    not np.isfinite(self.velocity).all() or abs(self.velocity[0])>=.01 or abs(self.velocity[1])>=.02 or
                    self.collection_pose is None or math.dist(pose[:2],self.collection_pose[:2])>.05 or
                    abs(heading_error(pose[2],self.collection_pose[2]))>SAMPLE_HEADING_TOLERANCE):
                self.abort('Unexpected movement at the sample stop')
        except ValueError as error:
            self.odometry_wall=-math.inf
            self.odometry_error=str(error)

    def mapping_received(self,msg):
        self.mapping=json.loads(msg.data);self.mapping_wall=time.monotonic()

    def soils_received(self,msg):
        self.soils=json.loads(msg.data);self.soils_wall=time.monotonic()

    def objects_received(self, msg):
        self.objects = json.loads(msg.data)
        self.objects_wall = time.monotonic()

    def detector_received(self,msg):
        self.detector=json.loads(msg.data);self.detector_wall=time.monotonic()

    def guard_received(self,msg):
        status=json.loads(msg.data)
        if status.get('sequence',0)<getattr(self,'guard_sequence',0):return
        self.guard=status;self.guard_wall=time.monotonic()
        if not hasattr(self,'run'):return
        if self.run.phase in SAMPLE_AT_SITE and status.get('command')!=[0.,0.]:
            self.abort('Nonzero command at the sample stop');return
        changed=(self.last_stop_generation is not None and status.get('stop_generation')!=self.last_stop_generation)
        self.last_stop_generation=status.get('stop_generation')
        if (changed and status.get('reason')=='chassis_contact' and status.get('mode')=='stopped' and
                status.get('odometry_resets',0)==self.run.epoch and self.run.phase in CONTACT_RECOVERABLE):
            self.pause_for_contact()
            return
        if (self.run.phase in ('exploration_paused','awaiting_target') and self.pause_stop_epoch is not None and
                status.get('stop_generation')==self.pause_stop_epoch and status.get('mode')=='stopped'):
            self.pause_stop_epoch=None
            changed=False
        if not self.expected_stop and ((self.starting and status.get('stop_generation')!=self.start_epoch) or
                changed and self.run.started is not None and self.run.phase not in ('complete','stopped')):
            self.abort('Stop or takeover invalidated the pending demo operation');return
        if self.run.phase=='ready':
            self.run.epoch=status.get('odometry_resets',0)
            self.manual_used |= status.get('mode')=='manual'
        elif status.get('odometry_resets',0)!=self.run.epoch:
            self.abort('Odometry epoch changed; Restart required')
        elif (self.run.phase in ACTIVE and not self.expected_stop and
              (not status.get('ready') or not self.starting and
               status.get('mode')!=('retreat' if self.run.phase=='contact_reversing' else 'autonomous'))):
            self.abort('Motion guard stopped or takeover: '+self.guard_cause())

    def health_details(self):
        now=time.monotonic()
        checks=dict(terrain_map=self.grid is not None,
            estimated_odometry=self.odometry_pose is not None and now-self.odometry_wall<.5,
            motion_guard=now-self.guard_wall<.6 and bool(self.guard.get('ready')),
            accepted_depth=now-self.mapping_wall<1. and not self.mapping.get('compute_error') and
                self.mapping.get('accepted_depth_stamp') is not None,
            detector=now-self.detector_wall<.6 and bool(self.detector.get('ready')) and not self.detector.get('error'))
        return dict(healthy=all(checks.values()),checks=checks,missing=[k for k,v in checks.items() if not v],
                    odometry_error=self.odometry_error)

    def healthy(self):
        return self.health_details()['healthy']

    def pose(self):
        # The latest common VIO/SLAM time is normally behind the ROS clock.
        # Asking the full chain for "now" would require future extrapolation.
        tf=self.tf.lookup_transform('map','drive_center',Time())
        if not -.05<=self.get_clock().now().nanoseconds/1e9-stamp_seconds(tf.header.stamp)<=.6:
            raise ValueError('Estimated drive-center pose is stale')
        p,q=tf.transform.translation,tf.transform.rotation
        return [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

    def transform_pose(self,pose,target='map'):
        source='odom' if target=='map' else 'map'
        # Use the valid full-chain pose time. RTAB-Map may publish map->odom
        # ahead of /clock for its transform tolerance; selecting that edge's
        # newest sample is not the same as selecting a fresh rover pose.
        anchor=self.tf.lookup_transform('map','drive_center',Time())
        age=self.get_clock().now().nanoseconds/1e9-stamp_seconds(anchor.header.stamp)
        if not -.05<=age<=.6:
            raise ValueError('Estimated map/odometry correction is stale')
        tf=self.tf.lookup_transform(target,source,Time.from_msg(anchor.header.stamp))
        xy=apply_tf([[pose[0],pose[1],0.]],tf)[0]
        q=tf.transform.rotation
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return [float(xy[0]),float(xy[1]),pose[2]+angle]

    def home_map(self):return self.transform_pose(self.run.home)

    def area(self):return float((self.mapping.get('measured_coverage') or {}).get('measured_area_m2',0.))

    def actual_plan(self,msg):
        if not hasattr(self,'run') or self.run.phase not in ACTIVE or not msg.poses:return
        if (self.goal is None and not self.pending) or self.current_target is None:return
        try:
            path=[pose_values(p) for p in msg.poses]
            # The planner also publishes paths requested for candidate scoring.
            if math.dist(path[-1][:2], self.current_target[:2]) > .15:return
            current=self.pose()
            if math.dist(path[0][:2], current[:2]) > .2:return
            keepout=self.run.preview['keepout'] if self.run.preview else None
            check_path(self.grid,path,current,self.current_target,keepout=keepout,avoid_center=self.contact_point)
            if self.current_purpose=='frontier':
                if not frontier_stop_clear(self.grid,path[-1]):
                    self.route_blocked('Planned frontier arrival lacks clear turning space', frontier_arrival=True)
                    return
                self.check_alternate_departure(path)
            elif self.current_purpose=='clearance' and not frontier_stop_clear(self.grid,path[-1],REPOSITION_ARRIVAL_MARGIN):
                raise ValueError('Planned arrival lacks clear turning space')
            self.active_path=path
        except ValueError as error:
            self.route_blocked('Nav2 route changed: '+str(error))

    def abort(self,reason):
        if not hasattr(self,'run'):return
        if self.run.phase!='stopped':
            self.run.stop(time.monotonic(),reason)
        self.selection_generation+=1
        self.cancel()
        self.nav_done=None
        self.mode='idle'
        self.auto_retry_pending=False
        if self.stop_client.service_is_ready():self.stop_client.call_async(Trigger.Request())
        self.flush()

    def route_blocked(self,reason,*,frontier_arrival=False):
        r=self.run
        if r.phase=='exploration_paused':
            return
        epoch=self.guard.get('stop_generation')
        if (r.phase not in EXPLORATION_ROUTE_PHASES or not self.healthy() or epoch is None or
                not self.stop_client.service_is_ready()):
            self.abort(reason)
            return
        if r.phase=='exploring' and self.frontier is not None:
            try:
                self.visited.append(self.transform_pose(self.frontier,'odom')[:2])
            except Exception:
                self.abort('Lost localization while pausing for a blocked route')
                return
        self.resume_return=(self.return_requested or r.phase=='exploration_return' or
                            r.phase=='exploration_reposition' and self.resume_return)
        self.frontier_arrival_blocked=frontier_arrival and r.phase=='exploring' and self.frontier is not None
        if self.active_path:
            self.failed_route=list(self.active_path)
        elif self.current_target is not None:
            try:self.failed_route=[self.pose(),list(self.current_target)]
            except Exception:self.failed_route=[]
        else:
            self.failed_route=[]
        self.selection_generation+=1
        self.cancel()
        self.nav_done=None
        self.frontier=None
        self.active_path=[]
        if self.frontier_future is not None:
            self.frontier_future.cancel()
            self.frontier_future=None
        self.pause_stop_epoch=epoch+1
        self.mode='idle'
        now=time.monotonic()
        self.auto_retry_pending=(self.frontier_arrival_blocked or bool(self.failed_route)) and self.auto_retry_count<MAX_AUTOMATIC_REPLANS
        self.auto_retry_until=now+4.
        r.transition('exploration_paused',now,reason)
        self.stop_client.call_async(Trigger.Request())
        self.flush()

    def pause_for_contact(self):
        r=self.run
        self.auto_retry_pending=False
        r.contact_from=r.phase
        self.contact_trail_since=r.phase_started
        if r.phase=='exploring' and self.frontier is not None:
            try:self.visited.append(self.transform_pose(self.frontier,'odom')[:2])
            except Exception:
                self.abort('Lost localization at chassis contact')
                return
        self.selection_generation+=1
        self.cancel()
        self.nav_done=None
        self.frontier=None
        self.active_path=[]
        if self.frontier_future is not None:
            self.frontier_future.cancel()
            self.frontier_future=None
        self.mode='idle'
        r.transition('contact_paused',time.monotonic(),'Chassis contact detected. Inspect the scene before backing up.')
        self.flush()

    def pause_for_sample_review(self,reason):
        r=self.run
        epoch=self.guard.get('stop_generation')
        if (r.phase not in ('sample_outbound','collecting') or epoch is None or not self.healthy() or
                not self.stop_client.service_is_ready()):
            self.abort(reason)
            return
        target=r.target
        raised=target['source']=='detected_object'
        try:
            site=(self.selected_object if raised else self.selected_site)(target['track_id'],False)
        except ValueError as error:
            self.abort(str(error))
            return
        target.update(odom_position=site['odom_position'],covariance=site['covariance'],
                      envelope_radius_m=object_radius(site) if raised else site['envelope_radius_m'])
        now=time.monotonic()
        self.selection_generation+=1
        self.cancel()
        self.nav_done=None
        self.mode='idle'
        self.active_path=[]
        self.pause_stop_epoch=epoch+1
        self.collection_pose=None
        self.sample_arrival_wall=None
        r.preview=r.collection_started=None
        r.sample_review_until=now+min(r.policy.selection_seconds,r.sample_time_left(now))
        r.transition('awaiting_target',now,reason+' Rover stopped. Preview and approve the updated trip.')
        self.stop_client.call_async(Trigger.Request())
        self.flush()

    async def stopped(self):
        self.expected_stop=True
        try:
            if not self.stop_client.service_is_ready():raise ValueError('Stop service unavailable')
            response=await self.wait(self.stop_client.call_async(Trigger.Request()),self.selection_generation,3.)
            if not response.success:raise ValueError(response.message)
            for _ in range(20):
                if (self.guard.get('mode')=='stopped' and self.guard.get('command')==[0.,0.] and
                        np.isfinite(self.velocity).all() and abs(self.velocity[0])<.01 and abs(self.velocity[1])<.02):return
                await self.pause(.05)
            raise ValueError('Stop acknowledgement did not reach status')
        finally:self.expected_stop=False

    async def pause(self,seconds):
        from rclpy.task import Future
        future=Future()
        timer=self.create_timer(seconds,lambda:future.set_result(None) if not future.done() else None,clock=self.wall_clock)
        try:await future
        finally:self.destroy_timer(timer)

    async def wait(self,future,token,timeout=8.):
        deadline=getattr(self,'operation_deadline',None)
        if deadline is not None:timeout=min(timeout,max(0.,deadline-time.monotonic()))
        return await super().wait(future,token,timeout)

    async def arm(self,token,*,retreat=False):
        if not self.healthy() or self.guard.get('mode')!='stopped':raise ValueError('Stopped healthy guard required')
        epoch=self.guard.get('stop_generation')
        client=self.retreat_client if retreat else self.enable_client
        future=client.call_async(Trigger.Request())
        succeeded=False
        try:
            answer=await self.wait(future,token,3.)
            if not answer.success:raise ValueError(answer.message)
            receipt=json.loads(answer.message)
            if receipt['stop_generation']!=epoch or not self.healthy():raise ValueError('Start interrupted')
            self.guard_sequence=receipt.get('status_sequence',0)
            succeeded=True
        finally:
            if not succeeded:
                future.add_done_callback(lambda _:self.stop_client.call_async(Trigger.Request()))

    async def protect_sample(self, keepout, token):
        # Install the exclusion in the motion guard before any sample drive is armed.
        odom_point=self.run.target['odom_position']
        disk=[*odom_point[:2],keepout[2]]
        message=Float64MultiArray(data=disk)
        for _ in range(10):
            self.valid_token(token)
            if self.guard.get('sample_keepout_odom')==disk:return
            self.sample_keepout_pub.publish(message)
            await self.pause(.1)
        raise ValueError('The motion guard did not confirm the sample exclusion')

    def request_exploration_return(self):
        self.return_requested=True
        if not self.waypoint_handoff_pending():
            self.cancel()
            self.nav_done=None
        if self.frontier_future is not None:
            self.frontier_future.cancel()
            self.frontier_future=None

    async def command(self,request,response):
        now=time.monotonic();r=self.run
        owns_operation=False
        try:
            r.verify(request.session_id,request.mission_id,request.client_token,request.epoch)
            if request.command=='heartbeat':
                if r.phase in ACTIVE and now-r.heartbeat_wall>r.policy.heartbeat_seconds:
                    self.check_deadlines(now)
                    self.abort(r.reason)
                if r.phase in ('stopped','complete'):
                    raise ValueError(r.reason or 'Mission has ended')
                r.heartbeat(now,request.sequence)
                response.success=True
            elif request.command=='return_now':
                if r.phase=='collecting':
                    raise ValueError('Wait for simulated collection to finish before returning')
                if r.phase=='awaiting_sample_return':
                    if not self.sample_return_ready():
                        raise ValueError('Return home requires a stationary rover and healthy navigation')
                    r.heartbeat_wall=now
                    r.request_sample_return(now)
                elif r.phase in ('sweeping','exploring'):
                    self.request_exploration_return()
                    r.event(now,'early_return_requested')
                else:
                    raise ValueError('Return is available during exploration or at the sample stop')
                response.success=True
            elif self.working:
                raise ValueError('Another bounded operation is running; Stop remains available')
            else:
                self.working=True
                owns_operation=True
                token=self.selection_generation
                if request.command in ('start_exploration','start_sample','preview_sample','alternate_sample',
                                       'resume_exploration','alternate_exploration','retreat_and_replan'):
                    self.starting=True;self.start_epoch=self.guard.get('stop_generation')
                if request.command=='start_exploration':
                    if r.phase!='ready' or self.manual_used:
                        raise ValueError('Use a fresh scene without manual pre-mapping')
                    if not self.healthy() or not self.mapping.get('clearance_confirmed'):
                        missing=self.health_details()['missing']
                        if not self.mapping.get('clearance_confirmed'):missing.append('operator launch-clearance confirmation')
                        raise ValueError('Autonomy not ready: '+', '.join(missing))
                    if self.mapping.get('publication_latency_p95_seconds') is None or self.mapping['publication_latency_p95_seconds']>.2:
                        raise ValueError('Wait for at least 30 healthy publication timings; p95 must be <=200 ms')
                    pose=self.pose()
                    check_path(self.grid,[pose,[*pose[:2],pose[2]+2*math.pi]],pose,pose)
                    # Full circle is explicitly sampled; a wrapped 2pi endpoint alone is insufficient.
                    if not all(footprint_clear(self.grid,*pose[:2],pose[2]+a) for a in np.linspace(0,2*math.pi,73)):
                        raise ValueError('Initial observation rotation is not fully known clear')
                    home=self.transform_pose(self.pose(),'odom')
                    await self.arm(token)
                    try:
                        r.start(time.monotonic(),request.client_token,home,r.epoch,self.area(),self.manual_used)
                        self.sweep=[pose[2]+math.pi/2*i for i in range(1,5)]
                        self.mode='demo'
                    except Exception:
                        self.abort('Exploration initialization failed after arming')
                        raise
                elif request.command in ('resume_exploration','alternate_exploration'):
                    self.auto_retry_pending=False
                    await self.resume_exploration(now,token,alternate=request.command=='alternate_exploration')
                elif request.command=='retreat_and_replan':
                    await self.start_contact_retreat(now,token)
                elif request.command=='select_target':
                    if request.target_id.startswith('object:'):
                        site = self.selected_object(request.target_id.removeprefix('object:'), require_fresh=False)
                        target = dict(source='detected_object', track_id=site['track_id'], epoch=r.epoch,
                                      class_name=site['class_name'], odom_position=site['odom_position'],
                                      covariance=site['covariance'], envelope_radius_m=object_radius(site))
                    elif request.target_id:
                        site=self.selected_site(request.target_id,require_fresh=False)
                        target=dict(source='detected_soil',track_id=site['track_id'],epoch=r.epoch,
                            odom_position=site['odom_position'],covariance=site['covariance'],
                            envelope_radius_m=site['envelope_radius_m'])
                    else:
                        if not np.isfinite([request.x,request.y]).all():raise ValueError('Invalid map click')
                        if self.grid.at(request.x,request.y)<0 or self.grid.at(request.x,request.y)>=90:
                            raise ValueError('Select measured traversable ground')
                        point=self.transform_pose([request.x,request.y,0.],target='odom')
                        target=dict(source='operator_selected',odom_position=[*point[:2],0.],envelope_radius_m=0.)
                    r.select(now,target)
                    self.sample_view_angles.clear()
                    self.selection_generation+=1
                elif request.command in ('preview_sample','alternate_sample'):
                    await self.plan_sample_preview(token,alternate=request.command=='alternate_sample')
                elif request.command=='start_sample':
                    if not self.healthy():raise ValueError('Navigation health is unavailable')
                    preview=r.preview
                    if preview is None or preview['id']!=request.preview_id:
                        raise ValueError('Preview expired or changed; request a new preview')
                    # The verified owner's approval is live contact. Later
                    # heartbeats can refresh it while the guard acknowledges.
                    r.heartbeat_wall=now
                    await self.protect_sample(preview['keepout'],token)
                    home=self.home_map()
                    if (math.dist(preview['home'][:2],home[:2])>.05 or
                            abs(heading_error(preview['home'][2],home[2]))>SAMPLE_HEADING_TOLERANCE):
                        raise ValueError('Home pose changed; request a new preview')
                    if math.dist(preview['target_map'],self.sample_target_map())>.05:
                        raise ValueError('Map correction changed the target; request a new preview')
                    r.validate_approval(time.monotonic(),request.preview_id,self.pose(),self.grid,self.target_current_signature())
                    if time.monotonic()-r.heartbeat_wall>r.policy.heartbeat_seconds:
                        raise ValueError('Operator heartbeat expired while checking sample approval')
                    armed=False
                    try:
                        await self.arm(token)
                        armed=True
                        self.valid_token(token)
                        activated=time.monotonic()
                        if activated-r.heartbeat_wall>r.policy.heartbeat_seconds:
                            raise ValueError('Operator heartbeat expired while arming the sample trip')
                        r.approve(activated,request.preview_id,self.pose(),self.grid,self.target_current_signature())
                        if time.monotonic()-r.heartbeat_wall>r.policy.heartbeat_seconds:
                            raise ValueError('Operator heartbeat expired before the sample trip could depart')
                        self.mode='demo'
                        self.send_demo(preview['stopping_pose'],'sample')
                    except Exception as error:
                        if r.phase!='stopped':
                            self.abort(str(error))
                        elif armed and token==self.selection_generation and self.stop_client.service_is_ready():
                            # A selection deadline can expire while the arm
                            # receipt is in flight and the mission still idle.
                            self.stop_client.call_async(Trigger.Request())
                        raise
                else:raise ValueError('Unknown demo command')
                response.success=True
                self.working=False
                self.starting=False
            response.message='Command accepted'
        except Exception as error:
            if request.command!='heartbeat':
                r.event(time.monotonic(),'command_rejected',command=request.command,reason=str(error))
            # Pure preflight rejections do not erase a successful first return.
            if owns_operation:
                if r.phase in ACTIVE:self.abort(str(error))
                self.working=False
                self.starting=False
            response.message=str(error)
        response.mission_id=r.mission_id
        response.preview_id=r.preview['id'] if r.preview else ''
        if request.command!='heartbeat':self.flush()
        return response

    def selected_site(self,ident,require_fresh):
        if (time.monotonic()-self.soils_wall>1. or not self.soils.get('ready') or
                self.soils.get('epoch')!=self.run.epoch):raise ValueError('Soil evidence unavailable')
        site=next((s for s in self.soils.get('sites',[]) if s['track_id']==ident and s['state']=='confirmed'),None)
        if site is None:raise ValueError('Soil site retired, ambiguous or unconfirmed')
        if ident in self.soils.get('ambiguous_track_ids',[]):raise ValueError('Soil association is ambiguous')
        max_age=STATIONARY_SOIL_EVIDENCE_SECONDS if self.run.phase=='collecting' else .6
        if require_fresh and (not site.get('geometry',{}).get('supported') or site.get('last_confidence',0.)<.5 or
                not 0.<=self.get_clock().now().nanoseconds/1e9-site.get('last_supported_stamp',-math.inf)<=max_age):
            raise ValueError('Selected soil site was not freshly reacquired with supported RGB-D evidence')
        return site

    def selected_object(self, ident, require_fresh):
        if (time.monotonic() - self.objects_wall > 1. or not self.objects.get('ready')
                or self.objects.get('schema_version') != 2 or self.objects.get('epoch') != self.run.epoch):
            raise ValueError('Object evidence unavailable')
        site = next((s for s in self.objects.get('objects', ()) if s['track_id'] == ident
                     and s['state'] == 'confirmed' and s['class_name'] in SOLID_CLASSES), None)
        if site is None:
            raise ValueError('Object is no longer confirmed')
        if self.objects.get('ambiguous_track_ids') and ident in self.objects['ambiguous_track_ids']:
            raise ValueError('Object association is ambiguous')
        if require_fresh and (not site.get('geometry', {}).get('supported')
                or site.get('last_supported_confidence', 0.) < .5
                or not 0. <= self.get_clock().now().nanoseconds / 1e9 - site.get('last_supported_stamp', -math.inf) <= .6):
            raise ValueError('Selected object needs fresh RGB-D evidence at the sample stop')
        return site

    def target_current_signature(self):
        target=dict(self.run.target)
        if target['source'] in ('detected_soil', 'detected_object'):
            raised = target['source'] == 'detected_object'
            site = self.selected_object(target['track_id'], False) if raised else self.selected_site(target['track_id'], False)
            if raised and site['class_name'] != target['class_name']:
                raise ValueError('Object class changed; select and preview again')
            shift=math.dist(site['odom_position'][:2],target['odom_position'][:2])
            if shift>(.05 if raised else SOIL_REFINEMENT_ALLOWANCE_M):
                raise SampleGeometryChanged('Target position changed')
            radius = object_radius(site) if raised else site['envelope_radius_m']
            limit=(self.run.preview or {}).get('keepout', [0.,0.,self.sample_keepout_radius(target)])[2]
            if radius+shift+.05>limit+1e-9:
                raise SampleGeometryChanged('Target envelope grew beyond the checked clearance')
        return target_signature(target)

    @staticmethod
    def sample_keepout_radius(target):
        allowance=SOIL_REFINEMENT_ALLOWANCE_M if target['source']=='detected_soil' else .025
        return max(.20,target.get('envelope_radius_m',0.)+.05+allowance)

    def sample_target_map(self):
        return self.transform_pose([*self.run.target['odom_position'][:2],0.])[:2]

    async def recovery_return(self,start,home,token,*,keepout=None):
        try:
            return None,await self.path(start,home,keepout=keepout)
        except ValueError:
            self.valid_token(token)
        # A direct home plan can require an unsafe turn at the current pose.
        # Check forward connectors to measured space where there is room to turn.
        for distance in (.50,.80):
            for offset in (0.,.65,-.65,1.25,-1.25):
                if time.monotonic()>=self.operation_deadline:
                    raise ValueError('No checked route home found within the replan time limit')
                yaw=start[2]+offset
                point=[start[0]+distance*math.cos(yaw),start[1]+distance*math.sin(yaw),yaw]
                if not frontier_stop_clear(self.grid,point,REPOSITION_ARRIVAL_MARGIN):
                    continue
                try:
                    outward=await self.path(start,point,keepout=keepout)
                    back=await self.path(point,home,keepout=keepout)
                    if not all(frontier_stop_clear(self.grid,p,REPOSITION_ARRIVAL_MARGIN)
                               for p in (point,outward[-1],back[0])):
                        continue
                    return point,[*outward,point,*back]
                except ValueError:
                    self.valid_token(token)
        raise ValueError('No checked route home or clear repositioning route is available')

    def contact_ready(self):
        guard=self.guard
        return (self.run.phase=='contact_paused' and guard.get('mode')=='stopped' and
                guard.get('in_contact') is False and (guard.get('fault') or {}).get('reason')=='chassis_contact'
                and self.sample_stationary() and self.healthy())

    async def start_contact_retreat(self,now,token):
        r=self.run
        if not self.contact_ready():
            raise ValueError('Backing up requires a stopped rover, fresh sensors, and cleared chassis contact')
        if not self.retreat_client.service_is_ready():
            raise ValueError('The reverse-only guard service is unavailable')
        history=[self.transform_pose(row['odom_pose']) for row in self.trails[-90:]
                 if row['wall']>=self.contact_trail_since]
        keepout=r.preview['keepout'] if r.preview else None
        retreat=recorded_retreat(self.grid,self.pose(),history,keepout)
        r.heartbeat_wall=now
        armed=False
        try:
            await self.arm(token,retreat=True)
            armed=True
            self.valid_token(token)
            now=time.monotonic()
            if r.phase!='contact_paused' or now-r.heartbeat_wall>r.policy.heartbeat_seconds:
                raise ValueError('Reverse authorization expired while arming')
            self.retreat_route=retreat
            self.retreat_progress=0.
            self.retreat_deadline=now+sample_reverse_reserve(r.policy.speed,self.rtf())
            self.mode='demo'
            r.transition('contact_reversing',now)
            r.event(now,'contact_retreat_started',route=retreat)
        except Exception:
            if armed:self.abort('Reverse authorization interrupted')
            raise

    async def advance_contact_retreat(self,now):
        r=self.run
        pose=self.pose()
        if now>=self.retreat_deadline:
            raise ValueError('Timed out while following the recorded reverse route')
        self.retreat_progress,next_pose=follow_recorded_retreat(self.retreat_route,pose,self.retreat_progress)
        cmd=Twist()
        if next_pose is not None:
            keepout=r.preview['keepout'] if r.preview else None
            check_path(self.grid,[pose,next_pose],pose,next_pose,keepout=keepout)
            yaw_error=heading_error(pose[2],next_pose[2])
            if abs(yaw_error)>.55:
                raise ValueError('Rover heading left the checked reverse route')
            cmd.linear.x=-min(.05,max(.02,(CONTACT_RETREAT_DISTANCE-self.retreat_progress)/1.4))
            cmd.angular.z=max(-.25,min(.25,2.4*yaw_error))
            self.retreat_pub.publish(cmd)
            return
        self.retreat_pub.publish(cmd)
        if not self.sample_stationary():return
        r.event(now,'contact_retreat_completed',distance_m=self.retreat_progress)
        await self.stopped()
        self.contact_point=[*self.retreat_route[0][:2],CONTACT_RETRY_CLEARANCE]
        self.starting=True
        self.start_epoch=self.guard.get('stop_generation')
        self.operation_deadline=time.monotonic()+7.
        try:
            await self.replan_after_contact()
        except ValueError as error:
            raise ValueError('Backed away. Alternate route unavailable: '+str(error)) from error
        finally:
            self.operation_deadline=None
            self.starting=False

    async def replan_after_contact(self):
        r=self.run
        token=self.selection_generation
        start=self.pose()
        home=self.home_map()
        keepout=r.preview['keepout'] if r.preview else None
        outbound_sample=r.contact_from=='sample_outbound'
        target=r.preview['stopping_pose'] if outbound_sample else home
        connector,route=await self.recovery_return(start,target,token,keepout=keepout)
        estimate=return_reserve(route,r.policy.speed,self.rtf())
        if outbound_sample:
            self.target_current_signature()
            self.return_path=await self.path(r.preview['backup_pose'],home,keepout=keepout)
            estimate+=return_reserve(self.return_path,r.policy.speed,self.rtf())
            estimate+=sample_reverse_reserve(r.policy.speed,self.rtf())+r.policy.collection_seconds
        now=time.monotonic()
        remaining=(r.sample_time_left(now) if keepout else r.policy.exploration_seconds-(now-r.started))
        if estimate>=remaining or now-r.heartbeat_wall>r.policy.heartbeat_seconds:
            raise ValueError('Replan needs a live operator and enough time for a safe return')
        await self.arm(token)
        self.valid_token(token)
        now=time.monotonic()
        if r.phase!='contact_reversing' or now-r.heartbeat_wall>r.policy.heartbeat_seconds:
            raise ValueError('Replan authorization expired while arming')
        if outbound_sample:
            r.preview['outward']=route
            r.preview['back']=self.return_path
        if connector is not None:
            r.transition('contact_rejoin',now)
            self.send_demo(connector,'clearance')
        else:
            self.continue_after_contact(now)

    def continue_after_contact(self,now):
        r=self.run
        next_phase=r.contact_from
        if next_phase in ('exploring','exploration_reposition'):
            next_phase='exploration_return' if self.return_requested or self.resume_return else 'exploring'
        self.nav_done=None
        self.last_route_check=-math.inf
        self.return_requested=False
        r.transition(next_phase,now)
        if next_phase=='sample_outbound':
            self.send_demo(r.preview['stopping_pose'],'sample')

    async def alternative_route(self,start,home,token):
        if not self.failed_route:
            raise ValueError('No recorded failed route to compare against')
        waypoints=[]
        for distance in (1.0,1.5,2.0):
            for offset in (.7,-.7,1.4,-1.4,2.1,-2.1,math.pi,0.):
                yaw=start[2]+offset
                point=[start[0]+distance*math.cos(yaw),start[1]+distance*math.sin(yaw),yaw]
                separation=distance_to_route(point,self.failed_route)
                if separation>=.65 and frontier_stop_clear(self.grid,point,REPOSITION_ARRIVAL_MARGIN):
                    waypoints.append((distance/separation,point))
        for _,point in sorted(waypoints,key=lambda entry:entry[0]):
            if time.monotonic()>=self.operation_deadline:break
            self.valid_token(token)
            try:
                outward=await self.path(start,point)
                separation=distance_to_route(outward[-1],self.failed_route)
                # This waypoint only gives the rover room to take another route.
                # Skip connectors that loop around to reach a nearby position.
                if separation<.60 or path_length(outward)>2.*math.dist(start[:2],point[:2]):
                    continue
                back=await self.path(point,home)
                if not all(frontier_stop_clear(self.grid,p,REPOSITION_ARRIVAL_MARGIN)
                           for p in (point,outward[-1],back[0])):
                    continue
                return point,[*outward,point,*back]
            except ValueError:
                self.valid_token(token)
        raise ValueError('No safe distinct route found on measured terrain')

    def check_alternate_departure(self,path):
        if self.alternate_route and any(distance_to_route(p,self.alternate_route)<.32
                                        for p in path if math.dist(p[:2],path[0][:2])<1.4):
            raise ValueError('Frontier route rejoins the failed corridor')

    def route_resume_ready(self):
        return (self.run.phase=='exploration_paused' and self.pause_stop_epoch is None and
                self.guard.get('mode')=='stopped' and not self.guard.get('fault') and
                self.sample_stationary() and self.healthy())

    async def resume_exploration(self,now,token,*,alternate=False,automatic=False):
        r=self.run
        if not self.route_resume_ready():
            raise ValueError('Resume requires the rover to be stopped with healthy navigation and no guard fault')
        if automatic:
            if now-r.heartbeat_wall>r.policy.heartbeat_seconds:
                raise ValueError('Operator heartbeat expired before the automatic replan')
        else:
            r.heartbeat_wall=now
        self.operation_deadline=now+3.5
        armed=False
        try:
            start=self.pose()
            planner=self.alternative_route if alternate else self.recovery_return
            point,route=await planner(start,self.home_map(),token)
            now=time.monotonic()
            reserve=return_reserve(route,r.policy.speed,self.rtf())
            if now-r.started+reserve>=r.policy.exploration_seconds:
                raise ValueError('Too little exploration time remains for the checked return')
            if now-r.heartbeat_wall>r.policy.heartbeat_seconds:
                raise ValueError('Operator heartbeat expired during the replan')
            await self.arm(token)
            armed=True
            self.valid_token(token)
            now=time.monotonic()
            if (r.phase!='exploration_paused' or now-r.heartbeat_wall>r.policy.heartbeat_seconds or
                    now-r.started+reserve>=r.policy.exploration_seconds):
                raise ValueError('Resume authorization expired while arming')
            self.mode='demo'
            self.return_path=route
            self.return_seconds=reserve
            self.last_route_check=-math.inf
            self.return_requested=False
            self.alternate_route=list(self.failed_route) if alternate else []
            r.transition('exploration_reposition' if point else 'exploration_return' if self.resume_return
                         else 'exploring',now)
            r.event(now,'exploration_resumed',reposition=point,alternate=alternate,automatic=automatic)
            if point:
                self.send_demo(point,'clearance')
        except Exception as error:
            if armed:
                self.abort(str(error))
            raise
        finally:
            self.operation_deadline=None

    async def automatic_replan(self,now,token):
        self.auto_retry_count+=1
        self.starting=True
        self.start_epoch=self.guard.get('stop_generation')
        try:
            # A failed arrival does not invalidate the approach corridor. Check
            # a way home and search again from the stopped pose when possible.
            alternate=not self.frontier_arrival_blocked
            try:
                await self.resume_exploration(now,token,alternate=alternate,automatic=True)
            except ValueError as error:
                if (not alternate or self.run.phase!='exploration_paused' or token!=self.selection_generation or
                        not self.route_resume_ready()):
                    raise
                now=time.monotonic()
                if now-self.run.heartbeat_wall>self.run.policy.heartbeat_seconds:
                    raise
                self.resume_return=True
                self.run.event(now,'automatic_return_fallback',reason=str(error))
                await self.resume_exploration(now,token,automatic=True)
        except Exception as error:
            if self.run.phase=='exploration_paused':
                self.run.reason='Automatic replan stopped: '+str(error)
                self.run.event(time.monotonic(),'automatic_replan_rejected',reason=str(error))
        finally:
            self.auto_retry_pending=False
            self.starting=False
            self.working=False

    def schedule_automatic_replan(self,now):
        r=self.run
        if r.phase!='exploration_paused' or not self.auto_retry_pending or self.working:return
        if now>=self.auto_retry_until or now-r.heartbeat_wall>r.policy.heartbeat_seconds:
            self.auto_retry_pending=False
            return
        if self.route_resume_ready():
            self.working=True
            self.executor.create_task(self.automatic_replan,now,self.selection_generation)

    async def plan_sample_preview(self,token,*,alternate=False):
        r=self.run
        if r.phase not in ('awaiting_target','preview_ready') or r.target is None:
            raise ValueError('Select a sample target first')
        if self.pause_stop_epoch is not None or self.guard.get('mode')!='stopped' or not self.sample_stationary():
            raise ValueError('Wait for the rover to stop before checking a sample path')
        previous,wall=r.preview,r.preview_wall
        if alternate and previous is None:
            raise ValueError('Preview one approach before looking for a different angle')
        if previous:
            r.preview=None
            r.transition('awaiting_target',time.monotonic())
        self.operation_deadline=time.monotonic()+24.
        try:
            await self.preview_sample(token,alternate=alternate)
        except Exception:
            if previous and token==self.selection_generation and r.phase=='awaiting_target':
                r.preview,r.preview_wall=previous,wall
                r.transition('preview_ready',time.monotonic())
            raise
        finally:
            self.operation_deadline=None

    async def preview_sample(self,token,*,alternate=False):
        target=self.run.target
        point=self.sample_target_map()
        start=self.pose();home=self.home_map()
        envelope=target.get('envelope_radius_m',0.)
        keepout=[*point,self.sample_keepout_radius(target)]
        candidates=collection_poses(self.grid,point,envelope,target.get('covariance'),
                                    raised_object=target['source']=='detected_object',
                                    detected_soil=target['source']=='detected_soil')
        if alternate:
            candidates=[pose for pose in candidates if all(
                abs(heading_error(pose[2],angle))>=math.radians(35)
                for angle in self.sample_view_angles)]
        candidates.sort(key=lambda p:(round(math.dist(p[:2],point),6),math.dist(p[:2],start[:2])))
        deadline=time.monotonic()+24.
        errors=[]
        budget_errors=[]
        for pose in candidates:
            if time.monotonic()>=deadline:break
            try:
                self.valid_token(token)
                backup=sample_reverse_pose(self.grid,pose,keepout)
                outward=await self.planned_path(start,pose,token)
                back=await self.planned_path(backup,home,token)
                a,b=[pose_values(p) for p in outward.poses],[pose_values(p) for p in back.poses]
                if self.contact_point is not None:
                    check_path(self.grid,a,start,pose,keepout=keepout,avoid_center=self.contact_point)
                    check_path(self.grid,b,backup,home,keepout=keepout,avoid_center=self.contact_point)
                self.run.set_preview(time.monotonic(),a,b,pose,self.pose(),self.grid,self.target_current_signature(),
                                     home=home,target_map=point,keepout=keepout,rtf=self.rtf())
                self.sample_view_angles.append(pose[2])
                return
            except SampleBudgetError as error:budget_errors.append(error)
            except ValueError as error:
                self.valid_token(token)
                errors.append(str(error))
        if budget_errors:
            raise min(budget_errors, key=lambda error:error.required)
        if alternate:
            raise ValueError('No clear round trip from a distinct approach angle')
        raise ValueError('No clear sample round trip found'+('. '+errors[-1] if errors else '. Select a site with more measured ground around it.'))

    def send_demo(self,target,purpose):
        if purpose in ('frontier','sweep') and self.return_requested:return
        if self.goal is not None or self.pending:raise ValueError('A navigation goal is already active')
        self.current_target,self.current_purpose=list(target),purpose
        self.active_path=[]
        self.goal_started=time.monotonic()
        self.pending=True
        self.nav_done=None
        self.waypoint_stop_deadline=None
        token=self.selection_generation
        generation=self.generation
        goal=NavigateToPose.Goal();goal.pose=self.selected_pose(target)
        if purpose in ('frontier','clearance'):
            goal.behavior_tree=self.frontier_bt if purpose=='frontier' else self.clearance_bt
        def accepted(future):
            handle=future.result()
            if token!=self.selection_generation or generation!=self.generation or self.run.phase not in ACTIVE:
                if handle.accepted:handle.cancel_goal_async()
                return
            self.pending=False
            if not handle.accepted:self.abort('Nav2 rejected autonomous goal');return
            self.goal=handle
            def completed(f):
                if token!=self.selection_generation or generation!=self.generation:return
                self.finish_navigation(f.result(),purpose)
            handle.get_result_async().add_done_callback(completed)
        self.nav.send_goal_async(goal).add_done_callback(accepted)

    def waypoint_kind(self):
        if self.current_purpose==WAYPOINT_PURPOSE.get(self.run.phase):
            return self.current_purpose
        return None

    def waypoint_handoff_pending(self):
        kind=self.waypoint_kind()
        return bool(kind) and (self.nav_done==kind or
                               self.goal is not None and self.waypoint_stop_deadline is not None)

    def waypoint_arrival_pose(self, now):
        kind=self.waypoint_kind()
        if (not kind or self.current_target is None or
                self.guard.get('mode')!='autonomous' or self.guard.get('fault') or not self.healthy() or
                now-self.run.heartbeat_wall>self.run.policy.heartbeat_seconds):
            return None
        try:
            pose=self.pose()
        except (ValueError,TransformException):
            return None
        radius,yaw=(FRONTIER_ARRIVAL_RADIUS,FRONTIER_ARRIVAL_YAW) if kind=='frontier' else (
            REPOSITION_ARRIVAL_RADIUS,REPOSITION_ARRIVAL_YAW)
        target=self.current_target
        path_yaw=self.active_path[-1][2] if self.active_path else target[2]
        if (math.dist(pose[:2],target[:2])>radius or abs(heading_error(pose[2],path_yaw))>yaw or
                not frontier_stop_clear(self.grid,pose,WAYPOINT_CLEARANCE_PAD)):
            return None
        if self.contact_point and math.dist(pose[:2],self.contact_point[:2])<self.contact_point[2]:
            return None
        keepout=self.run.preview['keepout'] if self.run.preview else None
        if keepout is not None and not footprint_avoids_disk(pose,keepout):
            return None
        return pose

    def check_waypoint_arrival(self, now):
        if self.goal is None:return
        if self.waypoint_stop_deadline is not None:
            if now>=self.waypoint_stop_deadline:
                self.abort('Nav2 did not acknowledge the waypoint stop')
            return
        if self.return_requested:return
        pose=self.waypoint_arrival_pose(now)
        if pose is None:return
        # Keep the goal handle until its terminal result. That prevents a new
        # drive from competing with a controller that is still stopping.
        self.waypoint_stop_deadline=now+2.
        self.goal.cancel_goal_async()
        self.run.event(now,'waypoint_stop_requested',purpose=self.current_purpose,pose=pose,
                       target=self.current_target)

    def finish_navigation(self,answer,purpose):
        self.goal=None
        stop_requested=self.waypoint_stop_deadline is not None
        self.waypoint_stop_deadline=None
        now=time.monotonic()
        reached=(answer.status==GoalStatus.STATUS_SUCCEEDED or
                 stop_requested and answer.status==GoalStatus.STATUS_CANCELED)
        if (not reached and answer.status==GoalStatus.STATUS_ABORTED and
                getattr(answer.result,'error_code',None)==NAV2_FAILED_TO_MAKE_PROGRESS):
            pose=self.waypoint_arrival_pose(now)
            if pose is not None:
                reached=True
                self.run.event(now,'waypoint_progress_result_accepted',purpose=purpose,pose=pose,
                               target=self.current_target)
        if not reached:
            self.route_blocked('Navigation failed: '+self.nav2_failure(answer))
            return
        self.nav_arrived_at=now
        self.nav_done=purpose

    async def path(self,start,target,*,keepout=None):
        result=await self.planned_path(start,target,self.selection_generation)
        values=[pose_values(p) for p in result.poses]
        check_path(self.grid,values,start,target,keepout=keepout,avoid_center=self.contact_point)
        return values

    def sample_stationary(self):
        return (self.guard.get('command')==[0.,0.] and np.isfinite(self.velocity).all() and
                abs(self.velocity[0])<.01 and abs(self.velocity[1])<.02)

    def waypoint_settled(self, now):
        """Require post-arrival feedback before leaving an observation or recovery stop."""
        arrival=self.nav_arrived_at
        if min(self.guard_wall,self.odometry_wall)>arrival and self.sample_stationary():
            if self.waypoint_arrival_pose(now) is None:
                raise ValueError('Rover settled outside the checked waypoint region')
            return True
        if now-arrival>=2.:
            raise ValueError('Rover did not settle after reaching the waypoint')
        return False

    def sample_return_ready(self):
        return (self.run.phase=='awaiting_sample_return' and self.run.preview is not None and
                self.guard.get('mode')=='autonomous' and not self.guard.get('fault') and
                self.sample_stationary() and self.healthy())

    def sample_motion(self, linear=0., angular=0.):
        message=Twist()
        message.linear.x,message.angular.z=linear,angular
        self.sample_motion_pub.publish(message)

    def advance_collection(self,now):
        r=self.run
        preview=r.preview
        point=self.sample_target_map()
        if math.dist(preview['target_map'],point)>.05:
            raise ValueError('Target map pose changed during sampling')
        self.target_current_signature()
        if r.target['source']=='detected_soil':
            site=self.selected_site(r.target['track_id'],False)
            point=self.transform_pose([*site['odom_position'][:2],0.])[:2]
        if r.phase=='sample_outbound':
            reserve=return_reserve(preview['back'],r.policy.speed,self.rtf())
            self.return_seconds=reserve+sample_reverse_reserve(r.policy.speed,self.rtf())
            if self.return_seconds+r.policy.collection_seconds>=r.sample_time_left(now):
                raise ValueError('Sampling return reserve exhausted; mission stopped')
            if self.goal is not None or self.pending or self.nav_done!='sample':return
        pose=self.pose()
        desired=math.atan2(point[1]-pose[1],point[0]-pose[0])
        error=heading_error(pose[2],desired)
        stationary=self.sample_stationary()
        if r.phase=='sample_outbound':
            if math.dist(pose[:2],preview['stopping_pose'][:2])>.08:
                raise ValueError('Nav2 stopped outside the sample approach tolerance')
            if self.sample_arrival_wall is None:
                if not stationary:return
                self.sample_arrival_wall=now
            if now-self.sample_arrival_wall>12.:
                raise ValueError('Could not align with or reacquire the selected sample')
            if abs(error)>math.radians(3):
                aligned=[*pose[:2],desired]
                check_path(self.grid,[pose,aligned],pose,aligned,keepout=preview['keepout'])
                self.sample_motion(angular=math.copysign(.15,error))
                return
            self.sample_motion()
            if not stationary:return
        fresh=True
        try:
            if r.target['source']=='detected_soil':
                self.selected_site(r.target['track_id'],True)
            elif r.target['source']=='detected_object':
                self.selected_object(r.target['track_id'],True)
        except ValueError:
            fresh=False
        if r.phase=='sample_outbound':
            if not fresh:return
            self.nav_done=None
            self.collection_pose=list(self.odometry_pose)
            r.event(now,'sample_aligned',heading_error_rad=error,position=pose[:2])
        r.collect(now,stationary,fresh and abs(error)<=SAMPLE_HEADING_TOLERANCE,self.healthy())
        if r.phase=='stopped':self.abort(r.reason)

    async def back_away_from_sample(self,now):
        r=self.run
        keepout=r.preview['keepout']
        pose=self.pose()
        if self.backup_start is None:
            if not self.sample_stationary():return
            end=sample_reverse_pose(self.grid,pose,keepout)
            path=await self.path(end,self.home_map(),keepout=keepout)
            self.return_seconds=return_reserve(path,r.policy.speed,self.rtf())
            reverse_time=sample_reverse_reserve(r.policy.speed,self.rtf())
            if self.return_seconds+reverse_time>=r.sample_time_left(time.monotonic()):
                raise ValueError('Insufficient time to back away and return from the sample')
            self.backup_start,self.backup_end=list(pose),end
            self.backup_deadline=time.monotonic()+reverse_time
            self.return_path=path
            r.event(now,'sample_reverse_started',start=pose,end=end)
        x,y,yaw=self.backup_start
        dx,dy=x-pose[0],y-pose[1]
        progress=dx*math.cos(yaw)+dy*math.sin(yaw)
        lateral=dy*math.cos(yaw)-dx*math.sin(yaw)
        if (now>=self.backup_deadline or not -.03<=progress<=SAMPLE_REVERSE_DISTANCE+.05 or
                abs(lateral)>.05 or abs(heading_error(pose[2],yaw))>SAMPLE_HEADING_TOLERANCE):
            raise ValueError('Reverse left the checked sample departure corridor or timed out')
        if progress<SAMPLE_REVERSE_DISTANCE-.02:
            check_path(self.grid,[pose,self.backup_end],pose,self.backup_end,keepout=keepout)
            self.sample_motion(linear=-min(r.policy.speed,SAMPLE_REVERSE_MAX_SPEED))
            return
        self.sample_motion()
        if not self.sample_stationary():return
        check_sample_turn(self.grid,pose,keepout)
        r.event(now,'sample_reverse_completed',distance_m=progress,lateral_error_m=lateral)
        self.home_attempts=0
        r.transition('sample_return',now)

    def exploration_trail(self):
        points=[entry['odom_pose'][:2] for entry in self.trails if entry['phase'] in EXPLORATION_ROUTE_PHASES]
        if not points:return ()
        xy=np.asarray(points)
        _,indices=np.unique(np.floor(xy/.20),axis=0,return_index=True)
        x,y,yaw=self.transform_pose([0.,0.,0.])
        c,s=math.cos(yaw),math.sin(yaw)
        return xy[indices] @ np.array([[c,s],[-s,c]])+[x,y]

    async def choose_frontier(self,goals):
        r=self.run
        survey=r.policy.mapping_seconds is not None
        generation=self.selection_generation
        checked=[]
        deferred=[]
        budget_rejected=False
        retrace_rejected=False
        last_candidate=len(goals)
        for index,candidate in enumerate(goals):
            if index>=last_candidate:break
            now=time.monotonic()
            reason=r.exploration_reason(now,self.return_path,self.rtf(),goals)
            if reason:
                r.transition('exploration_return',now,reason)
                return
            pose=candidate['pose']
            try:
                outward=await self.path(self.pose(),pose)
                self.check_alternate_departure(outward)
                overlap=exploration_overlap(outward,self.frontier_trail)
                retrace_seconds=path_length(outward)*overlap/r.policy.speed/min(1.,self.rtf())
                long_retrace=overlap>=.80 and retrace_seconds>MAX_FRONTIER_RETRACE_SECONDS
                if long_retrace and not survey:
                    retrace_rejected=True
                    r.event(time.monotonic(),'planning_candidate_rejected',
                            reason='Frontier requires a long retrace of earlier travel',pose=pose,
                            outward_trail_overlap=overlap,estimated_retrace_seconds=retrace_seconds)
                    continue
                back=await self.path(pose,self.home_map())
                if self.return_requested or r.phase!='exploring' or generation!=self.selection_generation:return
                outward_seconds=return_reserve(outward,r.policy.speed,self.rtf(),contingency_seconds=0.)
                back_seconds=return_reserve(back,r.policy.speed,self.rtf())
                remaining=r.policy.exploration_seconds-(time.monotonic()-r.started)
                if outward_seconds+back_seconds>=remaining:
                    budget_rejected=True
                    r.event(time.monotonic(),'planning_candidate_rejected',reason='Exploration round trip exceeds remaining budget',
                        pose=pose,outward_reserve_seconds=outward_seconds,return_reserve_seconds=back_seconds,
                        remaining_seconds=remaining)
                    continue
                if not all(frontier_stop_clear(self.grid,p) for p in (pose,outward[-1],back[0])):
                    raise ValueError('Frontier approach or departure lacks clear turning space')
                # Compare checked travel and viewing gain. The return route
                # must fit the reserve regardless of the frontier's score.
                score=candidate['expected_gain_m2']*(1.-.8*overlap)/(1.+outward_seconds)
                trip=(score,candidate,outward,back,outward_seconds,back_seconds,overlap)
                if long_retrace:
                    deferred.append(trip)
                else:
                    if not checked:last_candidate=min(last_candidate,index+ROUTE_COMPARISON_LIMIT)
                    checked.append(trip)
            except ValueError as error:
                if self.return_requested or r.phase!='exploring' or generation!=self.selection_generation:return
                self.visited.append(self.transform_pose(pose,'odom')[:2])
                r.event(time.monotonic(),'planning_candidate_rejected',reason=str(error),pose=pose)
        ranked=sorted(checked,key=lambda trip:-trip[0])+sorted(deferred,key=lambda trip:-trip[0])
        for score,candidate,outward,back,outward_seconds,back_seconds,overlap in ranked:
            pose=candidate['pose']
            now=time.monotonic()
            reason=r.exploration_reason(now,self.return_path,self.rtf(),goals)
            if reason:
                r.transition('exploration_return',now,reason)
                return
            remaining=r.policy.exploration_seconds-(now-r.started)
            if outward_seconds+back_seconds>=remaining:
                budget_rejected=True
                continue
            try:
                check_path(self.grid,outward,self.pose(),pose)
                check_path(self.grid,back,pose,self.home_map())
                if not all(frontier_stop_clear(self.grid,p) for p in (pose,outward[-1],back[0])):
                    raise ValueError('Frontier stop no longer has clear turning space')
            except ValueError as error:
                r.event(time.monotonic(),'planning_candidate_rejected',reason=str(error),pose=pose)
                continue
            r.event(time.monotonic(),'frontier_selected',pose=pose,expected_gain_m2=candidate['expected_gain_m2'],
                trail_novelty=candidate['trail_novelty'],outward_trail_overlap=overlap,
                outward_distance_m=path_length(outward),return_distance_m=path_length(back),
                outward_reserve_seconds=outward_seconds,return_reserve_seconds=back_seconds,
                remaining_seconds=remaining,route_score=score)
            self.frontier=pose
            self.send_demo(pose,'frontier');return
        reason='No checked route to a frontier remains'
        if budget_rejected and retrace_rejected:
            reason='Remaining frontiers exceed the return budget or retrace limit'
        elif budget_rejected:
            reason='No feasible frontier within the remaining return budget'
        elif retrace_rejected:
            reason='No frontier within the retrace limit'
        r.transition('exploration_return',time.monotonic(),reason)

    async def advance(self):
        r=self.run
        try:
            now=time.monotonic()
            if self.goal is not None and self.waypoint_stop_deadline is not None:return
            if self.return_requested and r.phase in ('sweeping','exploring'):
                if self.nav_done=='frontier' and not self.waypoint_settled(now):return
                self.nav_done=None
                r.transition('exploration_return',now,'Operator requested early return')
            if r.phase=='sweeping':
                if self.goal is not None or self.pending:return
                if self.nav_done:self.nav_done=None
                if self.sweep:
                    current=self.pose();target=[*current[:2],self.sweep.pop(0)]
                    check_path(self.grid,[current,target],current,target)
                    self.send_demo(target,'sweep')
                else:
                    r.baseline_area=r.visit_area=self.area()
                    r.transition('exploring',now)
            elif r.phase=='exploring':
                if self.nav_done=='frontier':
                    if not self.waypoint_settled(now):return
                    r.visited(now,self.transform_pose(self.pose(),'odom'),self.area())
                    self.visited.append(self.transform_pose(self.current_target,'odom')[:2]);self.nav_done=None
                    self.frontier=None
                    self.alternate_route=[]
                    self.auto_retry_count=0
                    self.last_route_check=-math.inf
                elif self.frontier is not None and not frontier_stop_clear(self.grid,self.frontier):
                    self.route_blocked('The frontier stop lost its obstacle clearance', frontier_arrival=True)
                    return
                start=self.pose();home=self.home_map()
                if now-self.last_route_check>=1.:
                    try:
                        return_path=await self.path(start,home)
                    except ValueError as error:
                        if not self.waypoint_handoff_pending():
                            self.route_blocked('Return route blocked: '+str(error))
                        return
                    if self.waypoint_handoff_pending():return
                    self.return_path=return_path
                    self.last_route_check=time.monotonic()
                    self.return_seconds=return_reserve(self.return_path,r.policy.speed,self.rtf())
                    reason=r.exploration_reason(time.monotonic(),self.return_path,self.rtf(),[True])
                    if reason:
                        self.cancel();self.nav_done=None;r.transition('exploration_return',now,reason);return
                if self.goal is not None or self.pending:return
                if self.frontier_future is None:
                    attempted=[self.transform_pose([*v,0.])[:2] for v in self.visited]
                    grid=Grid(self.grid.data.copy(),self.grid.resolution,self.grid.x,self.grid.y)
                    self.frontier_trail=self.exploration_trail()
                    limit=FRONTIER_CANDIDATE_LIMIT if r.policy.mapping_seconds is not None else FRONTIER_SHORTLIST_SIZE
                    self.frontier_future=self.worker.submit(observation_frontiers,grid,start,attempted,
                        max_goals=limit,speed=r.policy.speed,angular_speed=r.policy.angular_speed,home=home,
                        trail=self.frontier_trail)
                    return
                if not self.frontier_future.done():return
                goals=self.frontier_future.result();self.frontier_future=None
                r.event(time.monotonic(),'frontier_search',candidate_count=len(goals),
                    speed_mps=r.policy.speed,elapsed_seconds=time.monotonic()-r.started,
                    exploration_budget_seconds=r.policy.exploration_seconds)
                reason=r.exploration_reason(now,self.return_path,self.rtf(),goals)
                if reason:r.transition('exploration_return',now,reason);return
                await self.choose_frontier(goals)
            elif r.phase=='exploration_reposition' and self.nav_done=='clearance':
                if not self.waypoint_settled(now):return
                self.nav_done=None
                r.transition('exploration_return' if self.resume_return else 'exploring',now)
            elif r.phase in ('exploration_return','sample_return'):
                if self.goal is not None or self.pending:return
                if self.nav_done=='home':
                    self.nav_done=None
                    pose=self.pose();home=self.home_map()
                    distance=math.dist(pose[:2],home[:2])
                    angle=abs(math.atan2(math.sin(pose[2]-home[2]),math.cos(pose[2]-home[2])))
                    if distance>.08 or angle>math.radians(5):
                        if self.home_attempts>=2:raise ValueError('Home alignment failed')
                        self.home_attempts+=1
                    else:
                        await self.stopped()
                        r.event(time.monotonic(),'verified_home_arrival',position_error_m=distance,
                            heading_error_rad=angle,reference='estimated_drive_center',command=list(self.guard['command']))
                        r.home_arrived(time.monotonic());self.mode='idle';return
                keepout=r.preview['keepout'] if r.phase=='sample_return' else None
                self.return_path=await self.path(self.pose(),self.home_map(),keepout=keepout)
                self.send_demo(self.home_map(),'home')
            elif r.phase in ('sample_outbound','collecting'):
                self.advance_collection(now)
            elif r.phase=='sample_backing':
                await self.back_away_from_sample(now)
            elif r.phase=='contact_reversing':
                await self.advance_contact_retreat(now)
            elif r.phase=='contact_rejoin' and self.nav_done=='clearance':
                if not self.waypoint_settled(now):return
                self.continue_after_contact(now)
        except SampleGeometryChanged as error:
            self.pause_for_sample_review(str(error))
        except ValueError as error:
            self.route_blocked(str(error))
        except Exception as error:
            self.abort(str(error))
        finally:self.working=False

    def rtf(self):
        if len(self.rtf_samples)<2:return 1.
        a,b=self.rtf_samples[0],self.rtf_samples[-1]
        return max(.01,(b[1]-a[1])/max(.001,b[0]-a[0]))

    def check_deadlines(self,now):
        r=self.run;health=self.health_details()
        heartbeat_age=now-r.heartbeat_wall
        if r.phase in ACTIVE and (not health['healthy'] or heartbeat_age>r.policy.heartbeat_seconds):
            ages={}
            for name,attribute in (('estimated_odometry','odometry_wall'),('motion_guard','guard_wall'),
                                   ('accepted_depth','mapping_wall'),('detector','detector_wall')):
                received=getattr(self,attribute,-math.inf)
                ages[name]=now-received if math.isfinite(received) else None
            r.event(now,'autonomy_deadline_inputs',navigation_health=health,
                heartbeat_age_seconds=heartbeat_age,heartbeat_limit_seconds=r.policy.heartbeat_seconds,
                heartbeat_sequence=r.heartbeat_sequence,input_ages_seconds=ages,
                guard={key:self.guard.get(key) for key in ('mode','ready','reason','sequence')})
        r.deadlines(now,health['healthy'],health['missing'])

    def tick(self):
        if not hasattr(self,'run'):return
        now=time.monotonic();r=self.run
        self.check_deadlines(now)
        if r.phase=='stopped' and self.mode!='idle':self.abort(r.reason)
        self.check_waypoint_arrival(now)
        sim=self.get_clock().now().nanoseconds/1e9
        self.rtf_samples.append((now,sim));self.rtf_samples=self.rtf_samples[-150:]
        if self.odometry_pose is not None and r.phase in ACTIVE:
            phase='exploring' if r.phase=='sweeping' else r.phase
            previous=self.trails[-1] if self.trails else None
            new_segment=(previous is None or previous['phase_epoch']!=r.phase_started or
                         now-previous['wall']>1.5)
            odom=drive_pose(*self.odometry_pose)
            if (new_segment or math.dist(previous['odom_pose'][:2],odom[:2])>.02 or
                    now-previous['wall']>=1.):
                try:estimated=self.transform_pose(self.pose(),'odom')
                except Exception as error:self.abort('Trail localization unavailable: '+str(error));return
                if previous:
                    distance=math.dist(estimated[:2],previous['odom_pose'][:2])
                    new_segment |= distance>.20
                    if not new_segment:self.actual_distance+=distance
                segment=now if new_segment else previous['segment']
                self.trails.append(dict(phase=phase,phase_epoch=r.phase_started,segment=segment,
                                        odom_pose=estimated,wall=now,sim_time=sim))
                r.event(now,'trajectory',**self.trails[-1])
        self.schedule_automatic_replan(now)
        self.publish_status()
        if r.phase in ACTIVE and r.phase!='awaiting_sample_return' and not self.working:
            self.working=True;self.executor.create_task(self.advance)
        self.flush()

    def publish_status(self):
        if not hasattr(self,'run'):return
        now=time.monotonic();status=self.run.status(now)
        try:
            status['home']=self.home_map() if self.run.home is not None else None
            selected=self.trails[::max(1,math.ceil(len(self.trails)/2000))]
            correction=self.tf.lookup_transform('map','odom',Time())
            points=apply_tf([[*p['odom_pose'][:2],0.] for p in selected],correction) if selected else []
            status['trails']=[dict(phase=p['phase'],segment=p['segment'],pose=[float(q[0]),float(q[1])])
                              for p,q in zip(selected,points)]
            if self.run.target:
                status['target_map']=self.transform_pose([*self.run.target['odom_position'][:2],0.])[:2]
        except Exception:
            status['home']=None;status['trails']=[]
        outward, back = self.displayed_routes()
        waypoint_kind=self.waypoint_kind()
        waypoint=self.current_target if waypoint_kind and (
            self.goal is not None or self.pending or self.nav_done==waypoint_kind) else None
        status.update(coverage=self.mapping.get('measured_coverage'),
                      contact_ready=self.contact_ready(),
                      resume_ready=self.route_resume_ready() and not self.working,
                      auto_recovery=self.run.phase=='exploration_paused' and self.auto_retry_pending,
                      sample_return_ready=self.sample_return_ready(),
                      navigation_health=self.health_details(), distinct_route_available=bool(self.failed_route),
                      outward_path=outward, return_path=back,
                      frontier=waypoint,frontier_kind=waypoint_kind if waypoint else None,
                      return_reserve_seconds=self.return_seconds,
                      real_time_factor=self.rtf(),collection_elapsed_seconds=self.run.collection_elapsed,
                      collection_paused=self.run.phase=='collecting' and not self.run.collection_fresh,
                      actual_distance_m=self.actual_distance,
                      goal_active=self.goal is not None or self.pending)
        self.latest_status=status
        message=String();message.data=json.dumps(status,allow_nan=False);self.demo_pub.publish(message)
        legacy=String();legacy.data=json.dumps(dict(mission='idle' if self.run.phase not in ACTIVE else 'demo',home=status['home'],
            result=self.run.reason,starting=self.working,selection_state='none'))
        self.pub.publish(legacy)

    def displayed_routes(self):
        phase = self.run.phase
        if phase == 'preview_ready' and self.run.preview:
            p=self.run.preview
            return p['outward'], [p['stopping_pose'],p['backup_pose'],*p['back']]
        if phase=='sample_backing' and self.backup_start is not None:
            return [],[self.backup_start,self.backup_end,*self.return_path]
        if phase=='contact_reversing' and self.retreat_route:
            return [],self.retreat_route
        if self.goal is not None or self.pending:
            if (phase,self.current_purpose) in (('exploring','frontier'),('exploration_reposition','clearance'),
                                               ('contact_rejoin','clearance'),
                                               ('sample_outbound','sample')):
                return self.active_path, []
            if phase in ('exploration_return','sample_return') and self.current_purpose == 'home':
                return [], self.active_path
        return [], []

    def flush(self):
        while self.run.events:
            self.stream.write(json.dumps(self.run.events.pop(0),allow_nan=False)+'\n')
        self.stream.flush()
        now=time.monotonic()
        if hasattr(self,'latest_status') and now-self.last_snapshot>=1.:
            dest=self.output.with_suffix('.json');temp=dest.with_suffix('.tmp')
            temp.write_text(json.dumps(self.latest_status,indent=2,allow_nan=False)+'\n')
            temp.replace(dest);self.last_snapshot=now


def main():
    rclpy.init();node=DemoMission()
    try:rclpy.spin(node)
    finally:
        if node.run.phase not in ('complete','stopped'):node.abort('Demo process shutdown')
        else:
            node.cancel()
            if node.stop_client.service_is_ready():node.stop_client.call_async(Trigger.Request())
            node.flush()
        node.worker.shutdown(wait=False,cancel_futures=True)
        node.stream.close();node.destroy_node();rclpy.shutdown()
