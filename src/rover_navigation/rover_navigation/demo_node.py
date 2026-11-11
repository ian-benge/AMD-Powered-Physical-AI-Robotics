"""Opt-in two-mission ROS adapter. Only the existing guard publishes cmd_vel."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
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
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rover_navigation_interfaces.srv import DemoCommand

from .core import Grid, footprint_clear, turn_radius
from .demo_core import ACTIVE, DemoPolicy, DemoRun, collection_poses, observation_frontiers, path_length, return_reserve, target_signature
from .goal_checks import check_path, pose_values
from .mission_node import Mission
from .ros_utils import apply_tf, stamp_seconds
from .site_geometry import pose_covariance


class DemoMission(Mission):
    def __init__(self):
        super().__init__()
        for key,value in dict(session='',demo_output='',demo_profile='full').items():
            self.declare_parameter(key,value)
        session=Path(self.get_parameter('session').value).name
        output=Path(self.get_parameter('demo_output').value)
        if not session or not str(output) or output.exists():
            raise ValueError('Demo needs a new exclusive output and session')
        profile=self.get_parameter('demo_profile').value
        speed=self.get_parameter('navigation_speed').value
        if profile not in ('full','rehearsal'):
            raise ValueError('Unknown demo profile')
        policy=DemoPolicy.rehearsal(speed) if profile=='rehearsal' else DemoPolicy(speed=speed)
        self.run=DemoRun(session,policy)
        output.parent.mkdir(parents=True,exist_ok=True)
        self.stream=output.open('x')
        self.output=output
        self.stream.write(json.dumps(dict(kind='configuration',session_id=session,policy=self.run.status(time.monotonic())['policy']))+'\n')
        self.working=False
        self.expected_stop=False
        self.nav_done=None
        self.current_target=None
        self.current_purpose=''
        self.goal_started=0.
        self.odometry_pose=None
        self.odometry_wall=-math.inf
        self.odometry_error='Waiting for estimated odometry'
        self.velocity=[0.,0.]
        self.mapping={};self.mapping_wall=-math.inf
        self.soils={};self.soils_wall=-math.inf
        self.detector={};self.detector_wall=-math.inf
        self.manual_used=False
        self.last_stop_generation=None
        self.sweep=[]
        self.visited=[]
        self.trails=[]
        self.collection_pose=None
        self.return_path=[]
        self.outward_path=[]
        self.frontier=None
        self.return_seconds=None
        self.return_requested=False
        self.actual_distance=0.
        self.rtf_samples=[]
        self.last_snapshot=-math.inf
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='demo-frontiers')
        self.frontier_future=None
        self.search_generation=None
        self.create_service(DemoCommand,'/navigation/demo_command',self.command,callback_group=ReentrantCallbackGroup())
        self.demo_pub=self.create_publisher(String,'/navigation/demo_status',1)
        durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String,'/navigation/mapping_status',self.mapping_received,durable)
        self.create_subscription(String,'/navigation/soil_sites',self.soils_received,durable)
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
            if hasattr(self,'run') and self.run.phase=='collecting' and (
                    not np.isfinite(self.velocity).all() or abs(self.velocity[0])>=.01 or abs(self.velocity[1])>=.02 or
                    self.collection_pose is None or math.dist(pose[:2],self.collection_pose[:2])>.05):
                self.abort('Movement interrupted simulated collection')
        except ValueError as error:
            self.odometry_wall=-math.inf
            self.odometry_error=str(error)

    def mapping_received(self,msg):
        self.mapping=json.loads(msg.data);self.mapping_wall=time.monotonic()

    def soils_received(self,msg):
        self.soils=json.loads(msg.data);self.soils_wall=time.monotonic()

    def detector_received(self,msg):
        self.detector=json.loads(msg.data);self.detector_wall=time.monotonic()

    def guard_received(self,msg):
        status=json.loads(msg.data)
        if status.get('sequence',0)<getattr(self,'guard_sequence',0):return
        self.guard=status;self.guard_wall=time.monotonic()
        if not hasattr(self,'run'):return
        if self.run.phase=='collecting' and status.get('command')!=[0.,0.]:
            self.abort('Nonzero command interrupted simulated collection');return
        changed=(self.last_stop_generation is not None and status.get('stop_generation')!=self.last_stop_generation)
        self.last_stop_generation=status.get('stop_generation')
        if not self.expected_stop and ((self.starting and status.get('stop_generation')!=self.start_epoch) or
                changed and self.run.started is not None and self.run.phase not in ('complete','stopped')):
            self.abort('Stop or takeover invalidated the pending demo operation');return
        if self.run.phase=='ready':
            self.run.epoch=status.get('odometry_resets',0)
            self.manual_used |= status.get('mode')=='manual'
        elif status.get('odometry_resets',0)!=self.run.epoch:
            self.abort('Odometry epoch changed; Restart required')
        elif self.run.phase in ACTIVE and (not status.get('ready') or status.get('mode')!='autonomous' and not self.starting) and not self.expected_stop:
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
        tf=self.tf.lookup_transform(target,source,self.get_clock().now())
        xy=apply_tf([[pose[0],pose[1],0.]],tf)[0]
        q=tf.transform.rotation
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return [float(xy[0]),float(xy[1]),pose[2]+angle]

    def home_map(self):return self.transform_pose(self.run.home)

    def area(self):return float((self.mapping.get('measured_coverage') or {}).get('measured_area_m2',0.))

    def actual_plan(self,msg):
        if not hasattr(self,'run') or self.run.phase not in ACTIVE or not msg.poses:return
        try:
            path=[pose_values(p) for p in msg.poses]
            check_path(self.grid,path,path[0],path[-1])
        except ValueError as error:
            self.abort('Nav2 route changed to unknown/hazardous ground: '+str(error))

    def abort(self,reason):
        if not hasattr(self,'run'):return
        self.run.stop(time.monotonic(),reason)
        self.selection_generation+=1
        self.cancel()
        self.nav_done=None
        self.mode='idle'
        if self.stop_client.service_is_ready():self.stop_client.call_async(Trigger.Request())
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

    async def arm(self,token):
        if not self.healthy() or self.guard.get('mode')!='stopped':raise ValueError('Stopped healthy guard required')
        epoch=self.guard.get('stop_generation')
        future=self.enable_client.call_async(Trigger.Request())
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

    async def command(self,request,response):
        now=time.monotonic();r=self.run
        try:
            r.verify(request.session_id,request.mission_id,request.client_token,request.epoch)
            if request.command=='heartbeat':
                r.heartbeat(now,request.sequence)
                response.success=True
            elif request.command=='return_now':
                if r.phase not in ('sweeping','exploring'):raise ValueError('Return now requires active exploration')
                self.return_requested=True
                self.cancel();self.nav_done=None
                r.event(now,'early_return_requested')
                response.success=True
            elif self.working:
                raise ValueError('Another bounded operation is running; Stop remains available')
            else:
                self.working=True
                token=self.selection_generation
                if request.command in ('start_exploration','start_sample','preview_sample'):
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
                elif request.command=='select_target':
                    if request.target_id:
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
                    self.selection_generation+=1
                elif request.command=='preview_sample':
                    if r.phase=='preview_ready':r.transition('awaiting_target',now)
                    if r.phase!='awaiting_target' or r.target is None:raise ValueError('Select a sample target first')
                    self.operation_deadline=time.monotonic()+24.
                    try:await self.preview_sample(token)
                    finally:self.operation_deadline=None
                elif request.command=='start_sample':
                    if not self.healthy():raise ValueError('Navigation health is unavailable')
                    if r.preview is None or math.dist(r.preview['back'][-1][:2],self.home_map()[:2])>.05:
                        raise ValueError('Map correction changed home; request a new preview')
                    if math.dist(r.preview['target_map'],self.transform_pose([*r.target['odom_position'][:2],0.])[:2])>.05:
                        raise ValueError('Map correction changed the target; request a new preview')
                    r.approve(now,request.preview_id,self.pose(),self.grid,self.target_current_signature())
                    try:await self.arm(token)
                    except Exception:
                        r.stop(time.monotonic(),'Sample arming failed');raise
                    self.mode='demo'
                    self.send_demo(r.preview['stopping_pose'],'sample')
                else:raise ValueError('Unknown demo command')
                response.success=True
                self.working=False
                self.starting=False
            response.message='Command accepted'
        except Exception as error:
            if request.command!='heartbeat':
                r.event(time.monotonic(),'command_rejected',command=request.command,reason=str(error))
            # Pure preflight rejections do not erase a successful first return.
            if r.phase in ACTIVE and request.command!='heartbeat':self.abort(str(error))
            self.working=False if request.command!='heartbeat' else self.working
            if request.command!='heartbeat':self.starting=False
            response.message=str(error)
        response.mission_id=r.mission_id
        response.preview_id=r.preview['id'] if r.preview else ''
        self.flush()
        return response

    def selected_site(self,ident,require_fresh):
        if (time.monotonic()-self.soils_wall>1. or not self.soils.get('ready') or
                self.soils.get('epoch')!=self.run.epoch):raise ValueError('Soil evidence unavailable')
        site=next((s for s in self.soils.get('sites',[]) if s['track_id']==ident and s['state']=='confirmed'),None)
        if site is None:raise ValueError('Soil site retired, ambiguous or unconfirmed')
        if ident in self.soils.get('ambiguous_track_ids',[]):raise ValueError('Soil association is ambiguous')
        if require_fresh and (not site.get('geometry',{}).get('supported') or site.get('last_confidence',0.)<.5 or
                not 0.<=self.get_clock().now().nanoseconds/1e9-site.get('last_supported_stamp',-math.inf)<=.6):
            raise ValueError('Selected soil site was not freshly reacquired with supported RGB-D evidence')
        return site

    def target_current_signature(self):
        target=dict(self.run.target)
        if target['source']=='detected_soil':
            site=self.selected_site(target['track_id'],False)
            if math.dist(site['odom_position'],target['odom_position'])>.05:
                raise ValueError('Target position changed; select and preview again')
            if site['envelope_radius_m']>target['envelope_radius_m']+.025:
                raise ValueError('Target envelope grew; select and preview again')
        return target_signature(target)

    async def preview_sample(self,token):
        target=self.run.target
        point=self.transform_pose([*target['odom_position'][:2],0.])
        start=self.pose();home=self.home_map()
        candidates=collection_poses(self.grid,point[:2],target.get('envelope_radius_m',0.),target.get('covariance'))
        candidates.sort(key=lambda p:math.dist(p[:2],start[:2]))
        deadline=time.monotonic()+24.
        errors=[]
        for pose in candidates[:3]:
            if time.monotonic()>=deadline:break
            try:
                outward=await self.planned_path(start,pose,token)
                back=await self.planned_path(pose,home,token)
                a,b=[pose_values(p) for p in outward.poses],[pose_values(p) for p in back.poses]
                self.run.set_preview(time.monotonic(),a,b,pose,self.pose(),self.grid,self.target_current_signature(),self.rtf())
                self.run.preview['target_map']=point[:2]
                self.outward_path,self.return_path=a,b
                return
            except ValueError as error:errors.append(str(error))
        raise ValueError('No safe budgeted sample round trip: '+'; '.join(errors[-3:]))

    def send_demo(self,target,purpose):
        if purpose in ('frontier','sweep') and self.return_requested:return
        if self.goal is not None or self.pending:raise ValueError('A navigation goal is already active')
        self.current_target,self.current_purpose=list(target),purpose
        self.goal_started=time.monotonic()
        self.pending=True
        self.nav_done=None
        token=self.selection_generation
        generation=self.generation
        goal=NavigateToPose.Goal();goal.pose=self.selected_pose(target)
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
                self.goal=None
                answer=f.result()
                if answer.status!=GoalStatus.STATUS_SUCCEEDED:
                    self.abort('Navigation failed: '+self.nav2_failure(answer));return
                self.nav_done=purpose
            handle.get_result_async().add_done_callback(completed)
        self.nav.send_goal_async(goal).add_done_callback(accepted)

    async def path(self,start,target):
        result=await self.planned_path(start,target,self.selection_generation)
        values=[pose_values(p) for p in result.poses]
        check_path(self.grid,values,start,target)
        return values

    async def advance(self):
        r=self.run
        try:
            now=time.monotonic()
            if self.return_requested and r.phase in ('sweeping','exploring'):
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
                    r.visited(now,self.transform_pose(self.current_target,'odom'),self.area())
                    self.visited.append(self.transform_pose(self.current_target,'odom')[:2]);self.nav_done=None
                start=self.pose();home=self.home_map()
                if now-self.last_route_check>=1.:
                    self.return_path=await self.path(start,home)
                    self.last_route_check=time.monotonic()
                    self.return_seconds=return_reserve(self.return_path,r.policy.speed,self.rtf())
                    reason=r.exploration_reason(time.monotonic(),self.return_path,self.rtf(),[True])
                    if reason:
                        self.cancel();self.nav_done=None;r.transition('exploration_return',now,reason);return
                if self.goal is not None or self.pending:return
                if self.frontier_future is None:
                    attempted=[self.transform_pose([*v,0.])[:2] for v in self.visited]
                    grid=Grid(self.grid.data.copy(),self.grid.resolution,self.grid.x,self.grid.y)
                    self.frontier_future=self.worker.submit(observation_frontiers,grid,start,attempted)
                    return
                if not self.frontier_future.done():return
                goals=self.frontier_future.result();self.frontier_future=None
                reason=r.exploration_reason(now,self.return_path,self.rtf(),goals)
                if reason:r.transition('exploration_return',now,reason);return
                for candidate in goals:
                    pose=candidate['pose']
                    try:
                        outward=await self.path(self.pose(),pose)
                        back=await self.path(pose,self.home_map())
                        if time.monotonic()-r.started+return_reserve(outward,r.policy.speed,self.rtf())+return_reserve(back,r.policy.speed,self.rtf())>=r.policy.exploration_seconds:
                            continue
                        self.outward_path,self.return_path=outward,back
                        self.frontier=pose
                        self.send_demo(pose,'frontier');return
                    except ValueError as error:
                        self.visited.append(self.transform_pose(pose,'odom')[:2])
                        r.event(time.monotonic(),'planning_candidate_rejected',reason=str(error),pose=pose)
                r.transition('exploration_return',time.monotonic(),'No feasible budgeted frontier')
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
                self.return_path=await self.path(self.pose(),self.home_map())
                self.send_demo(self.home_map(),'home')
            elif r.phase in ('sample_outbound','collecting'):
                if r.phase!='collecting' and now-self.last_route_check>=1.:
                    self.target_current_signature()
                    if math.dist(r.preview['target_map'],self.transform_pose([*r.target['odom_position'][:2],0.])[:2])>.05:
                        raise ValueError('Target map correction changed during sampling')
                    self.return_path=await self.path(self.pose(),self.home_map())
                    self.return_seconds=return_reserve(self.return_path,r.policy.speed,self.rtf())
                    if now-r.sample_started+self.return_seconds+10.>=r.policy.sampling_seconds:
                        raise ValueError('Sampling return reserve exhausted; mission stopped')
                    self.last_route_check=time.monotonic()
                if self.goal is not None or self.pending:return
                if r.phase=='sample_outbound' and self.nav_done!='sample':return
                if self.collection_pose is None:self.collection_pose=list(self.odometry_pose)
                stationary=(self.guard.get('command')==[0.,0.] and np.isfinite(self.velocity).all() and
                            abs(self.velocity[0])<.01 and abs(self.velocity[1])<.02 and
                            math.dist(self.collection_pose[:2],self.odometry_pose[:2])<=.05)
                fresh=True
                if r.target['source']=='detected_soil':
                    try:self.selected_site(r.target['track_id'],True)
                    except ValueError:fresh=False
                if not stationary and r.phase=='sample_outbound':return
                self.nav_done=None
                r.collect(time.monotonic(),stationary,fresh,self.healthy())
                if r.phase=='stopped':self.abort(r.reason)
                elif r.phase=='sample_return':self.home_attempts=0
        except Exception as error:
            self.abort(str(error))
        finally:self.working=False

    def rtf(self):
        if len(self.rtf_samples)<2:return 1.
        a,b=self.rtf_samples[0],self.rtf_samples[-1]
        return max(.01,(b[1]-a[1])/max(.001,b[0]-a[0]))

    def tick(self):
        if not hasattr(self,'run'):return
        now=time.monotonic();r=self.run
        r.deadlines(now,self.healthy())
        if r.phase=='stopped' and self.mode!='idle':self.abort(r.reason)
        sim=self.get_clock().now().nanoseconds/1e9
        self.rtf_samples.append((now,sim));self.rtf_samples=self.rtf_samples[-150:]
        if self.odometry_pose is not None and r.phase in ACTIVE:
            phase='exploring' if r.phase=='sweeping' else r.phase
            if not self.trails or math.dist(self.trails[-1]['odom_pose'][:2],self.odometry_pose[:2])>.02 or now-self.trails[-1]['wall']>=1.:
                try:estimated=self.transform_pose(self.pose(),'odom')
                except Exception as error:self.abort('Trail localization unavailable: '+str(error));return
                if self.trails:self.actual_distance+=math.dist(estimated[:2],self.trails[-1]['odom_pose'][:2])
                self.trails.append(dict(phase=phase,odom_pose=estimated,wall=now,sim_time=sim))
                r.event(now,'trajectory',**self.trails[-1])
        self.publish_status()
        if r.phase in ACTIVE and not self.working:
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
            status['trails']=[dict(phase=p['phase'],pose=[float(q[0]),float(q[1])]) for p,q in zip(selected,points)]
            if self.run.target:
                status['target_map']=self.transform_pose([*self.run.target['odom_position'][:2],0.])[:2]
        except Exception:
            status['home']=None;status['trails']=[]
        status.update(coverage=self.mapping.get('measured_coverage'),
                      navigation_health=self.health_details(),
                      outward_path=self.outward_path if self.run.phase in ('exploring','preview_ready','sample_outbound') else [],
                      return_path=self.return_path if self.run.phase not in ('complete','stopped') else [],
                      frontier=self.frontier if self.run.phase=='exploring' else None,return_reserve_seconds=self.return_seconds,
                      real_time_factor=self.rtf(),collection_elapsed_seconds=(0. if self.run.collection_started is None else max(0.,now-self.run.collection_started)),
                      actual_distance_m=self.actual_distance,
                      goal_active=self.goal is not None or self.pending)
        self.latest_status=status
        message=String();message.data=json.dumps(status,allow_nan=False);self.demo_pub.publish(message)
        legacy=String();legacy.data=json.dumps(dict(mission='idle' if self.run.phase not in ACTIVE else 'demo',home=status['home'],
            result=self.run.reason,starting=self.working,selection_state='none'))
        self.pub.publish(legacy)

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
