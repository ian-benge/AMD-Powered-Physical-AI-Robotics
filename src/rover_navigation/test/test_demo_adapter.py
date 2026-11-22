"""Cancellation and arrival races in the real ROS adapter, with transport doubles."""
import asyncio
import importlib
import json
import math
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock,MagicMock,patch
import numpy as np

from rover_navigation.demo_core import DemoRun,DemoPolicy,SampleGeometryChanged,distance_to_route,target_signature
from rover_navigation.core import Grid


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names=('rclpy','rclpy.node','rclpy.action','rclpy.callback_groups','rclpy.clock','rclpy.qos','rclpy.time',
               'action_msgs','action_msgs.msg','geometry_msgs','geometry_msgs.msg','nav_msgs','nav_msgs.msg',
               'nav2_msgs','nav2_msgs.action','std_msgs','std_msgs.msg','std_srvs','std_srvs.srv','tf2_ros',
               'visualization_msgs','visualization_msgs.msg','rover_navigation_interfaces','rover_navigation_interfaces.srv')
        modules={n:MagicMock() for n in names};modules['rclpy.node'].Node=object
        modules['tf2_ros'].TransformException=RuntimeError
        with patch.dict(sys.modules,modules):cls.module=importlib.import_module('rover_navigation.demo_node')
    def node(self):
        n=self.module.DemoMission.__new__(self.module.DemoMission)
        n.run=DemoRun('one',DemoPolicy.rehearsal());n.guard_sequence=0;n.last_stop_generation=7
        n.guard=dict(ready=True,mode='stopped',stop_generation=7,sequence=1,command=[0.,0.])
        n.selection_generation=2;n.expected_stop=False;n.starting=False;n.start_epoch=None
        n.mode='idle';n.manual_used=False;n.working=False;n.nav_done=None;n.nav_arrived_at=None
        n.waypoint_stop_deadline=None
        n.last_route_check=float('inf')
        n.return_requested=False
        n.pause_stop_epoch=None;n.resume_return=False;n.frontier=None;n.frontier_future=None
        n.contact_point=None;n.retreat_route=None
        n.frontier_bt='';n.clearance_bt='';n.sample_view_angles=[];n.failed_route=[];n.alternate_route=[]
        n.frontier_arrival_blocked=False
        n.frontier_trail=();n.trails=[]
        n.auto_retry_pending=False;n.auto_retry_count=0;n.auto_retry_until=0.
        n.active_path=[];n.current_target=None;n.current_purpose=''
        n.stop_client=MagicMock();n.cancel=MagicMock();n.flush=MagicMock()
        return n
    def message(self,**fields):
        value=dict(ready=True,mode='stopped',stop_generation=7,sequence=2,command=[0.,0.],odometry_resets=0)
        value.update(fields);return NS(data=json.dumps(value))
    def test_stop_during_initial_arm_invalidates_start(self):
        n=self.node();n.starting=True;n.start_epoch=7
        n.guard_received(self.message(stop_generation=8))
        self.assertEqual(n.run.phase,'stopped');self.assertEqual(n.selection_generation,3)
        n.stop_client.call_async.assert_called_once()
    def test_stop_after_preview_cancels_future_approval(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.)
        n.run.phase='preview_ready';n.run.preview={'id':'old'}
        n.guard_received(self.message(stop_generation=8))
        self.assertEqual(n.run.phase,'stopped');self.assertIsNone(n.run.preview)
    def test_expected_first_home_stop_does_not_cancel_waiting_transition(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.)
        n.run.phase='exploration_return';n.expected_stop=True
        n.guard_received(self.message(stop_generation=8))
        self.assertEqual(n.run.phase,'exploration_return')
        n.run.home_arrived(10.);self.assertEqual(n.run.phase,'awaiting_target')
    def test_old_status_before_arm_receipt_is_ignored(self):
        n=self.node();n.guard_sequence=10
        n.guard_received(self.message(sequence=9,stop_generation=8))
        self.assertEqual(n.run.phase,'ready');self.assertEqual(n.last_stop_generation,7)
    def test_transient_stopped_status_during_arming_cannot_cancel_or_authorize_motion(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.);n.run.phase='sample_outbound'
        n.starting=True;n.start_epoch=7
        n.guard_received(self.message())
        self.assertEqual(n.run.phase,'sample_outbound');n.stop_client.call_async.assert_not_called()
    def test_takeover_and_sensor_loss_abort_active_exploration(self):
        for fields in (dict(mode='manual',stop_generation=8),dict(mode='autonomous',ready=False)):
            n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.);n.run.phase='exploring'
            n.guard_received(self.message(**fields));self.assertEqual(n.run.phase,'stopped')
    def test_epoch_reset_aborts_even_while_waiting_at_home(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.);n.run.phase='awaiting_target'
        n.guard_received(self.message(odometry_resets=1));self.assertEqual(n.run.phase,'stopped')
    def test_nonzero_command_during_collection_aborts_without_sample(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.);n.run.phase='collecting'
        n.guard_received(self.message(mode='autonomous',command=[.01,0.]))
        self.assertEqual(n.run.phase,'stopped');self.assertIsNone(n.run.sample)
    def test_late_navigation_acceptance_is_cancelled(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.)
        n.goal=None;n.pending=False;n.generation=1;n.nav=MagicMock();n.selected_pose=MagicMock()
        n.send_demo([1,0,0],'frontier')
        callback=n.nav.send_goal_async.return_value.add_done_callback.call_args.args[0]
        n.run.stop(1.,'Stop');n.selection_generation+=1
        handle=MagicMock(accepted=True);callback(NS(result=lambda:handle))
        handle.cancel_goal_async.assert_called_once()
    def test_return_now_is_accepted_during_pending_planning(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.);n.run.phase='exploring';n.working=True
        request=NS(command='return_now',session_id='one',mission_id=n.run.mission_id,client_token=n.run.owner,epoch=0)
        response=asyncio.run(n.command(request,NS(success=False)))
        self.assertTrue(response.success);self.assertTrue(n.return_requested)
        self.assertTrue(n.working);self.assertNotEqual(n.run.phase,'stopped');n.cancel.assert_called_once()
    def exploration_start(self,n):
        n.healthy=lambda:True;n.mapping=dict(clearance_confirmed=True,publication_latency_p95_seconds=.1)
        n.pose=MagicMock(return_value=[0.,0.,0.]);n.transform_pose=MagicMock(return_value=[0.,0.,0.])
        n.arm=AsyncMock();n.area=lambda:3.;n.grid=MagicMock()
        return NS(command='start_exploration',session_id='one',mission_id=n.run.mission_id,
                  client_token='operator-token-one',epoch=0)
    def test_home_transform_failure_cannot_arm_the_rover(self):
        n=self.node();request=self.exploration_start(n)
        n.transform_pose.side_effect=ValueError('No fresh map correction')
        with patch.object(self.module,'check_path'),patch.object(self.module,'footprint_clear',return_value=True):
            response=asyncio.run(n.command(request,NS(success=False)))
        self.assertFalse(response.success);n.arm.assert_not_awaited();self.assertEqual(n.run.phase,'ready')
    def test_failure_after_arm_stops_and_invalidates_initialization(self):
        n=self.node();request=self.exploration_start(n)
        n.run.start=MagicMock(side_effect=ValueError('Initialization interrupted'))
        with patch.object(self.module,'check_path'),patch.object(self.module,'footprint_clear',return_value=True):
            response=asyncio.run(n.command(request,NS(success=False)))
        self.assertFalse(response.success);n.arm.assert_awaited_once();self.assertEqual(n.run.phase,'stopped')
        n.stop_client.call_async.assert_called_once();n.cancel.assert_called_once()
    def test_early_return_fences_late_frontier_dispatch(self):
        n=self.node();n.return_requested=True;n.nav=MagicMock()
        n.send_demo([1,0,0],'frontier');n.nav.send_goal_async.assert_not_called()

    def test_map_only_shows_the_current_goal_route_during_return(self):
        n = self.frontier_node()
        frontier = [[0.,0.,0.], [1.,0.,0.]]
        n.active_path = frontier
        n.current_purpose = 'frontier'
        n.goal = object()
        self.assertEqual(n.displayed_routes(), (frontier, []))
        n.run.transition('exploration_return', 100.)
        self.assertEqual(n.displayed_routes(), ([], []))
        n.current_purpose = 'home'
        n.current_target = [0.,0.,0.]
        n.pose = lambda: [1.,0.,0.]
        n.active_path = []
        home = [[1.,0.,0.], [0.,0.,0.]]
        speculative = [[2.,0.,0.], [0.,0.,0.]]
        with patch.object(self.module, 'pose_values', side_effect=lambda pose:pose):
            n.actual_plan(NS(poses=speculative))
            self.assertEqual(n.displayed_routes(), ([], []))
            n.actual_plan(NS(poses=home))
            self.assertEqual(n.displayed_routes(), ([], home))
    def frontier_node(self):
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.)
        n.run.phase='exploring';n.goal=None;n.pending=False;n.visited=[]
        n.pose=lambda:[0.,0.,0.];n.home_map=lambda:[0.,0.,0.]
        n.transform_pose=lambda pose,*args:list(pose)
        n.return_path=[[0.,0.,0.]];n.rtf=lambda:1.
        n.grid=Grid(np.zeros((160,160),np.int8),.05,-4.,-4.)
        return n
    def test_worker_uses_qualified_mission_speed_and_a_map_snapshot(self):
        n=self.frontier_node();n.frontier_future=None;n.worker=MagicMock()
        asyncio.run(n.advance())
        args=n.worker.submit.call_args
        self.assertEqual(args.kwargs,dict(max_goals=24,speed=.05,angular_speed=.3,home=[0.,0.,0.],trail=()))
        self.assertIsNot(args.args[1].data,n.grid.data)

    def test_survey_search_submits_all_generated_frontier_candidates(self):
        n=self.frontier_node();n.run.policy=DemoPolicy.survey();n.worker=MagicMock()
        asyncio.run(n.advance())
        args=n.worker.submit.call_args
        self.assertEqual(args.kwargs['max_goals'],80)
        self.assertEqual(args.kwargs['speed'],.15)

    def test_next_frontier_waits_for_fresh_stop_after_nav2_reaches_arrival_region(self):
        n=self.frontier_node();n.worker=MagicMock()
        n.nav_done='frontier';n.nav_arrived_at=100.;n.current_target=[0.,0.,0.]
        n.current_purpose='frontier'
        n.guard['mode']='autonomous';n.run.heartbeat_wall=100.;n.healthy=lambda:True
        n.path=AsyncMock(return_value=[[0.,0.,0.]])
        n.area=lambda:6.
        n.guard_wall=99.9;n.odometry_wall=100.1;n.velocity=[0.,0.]
        with patch.object(self.module.time,'monotonic',return_value=100.2):
            asyncio.run(n.advance())
            n.guard_wall=100.15;n.velocity=[0.,.04]
            asyncio.run(n.advance())
            self.assertEqual(n.nav_done,'frontier')
            self.assertEqual(n.run.frontier_visits,[])
            n.worker.submit.assert_not_called()
            n.velocity=[0.,0.]
            asyncio.run(n.advance())
        self.assertIsNone(n.nav_done)
        self.assertEqual(len(n.run.frontier_visits),1)
        n.worker.submit.assert_called_once()

    def test_frontier_and_recovery_stop_once_inside_the_wider_region(self):
        for phase,purpose,offset in (('exploring','frontier',.235),('exploration_reposition','clearance',.19)):
            n=self.frontier_node()
            n.run.phase=phase;n.current_purpose=purpose;n.current_target=[offset,0.,0.]
            n.guard['mode']='autonomous';n.run.heartbeat_wall=100.;n.healthy=lambda:True
            previous_goal=n.goal=MagicMock()
            with patch.object(self.module.time,'monotonic',return_value=100.2),patch.object(
                    self.module,'GoalStatus',NS(STATUS_SUCCEEDED=4,STATUS_CANCELED=5,STATUS_ABORTED=6)):
                n.check_waypoint_arrival(100.2)
                n.check_waypoint_arrival(100.3)
                previous_goal.cancel_goal_async.assert_called_once()
                asyncio.run(n.advance())
                self.assertIs(n.goal,previous_goal)
                self.assertIsNone(n.nav_done)
                n.finish_navigation(NS(status=5),purpose)
            self.assertEqual(n.nav_done,purpose)
            self.assertIsNone(n.goal)

    def test_progress_abort_counts_as_arrival_only_at_a_clear_ready_waypoint(self):
        for distance,clear,ready,code,expected in ((.235,True,True,105,True),(.41,True,True,105,False),
                (.235,False,True,105,False),(.235,True,False,105,False),(.235,True,True,106,False)):
            n=self.frontier_node()
            n.current_purpose='frontier';n.current_target=[distance,0.,0.]
            n.guard['mode']='autonomous';n.run.heartbeat_wall=100.;n.healthy=lambda:ready
            n.route_blocked=MagicMock()
            if not clear:n.grid.data[n.grid.cell(0.,0.)]=100
            answer=NS(status=6,result=NS(error_code=code,error_msg='progress'))
            with patch.object(self.module.time,'monotonic',return_value=100.2),patch.object(
                    self.module,'GoalStatus',NS(STATUS_SUCCEEDED=4,STATUS_CANCELED=5,STATUS_ABORTED=6)):
                n.finish_navigation(answer,'frontier')
            self.assertEqual(n.nav_done,'frontier' if expected else None)
            self.assertEqual(n.route_blocked.call_count,0 if expected else 1)

    def test_unacknowledged_waypoint_stop_never_dispatches_the_next_goal(self):
        n=self.frontier_node();n.goal=MagicMock();n.waypoint_stop_deadline=102.
        n.abort=MagicMock()
        n.check_waypoint_arrival(102.1)
        n.abort.assert_called_once_with('Nav2 did not acknowledge the waypoint stop')

    def test_return_plan_cannot_interrupt_waypoint_handoff(self):
        n=self.frontier_node();n.worker=MagicMock();n.send_demo=MagicMock()
        n.current_purpose='frontier';n.current_target=[.235,0.,0.];n.goal=MagicMock()
        n.guard['mode']='autonomous';n.run.heartbeat_wall=100.;n.healthy=lambda:True
        n.last_route_check=-math.inf
        async def path_fetched_during_arrival(*_):
            n.goal=None;n.nav_done='frontier';n.nav_arrived_at=100.2
            return [[0.,0.,0.],[20.,0.,0.]]
        n.path=path_fetched_during_arrival
        with patch.object(self.module.time,'monotonic',return_value=100.2):
            asyncio.run(n.advance())
        self.assertEqual(n.run.phase,'exploring')
        self.assertEqual(n.nav_done,'frontier')
        n.worker.submit.assert_not_called()
        n.request_exploration_return()
        n.cancel.assert_not_called()
        n.guard_wall=100.3;n.odometry_wall=100.3;n.velocity=[0.,.04]
        with patch.object(self.module.time,'monotonic',return_value=100.4):
            asyncio.run(n.advance())
            n.send_demo.assert_not_called()
            n.velocity=[0.,0.]
            n.path=AsyncMock(return_value=[[0.,0.,0.]])
            asyncio.run(n.advance())
        self.assertEqual(n.run.phase,'exploration_return')
        n.send_demo.assert_called_once_with([0.,0.,0.],'home')
    def candidate(self,pose):
        return dict(pose=pose,expected_gain_m2=8.,trail_novelty=1.,score=.1)

    def test_budget_rejection_records_why_and_returns_without_dispatch(self):
        n=self.frontier_node();pose=[3.,0.,0.]
        n.frontier_future=NS(done=lambda:True,result=lambda:[self.candidate(pose)])
        n.path=AsyncMock(side_effect=[[[0,0,0],[10,0,0]],[[10,0,0],[0,0,0]]])
        n.send_demo=MagicMock()
        with patch.object(self.module.time,'monotonic',return_value=100.):asyncio.run(n.advance())
        self.assertEqual(n.run.phase,'exploration_return');n.send_demo.assert_not_called()
        rejected=next(e for e in n.run.events if e['kind']=='planning_candidate_rejected')
        self.assertEqual(rejected['remaining_seconds'],260.)
        self.assertGreater(rejected['outward_reserve_seconds']+rejected['return_reserve_seconds'],260.)
    def test_rejected_candidate_does_not_prevent_a_safe_budgeted_alternative(self):
        n=self.frontier_node();poses=[[1.,0.,0.],[.8,0.,0.]]
        n.frontier_future=NS(done=lambda:True,result=lambda:[self.candidate(p) for p in poses])
        n.path=AsyncMock(side_effect=[ValueError('Unknown footprint'),[[0,0,0],poses[1]],[poses[1],[0,0,0]]])
        n.send_demo=MagicMock()
        with patch.object(self.module.time,'monotonic',return_value=100.):asyncio.run(n.advance())
        n.send_demo.assert_called_once_with(poses[1],'frontier')
        self.assertEqual(n.run.phase,'exploring')
        self.assertTrue(any(e['kind']=='frontier_selected' for e in n.run.events))
    def test_frontier_admits_checked_trip_with_one_shared_contingency(self):
        n=self.frontier_node();pose=[3.,0.,0.]
        n.frontier_future=NS(done=lambda:True,result=lambda:[self.candidate(pose)])
        n.path=AsyncMock(side_effect=[[[0.,0.,0.],pose],[pose,[0.,0.,0.]]])
        n.send_demo=MagicMock()
        with patch.object(self.module.time,'monotonic',return_value=148.):
            asyncio.run(n.advance())
        n.send_demo.assert_called_once_with(pose,'frontier')
        event=next(e for e in n.run.events if e['kind']=='frontier_selected')
        self.assertEqual(event['outward_reserve_seconds'],90.)
        self.assertEqual(event['return_reserve_seconds'],120.)

    def test_checks_a_second_outbound_path_when_first_retraces_the_trail(self):
        n=self.frontier_node()
        start=[0.,0.,0.]
        repeated=[2.,0.,0.]
        fresh=[-2.,0.,0.]
        n.frontier_trail=np.column_stack((np.linspace(0,2,11),np.zeros(11)))
        n.path=AsyncMock(side_effect=[[start,repeated],[repeated,start],[start,fresh],[fresh,start]])
        n.send_demo=MagicMock()
        with patch.object(self.module.time,'monotonic',return_value=100.):
            asyncio.run(n.choose_frontier([self.candidate(repeated),self.candidate(fresh)]))
        n.send_demo.assert_called_once_with(fresh,'frontier')
        self.assertEqual(n.path.await_count,4)
        event=next(e for e in n.run.events if e['kind']=='frontier_selected')
        self.assertLess(event['outward_trail_overlap'],.25)

    def test_prefers_outward_frontier_to_parallel_pass_near_the_previous_leg(self):
        n=self.frontier_node()
        n.grid=Grid(np.zeros((800,800),np.int8),.05,-20.,-20.)
        n.run.policy=DemoPolicy(speed=.15)
        start=[-7.89,12.40,2.64]
        n.pose=lambda:start
        n.frontier_trail=np.column_stack((np.full(41,-7.),np.linspace(4.,12.,41)))
        parallel=[-8.97,9.58,-2.68]
        outward=[-9.10,15.2,2.0]
        n.path=AsyncMock(side_effect=lambda a,b:[a,b])
        n.send_demo=MagicMock()
        with patch.object(self.module.time,'monotonic',return_value=300.):
            asyncio.run(n.choose_frontier([self.candidate(parallel),self.candidate(outward)]))
        self.assertEqual(n.path.await_count,4)
        n.send_demo.assert_called_once_with(outward,'frontier')

    def test_long_retrace_is_skipped_before_its_return_plan(self):
        start=[2.03,-17.31,0.]
        retrace=[.83,3.03,1.79]
        nearby=[4.53,-17.31,0.]
        for poses in ([retrace], [retrace,nearby]):
            with self.subTest(has_nearby=len(poses)>1):
                n=self.frontier_node()
                n.grid=Grid(np.zeros((800,800),np.int8),.05,-20.,-20.)
                n.run.policy=DemoPolicy(speed=.15)
                n.pose=lambda:start
                n.frontier_trail=np.linspace(start[:2],retrace[:2],101)
                n.path=AsyncMock(side_effect=lambda a,b:[a,b])
                n.send_demo=MagicMock()
                with patch.object(self.module.time,'monotonic',return_value=254.):
                    asyncio.run(n.choose_frontier([self.candidate(p) for p in poses]))
                rejected=next(e for e in n.run.events if e['kind']=='planning_candidate_rejected')
                self.assertGreater(rejected['estimated_retrace_seconds'],120.)
                self.assertNotIn((retrace,n.home_map()),[call.args for call in n.path.await_args_list])
                if len(poses)>1:
                    n.send_demo.assert_called_once_with(nearby,'frontier')
                else:
                    self.assertEqual(n.run.phase,'exploration_return')
                    n.send_demo.assert_not_called()

    def test_survey_can_retrace_to_unmapped_ground_and_prioritizes_a_local_route(self):
        n=self.frontier_node();n.run.policy=DemoPolicy.survey()
        n.grid=Grid(np.zeros((800,800),np.int8),.05,-20.,-20.)
        start=[13.5,2.2,1.6]
        n.pose=lambda:start
        n.frontier_trail=np.linspace([0.,.8],start[:2],61)
        retraces=[[3.125,1.475,1.30],[3.175,.975,1.57],[3.225,1.675,1.30]]
        nearby=[14.5,3.4,1.2]
        n.path=AsyncMock(side_effect=lambda a,b:[a,b])
        n.send_demo=MagicMock()
        with patch.object(self.module.time,'monotonic',return_value=131.9):
            asyncio.run(n.choose_frontier([self.candidate(p) for p in retraces]))
            self.assertIn(n.send_demo.call_args.args[0],retraces)
            n.send_demo.reset_mock()
            asyncio.run(n.choose_frontier([self.candidate(p) for p in [*retraces,nearby]]))
            n.send_demo.assert_called_once_with(nearby,'frontier')
        self.assertEqual(n.run.phase,'exploring')

    def test_survey_mapping_timer_cancels_the_active_frontier_for_a_checked_return(self):
        n=self.frontier_node();n.run.policy=DemoPolicy.survey()
        n.last_route_check=1498.
        n.goal=object()
        n.path=AsyncMock(return_value=[[0.,0.,0.]])
        with patch.object(self.module.time,'monotonic',return_value=1500.):
            asyncio.run(n.advance())
        self.assertEqual(n.run.phase,'exploration_return')
        self.assertEqual(n.run.reason,'Survey mapping time reached')
        n.cancel.assert_called_once()

    def test_blocked_return_pauses_and_requires_explicit_healthy_resume(self):
        n=self.frontier_node()
        n.guard['mode']='autonomous'
        n.frontier=[1.,0.,0.]
        n.healthy=lambda:True
        n.last_route_check=0.
        n.path=AsyncMock(side_effect=ValueError('Path footprint crosses hazardous ground'))
        n.velocity=[0.,0.]
        n.arm=AsyncMock()
        request=NS(command='resume_exploration',session_id='one',mission_id=n.run.mission_id,
                   client_token=n.run.owner,epoch=0)
        with patch.object(self.module.time,'monotonic',return_value=100.):
            asyncio.run(n.advance())
            self.assertEqual(n.run.phase,'exploration_paused')
            self.assertEqual(n.visited,[[1.,0.]])
            n.stop_client.call_async.assert_called_once()
            n.arm.assert_not_awaited()
            n.guard_received(self.message(stop_generation=8))
            self.assertEqual(n.run.phase,'exploration_paused')
            self.assertIsNone(n.pause_stop_epoch)
            result=asyncio.run(n.command(request,NS(success=False)))
            self.assertFalse(result.success)
            n.arm.assert_not_awaited()
            n.path=AsyncMock(return_value=[[0.,0.,0.]])
            n.guard['fault']={'reason':'chassis_contact'}
            self.assertFalse(asyncio.run(n.command(request,NS(success=False))).success)
            n.arm.assert_not_awaited()
            n.guard['fault']=None
            self.assertTrue(asyncio.run(n.command(request,NS(success=False))).success)
        self.assertEqual(n.run.phase,'exploring')
        self.assertEqual(n.run.started,0.)
        n.arm.assert_awaited_once()

    def test_resume_uses_checked_forward_connector_when_direct_return_is_blocked(self):
        n=self.frontier_node()
        n.healthy=lambda:True
        n.velocity=[0.,0.]
        n.run.phase='exploration_paused'
        origin=[0.,0.,0.]
        connector=[.5,0.,0.]
        n.path=AsyncMock(side_effect=[ValueError('Direct turn blocked'),[origin,connector],[connector,origin]])
        n.arm=AsyncMock()
        n.send_demo=MagicMock()
        request=NS(command='resume_exploration',session_id='one',mission_id=n.run.mission_id,
                   client_token=n.run.owner,epoch=0)
        with patch.object(self.module.time,'monotonic',return_value=100.):
            result=asyncio.run(n.command(request,NS(success=False)))
        self.assertTrue(result.success,result.message)
        self.assertEqual(n.run.phase,'exploration_reposition')
        n.send_demo.assert_called_once_with(connector,'clearance')
        self.assertEqual(n.path.await_count,3)

    def test_explicit_alternate_starts_in_a_different_corridor(self):
        n=self.frontier_node()
        n.healthy=lambda:True
        n.velocity=[0.,0.]
        n.run.phase='exploration_paused'
        n.failed_route=[[0.,0.,0.],[3.,0.,0.]]
        n.path=AsyncMock(side_effect=lambda start,end: [start,end])
        n.arm=AsyncMock()
        n.send_demo=MagicMock()
        request=NS(command='alternate_exploration',session_id='one',mission_id=n.run.mission_id,
                   client_token=n.run.owner,epoch=0)
        with patch.object(self.module.time,'monotonic',return_value=100.):
            result=asyncio.run(n.command(request,NS(success=False)))
        self.assertTrue(result.success,result.message)
        self.assertEqual(n.run.phase,'exploration_reposition')
        point,purpose=n.send_demo.call_args.args
        self.assertEqual(purpose,'clearance')
        self.assertGreaterEqual(distance_to_route(point,n.failed_route),.65)
        self.assertEqual(n.alternate_route,n.failed_route)

    def test_alternate_skips_a_loop_to_a_nearby_reposition_waypoint(self):
        n=self.frontier_node()
        start=[0.,0.,0.]
        home=[-2.,0.,0.]
        n.failed_route=[start,[3.,0.,0.]]
        n.operation_deadline=103.5
        offered=[]

        async def plan(a,b):
            if a!=start:
                return [a,b]
            offered.append(b)
            return [a,[2.2,0.,0.],b] if len(offered)==1 else [a,b]

        n.path=AsyncMock(side_effect=plan)
        with patch.object(self.module.time,'monotonic',return_value=100.):
            point,route=asyncio.run(n.alternative_route(start,home,n.selection_generation))
        self.assertEqual(len(offered),2)
        self.assertEqual(point,offered[1])
        self.assertEqual(route[-1],home)

    def test_automatic_replan_waits_for_stop_and_keeps_existing_owner_lease(self):
        n=self.frontier_node()
        n.healthy=lambda:True
        n.guard['mode']='autonomous'
        n.velocity=[0.,0.]
        n.active_path=[[0.,0.,0.],[3.,0.,0.]]
        n.executor=MagicMock()
        n.arm=AsyncMock()
        n.send_demo=MagicMock()
        connector=[0.,1.,math.pi/2]
        n.alternative_route=AsyncMock(return_value=(connector,[[0.,0.,0.],connector,[0.,0.,0.]]))
        n.run.heartbeat_wall=99.8
        with patch.object(self.module.time,'monotonic',return_value=100.):
            n.route_blocked('Navigation failed: Nav2 aborted, error 105')
            n.schedule_automatic_replan(100.)
            n.executor.create_task.assert_not_called()
            n.guard_received(self.message(stop_generation=8))
            n.schedule_automatic_replan(100.)
            task,now,token=n.executor.create_task.call_args.args
            asyncio.run(task(now,token))
        self.assertEqual(n.run.heartbeat_wall,99.8)
        self.assertEqual(n.run.phase,'exploration_reposition')
        self.assertEqual(n.auto_retry_count,1)
        n.send_demo.assert_called_once_with(connector,'clearance')
        self.assertTrue(any(e.get('automatic') for e in n.run.events))

    def test_lost_frontier_clearance_replans_from_the_stopped_pose(self):
        n=self.frontier_node()
        n.healthy=lambda:True
        n.guard['mode']='autonomous'
        n.velocity=[0.,0.]
        blocked=[1.75,0.,0.]
        n.frontier=blocked
        n.grid.data[n.grid.cell(1.75,.6)]=100
        n.active_path=[n.pose(),blocked]
        n.run.heartbeat_wall=99.8
        n.arm=AsyncMock()
        n.send_demo=MagicMock()
        n.alternative_route=AsyncMock()
        n.path=AsyncMock(side_effect=lambda a,b,**kwargs:[a,b])
        with patch.object(self.module.time,'monotonic',return_value=100.):
            asyncio.run(n.advance())
            self.assertEqual(n.run.phase,'exploration_paused')
            self.assertEqual(n.visited,[blocked[:2]])
            self.assertTrue(n.frontier_arrival_blocked)
            n.arm.assert_not_awaited()
            n.guard_received(self.message(stop_generation=8))
            asyncio.run(n.automatic_replan(100.,n.selection_generation))
            self.assertEqual(n.run.phase,'exploring')
            n.alternative_route.assert_not_awaited()
            n.send_demo.assert_not_called()
            self.assertEqual(n.alternate_route,[])
            next_frontier=[0.,1.75,0.]
            asyncio.run(n.choose_frontier([self.candidate(next_frontier)]))
        n.arm.assert_awaited_once()
        n.send_demo.assert_called_once_with(next_frontier,'frontier')

    def test_automatic_replan_does_not_arm_on_expired_lease_or_exhausted_retries(self):
        for heartbeat,count in ((97.,0),(99.8,2)):
            n=self.frontier_node()
            n.healthy=lambda:True
            n.velocity=[0.,0.]
            n.active_path=[[0.,0.,0.],[3.,0.,0.]]
            n.executor=MagicMock();n.arm=AsyncMock()
            n.run.heartbeat_wall=heartbeat;n.auto_retry_count=count
            with patch.object(self.module.time,'monotonic',return_value=100.):
                n.route_blocked('Route blocked')
                n.guard_received(self.message(stop_generation=8))
                n.schedule_automatic_replan(100.)
            self.assertEqual(n.run.phase,'exploration_paused')
            n.executor.create_task.assert_not_called()
            n.arm.assert_not_awaited()

    def test_failed_alternate_checks_a_route_home_with_the_same_operator_lease(self):
        n=self.frontier_node()
        n.healthy=lambda:True;n.velocity=[0.,0.]
        n.run.phase='exploration_paused';n.run.heartbeat_wall=99.8
        n.alternative_route=AsyncMock(side_effect=ValueError('No safe distinct route'))
        n.recovery_return=AsyncMock(return_value=(None,[[0.,0.,0.]]))
        n.arm=AsyncMock()
        with patch.object(self.module.time,'monotonic',return_value=100.):
            asyncio.run(n.automatic_replan(100.,n.selection_generation))
        self.assertEqual(n.run.phase,'exploration_return')
        self.assertEqual(n.run.heartbeat_wall,99.8)
        n.recovery_return.assert_awaited_once()
        n.arm.assert_awaited_once()

    def test_contact_pauses_and_only_the_explicit_owner_can_authorize_retrace(self):
        n=self.frontier_node()
        n.healthy=lambda:True
        n.run.phase_started=5.
        n.velocity=[0.,0.]
        n.pose=lambda:[.60,0.,0.]
        n.trails=[dict(wall=6.+i,odom_pose=[i*.05,0.,0.]) for i in range(13)]
        n.retreat_client=MagicMock()
        n.arm=AsyncMock()
        contact=dict(stop_generation=8,reason='chassis_contact',fault={'reason':'chassis_contact'})
        n.guard_received(self.message(in_contact=True,**contact))
        self.assertEqual(n.run.phase,'contact_paused')
        request=NS(command='retreat_and_replan',session_id='one',mission_id=n.run.mission_id,
                   client_token=n.run.owner,epoch=0)
        with patch.object(self.module.time,'monotonic',return_value=100.):
            self.assertFalse(asyncio.run(n.command(request,NS(success=False))).success)
            n.arm.assert_not_awaited()
            n.guard_received(self.message(in_contact=False,**contact))
            request.client_token='different-operator-token'
            self.assertFalse(asyncio.run(n.command(request,NS(success=False))).success)
            n.arm.assert_not_awaited()
            request.client_token=n.run.owner
            result=asyncio.run(n.command(request,NS(success=False)))
        self.assertTrue(result.success,result.message)
        self.assertEqual(n.run.phase,'contact_reversing')
        n.arm.assert_awaited_once_with(n.selection_generation,retreat=True)
        self.assertAlmostEqual(n.retreat_route[-1][0],.1)
        n.retreat_pub=MagicMock()
        n.stopped=AsyncMock()
        n.replan_after_contact=AsyncMock()
        n.pose=lambda:[.10,0.,0.]
        n.retreat_progress=.45
        asyncio.run(n.advance_contact_retreat(111.))
        n.stopped.assert_awaited_once()
        n.replan_after_contact.assert_awaited_once()
        self.assertEqual(n.contact_point,[.60,0.,.18])

    def test_soil_refinement_from_v37_fits_checked_envelope(self):
        n=self.sample_node()
        old=[3.6156123374,-2.9914112902,-.0022780826]
        latest=[3.640869392,-3.018871104,-.0099285954]
        n.run.target=dict(source='detected_soil',track_id='0:4',odom_position=old,
                          envelope_radius_m=.67542481298)
        n.run.preview['keepout']=[*old[:2],n.sample_keepout_radius(n.run.target)]
        site=dict(odom_position=latest,envelope_radius_m=.71470486769)
        n.selected_site=MagicMock(return_value=site)
        self.assertEqual(n.target_current_signature(),target_signature(n.run.target))
        site['envelope_radius_m']=.94
        with self.assertRaisesRegex(ValueError,'grew beyond the checked clearance'):
            n.target_current_signature()

    def test_v47_soil_refinement_fits_the_approved_larger_envelope(self):
        n=self.sample_node()
        original=[.56457518351,5.86932956687,.01049308247]
        n.run.target=dict(source='detected_soil',track_id='0:4',odom_position=original,
                          envelope_radius_m=.75511164679)
        n.run.preview['keepout']=[*original[:2],n.sample_keepout_radius(n.run.target)]
        site=dict(odom_position=[.50186833613,5.73227196332,-.01989420514],envelope_radius_m=.87245595739)
        n.selected_site=MagicMock(return_value=site)
        self.assertEqual(n.target_current_signature(),target_signature(n.run.target))
        site['envelope_radius_m']=.93
        with self.assertRaisesRegex(SampleGeometryChanged,'grew beyond the checked clearance'):
            n.target_current_signature()

    def test_soil_collection_tolerates_report_cadence_but_rejects_stale_evidence(self):
        n=self.sample_node()
        site=dict(track_id='0:4',state='confirmed',geometry=dict(supported=True),
                  last_confidence=.82,last_supported_stamp=14.25)
        n.soils=dict(ready=True,epoch=0,sites=[site]);n.soils_wall=100.
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=15_000_000_000))
        with patch.object(self.module.time,'monotonic',return_value=100.2):
            n.run.phase='sample_outbound'
            with self.assertRaisesRegex(ValueError,'not freshly reacquired'):
                n.selected_site('0:4',True)
            n.run.phase='collecting'
            self.assertIs(n.selected_site('0:4',True),site)
            site['last_supported_stamp']=14.
            with self.assertRaisesRegex(ValueError,'not freshly reacquired'):
                n.selected_site('0:4',True)

    def test_growing_soil_pauses_for_updated_preview_without_resetting_budget(self):
        n=self.sample_node()
        n.run.approve(24.,n.run.preview['id'],n.home_map(),n.grid,target_signature(n.run.target))
        n.mode='demo';n.guard['mode']='autonomous'
        n.run.target=dict(source='detected_soil',track_id='0:4',odom_position=[3.,0.,0.],
                          envelope_radius_m=1.)
        n.selected_site=MagicMock(return_value={'odom_position':[3.03,0.,0.],
            'envelope_radius_m':1.14,'covariance':np.eye(3).tolist()})
        n.advance_collection=MagicMock(side_effect=SampleGeometryChanged('Geometry grew'))
        old_deadline=n.run.sample_started+n.run.sample_time_left(24.)
        with patch.object(self.module.time,'monotonic',return_value=48.):
            asyncio.run(n.advance())
        self.assertEqual(n.run.phase,'awaiting_target')
        self.assertIsNone(n.run.preview)
        self.assertEqual(n.run.target['envelope_radius_m'],1.14)
        self.assertAlmostEqual(48.+n.run.sample_time_left(48.),old_deadline)
        n.stop_client.call_async.assert_called_once()
        n.guard_received(self.message(stop_generation=8))
        self.assertIsNone(n.pause_stop_epoch)
        self.assertEqual(n.run.phase,'awaiting_target')
        n.run.validate_approval=MagicMock()
        n.run.approve(50.,'new',n.home_map(),n.grid,'new-target')
        self.assertEqual(n.run.sample_started,24.)

    def test_alternate_sample_previews_a_new_side_and_keeps_prior_preview_if_exhausted(self):
        n=self.sample_node()
        n.velocity=[0.,0.]
        n.sample_view_angles=[0.]
        old_id=n.run.preview['id']
        opposite=[2.,1.,-math.pi/2]
        n.planned_path=AsyncMock(side_effect=lambda start,end,token:NS(poses=[start,end]))
        with (
            patch.object(self.module.time,'monotonic',return_value=23.),
            patch.object(self.module,'collection_poses',return_value=[[1.,0.,0.],opposite]),
            patch.object(self.module,'pose_values',side_effect=lambda pose:pose),
        ):
            asyncio.run(n.plan_sample_preview(n.selection_generation,alternate=True))
            self.assertEqual(n.run.preview['stopping_pose'],opposite)
            self.assertNotEqual(n.run.preview['id'],old_id)
            saved=n.run.preview
            with self.assertRaisesRegex(ValueError,'distinct approach'):
                asyncio.run(n.plan_sample_preview(n.selection_generation,alternate=True))
        self.assertEqual(n.run.phase,'preview_ready')
        self.assertIs(n.run.preview,saved)
    def test_latest_common_drive_center_pose_accepts_vio_delay_but_rejects_stale_tf(self):
        n=self.node();n.tf=MagicMock();n.get_clock=MagicMock()
        n.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        tf=NS(header=NS(stamp=NS(sec=9,nanosec=900_000_000)),transform=NS(
            translation=NS(x=.13,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        n.tf.lookup_transform.return_value=tf;self.assertEqual(n.pose(),[.13,0.,0.])
        tf.header.stamp=NS(sec=9,nanosec=0)
        with self.assertRaises(ValueError):n.pose()
    def test_home_and_frontier_reprojection_use_latest_fresh_map_correction(self):
        n=self.node();n.tf=MagicMock();n.get_clock=MagicMock()
        n.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        yaw=.2
        tf=NS(header=NS(stamp=NS(sec=9,nanosec=998_000_000)),transform=NS(
            translation=NS(x=.4,y=-.3,z=0.),rotation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))))
        n.tf.lookup_transform.return_value=tf
        latest=object()
        anchored=object()
        with patch.object(self.module,'Time',return_value=latest) as clock:
            clock.from_msg.return_value=anchored
            for target,source in (('map','odom'),('odom','map')):
                n.tf.lookup_transform.reset_mock()
                result=n.transform_pose([1.,2.,.5],target)
                self.assertEqual(n.tf.lookup_transform.call_count,2)
                self.assertEqual(n.tf.lookup_transform.call_args_list[0].args,('map','drive_center',latest))
                self.assertEqual(n.tf.lookup_transform.call_args_list[1].args,(target,source,anchored))
                clock.from_msg.assert_called_with(tf.header.stamp)
                self.assertAlmostEqual(result[0],.4+math.cos(yaw)-2*math.sin(yaw))
                self.assertAlmostEqual(result[1],-.3+math.sin(yaw)+2*math.cos(yaw))
                self.assertAlmostEqual(result[2],.7)
    def test_map_correction_staleness_or_future_stamp_cannot_reproject_home(self):
        n=self.node();n.tf=MagicMock();n.get_clock=MagicMock()
        n.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        for stamp in (NS(sec=9,nanosec=300_000_000),NS(sec=10,nanosec=100_000_000)):
            n.tf.lookup_transform.return_value=NS(header=NS(stamp=stamp))
            with self.assertRaisesRegex(ValueError,'correction is stale'):n.transform_pose([0.,0.,0.])
    def test_future_dated_correction_is_queried_at_fresh_full_chain_time(self):
        n=self.node();n.tf=MagicMock();n.get_clock=MagicMock()
        n.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        anchor=NS(header=NS(stamp=NS(sec=9,nanosec=980_000_000)))
        correction=NS(header=NS(stamp=anchor.header.stamp),transform=NS(
            translation=NS(x=.4,y=-.3,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        def lookup(target,source,stamp):
            if source=='drive_center':return anchor
            self.assertEqual(stamp,('pose-time',9,980_000_000))
            return correction
        n.tf.lookup_transform.side_effect=lookup
        with patch.object(self.module,'Time') as clock:
            clock.from_msg.side_effect=lambda s:('pose-time',s.sec,s.nanosec)
            self.assertEqual(n.transform_pose([1.,2.,0.]),[1.4,1.7,0.])
        # Unavailable interpolation must still reject rather than use an old or
        # future correction, extrapolate, or silently return identity.
        n.tf.lookup_transform.side_effect=[anchor,RuntimeError('No transform at pose time')]
        with self.assertRaisesRegex(RuntimeError,'No transform'):n.transform_pose([0.,0.,0.])
    def test_health_identifies_missing_odometry_and_preserves_covariance_rejection(self):
        n=self.node();now=self.module.time.monotonic();n.grid=object()
        n.odometry_pose=None;n.odometry_wall=-float('inf');n.odometry_error='Waiting for estimated odometry'
        n.guard_wall=n.mapping_wall=n.detector_wall=now
        n.mapping=dict(accepted_depth_stamp=1.);n.detector=dict(ready=True)
        self.assertEqual(n.health_details()['missing'],['estimated_odometry'])
        message=NS(header=NS(frame_id='odom'),child_frame_id='base_link',
            pose=NS(covariance=[0.]*36,pose=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
            twist=NS(twist=NS(linear=NS(x=0.),angular=NS(z=0.))))
        n.odom_received(message);self.assertTrue(n.healthy());self.assertEqual(n.odometry_error,'')
        message.pose.covariance[0]=.04
        n.odom_received(message);self.assertFalse(n.healthy())
        self.assertEqual(n.health_details()['missing'],['estimated_odometry'])
        self.assertIn('Localization uncertain',n.odometry_error)
    def test_stopping_settle_does_not_consume_arrival_notification(self):
        n=self.sample_node();n.run.approve(24.,n.run.preview['id'],n.home_map(),n.grid,
                                           target_signature(n.run.target))
        n.nav_done='sample';n.pose=lambda:[1.,0.,0.]
        n.odometry_pose=[1,0,0];n.velocity=[.02,0]
        with patch.object(self.module.time,'monotonic',return_value=25.):
            asyncio.run(n.advance());self.assertEqual(n.nav_done,'sample');self.assertEqual(n.run.phase,'sample_outbound')
            n.velocity=[0,0];asyncio.run(n.advance());self.assertEqual(n.run.phase,'collecting')

    def sample_node(self):
        n=self.frontier_node()
        n.run.transition('exploration_return',10.)
        n.run.home_arrived(20.)
        n.run.select(21.,dict(source='operator_selected',odom_position=[2.,0.,0.]))
        # Recorded v33 planner lattice endpoint was 7.8 cm from the requested home.
        home=n.home_map()
        n.run.set_preview(22.,[home,[1.,0.,0.]],[[.5,0.,0.],[.078,0.,0.]],
                          [1.,0.,0.],home,n.grid,target_signature(n.run.target),
                          home=home,target_map=[2.,0.],keepout=[2.,0.,.20])
        n.healthy=lambda:True
        n.sample_arrival_wall=None
        n.backup_start=None
        n.sample_motion=MagicMock()
        return n

    def test_approval_compares_home_to_the_requested_pose(self):
        for shift in (0.,.06):
            n=self.sample_node()
            n.home_map=lambda:[shift,0.,0.]
            n.protect_sample=AsyncMock()
            n.arm=AsyncMock()
            n.send_demo=MagicMock()
            request=NS(command='start_sample',session_id='one',mission_id=n.run.mission_id,
                       client_token=n.run.owner,epoch=0,preview_id=n.run.preview['id'])
            with patch.object(self.module.time,'monotonic',return_value=24.):
                response=asyncio.run(n.command(request,NS(success=False)))
            self.assertEqual(response.success,shift==0.)
            if shift==0.:
                n.arm.assert_awaited_once()
                n.send_demo.assert_called_once_with([1.,0.,0.],'sample')
            else:
                self.assertIn('Home pose changed',response.message)
                n.arm.assert_not_awaited()

    def test_sample_approval_restarts_expired_exploration_lease(self):
        for arm_elapsed,expected in ((.05,True),(1.02,False)):
            n=self.sample_node()
            n.run.heartbeat_wall=-7.
            n.protect_sample=AsyncMock()
            wall=24.
            async def arm(token):
                nonlocal wall
                self.assertEqual(n.run.phase,'preview_ready')
                wall+=arm_elapsed
                n.run.deadlines(wall)
            n.arm=arm
            n.send_demo=MagicMock()
            request=NS(command='start_sample',session_id='one',mission_id=n.run.mission_id,
                       client_token=n.run.owner,epoch=0,preview_id=n.run.preview['id'])
            with patch.object(self.module.time,'monotonic',side_effect=lambda:wall):
                response=asyncio.run(n.command(request,NS(success=False)))
            self.assertEqual(response.success,expected,response.message)
            if expected:
                n.run.deadlines(wall)
                self.assertEqual(n.run.phase,'sample_outbound')
                n.send_demo.assert_called_once()
            else:
                self.assertEqual(n.run.phase,'stopped')
                n.send_demo.assert_not_called()
                n.stop_client.call_async.assert_called_once()

    def test_selection_deadline_while_waiting_for_arm_stops_guard(self):
        n=self.sample_node()
        n.run.selection_started=-35.9
        n.protect_sample=AsyncMock()
        wall=24.
        async def arm(token):
            nonlocal wall
            wall+=.2
            n.run.deadlines(wall)
        n.arm=arm
        n.send_demo=MagicMock()
        request=NS(command='start_sample',session_id='one',mission_id=n.run.mission_id,
                   client_token=n.run.owner,epoch=0,preview_id=n.run.preview['id'])
        with patch.object(self.module.time,'monotonic',side_effect=lambda:wall):
            response=asyncio.run(n.command(request,NS(success=False)))
        self.assertFalse(response.success)
        self.assertIn('selection timed out',n.run.reason)
        n.send_demo.assert_not_called()
        n.stop_client.call_async.assert_called_once()

    def test_sample_faces_site_then_reverses_without_turning_before_return(self):
        n=self.sample_node()
        n.run.approve(24.,n.run.preview['id'],n.home_map(),n.grid,target_signature(n.run.target))
        n.nav_done='sample'
        n.velocity=[0.,0.]
        n.pose=lambda:[1.,0.,.10]
        n.advance_collection(25.)
        n.sample_motion.assert_called_once_with(angular=-.15)
        self.assertEqual(n.run.phase,'sample_outbound')
        n.pose=lambda:[1.,0.,0.]
        n.odometry_pose=[1.,0.,0.]
        n.advance_collection(26.)
        self.assertEqual(n.run.phase,'collecting')
        n.guard['mode']='autonomous'
        request=NS(command='return_now',session_id='one',mission_id=n.run.mission_id,
                   client_token=n.run.owner,epoch=0)
        with patch.object(self.module.time,'monotonic',return_value=29.4):
            response=asyncio.run(n.command(request,NS(success=False)))
        self.assertFalse(response.success)
        self.assertEqual(n.run.phase,'collecting')
        self.assertFalse(n.sample_return_ready())
        n.advance_collection(36.)
        self.assertEqual(n.run.phase,'awaiting_sample_return')
        with patch.object(self.module.time,'monotonic',return_value=36.5):
            self.assertTrue(asyncio.run(n.command(request,NS(success=False))).success)
        self.assertEqual(n.run.phase,'sample_backing')
        n.path=AsyncMock(return_value=[[.5,0.,0.],[0.,0.,0.]])
        n.send_demo=MagicMock()
        with patch.object(self.module.time,'monotonic',return_value=37.):
            asyncio.run(n.advance())
        n.path.assert_awaited_once_with([.5,0.,0.],[0.,0.,0.],keepout=[2.,0.,.20])
        n.sample_motion.assert_called_with(linear=-.05)
        n.send_demo.assert_not_called()
        n.pose=lambda:[.5,0.,0.]
        n.velocity=[-.04,0.]
        asyncio.run(n.back_away_from_sample(46.))
        self.assertEqual(n.run.phase,'sample_backing')
        n.sample_motion.assert_called_with()
        n.velocity=[0.,0.]
        asyncio.run(n.back_away_from_sample(47.))
        self.assertEqual(n.run.phase,'sample_return')
        n.send_demo.assert_not_called()
        with patch.object(self.module.time,'monotonic',return_value=47.2):
            asyncio.run(n.advance())
        n.send_demo.assert_called_once_with(n.home_map(),'home')
        n.nav_done='home';n.pose=n.home_map;n.stopped=AsyncMock()
        with patch.object(self.module.time,'monotonic',return_value=51.):
            asyncio.run(n.advance())
        n.stopped.assert_awaited_once()
        self.assertEqual(n.run.phase,'complete')
        self.assertTrue(n.run.sample['delivered'])

    def test_deadline_records_heartbeat_expiry_without_extending_the_lease(self):
        n=self.node();n.run.start(10.,'operator-token-one',[0,0,0],0,0.)
        n.run.heartbeat(10.5,1);n.health_details=lambda:dict(healthy=True,checks={},missing=[])
        n.odometry_wall=n.guard_wall=n.mapping_wall=n.detector_wall=11.5
        n.check_deadlines(11.51)
        self.assertEqual(n.run.phase,'stopped')
        event=next(e for e in n.run.events if e['kind']=='autonomy_deadline_inputs')
        self.assertAlmostEqual(event['heartbeat_age_seconds'],1.01)
        self.assertEqual(event['heartbeat_limit_seconds'],1.)
        self.assertTrue(event['navigation_health']['healthy'])
        self.assertNotIn('operator-token-one',json.dumps(event))

    def test_heartbeat_after_deadline_cannot_renew_before_the_next_tick(self):
        n=self.node();n.run.start(10.,'operator-token-one',[0,0,0],0,0.)
        n.run.heartbeat(10.5,1);n.health_details=lambda:dict(healthy=True,checks={},missing=[])
        request=NS(command='heartbeat',session_id='one',mission_id=n.run.mission_id,
                   client_token=n.run.owner,epoch=0,sequence=2)
        with patch.object(self.module.time,'monotonic',return_value=11.51):
            response=asyncio.run(n.command(request,NS(success=False)))
        self.assertFalse(response.success)
        self.assertEqual(n.run.heartbeat_wall,10.5)
        self.assertEqual(n.run.phase,'stopped')
        self.assertEqual(response.message,'Operator heartbeat lost')
        n.stop_client.call_async.assert_called_once()

    def test_deadline_records_transient_input_loss_with_a_live_heartbeat(self):
        n=self.node();n.run.start(10.,'operator-token-one',[0,0,0],0,0.)
        n.run.heartbeat(11.4,2)
        n.health_details=lambda:dict(healthy=False,checks=dict(detector=False),missing=['detector'])
        n.detector_wall=10.8
        n.check_deadlines(11.5)
        self.assertEqual(n.run.phase,'stopped')
        self.assertEqual(n.run.reason,'Navigation inputs became unhealthy (detector)')
        event=next(e for e in n.run.events if e['kind']=='autonomy_deadline_inputs')
        self.assertAlmostEqual(event['heartbeat_age_seconds'],.1)
        self.assertAlmostEqual(event['input_ages_seconds']['detector'],.7)
        self.assertEqual(event['navigation_health']['missing'],['detector'])
        self.assertIsNone(event['input_ages_seconds']['accepted_depth'])

    def test_healthy_inputs_and_live_heartbeat_continue_without_stop_event(self):
        n=self.node();n.run.start(10.,'operator-token-one',[0,0,0],0,0.)
        n.run.heartbeat(10.5,1);n.health_details=lambda:dict(healthy=True,checks={},missing=[])
        n.check_deadlines(11.49)
        self.assertEqual(n.run.phase,'sweeping')
        self.assertFalse(any(e['kind']=='autonomy_deadline_inputs' for e in n.run.events))


if __name__=='__main__':unittest.main()
