"""Cancellation and arrival races in the real ROS adapter, with transport doubles."""
import asyncio
import importlib
import json
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock,MagicMock,patch

from rover_navigation.demo_core import DemoRun,DemoPolicy


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
        n.mode='idle';n.manual_used=False;n.working=False;n.nav_done=None
        n.last_route_check=float('inf')
        n.return_requested=False
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
    def test_latest_common_drive_center_pose_accepts_vio_delay_but_rejects_stale_tf(self):
        n=self.node();n.tf=MagicMock();n.get_clock=MagicMock()
        n.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        tf=NS(header=NS(stamp=NS(sec=9,nanosec=900_000_000)),transform=NS(
            translation=NS(x=.13,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        n.tf.lookup_transform.return_value=tf;self.assertEqual(n.pose(),[.13,0.,0.])
        tf.header.stamp=NS(sec=9,nanosec=0)
        with self.assertRaises(ValueError):n.pose()
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
        n=self.node();n.run.start(0.,'operator-token-one',[0,0,0],0,0.)
        n.run.phase='sample_outbound';n.run.target={'source':'operator_selected'}
        n.goal=None;n.pending=False;n.nav_done='sample';n.collection_pose=None
        n.odometry_pose=[1,0,0];n.velocity=[.02,0];n.guard['command']=[0.,0.]
        n.healthy=lambda:True
        asyncio.run(n.advance());self.assertEqual(n.nav_done,'sample');self.assertEqual(n.run.phase,'sample_outbound')
        n.velocity=[0,0];asyncio.run(n.advance());self.assertEqual(n.run.phase,'collecting')


if __name__=='__main__':unittest.main()
