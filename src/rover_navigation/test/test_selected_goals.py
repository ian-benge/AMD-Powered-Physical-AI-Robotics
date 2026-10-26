import asyncio
import importlib
import json
import math
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

import numpy as np

from rover_navigation.core import Grid, footprint_clear, transform
from rover_navigation.goal_checks import check_path, overlay_hazards, pose_values


def pose(x=0., y=0., yaw=0., frame='map'):
    return NS(header=NS(frame_id=frame), pose=NS(position=NS(x=x, y=y, z=0.),
              orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))


class GoalGeometryTests(unittest.TestCase):
    def grid(self):
        return Grid(np.zeros((120, 120), dtype=np.int8), .05, -3., -3.)

    def test_invalid_frame_nan_and_quaternion_rejected(self):
        for msg in (pose(frame='odom'), pose(x=float('nan')), pose()):
            if msg.header.frame_id == 'map' and math.isfinite(msg.pose.position.x):
                msg.pose.orientation.w = 0.
            with self.assertRaises(ValueError):
                pose_values(msg)

    def test_entire_path_checked_between_sparse_poses(self):
        grid = self.grid()
        check_path(grid, [(0., 0., 0.), (1., 0., 0.)], (0., 0., 0.), (1., 0., 0.))
        grid.data[grid.cell(.5, .15)] = 100
        with self.assertRaises(ValueError):
            check_path(grid, [(0., 0., 0.), (1., 0., 0.)], (0., 0., 0.), (1., 0., 0.))

    def test_new_hazard_overrides_old_free_and_reprojects_after_correction(self):
        grid = self.grid()
        observed = np.array([[.5, 0., 0.]])
        overlay_hazards(grid, transform(observed, [1., 0., 0.], [0., 0., 0., 1.]))
        self.assertEqual(grid.at(1.5, 0.), 100)
        self.assertEqual(grid.at(.5, 0.), 0)
        grid.data[:] = -1
        overlay_hazards(grid, observed)
        self.assertEqual(grid.at(.5, 0.), 100)
        self.assertEqual(grid.at(2., 0.), -1)


class SelectedMissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        names = ('rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.callback_groups', 'rclpy.clock',
                 'rclpy.qos', 'rclpy.time', 'action_msgs', 'action_msgs.msg', 'geometry_msgs',
                 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'nav2_msgs', 'nav2_msgs.action',
                 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'tf2_ros',
                 'visualization_msgs', 'visualization_msgs.msg')
        modules = {name: MagicMock() for name in names}
        modules['rclpy.node'].Node = object
        modules['tf2_ros'].TransformException = RuntimeError
        with patch.dict(sys.modules, modules):
            cls.module = importlib.import_module('rover_navigation.mission_node')

    def node(self):
        node = self.module.Mission.__new__(self.module.Mission)
        node.mode, node.starting, node.home = 'idle', False, None
        node.guard = dict(mode='stopped', ready=True, stop_generation=7)
        node.guard_wall = self.module.time.monotonic()
        node.guard_sequence = node.home_attempts = 0
        node.selection_state, node.selection_reason = 'ready', ''
        node.selection_generation = 4
        node.selected, node.preview_home = (1., 0., 0.), (0., 0., 0.)
        node.grid = Grid(np.zeros((120, 120), dtype=np.int8), .05, -3., -3.)
        node.pose = MagicMock(return_value=(0., 0., 0.))
        node.publish_status = MagicMock()
        node.begin = MagicMock()
        node.stop_client, node.enable_client, node.nav, node.planner = MagicMock(), MagicMock(), MagicMock(), MagicMock()
        node.validate = AsyncMock()
        node.wait = AsyncMock(return_value=NS(success=True, message=json.dumps(dict(stop_generation=7))))
        return node

    def response(self):
        return NS(success=False, message='')

    def test_start_revalidates_then_enables_and_begins(self):
        node = self.node()
        reply = asyncio.run(node.start_selected(None, self.response()))
        self.assertTrue(reply.success)
        node.validate.assert_awaited_once()
        node.begin.assert_called_once_with('waypoints', [(1., 0., 0.)])

    def test_map_changed_after_preview_rejects_start_without_arming(self):
        node = self.node()
        node.grid.data[node.grid.cell(1., .1)] = 100
        async def validate(_):
            node.preview_paths = []
            node.check_preview(node.pose(), node.preview_home)
        node.validate.side_effect = validate
        reply = asyncio.run(node.start_selected(None, self.response()))
        self.assertFalse(reply.success)
        node.enable_client.call_async.assert_not_called()

    def test_stop_during_planning_invalidates_pending_start(self):
        node = self.node()
        async def cancelled(_):
            node.selection_generation += 1
            raise ValueError('Stopped')
        node.validate.side_effect = cancelled
        self.assertFalse(asyncio.run(node.start_selected(None, self.response())).success)
        node.begin.assert_not_called()
        node.enable_client.call_async.assert_not_called()

    def test_stop_epoch_changed_during_arm_cannot_begin(self):
        node = self.node()
        node.wait.return_value.message = json.dumps(dict(stop_generation=8))
        self.assertFalse(asyncio.run(node.start_selected(None, self.response())).success)
        node.begin.assert_not_called()
        node.stop_client.call_async.assert_called()

    def test_late_planner_acceptance_is_cancelled(self):
        node = self.node()
        node.selected_pose = MagicMock()
        node.wait.side_effect = ValueError('Cancelled')
        with self.assertRaises(ValueError):
            asyncio.run(node.planned_path((0., 0., 0.), (1., 0., 0.), 4))
        pending = node.planner.send_goal_async.return_value
        callback = pending.add_done_callback.call_args.args[0]
        callback(pending)
        pending.result.return_value.cancel_goal_async.assert_called_once()

    def test_takeover_and_sensor_loss_cancel_active_mission(self):
        for guard in (dict(mode='manual', ready=True, stop_generation=8),
                      dict(mode='stopped', ready=False, stop_generation=7)):
            node = self.node()
            node.mode = 'waypoints'
            node.abort = MagicMock()
            node.guard_received(NS(data=json.dumps(guard)))
            node.abort.assert_called_once()

    def test_replacement_selection_during_mission_is_rejected(self):
        node = self.node()
        node.mode = 'waypoints'
        node.select_goal(pose(2.))
        self.assertEqual(node.selected, (1., 0., 0.))

    def test_return_route_loss_stops_without_recovery(self):
        node = self.node()
        node.mode, node.home = 'waypoints', (2., 0., 0.)
        node.started_wall = self.module.time.monotonic()
        node.last_route_check = -math.inf
        node.get_parameter = MagicMock(return_value=NS(value=300.))
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 10**9
        node.abort = MagicMock()
        node.grid.data[:, 80] = 100
        node.tick()
        node.abort.assert_called_once_with('No verified safe return route remains')

    def test_status_published_before_arming_cannot_cancel_new_mission(self):
        node = self.node()
        node.wait = AsyncMock(return_value=NS(success=True, message=json.dumps(
            dict(stop_generation=7, status_sequence=40))))
        node.begin = MagicMock(side_effect=lambda mode, route: setattr(node, 'mode', mode))
        node.abort = MagicMock()
        self.assertTrue(asyncio.run(node.start_selected(None, self.response())).success)
        node.guard_received(NS(data=json.dumps(dict(mode='stopped', ready=True, stop_generation=7, sequence=39))))
        node.abort.assert_not_called()
        node.guard_received(NS(data=json.dumps(dict(mode='stopped', ready=True, stop_generation=8, sequence=41,
                                                     reason='operator_stop'))))
        node.abort.assert_called_once()
        self.assertIn('operator_stop', node.abort.call_args.args[0])

    def test_navigation_failure_reports_nav2_outcome_and_guard_fault(self):
        node = self.node()
        node.mode, node.generation, node.goal, node.pending = 'returning', 3, None, False
        node.get_clock = MagicMock()
        node.abort = MagicMock()
        node.guard = dict(mode='stopped', reason='operator_stop', fault=dict(reason='map_localization_stale'))
        node.send((0., 0., 0.))
        accepted = node.nav.send_goal_async.return_value.add_done_callback.call_args.args[0]
        handle = MagicMock(accepted=True)
        accepted(NS(result=lambda: handle))
        completed = handle.get_result_async.return_value.add_done_callback.call_args.args[0]
        completed(NS(result=lambda: NS(status=5, result=NS(error_code=0, error_msg=''))))
        message = node.abort.call_args.args[0]
        self.assertIn('Nav2 cancelled', message)
        self.assertIn('guard fault map_localization_stale', message)

    def test_return_rotates_in_place_to_exact_home_heading(self):
        node = self.node()
        node.mode, node.home = 'returning', (0., 0., math.radians(15.))
        node.get_parameter = MagicMock(return_value=NS(value=4.))
        node.abort = MagicMock()
        # Lattice goals end on the nearest of its uneven headings (26.57 degrees here).
        node.pose.return_value = (.012, -.01, math.atan2(1., 2.))
        node.finish_return()
        node.abort.assert_not_called()
        self.assertEqual(node.mode, 'aligning')
        x, y, yaw = node.route[0]
        self.assertEqual(node.grid.cell(x, y), node.grid.cell(.012, -.01))  # same planner cell: rotate only
        self.assertAlmostEqual(yaw, math.radians(15.))
        node.pose.return_value = (.012, -.01, math.radians(12.))
        node.finish_return()
        node.abort.assert_called_once_with('Home reached')

    def test_pivot_offset_after_final_turn_gets_bounded_reapproach(self):
        node = self.node()
        node.mode, node.home = 'returning', (0., 0., 0.)
        node.get_parameter = lambda name: NS(value=dict(
            home_yaw_tolerance_deg=5., home_position_tolerance_m=.08, home_reapproaches=2)[name])
        node.abort = MagicMock()
        node.pose.return_value = (-.26, 0., 0.)  # front-axle pivot after a 180 degree turn at home
        node.finish_return()
        node.finish_return()
        self.assertEqual((node.mode, node.home_attempts), ('returning', 2))
        node.abort.assert_not_called()
        node.finish_return()
        node.abort.assert_called_once_with('Home reached')

    def test_home_alignment_requires_a_clear_rotation_sweep(self):
        node = self.node()
        node.mode, node.home = 'returning', (0., 0., math.pi/2)
        node.get_parameter = MagicMock(return_value=NS(value=4.))
        node.abort = MagicMock()
        node.grid.data[node.grid.cell(-.275, -.275)] = 100  # swept only by the rear overhang mid-turn
        self.assertTrue(footprint_clear(node.grid, 0., 0., 0.))
        self.assertTrue(footprint_clear(node.grid, 0., 0., math.pi/2))
        node.finish_return()
        self.assertIn('alignment is not verified clear', node.abort.call_args.args[0])
        self.assertEqual(node.mode, 'returning')

    def test_alignment_goal_is_sent_without_triggering_another_return(self):
        node = self.node()
        node.mode, node.home, node.route = 'aligning', (0., 0., .26), [(.025, -.025, .26)]
        node.goal, node.pending, node.started = None, False, 0.
        node.started_wall = self.module.time.monotonic()
        node.last_route_check = -math.inf
        node.get_parameter = MagicMock(return_value=NS(value=300.))
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.nanoseconds = 299*10**9
        node.cancel, node.send = MagicMock(), MagicMock()
        node.tick()
        node.cancel.assert_not_called()
        node.send.assert_called_once_with((.025, -.025, .26))
