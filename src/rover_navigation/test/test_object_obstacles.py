import unittest

import numpy as np

from rover_navigation.core import Grid, cause_names, swept_clear
from rover_navigation.demo_core import collection_poses
from rover_navigation.object_obstacles import ObjectObstacles


def track(ident='0:1', name='rock', state='confirmed', point=(1., 0., .1), epoch=0):
    return dict(track_id=ident, class_name=name, state=state, epoch=epoch,
                odom_position=point, extent=[.18, .14, .12], covariance=(np.eye(3) * .0025).tolist())


def report(*objects, tentative=(), ready=True, epoch=0, stamp=1.):
    return dict(schema_version=2, frame='odom', ready=ready, epoch=epoch,
                accepted_stamp=stamp, objects=objects, tentative_objects=tentative)


def grids():
    return (Grid(np.zeros((160, 160), np.int8), .05, -4., -4.),
            Grid(np.zeros((160, 160), np.int8), .05, -4., -4.))


class ObjectObstacleTests(unittest.TestCase):
    def test_blocks_footprint_and_keeps_unknown_and_depth_hazards(self):
        obstacles = ObjectObstacles()
        obstacles.update(report(track(), track('0:2', 'pale_soil_patch', point=(-1., 0., 0.)),
                                tentative=[track('0:3', state='tentative', point=(0., -1., .1))]))
        local, reasons = grids()
        local.data[local.cell(1.025, .075)] = -1
        local.data[local.cell(-2., 0.)] = 100
        reasons.data[reasons.cell(1., 0.)] = 2
        obstacles.apply(local, reasons)
        self.assertEqual(local.at(1., 0.), 100)
        self.assertEqual(set(cause_names(reasons.at(1., 0.))), {'excessive_slope', 'tracked_object'})
        self.assertEqual(local.at(1.025, .075), -1)
        self.assertEqual(local.at(-2., 0.), 100)
        self.assertEqual(local.at(-1., 0.), 0)
        self.assertEqual(local.at(0., -1.), 0)
        self.assertFalse(swept_clear(local, (.57, 0., 0.), .1, 0.))
        self.assertTrue(swept_clear(local, (.57, -.6, 0.), .1, 0.))

    def test_rotated_map_projection_and_selected_object_stay_clear(self):
        obstacles = ObjectObstacles()
        obstacles.update(report(track()))
        global_grid, reasons = grids()
        obstacles.apply(global_grid, reasons, lambda points: [(-y, x, z) for x, y, z in points])
        self.assertEqual(global_grid.at(0., 1.), 100)
        self.assertEqual(global_grid.at(1., 0.), 0)
        with self.assertRaises(ValueError):
            collection_poses(global_grid, [0., 1.])
        poses = collection_poses(global_grid, [0., 1.], .32, raised_object=True)
        self.assertTrue(poses)
        self.assertTrue(all(np.linalg.norm(np.asarray(p[:2]) - [0., 1.]) >= 1.49 for p in poses))
        self.assertEqual(global_grid.at(0., 1.), 100)

    def test_suspension_and_demotion_retain_obstacle_until_retirement(self):
        obstacles = ObjectObstacles()
        obstacles.update(report(track()))
        obstacles.update(report(ready=False, stamp=2.))
        obstacles.update(report(tentative=[track(state='unverified')], stamp=3.))
        obstacles.update(report(stamp=2.))
        self.assertIn('0:1', obstacles.disks)
        obstacles.update(report(stamp=4.))
        self.assertFalse(obstacles.disks)
        obstacles.update(report(track(), stamp=5.))
        obstacles.update(report(epoch=1, stamp=.5))
        obstacles.update(report(track(), stamp=9.))
        self.assertFalse(obstacles.disks)
