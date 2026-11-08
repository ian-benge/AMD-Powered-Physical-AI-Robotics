"""Larger-map boundaries and 0.20 m/s leases through production safety rules."""
import math
import unittest

import numpy as np

from rover_navigation.core import Grid, TerrainMemory, mask_outside_memory, swept_clear
from rover_navigation.manual_input import ManualInput
from rover_navigation.safety import SafetyGate


class EnvironmentV2SafetyTests(unittest.TestCase):
    def test_all_four_local_boundaries_remain_unknown_despite_fresh_hazards(self):
        memory = TerrainMemory(40., .025)
        self.assertEqual(memory.grid.data.shape, (1600,1600))
        for cx,cy in ((19.,0.),(-19.,0.),(0.,19.),(0.,-19.)):
            local = Grid(np.full((240,240),100,np.int8),.025,cx-3.,cy-3.)
            mask_outside_memory(local,memory.grid)
            x = local.x+(np.arange(240)+.5)*.025
            y = local.y+(np.arange(240)+.5)*.025
            inside = (abs(y)<20.)[:,None] & (abs(x)<20.)[None,:]
            self.assertTrue((local.data[~inside]==-1).all())
            self.assertTrue((local.data[inside]==100).all())

    def gate(self, now=10.):
        gate=SafetyGate()
        gate.clock(1.,now)
        for name in ('depth','pose'):gate.health(name,now)
        self.assertTrue(gate.select('manual',now))
        gate.command('manual',.2,0.,now)
        return gate

    def test_unknown_and_hazard_ground_stop_point_two_and_stay_latched(self):
        for value in (-1,100):
            gate=self.gate()
            grid=Grid(np.zeros((240,240),np.int8),.025,-3.,-3.)
            grid.data[:,round((.15+3.)/.025):]=value
            self.assertEqual(gate.output(10.05,lambda v,w:swept_clear(grid,(0.,0.,0.),v,w)),(0.,0.))
            self.assertEqual(gate.mode,'stopped')
            grid.data[:]=0
            gate.command('manual',.2,0.,10.06)
            self.assertEqual(gate.output(10.07,lambda *_:True),(0.,0.))
            self.assertTrue(gate.select('manual',10.08))
            self.assertEqual(gate.output(10.09,lambda *_:True),(0.,0.))
            gate.command('manual',.2,0.,10.1)
            self.assertEqual(gate.output(10.11,lambda *_:True),(.2,0.))

    def test_point_two_stale_clock_pose_depth_and_command_stop_and_require_rearm(self):
        for stale in ('clock','pose','depth','command'):
            gate=self.gate()
            if stale=='clock':gate.clock_received=9.
            elif stale in ('pose','depth'):gate.sensors[stale]=8.
            now=10.36 if stale=='command' else 10.05
            self.assertEqual(gate.output(now,lambda *_:True),(0.,0.))
            self.assertEqual(gate.mode,'stopped')
            gate.clock(2.,now)
            for name in ('depth','pose'):gate.health(name,now)
            gate.command('manual',.2,0.,now)
            self.assertEqual(gate.output(now,lambda *_:True),(0.,0.))

    def test_released_or_expired_browser_lease_cannot_replay_point_two(self):
        for released in (True,False):
            control=ManualInput(max_linear=.2,max_angular=.3,initial_linear=.05)
            token=control.arm();now=control.received
            control.command(dict(session=token,sequence=0,linear=.2,angular=0.),now)
            if released:control.release(token)
            with self.assertRaises(ValueError):
                control.command(dict(session=token,sequence=1,linear=.2,angular=0.),now+(.01 if released else .26))
            self.assertIsNone(control.token)


if __name__=='__main__':unittest.main()
