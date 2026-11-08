import unittest

import numpy as np

from rover_navigation.core import depth_points
from rover_navigation.sample_sites import depth_location
from rover_navigation.site_geometry import measure


class RenderingDepthContractTests(unittest.TestCase):
    def test_distant_rgb_without_usable_depth_cannot_position_an_object_or_clear_ground(self):
        k=[30.,0.,20.,0.,30.,20.,0.,0.,1.]
        box=[10.,10.,30.,30.]
        for distance in (np.inf,9.,20.):
            depth=np.full((40,40),distance,dtype='<f4')
            self.assertEqual(depth_points(depth.tobytes(),'32FC1',40,40,160,False,k).shape,(0,3))
            with self.assertRaises(ValueError):
                depth_location(depth.tobytes(),'32FC1',40,40,160,False,k,box)
            with self.assertRaises(ValueError):
                measure(depth,k,box,[0,0,0],[0,0,0,1],np.zeros((6,6)))


if __name__=='__main__': unittest.main()
