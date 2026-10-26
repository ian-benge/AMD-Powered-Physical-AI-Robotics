import unittest
import xml.etree.ElementTree as ET
from rover_navigation.fixtures import NAMES, fixture_world


class FixtureTests(unittest.TestCase):
    def test_every_fixture_has_physical_ground_and_no_generated_map(self):
        for name in NAMES:
            with self.subTest(name=name):
                root = ET.fromstring(fixture_world(name))
                self.assertTrue(root.findall('.//collision'))
                self.assertIsNone(root.find('.//heightmap'))
                self.assertEqual(fixture_world(name), fixture_world(name))

    def test_dropoff_has_a_real_physical_gap_and_no_paint_covering_it(self):
        root = ET.fromstring(fixture_world('dropoff'))
        for model in root.findall('.//model'):
            x, _, z, *_ = map(float, model.findtext('pose').split())
            if 1. < x < 2.:
                self.assertLess(z, -.5)

    def test_narrow_gap_is_less_than_the_padded_rover_width(self):
        root = ET.fromstring(fixture_world('narrow_gap'))
        wall = root.find('.//model[@name="gap_wall_1"]')
        y = float(wall.findtext('pose').split()[1])
        width = float(wall.findtext('.//collision/geometry/box/size').split()[1])
        self.assertAlmostEqual(2*y-width, .35)
        self.assertLess(2*y-width, .34+2*.025)

    def test_unknown_fixture_rejected(self):
        with self.assertRaises(ValueError):
            fixture_world('not-a-fixture')

    def test_flat_edge_preserves_air_gap_and_launch_disk(self):
        root = ET.fromstring(fixture_world('flat_edge'))
        platform = root.find('.//model[@name="ground"]')
        self.assertEqual(platform.findtext('.//collision/geometry/box/size'), '3.0 10.0 0.2')
        for model in root.findall('.//model'):
            x, _, z, *_ = map(float, model.findtext('pose').split())
            if model.attrib['name'].startswith('paint_'):
                self.assertLessEqual(abs(x), 1.25)
            if model.attrib['name'].startswith('backdrop'):
                self.assertGreater(x, 2.9)
                self.assertGreater(z, 0.)
