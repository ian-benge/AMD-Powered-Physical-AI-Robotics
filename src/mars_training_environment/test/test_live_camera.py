"""Real rover Xacro expansion for capture and the lighter browser-driving camera."""
import copy
import sys
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

from mars_training_environment.common import read_config
from mars_training_environment.robot import live_camera_config, robot_description
from mars_training_environment.environment_v2 import configuration


class LiveCameraTests(unittest.TestCase):
    def test_ogre_capture_omits_unsupported_sensor_and_keeps_rgbd_and_robot(self):
        root = Path(__file__).resolve().parents[3]
        packages = types.ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda name: str(root/'src'/name)
        config = configuration(read_config())
        with patch.dict(sys.modules, {'ament_index_python': types.ModuleType('ament_index_python'),
                                     'ament_index_python.packages': packages}):
            ogre = ET.fromstring(robot_description(config))
            config['camera']['render_engine'] = 'ogre2'
            legacy = ET.fromstring(robot_description(config))
        self.assertEqual([s.get('type') for s in ogre.findall('.//sensor')], ['rgbd_camera'])
        self.assertEqual([s.get('type') for s in legacy.findall('.//sensor')], ['rgbd_camera', 'boundingbox_camera'])
        self.assertEqual(ET.tostring(ogre.find('.//sensor')), ET.tostring(legacy.find('.//sensor')))
        self.assertEqual(ET.tostring(ogre.find('ros2_control')), ET.tostring(legacy.find('ros2_control')))
        self.assertEqual([ET.tostring(n) for n in ogre.findall('joint')],
                         [ET.tostring(n) for n in legacy.findall('joint')])

    def test_live_resolution_keeps_fov_and_saved_capture_config(self):
        original = read_config()
        saved = copy.deepcopy(original)
        live = live_camera_config(original)
        self.assertEqual((live['camera']['width'], live['camera']['height']), (960, 540))
        self.assertEqual(live['camera']['horizontal_fov_rad'], original['camera']['horizontal_fov_rad'])
        self.assertEqual(live['camera']['mount_xyz'], original['camera']['mount_xyz'])
        self.assertEqual(original, saved)
        self.assertEqual(live_camera_config(original, 640)['camera']['height'], 360)
        with self.assertRaises(ValueError):
            live_camera_config(original, 100)

    def test_real_robot_keeps_capture_sensors_and_uses_only_rgb_for_live(self):
        root = Path(__file__).resolve().parents[3]
        packages = types.ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda name: str(root / 'src' / name)
        with patch.dict(sys.modules, {'ament_index_python': types.ModuleType('ament_index_python'),
                                     'ament_index_python.packages': packages}):
            config = read_config()
            capture = ET.fromstring(robot_description(config))
            live = ET.fromstring(robot_description(live_camera_config(config), live=True))
        capture_sensors, live_sensors = capture.findall('.//sensor'), live.findall('.//sensor')
        self.assertEqual([s.get('type') for s in capture_sensors], ['rgbd_camera', 'boundingbox_camera'])
        self.assertEqual([s.get('type') for s in live_sensors], ['camera'])
        rgb = live_sensors[0]
        self.assertEqual(rgb.findtext('topic'), '/oak/image')
        self.assertEqual(rgb.findtext('camera/image/width'), '960')
        self.assertEqual(rgb.findtext('camera/image/height'), '540')
        self.assertIsNone(rgb.find('camera/depth_camera'))
        self.assertEqual(capture_sensors[0].findtext('camera/image/width'), str(config['camera']['width']))
        self.assertEqual([n.get('name') for n in capture.findall('joint')], [n.get('name') for n in live.findall('joint')])
        self.assertEqual(ET.tostring(capture.find('ros2_control')), ET.tostring(live.find('ros2_control')))

    def test_real_sensor_emits_independent_rgb_and_depth_clips(self):
        root = Path(__file__).resolve().parents[3]
        packages = types.ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda name: str(root / 'src' / name)
        config=read_config()
        config['camera']['rgb_far_m']=20
        with patch.dict(sys.modules, {'ament_index_python': types.ModuleType('ament_index_python'),
                                     'ament_index_python.packages': packages}):
            camera=ET.fromstring(robot_description(config,navigation=True)).find('.//sensor/camera')
        self.assertEqual(float(camera.findtext('clip/far')),20)
        self.assertEqual(float(camera.findtext('depth_camera/clip/far')),8)
        self.assertEqual(float(camera.findtext('clip/near')),.3)


if __name__ == '__main__':
    unittest.main()
