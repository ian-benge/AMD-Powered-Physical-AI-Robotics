import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from mars_training_environment.common import read_config
from mars_training_environment.robot import live_camera_config, robot_description


ROOT = Path(__file__).resolve().parents[3]


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class LaunchTests(unittest.TestCase):
    def stubs(self, folder):
        modules = {name: types.ModuleType(name) for name in (
            'ament_index_python', 'ament_index_python.packages', 'launch', 'launch.actions',
            'launch.event_handlers', 'launch.launch_description_sources', 'launch.substitutions',
            'launch_ros', 'launch_ros.actions')}
        modules['ament_index_python.packages'].get_package_share_directory = lambda name: str(
            folder/name if name in ('nav2_smac_planner', 'ros_gz_sim') else ROOT/'src'/name)
        modules['launch'].LaunchDescription = list
        for name in ('DeclareLaunchArgument', 'OpaqueFunction', 'RegisterEventHandler'):
            setattr(modules['launch.actions'], name, lambda *args, **kw: dict(kind='action', args=args, **kw))
        modules['launch.actions'].IncludeLaunchDescription = lambda *args, **kw: dict(kind='world', **kw)
        modules['launch.event_handlers'].OnProcessExit = lambda **kw: kw
        modules['launch.launch_description_sources'].PythonLaunchDescriptionSource = str
        modules['launch.substitutions'].LaunchConfiguration = Configuration
        modules['launch_ros.actions'].Node = lambda **kw: dict(kind='node', **kw)
        return modules

    def test_isolated_world_sensor_profiles_and_truth_ownership(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory).resolve()
            lattice = folder/'nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json'
            lattice.parent.mkdir(parents=True)
            lattice.write_text('{}')
            source = folder/'bundle/scenes/scene_050000/scene.json'
            source.parent.mkdir(parents=True)
            assets = folder/'bundle/assets/catalog.json'
            assets.parent.mkdir()
            assets.write_text('[]')
            source.write_text(json.dumps(dict(id='scene_050000', config=read_config(), ground={'layers': []},
                                             objects=[], lighting=dict(azimuth=0., elevation=1., intensity=1., ambient=.4, sky_rgb=[.5]*3))))
            world = source.parent/'world.sdf'
            world.write_text('preserved-source-world')
            saved = source.read_bytes()
            context = dict(scene=str(source), session=str(folder/'run'), database='', gui='false', camera_pitch='0.4',
                           waypoints='[[1,0,0]]', budget_seconds='600.', fixture='', terrain_collision='visual', preview='false',
                           rviz='false', rviz_software_rendering='1', mission_controls='false', navigation_speed='0.10',
                           detector_model='', detector_backend='remote_http', detector_url='http://127.0.0.1:8775',
                           mapping_on_confirm='false', controls_port='8774', terrain_backend='cpu',
                           terrain_url='http://127.0.0.1:8782')
            spec = importlib.util.spec_from_file_location('tested_navigation', ROOT/'src/rover_navigation/launch/navigation.launch.py')
            launch = importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules, self.stubs(folder)):
                spec.loader.exec_module(launch)
                nodes = launch.setup(context)
                for invalid in ('nan', 'inf', '-0.1', str(math.pi/2)):
                    bad = dict(context, camera_pitch=invalid, session=str(folder/'invalid-pitch'))
                    with self.assertRaises(ValueError):
                        launch.setup(bad)
                    self.assertFalse((folder/'invalid-pitch').exists())
                database = folder/'saved.db'
                database.write_bytes(b'source-database')
                localization = dict(context, session=str(folder/'localized'), database=str(database), preview='true',
                                    rviz='true', mission_controls='true', terrain_collision='scene', detector_model='/trained.pt',
                                    mapping_on_confirm='true')
                localized_nodes = launch.setup(localization)
                with self.assertRaises(ValueError):
                    launch.setup(context)
                capture = ET.fromstring(robot_description(read_config()))
                live = ET.fromstring(robot_description(live_camera_config(read_config()), live=True))
                nav = ET.parse(folder/'run/robot.urdf').getroot()
            self.assertEqual(source.read_bytes(), saved)
            self.assertEqual(database.read_bytes(), b'source-database')
            self.assertEqual((folder/'localized/rtabmap.db').read_bytes(), b'source-database')
            viewer = next(n for n in localized_nodes if n.get('executable') == 'live_bridge')
            self.assertEqual(viewer['parameters'][0]['control_mode'], 'gazebo')
            self.assertIn('rviz2', [n.get('executable') for n in localized_nodes])
            self.assertIn('mission_controls', [n.get('executable') for n in localized_nodes])
            controls = next(n for n in localized_nodes if n.get('executable') == 'mission_controls')
            self.assertEqual(controls['parameters'][0]['session'], str(folder/'localized'))
            terrain = next(n for n in localized_nodes if n.get('executable') == 'terrain')
            self.assertTrue(terrain['parameters'][1]['mapping_on_confirm'])
            self.assertIn('evaluate', [n.get('executable') for n in localized_nodes])
            self.assertIn('sample_sites', [n.get('executable') for n in localized_nodes])
            detector = next(n for n in localized_nodes if n.get('executable') == 'rock_detector')
            self.assertEqual(detector['parameters'][1]['backend'], 'remote_http')
            self.assertEqual(detector['parameters'][1]['model_path'], '/trained.pt')
            self.assertNotIn('rock_detector', [n.get('executable') for n in nodes])
            nav_params = next(n for n in localized_nodes if n.get('executable') == 'planner_server')['parameters'][0]
            import yaml
            configured = yaml.safe_load(Path(nav_params).read_text())
            self.assertEqual(configured['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'], .1)
            from rover_navigation.core import DRIVE_CENTER_X, FOOTPRINT
            self.assertEqual(configured['bt_navigator']['ros__parameters']['robot_base_frame'], 'drive_center')
            for costmap in ('local_costmap', 'global_costmap'):
                values = configured[costmap][costmap]['ros__parameters']
                self.assertEqual(values['robot_base_frame'], 'drive_center')
                corners = json.loads(values['footprint'])
                self.assertEqual(max(c[0] for c in corners), round(FOOTPRINT[0], 4))
                self.assertEqual(min(c[0] for c in corners), round(FOOTPRINT[1], 4))
            self.assertEqual(json.loads((folder/'run/session.json').read_text())['drive_center_x'], DRIVE_CENTER_X)
            spawned = [n for a in nodes if a.get('kind') == 'action' for e in a.get('args', ())
                       if isinstance(e, dict) for n in e.get('on_exit', [])]
            wheels = next(n for n in spawned if n['arguments'][0] == 'diff_drive_controller')
            calibration = Path(wheels['arguments'][wheels['arguments'].index('--param-file')+1])
            self.assertEqual(calibration.parent, folder/'run')
            self.assertEqual(yaml.safe_load(calibration.read_text())['diff_drive_controller']['ros__parameters']
                             ['wheel_separation_multiplier'], 1.)
            slam = next(n for n in localized_nodes if n.get('executable') == 'rtabmap')
            self.assertEqual(slam['parameters'][1]['Mem/IncrementalMemory'], 'false')
            self.assertEqual(world.read_text(), 'preserved-source-world')
            def terrain_collision(session):
                link = ET.parse(folder/session/'world.sdf').getroot().find("world/model[@name='mars_terrain']/link")
                return link.findtext('collision/geometry/mesh/uri'), link.findtext('visual/geometry/mesh/uri')
            collision, visual = terrain_collision('run')
            self.assertEqual(collision, visual)  # physics meets the surface the camera measures
            self.assertEqual(visual, str(source.parent/'terrain.dae'))
            self.assertEqual(terrain_collision('localized')[0], str(source.parent/'terrain_collision.dae'))
            self.assertEqual(json.loads((folder/'run/session.json').read_text())['terrain_collision'], 'visual')
            self.assertEqual([s.get('type') for s in capture.findall('.//sensor')], ['rgbd_camera', 'boundingbox_camera'])
            self.assertEqual([s.get('type') for s in live.findall('.//sensor')], ['camera'])
            self.assertEqual([s.get('type') for s in nav.findall('.//sensor')], ['rgbd_camera', 'contact'])
            self.assertEqual(nav.findtext('.//sensor[@type="contact"]/contact/collision'), 'base_link_collision')
            self.assertEqual(nav.findtext('.//sensor[@type="contact"]/contact/topic'), '/navigation/chassis_contacts')
            self.assertEqual(nav.findtext('.//sensor/camera/image/width'), '960')
            centre = nav.find("joint[@name='drive_center_joint']")
            self.assertEqual((centre.find('parent').get('link'), centre.find('child').get('link')),
                             ('base_link', 'drive_center'))
            self.assertEqual(centre.find('origin').get('xyz'), '0.13 0 0')
            self.assertIsNone(capture.find("joint[@name='drive_center_joint']"))
            self.assertIsNone(live.find("joint[@name='drive_center_joint']"))
            executables = [n.get('executable') for n in nodes]
            self.assertNotIn('odom_gt', executables)
            self.assertNotIn('live_bridge', executables)
            self.assertNotIn('gazebo_drive', executables)
            self.assertIn('rgbd_odometry', executables)
            for sensor_node in (n for n in nodes if n.get('executable') in ('rgbd_odometry', 'rtabmap')):
                # Reject adjacent 30 Hz frames even when delivery drops/bursts.
                interval = sensor_node['parameters'][1]['approx_sync_max_interval']
                self.assertGreater(interval, 0.)
                self.assertLess(interval, 1./30.)
            publishers = [n for n in nodes if n.get('package', '').startswith('nav2_') and n.get('executable') != 'lifecycle_manager']
            self.assertTrue(all(('cmd_vel', '/navigation/autonomous_cmd_vel') in n['remappings'] for n in publishers))

    def test_profile_flags_cannot_silently_remove_depth(self):
        with self.assertRaises(ValueError):
            robot_description(read_config(), live=True, navigation=True)


if __name__ == '__main__':
    unittest.main()
