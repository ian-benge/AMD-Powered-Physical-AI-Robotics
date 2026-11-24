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
            context.update(adaptive_terrain='true', sensor_headless='true', sensor_render_engine='ogre2', camera_rgb_far='8')
            context.update(manual_max_linear_speed='0.50',manual_max_angular_speed='0.60')
            context.update(map_size_m='auto',manual_initial_linear_speed='-1',camera_rate_hz='30')
            spec = importlib.util.spec_from_file_location('tested_navigation', ROOT/'src/rover_navigation/launch/navigation.launch.py')
            launch = importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules, self.stubs(folder)):
                spec.loader.exec_module(launch)
                nodes = launch.setup(context)
                reduced = launch.setup(dict(context,camera_rate_hz='15',session=str(folder/'reduced-rate')))
                rate_sensor = ET.parse(folder/'reduced-rate/robot.urdf').find('.//sensor')
                self.assertEqual(float(rate_sensor.findtext('update_rate')),15.)
                saved_camera=json.loads((folder/'reduced-rate/session.json').read_text())['camera']
                self.assertEqual(saved_camera['rate_hz'],15.)
                self.assertEqual((saved_camera['width'],saved_camera['height']),(960,540))
                self.assertEqual(json.loads((folder/'run/session.json').read_text())['camera']['rate_hz'],30.)
                for invalid_rate in ('nan','inf','0','8','9','31'):
                    with self.assertRaises(ValueError):
                        launch.setup(dict(context,camera_rate_hz=invalid_rate,session=str(folder/'invalid-rate')))
                    self.assertFalse((folder/'invalid-rate').exists())
                for invalid in ('other', 'ogre'):
                    with self.assertRaises(ValueError):
                        launch.setup(dict(context, sensor_render_engine=invalid, session=str(folder/'invalid-renderer')))
                    self.assertFalse((folder/'invalid-renderer').exists())
                alternate = launch.setup(dict(context, gui='true', sensor_render_engine='ogre', sensor_headless='false',
                                              session=str(folder/'renderer-check')))
                alternate_world = next(n for n in alternate if n.get('kind') == 'world')
                alternate_args = dict(alternate_world['launch_arguments'])['gz_args']
                self.assertIn('--render-engine-server ogre ', alternate_args)
                self.assertNotIn('--headless-rendering', alternate_args)
                extended = launch.setup(dict(context, sensor_render_engine='ogre', sensor_headless='false',
                                              camera_rgb_far='20', session=str(folder/'extended-rgb')))
                extended_camera=ET.parse(folder/'extended-rgb/robot.urdf').find('.//sensor/camera')
                self.assertEqual(float(extended_camera.findtext('clip/far')),20)
                self.assertEqual(float(extended_camera.findtext('depth_camera/clip/far')),8)
                self.assertEqual(json.loads((folder/'extended-rgb/session.json').read_text())['camera']['far_clip_mode'],'axial')
                for invalid in ('nan','inf','7','21'):
                    with self.assertRaises(ValueError):
                        launch.setup(dict(context,camera_rgb_far=invalid,session=str(folder/'invalid-rgb')))
                for invalid in ('nan', 'inf', '-0.1', str(math.pi/2)):
                    bad = dict(context, camera_pitch=invalid, session=str(folder/'invalid-pitch'))
                    with self.assertRaises(ValueError):
                        launch.setup(bad)
                    self.assertFalse((folder/'invalid-pitch').exists())
                database = folder/'saved.db'
                database.write_bytes(b'source-database')
                localization = dict(context, session=str(folder/'localized'), database=str(database), preview='true',
                                    gui='true', rviz='true', mission_controls='true', terrain_collision='scene', detector_model='/trained.pt',
                                    mapping_on_confirm='true')
                localized_nodes = launch.setup(localization)
                demo=launch.setup(dict(localization,demo_mode='true',demo_profile='rehearsal',map_size_m='40',
                    database='',camera_rate_hz='15',navigation_speed='0.15',manual_max_linear_speed='0.15',
                    session=str(folder/'demo')))
                demo_mission=next(n for n in demo if n.get('executable')=='demo_mission')
                legacy_slam=next(n for n in localized_nodes if n.get('executable')=='rtabmap')
                demo_executables={n.get('executable') for n in demo}
                self.assertTrue({'rgbd_odometry','terrain','sample_sites','mission_controls'} <= demo_executables)
                self.assertFalse({'rtabmap','live_bridge'} & demo_executables)
                self.assertTrue(any(n.get('executable')=='live_bridge' for n in localized_nodes))
                self.assertTrue(legacy_slam['parameters'][1]['publish_tf'])
                self.assertIn(('grid_map', '/map'), legacy_slam['remappings'])
                demo_session=json.loads((folder/'demo/session.json').read_text())
                self.assertFalse(demo_session['slam_graph_enabled'])
                self.assertEqual(demo_session['map_edge_setback_m'],3.)
                self.assertIn(('/navigation/terrain_map','/navigation/demo_planning_map'),demo_mission['remappings'])
                anchors=[n for n in demo if n.get('executable')=='static_transform_publisher']
                self.assertEqual(len(anchors), 1)
                self.assertEqual(anchors[0]['arguments'], ['--frame-id', 'map', '--child-frame-id', 'odom'])
                self.assertFalse(any(n.get('executable')=='static_transform_publisher' for n in localized_nodes))
                guard=next(n for n in demo if n.get('executable')=='motion_guard')['parameters'][1]
                self.assertTrue(guard['fixed_navigation_map'])
                self.assertEqual(guard['max_linear_speed'], .15)
                self.assertEqual(demo_mission['parameters'][1]['navigation_speed'], .15)
                self.assertEqual(demo_mission['parameters'][1]['demo_profile'],'rehearsal')
                self.assertEqual(demo_mission['parameters'][1]['session'],str(folder/'demo'))
                import yaml
                demo_nav = yaml.safe_load((folder/'demo/nav2.yaml').read_text())
                self.assertEqual(demo_nav['global_costmap']['global_costmap']['ros__parameters']['ground']['map_topic'],
                                 '/navigation/demo_planning_map')
                demo_controller = demo_nav['controller_server']['ros__parameters']
                self.assertEqual(demo_controller['FollowPath']['plugin'],
                                 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
                self.assertTrue(demo_controller['FollowPath']['use_collision_detection'])
                self.assertEqual(demo_controller['FollowPath']['rotate_to_heading_angular_vel'], .30)
                self.assertEqual(demo_controller['FollowPath']['desired_linear_vel'], .15)
                self.assertIs(demo_controller['FollowPath']['stateful'], False)
                self.assertIs(demo_controller['goal_checker']['stateful'], False)
                self.assertEqual(demo_controller['goal_checker']['xy_goal_tolerance'], .06)
                self.assertEqual(demo_controller['goal_checker']['yaw_goal_tolerance'], .087)
                self.assertEqual(demo_controller['goal_checker']['plugin'],'nav2_controller::StoppedGoalChecker')
                for purpose,radius,yaw in (('frontier',.22,.28),('clearance',.12,.21)):
                    checker=purpose+'_goal_checker'
                    self.assertEqual(demo_controller[checker]['plugin'],'nav2_controller::SimpleGoalChecker')
                    self.assertEqual(demo_controller[checker]['xy_goal_tolerance'], radius)
                    self.assertEqual(demo_controller[checker]['yaw_goal_tolerance'], yaw)
                    tree=Path(demo_mission['parameters'][1][purpose+'_bt'])
                    self.assertEqual(ET.parse(tree).find('.//FollowPath').get('goal_checker_id'),checker)
                with self.assertRaises(ValueError):
                    launch.setup(dict(localization,demo_mode='true',database='',camera_rate_hz='10',
                                      navigation_speed='0.16',session=str(folder/'demo-too-fast')))
                self.assertEqual(demo_controller['progress_checker']['movement_time_allowance'], 20.)
                front = ('0.9', '0.8', '1 0 0')
                for run, rear in (('localized', front), ('demo', ('0.25', '0.8', '0 1 0'))):
                    robot = ET.parse(folder/run/'robot.urdf')
                    for wheel, expected in (('front_left', front), ('front_right', front),
                                            ('rear_left', rear), ('rear_right', rear)):
                        contact = robot.find(f"gazebo[@reference='{wheel}_wheel']")
                        self.assertEqual(tuple(contact.findtext(k) for k in ('mu1', 'mu2', 'fdir1')), expected)
                for executable in ('terrain','sample_sites','mission_controls'):
                    node=next(n for n in demo if n.get('executable')==executable)
                    self.assertTrue(any(p.get('demo_mode') is True for p in node['parameters'] if isinstance(p,dict)))
                for rate in ('8','10'):
                    run=folder/f'demo-{rate}hz'
                    launch.setup(dict(localization,demo_mode='true',demo_profile='rehearsal',database='',
                        camera_rate_hz=rate,gui='false',rviz='false',session=str(run)))
                    low_sensor=ET.parse(run/'robot.urdf').find('.//sensor')
                    self.assertEqual(float(low_sensor.findtext('update_rate')),float(rate))
                    self.assertEqual(json.loads((run/'session.json').read_text())['camera']['rate_hz'],float(rate))
                for unsupported in ('7','9','11','14','30'):
                    with self.assertRaises(ValueError):launch.setup(dict(localization,demo_mode='true',database='',
                        camera_rate_hz=unsupported,session=str(folder/'demo-unsupported')))
                with self.assertRaises(ValueError):launch.setup(dict(localization,demo_mode='true',demo_profile='full',camera_rate_hz='15',session=str(folder/'demo-invalid')))
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
            world_launch = next(n for n in localized_nodes if n.get('kind') == 'world')
            self.assertIn('--headless-rendering', dict(world_launch['launch_arguments'])['gz_args'])
            self.assertNotIn('-s ', dict(world_launch['launch_arguments'])['gz_args'])
            terrain_node = next(n for n in localized_nodes if n.get('executable') == 'terrain')
            self.assertTrue(terrain_node['parameters'][1]['adaptive_terrain'])
            self.assertTrue(terrain_node['parameters'][1]['evidence_output'].endswith('terrain-evidence-events.jsonl'))
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
            self.assertNotIn('stateful', configured['controller_server']['ros__parameters']['FollowPath'])
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
