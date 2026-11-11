"""An isolated Harmonic RGB-D world, RTAB-Map, guarded Nav2 and mission services."""
import copy
import json
import math
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from mars_training_environment.generate import write_world
    from mars_training_environment.robot import live_camera_config
    from rover_navigation.fixtures import fixture_world
    from rover_navigation.coverage import ground_view
    source = Path(LaunchConfiguration('scene').perform(context)).resolve()
    try:
        demo_mode = LaunchConfiguration('demo_mode').perform(context) == 'true'
    except KeyError:  # historical source-only launch fixtures
        demo_mode = False
    speed = float(LaunchConfiguration('navigation_speed').perform(context))
    pitch = float(LaunchConfiguration('camera_pitch').perform(context))
    engine = LaunchConfiguration('sensor_render_engine').perform(context)
    rgb_far = float(LaunchConfiguration('camera_rgb_far').perform(context))
    camera_rate = float(LaunchConfiguration('camera_rate_hz').perform(context))
    if not math.isfinite(camera_rate) or not 10 <= camera_rate <= 30:
        raise ValueError('camera_rate_hz must be finite and in [10, 30] Hz')
    if not math.isfinite(rgb_far) or not 8 <= rgb_far <= 20:
        raise ValueError('camera_rgb_far must be finite and in [8, 20] metres')
    if engine not in ('ogre2', 'ogre'):
        raise ValueError('sensor_render_engine must be ogre2 or ogre')
    if engine == 'ogre' and LaunchConfiguration('sensor_headless').perform(context) == 'true':
        raise ValueError('Ogre sensor rendering requires sensor_headless:=false and an active display')
    if not math.isfinite(pitch) or not 0 <= pitch < math.pi/2:
        raise ValueError('camera_pitch must be finite and in [0, pi/2) radians')
    if not math.isfinite(speed) or not 0 < speed <= .2:
        raise ValueError('navigation_speed must be in (0, 0.20] m/s')
    if demo_mode and (speed>.10 or camera_rate!=15 or LaunchConfiguration('database').perform(context)):
        raise ValueError('Demo requires a fresh map, 15 Hz RGB-D and speed <=0.10 m/s')
    from rover_navigation.manual_input import ManualInput
    initial=float(LaunchConfiguration('manual_initial_linear_speed').perform(context))
    ManualInput(max_linear=float(LaunchConfiguration('manual_max_linear_speed').perform(context)),
                max_angular=float(LaunchConfiguration('manual_max_angular_speed').perform(context)),
                initial_linear=None if initial==-1 else initial)
    run = Path(LaunchConfiguration('session').perform(context)).expanduser().resolve()
    if run.exists():
        raise ValueError('Use a new session directory; map/session evidence is never overwritten')
    database = LaunchConfiguration('database').perform(context)
    if database and not Path(database).expanduser().is_file():
        raise ValueError('Localization requires an existing RTAB-Map database')
    run.mkdir(parents=True)
    scene = copy.deepcopy(json.loads(source.read_text()))
    from mars_training_environment.environment_v2 import operating_map_size
    scene['operating_map_size_m']=operating_map_size(scene,LaunchConfiguration('map_size_m').perform(context))
    scene['config'] = live_camera_config(scene['config'], 960)
    scene['config']['camera']['rate_hz'] = camera_rate
    # A navigation-only pitch closes the near-ground gap from the original
    # high/mildly tilted camera. Capture/live profiles and the source scene stay intact.
    scene['config']['camera']['tilt_rad'] = pitch
    scene['config']['camera']['rgb_far_m'] = rgb_far
    scene['config']['camera']['far_clip_mode'] = 'axial' if engine == 'ogre' else 'radial'
    scene['config']['camera']['near_clip_mode'] = 'occlude' if engine == 'ogre' else 'clip'
    scene['config']['camera']['render_engine'] = engine
    ground_view(scene['config']['camera'])  # validate the complete experiment geometry
    bundle = source.parents[2]
    assets = json.loads((bundle/'assets/catalog.json').read_text())
    fixture = LaunchConfiguration('fixture').perform(context)
    terrain_collision = LaunchConfiguration('terrain_collision').perform(context)
    if terrain_collision not in ('visual', 'scene'):
        raise ValueError('terrain_collision must be visual or scene')
    if fixture:
        (run/'world.sdf').write_text(fixture_world(fixture))
    else:
        write_world(bundle, scene, assets, output_path=run/'world.sdf')
    world_xml = ET.parse(run/'world.sdf')
    terrain = world_xml.getroot().find("world/model[@name='mars_terrain']/link")
    if terrain is not None and terrain_collision == 'visual':
        # The camera measures the visual mesh; the scene's coarser collision
        # mesh is smoother, so the wheels would not meet the slopes navigation
        # avoids. Collide with the surface the camera sees.
        terrain.find('collision/geometry/mesh/uri').text = terrain.find('visual/geometry/mesh/uri').text
    ET.SubElement(world_xml.getroot().find('world'), 'plugin',
                  {'filename': 'gz-sim-contact-system', 'name': 'gz::sim::systems::Contact'})
    world_xml.write(run/'world.sdf', encoding='utf-8', xml_declaration=True)
    if database:
        shutil.copy2(Path(database).expanduser(), run/'rtabmap.db')
    return build_nodes(context, scene, run, source, database)


def build_nodes(context, scene, run, source, database):
    from mars_training_environment.robot import robot_description
    from rover_navigation.core import DRIVE_CENTER_X, FOOTPRINT
    from rover_navigation.coverage import ground_view
    camera = scene['config']['camera']
    try: demo_mode = LaunchConfiguration('demo_mode').perform(context) == 'true'
    except KeyError: demo_mode = False
    demo_profile = LaunchConfiguration('demo_profile').perform(context) if demo_mode else 'full'
    description = robot_description(scene['config'], navigation=True, drive_center_x=DRIVE_CENTER_X)
    (run/'robot.urdf').write_text(description)
    (run/'session.json').write_text(json.dumps(dict(scene=str(source), camera=camera, database=database,
                                                   container_image=os.environ.get('NAV_IMAGE', 'rover-navigation'),
                                                   install_relative=os.environ.get('NAV_INSTALL_REL', 'install_navigation'),
                                                   predicted_ground_view=ground_view(camera),
                                                   drive_center_x=DRIVE_CENTER_X,
                                                   terrain_collision=LaunchConfiguration('terrain_collision').perform(context),
                                                   terrain_backend=LaunchConfiguration('terrain_backend').perform(context),
                                                   terrain_url=LaunchConfiguration('terrain_url').perform(context),
                                                   adaptive_terrain=LaunchConfiguration('adaptive_terrain').perform(context),
                                                   sensor_headless=LaunchConfiguration('sensor_headless').perform(context),
                                                   sensor_render_engine=LaunchConfiguration('sensor_render_engine').perform(context),
                                                   fixture=LaunchConfiguration('fixture').perform(context),
                                                   navigation_speed=float(LaunchConfiguration('navigation_speed').perform(context)),
                                                   manual_max_linear_speed=float(LaunchConfiguration('manual_max_linear_speed').perform(context)),
                                                   manual_max_angular_speed=float(LaunchConfiguration('manual_max_angular_speed').perform(context)),
                                                   manual_initial_linear_speed=float(LaunchConfiguration('manual_initial_linear_speed').perform(context)),
                                                   map_size_m=scene['operating_map_size_m'],
                                                   demo_mode=demo_mode,demo_profile=demo_profile,
                                                   gui=LaunchConfiguration('gui').perform(context),
                                                   rviz=LaunchConfiguration('rviz').perform(context),
                                                   mission_controls=LaunchConfiguration('mission_controls').perform(context),
                                                   detector_model=LaunchConfiguration('detector_model').perform(context),
                                                   detector_backend=LaunchConfiguration('detector_backend').perform(context),
                                                   detector_url=LaunchConfiguration('detector_url').perform(context),
                                                   mapping_on_confirm=LaunchConfiguration('mapping_on_confirm').perform(context),
                                                   fastdds_profile=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')), indent=2))
    bridge_entries = []
    for ros, gz, ros_type, gz_type in (
        ('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
        ('/odom_gt', '/odom_gt', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        ('/navigation/chassis_contacts', '/navigation/chassis_contacts', 'ros_gz_interfaces/msg/Contacts', 'gz.msgs.Contacts'),
        (camera['image_topic'], '/oak/image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
        (camera['depth_topic'], '/oak/depth_image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
        (camera['camera_info_topic'], '/oak/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')):
        bridge_entries.append(dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=ros_type,
                                   gz_type_name=gz_type, direction='GZ_TO_ROS'))
    (run/'bridge.yaml').write_text(yaml.safe_dump(bridge_entries))
    share = Path(get_package_share_directory('rover_navigation'))
    params = yaml.safe_load((share/'config/nav2.yaml').read_text())
    lattice = Path(get_package_share_directory('nav2_smac_planner'))/'sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json'
    if not lattice.is_file():
        raise ValueError('Installed Nav2 lacks Jazzy differential-drive lattice primitives')
    params['planner_server']['ros__parameters']['GridBased']['lattice_filepath'] = str(lattice)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(share/'config/navigate.xml')
    params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = float(LaunchConfiguration('navigation_speed').perform(context))
    front, rear, half = (round(v, 4) for v in FOOTPRINT)
    footprint = json.dumps([[front, half], [front, -half], [rear, -half], [rear, half]])
    params['bt_navigator']['ros__parameters']['robot_base_frame'] = 'drive_center'
    for costmap in ('local_costmap', 'global_costmap'):
        configured = params[costmap][costmap]['ros__parameters']
        configured['robot_base_frame'], configured['footprint'] = 'drive_center', footprint
    (run/'nav2.yaml').write_text(yaml.safe_dump(params))
    sim = {'use_sim_time': True}
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r '+('' if LaunchConfiguration('gui').perform(context) == 'true' else '-s ')+
                          ('--headless-rendering ' if LaunchConfiguration('sensor_headless').perform(context) == 'true' else '')+
                          '--render-engine-server '+LaunchConfiguration('sensor_render_engine').perform(context)+' '+str(run/'world.sdf'),
                          'on_exit_shutdown': 'true'}.items())
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[sim, {'config_file': str(run/'bridge.yaml')}])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[sim, {'robot_description': description}])
    spawn = Node(package='ros_gz_sim', executable='create', arguments=['-topic', 'robot_description', '-name', 'four_wheel_bot', '-z', '.09'])
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    # Harmonic turns this rover about its front axle without wheel slip, so the
    # shared 1.5 multiplier would turn 1.5x the rate the guard's swept check assumes.
    (run/'controllers_navigation.yaml').write_text(yaml.safe_dump(
        {'diff_drive_controller': {'ros__parameters': {'wheel_separation_multiplier': 1.0}}}))
    wheels = Node(package='controller_manager', executable='spawner',
                  arguments=['diff_drive_controller', '--param-file', str(run/'controllers_navigation.yaml')])
    stamper = Node(package='four_wheel_sim', executable='twist_stamper',
                   parameters=[sim, {'watchdog_timeout_wall_sec': .35}])
    remaps = [('rgb/image', camera['image_topic']), ('depth/image', camera['depth_topic']),
              ('rgb/camera_info', camera['camera_info_topic'])]
    vo = Node(package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', parameters=[sim, {
        'frame_id': 'base_link', 'odom_frame_id': 'odom', 'publish_tf': True,
        'approx_sync': True, 'approx_sync_max_interval': .015,
        'qos': 2, 'qos_camera_info': 2, 'Odom/ResetCountdown': '0'}], remappings=remaps+[('odom', '/odom')])
    slam = Node(package='rtabmap_slam', executable='rtabmap', namespace='rtabmap', parameters=[sim, {
        'frame_id': 'base_link', 'odom_frame_id': 'odom', 'map_frame_id': 'map',
        'subscribe_depth': True, 'approx_sync': True, 'approx_sync_max_interval': .015,
        'qos_image': 2, 'qos_camera_info': 2,
        'database_path': str(run/'rtabmap.db'),
        'Mem/IncrementalMemory': 'false' if database else 'true',
        'Mem/InitWMWithAllNodes': 'true' if database else 'false',
        'Grid/FromDepth': 'true', 'Grid/CellSize': '.05', 'Grid/RangeMax': '8.0',
        'cloud_decimation': 3, 'cloud_voxel_size': .025,
        'Grid/NormalsSegmentation': 'true'}], remappings=remaps+[('odom', '/odom'), ('grid_map', '/map')])
    nodes = [gazebo, bridge, rsp, spawn, stamper, vo, slam,
             RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
             RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[wheels]))]
    for name in ('terrain', 'motion_guard', 'mission'):
        options = {'budget_seconds': float(LaunchConfiguration('budget_seconds').perform(context)),
                   'navigation_speed': float(LaunchConfiguration('navigation_speed').perform(context)),
                   'waypoints': LaunchConfiguration('waypoints').perform(context)} if name == 'mission' else {}
        if name == 'terrain':
            options['demo_mode']=demo_mode
            options['map_size_m']=scene['operating_map_size_m']
            options['mapping_on_confirm'] = LaunchConfiguration('mapping_on_confirm').perform(context) == 'true'
            options['terrain_backend'] = LaunchConfiguration('terrain_backend').perform(context)
            options['terrain_url'] = LaunchConfiguration('terrain_url').perform(context)
            options['adaptive_terrain'] = LaunchConfiguration('adaptive_terrain').perform(context) == 'true'
            options['evidence_output'] = str(run/'terrain-evidence-events.jsonl')
        if name=='motion_guard' and demo_mode:
            options.update(max_linear_speed=.1,max_angular_speed=.3)
        if name=='mission' and demo_mode:
            options.update(session=str(run),demo_output=str(run/'demo-events.jsonl'),demo_profile=demo_profile)
        nodes.append(Node(package='rover_navigation', executable='demo_mission' if name=='mission' and demo_mode else name,
                          output='screen', parameters=[sim, options]))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='navigation_rviz',
                          arguments=['-d', str(share/'config/navigation.rviz')], parameters=[sim],
                          additional_env={'LIBGL_ALWAYS_SOFTWARE': LaunchConfiguration('rviz_software_rendering').perform(context)}))
    if LaunchConfiguration('mission_controls').perform(context) == 'true':
        nodes.append(Node(package='rover_navigation', executable='mission_controls', output='screen',
                          parameters=[{'session': str(run), 'port': int(LaunchConfiguration('controls_port').perform(context)),
                                       'demo_mode':demo_mode,
                                       'platform_temperature_file':str(run.parent/'demo-platform-temperature.json') if demo_mode else '',
                                       'max_linear_speed':float(LaunchConfiguration('manual_max_linear_speed').perform(context)),
                                       'initial_linear_speed':float(LaunchConfiguration('manual_initial_linear_speed').perform(context)),
                                       'max_angular_speed':float(LaunchConfiguration('manual_max_angular_speed').perform(context))}]))
        nodes.append(Node(package='rover_navigation', executable='evaluate', output='screen',
                          parameters=[sim, {'output': str(run/'mission-events.jsonl')}]))
    if LaunchConfiguration('preview').perform(context) == 'true':
        # The existing gazebo control mode is camera-only in LiveBridge; no
        # GazeboDrive node is launched here. This viewer never publishes motion.
        nodes.append(Node(package='rover_perception', executable='live_bridge', name='navigation_camera_viewer',
                          parameters=[{'image_topic': camera['image_topic'], 'port': 8773,
                                       'control_mode': 'gazebo', 'use_sim_time': False}]))
    detector_model = LaunchConfiguration('detector_model').perform(context)
    if detector_model:
        backend = LaunchConfiguration('detector_backend').perform(context)
        if backend not in ('ultralytics', 'remote_http'):
            raise ValueError('A configured detector requires ultralytics or remote_http; none is not inference')
        nodes.append(Node(package='rover_perception', executable='rock_detector', name='rock_detector', output='screen',
                          parameters=[sim, {'image_topic': camera['image_topic'], 'backend': backend,
                                            'model_path': detector_model,
                                            'detector_url': LaunchConfiguration('detector_url').perform(context)}]))
        nodes.append(Node(package='rover_navigation', executable='sample_sites', output='screen',
                          parameters=[sim, {'output': str(run/'sample-sites.json'),'demo_mode':demo_mode}]))
    lifecycle_nodes = ['planner_server', 'controller_server', 'bt_navigator']
    for name in lifecycle_nodes:
        nodes.append(Node(package='nav2_'+{'planner_server': 'planner', 'controller_server': 'controller', 'bt_navigator': 'bt_navigator'}[name],
                          executable=name, name=name, output='screen', parameters=[str(run/'nav2.yaml')],
                          remappings=[('cmd_vel', '/navigation/autonomous_cmd_vel')]))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                      parameters=[sim, {'autostart': True, 'node_names': lifecycle_nodes}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene'), DeclareLaunchArgument('session'),
        DeclareLaunchArgument('database', default_value=''),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('sensor_headless', default_value='true'),
        DeclareLaunchArgument('sensor_render_engine', default_value='ogre2'),
        DeclareLaunchArgument('adaptive_terrain', default_value='true'),
        DeclareLaunchArgument('camera_pitch', default_value='0.4'),
        DeclareLaunchArgument('camera_rgb_far', default_value='8'),
        DeclareLaunchArgument('camera_rate_hz', default_value='30'),
        DeclareLaunchArgument('fixture', default_value=''),
        DeclareLaunchArgument('terrain_collision', default_value='visual'),
        DeclareLaunchArgument('preview', default_value='false'),
        DeclareLaunchArgument('terrain_backend', default_value='rocm_http'),
        DeclareLaunchArgument('terrain_url', default_value='http://127.0.0.1:8782'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_software_rendering', default_value='1'),
        DeclareLaunchArgument('mission_controls', default_value='false'),
        DeclareLaunchArgument('controls_port', default_value='8774'),
        DeclareLaunchArgument('manual_max_linear_speed', default_value='0.50'),
        DeclareLaunchArgument('manual_initial_linear_speed', default_value='-1'),
        DeclareLaunchArgument('map_size_m', default_value='auto'),
        DeclareLaunchArgument('manual_max_angular_speed', default_value='0.60'),
        DeclareLaunchArgument('mapping_on_confirm', default_value='false'),
        DeclareLaunchArgument('navigation_speed', default_value='0.10'),
        DeclareLaunchArgument('detector_model', default_value=''),
        DeclareLaunchArgument('detector_backend', default_value='remote_http'),
        DeclareLaunchArgument('detector_url', default_value='http://127.0.0.1:8775'),
        DeclareLaunchArgument('waypoints', default_value=''),
        DeclareLaunchArgument('demo_mode', default_value='false'),
        DeclareLaunchArgument('demo_profile', default_value='full'),
        DeclareLaunchArgument('budget_seconds', default_value='600.0'), OpaqueFunction(function=setup)])
