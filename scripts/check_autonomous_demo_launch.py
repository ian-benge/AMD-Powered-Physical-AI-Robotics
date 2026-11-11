#!/usr/bin/env python3
"""Construct the actual ROS launch graph, without executing its actions."""
import argparse
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--representative',type=int,required=True)
    p.add_argument('--fixture',choices=('flat',),default='')
    p.add_argument('--gui',choices=('false','true'),default='false')
    a=p.parse_args();app=Path('/work')
    config=json.loads((app/'DEMO_CONFIGURATION.json').read_text());selected=config['qualified_scenes'][a.representative-1]
    path=app/'source/src/rover_navigation/launch/navigation.launch.py'
    spec=importlib.util.spec_from_file_location('actual_demo_launch',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext();context.launch_configurations.update(scene='/scenes/scenes/'+selected['scene']+'/scene.json',
        session='/work/checks/actual-launch-'+str(a.representative)+'-gui-'+a.gui+('-contact' if a.fixture else ''))
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument) and action.name not in context.launch_configurations:
            context.launch_configurations[action.name]=perform_substitutions(context,action.default_value)
    context.launch_configurations.update(demo_mode='false' if a.fixture else 'true',fixture=a.fixture,demo_profile='rehearsal',camera_rate_hz='15',
        navigation_speed='0.05',manual_max_linear_speed='0.10',manual_max_angular_speed='0.30',manual_initial_linear_speed='0.05',
        sensor_render_engine='ogre',sensor_headless='false',camera_rgb_far='20',gui=a.gui,rviz='false',
        preview='true',mission_controls='true',mapping_on_confirm='true',terrain_collision='scene',
        detector_model=config['checkpoint'],detector_url='http://127.0.0.1:8778')
    actions=module.setup(context)
    output=Path(context.launch_configurations['session']);session=json.loads((output/'session.json').read_text())
    assert session['demo_mode']==(not bool(a.fixture)) and session['camera']['rate_hz']==15 and session['map_size_m']==40.
    assert session['fixture']==a.fixture
    assert session['database']=='' and session['navigation_speed']==.05 and session['camera']['render_engine']=='ogre'
    assert session['gui']==a.gui and session['rviz']=='false' and session['sensor_headless']=='false'
    gazebo=next(action for action in actions if hasattr(action,'launch_arguments') and
                'gz_args' in dict(action.launch_arguments))
    argument=dict(gazebo.launch_arguments)['gz_args']
    gz_args=argument if isinstance(argument,str) else perform_substitutions(context,argument)
    assert ('-s ' in gz_args)==(a.gui=='false')
    assert '--headless-rendering' not in gz_args and '--render-engine-server ogre ' in gz_args
    sensor=ET.parse(output/'robot.urdf').find('.//sensor');assert float(sensor.findtext('update_rate'))==15.
    (output/'LAUNCH_GRAPH_REVIEW.json').write_text(json.dumps(dict(result='ACTUAL_LAUNCH_CONSTRUCTION_PASS',
        actions=len(actions),scene=selected['scene'],fixture=a.fixture,gui=a.gui,sensor_headless=False,
        rendering_started=False,motion_requested=False),indent=2)+'\n')
    print('ACTUAL_DEMO_LAUNCH_GRAPH_PASS',selected['density'],len(actions))


if __name__=='__main__':main()
