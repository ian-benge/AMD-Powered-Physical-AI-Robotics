"""Deterministic physical hazard worlds for Gazebo acceptance, not map priors."""
import math
import random


NAMES = ('flat', 'flat_edge', 'slope', 'step', 'dropoff', 'narrow_gap', 'blocked_return')


def fixture_world(name):
    if name not in NAMES:
        raise ValueError('Unknown navigation fixture: '+name)
    models = []

    def box(label, center, size, pitch=0.):
        xyz, dimensions = ' '.join(map(str, center)), ' '.join(map(str, size))
        models.append(f'''<model name="{label}"><static>true</static><pose>{xyz} 0 {pitch} 0</pose>
<link name="body"><collision name="solid"><geometry><box><size>{dimensions}</size></box></geometry></collision>
<visual name="surface"><geometry><box><size>{dimensions}</size></box></geometry>
<material><ambient>0.5 0.30 0.18 1</ambient><diffuse>0.5 0.30 0.18 1</diffuse></material></visual></link></model>''')

    if name == 'dropoff':
        box('launch_platform', (-2., 0., -.1), (6., 10., .2))
        box('far_platform', (3.5, 0., -.1), (3., 10., .2))
        box('pit_bottom', (1.5, 0., -.9), (1., 10., .2))
    elif name == 'flat_edge':
        # Clear 1.1 m launch disk, then a visible physical platform edge at x=1.5.
        box('ground', (0., 0., -.1), (3., 10., .2))
    else:
        box('ground', (0., 0., -.1), (10., 10., .2))
    if name == 'slope':
        angle = math.atan2(.6, 2.)
        box('steep_ramp', (2., 0., .3-.05*math.cos(angle)), (math.hypot(2., .6), 3., .1), -angle)
    if name == 'step':
        box('high_step', (2., 0., .075), (.8, 3., .15))
    if name == 'narrow_gap':
        for sign in (-1, 1):
            box('gap_wall_'+str(sign), (2., sign*2.5875, .35), (.2, 4.825, .7))
    if name == 'blocked_return':
        # Move this gate to y=0 through Gazebo's set_pose service AFTER the rover
        # passes x=1.5. There is deliberately no navigation-side ground truth hook.
        box('return_gate', (.8, 8., .35), (.3, 10., .7))
    # Seeded, nonrepeating visual ground details give RGB-D odometry image features.
    # They do not span the drop-off and introduce no physical obstacle geometry.
    rng = random.Random(19)
    if name == 'flat_edge':
        # A textured vertical backdrop gives visual odometry real RGB-D features.
        # It is separated from the platform by a 1.5 m air gap; it supplies no
        # floor or navigation prior across the edge under test.
        box('edge_backdrop', (3., 0., 1.5), (.04, 4., 3.))
        for i in range(19):
            for j in range(14):
                shade = rng.uniform(.08, .95)
                models.append(f'''<model name="backdrop_paint_{i}_{j}"><static>true</static><pose>2.979 {-1.8+i*.2} {.15+j*.2} 0 0 0</pose>
<link name="paint"><visual name="patch"><geometry><box><size>.001 .16 .16</size></box></geometry>
<material><ambient>{shade} {shade} {shade} 1</ambient>
<diffuse>{shade} {shade} {shade} 1</diffuse></material></visual></link></model>''')
    for i in range(20):
        for j in range(20):
            x, y = -4.75+i*.5, -4.75+j*.5
            if name == 'dropoff' and 1. <= x < 2.:
                continue
            if name == 'flat_edge' and abs(x) > 1.25:
                continue
            shade = rng.uniform(.6, 1.)
            models.append(f'''<model name="paint_{i}_{j}"><static>true</static><pose>{x} {y} .001 0 0 0</pose>
<link name="paint"><visual name="patch"><geometry><box><size>.45 .45 .001</size></box></geometry>
<material><ambient>{.5*shade} {.3*shade} {.18*shade} 1</ambient>
<diffuse>{.5*shade} {.3*shade} {.18*shade} 1</diffuse></material></visual></link></model>''')
    if name == 'flat_edge':
        # Distinctive near-ground corners provide translational parallax too;
        # a distant planar backdrop alone is a poor motion reference.
        for i in range(13):
            for j in range(25):
                shade = rng.uniform(.05, .95)
                x, y = -1.2+i*.2+rng.uniform(-.025, .025), -1.8+j*.15+rng.uniform(-.02, .02)
                models.append(f'''<model name="edge_detail_{i}_{j}"><static>true</static><pose>{x} {y} .002 0 0 0</pose>
<link name="paint"><visual name="patch"><geometry><box><size>.09 .08 .001</size></box></geometry>
<material><ambient>{shade} {shade} {shade} 1</ambient>
<diffuse>{shade} {shade} {shade} 1</diffuse></material></visual></link></model>''')
    return '''<sdf version="1.9"><world name="mars_sampling">
<physics name="default" type="dart"><max_step_size>.002</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<gravity>0 0 -9.81</gravity><scene><ambient>.5 .5 .5 1</ambient><background>.5 .4 .3 1</background></scene>
<light name="sun" type="directional"><direction>-.5 -.2 -1</direction><diffuse>.9 .85 .8 1</diffuse></light>
'''+''.join(models)+'</world></sdf>'
