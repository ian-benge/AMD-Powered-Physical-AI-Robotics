#!/usr/bin/env python3
"""Deterministic evaluator-only Gazebo fixture; never connects to navigation.

Generate with the repository packages on PYTHONPATH. Run in its own
GZ_PARTITION, with no ROS bridges, detector, controller or navigation nodes.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from xml.sax.saxutils import escape

import cv2
import numpy as np

from mars_training_environment.meshes import sphere_mesh, write_dae


def generate(output, rgb_far=8., depth_far=8., asset_root=None):
    if not math.isfinite(rgb_far) or not 8 <= rgb_far <= 20 or depth_far != 8:
        raise ValueError('Fixture requires 8..20 m RGB and exactly 8 m usable depth')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    resource = Path(asset_root) if asset_root is not None else output
    # All pixels in these textures are known. RGBA channel presence alone
    # must not decide whether a surface needs blending.
    for name, alpha in [('opaque_rgb', None), ('opaque_rgba', 255), ('translucent', 100)]:
        image = np.zeros((32, 32, 3 if alpha is None else 4), np.uint8)
        image[:, :, :3] = [70, 120, 210]
        image[::4, :, :3] = [80, 145, 230]
        if alpha is not None:
            image[:, :, 3] = alpha
        cv2.imwrite(str(output/f'{name}.png'), image)
    vertices, faces = sphere_mesh(3)
    write_dae(output/'rock.dae', vertices, faces)
    items = []
    def box(name, xyz, size, color, texture='opaque_rgb'):
        items.append(dict(name=name, xyz=xyz, size=size, color=color, texture=texture, shape='box'))
    box('ground', [0,0,-.1], [50,40,.2], [.35,.20,.10])
    # Tall range targets reach above the horizon of the downward camera.
    for name,x,y in [('range4',4,-2.4),('range7',7,-2.2),('range9',9,-.8),
                     ('range15',15,2.4),('range19',19,5.7),('range21',21,12.6)]:
        box(name,[x,y,.8],[.6,.7,1.6],[.1,.8,.1])
    box('near_clipped',[.2,.15,.70],[.03,.02,.02],[1,0,0])
    box('overlap_front',[4,.3,.65],[.4,.5,.9],[.9,.1,.1], 'opaque_rgba')
    box('overlap_back',[5,.375,.65],[.6,.8,1.1],[.1,.1,.9])
    box('ridge',[5,-2.5,.55],[1,1,1.1],[.6,.3,.1])
    items.append(dict(name='buried_rock',xyz=[3,1.4,.05],size=[.7]*3,color=[.8,.8,.1],texture='opaque_rgba',shape='rock'))
    items.append(dict(name='occluded_rock',xyz=[7,-3.5,.2],size=[.7]*3,color=[.8,.1,.8],texture='opaque_rgb',shape='rock'))
    box('transparent_panel',[3,-.5,.8],[.05,.4,.8],[1,1,1], 'translucent')
    models=[]
    for item in items:
        name,xyz,size,color=(item[k] for k in ('name','xyz','size','color'))
        geometry = (f'<box><size>{" ".join(map(str,size))}</size></box>' if item['shape']=='box' else
                    f'<mesh><uri>{escape((resource/"rock.dae").as_posix())}</uri><scale>{" ".join(map(str,size))}</scale></mesh>')
        material=(f'<material><ambient>{" ".join(map(str,color))} 1</ambient><diffuse>{" ".join(map(str,color))} 1</diffuse>'
                  f'<pbr><metal><albedo_map>{escape((resource/(item["texture"]+".png")).as_posix())}</albedo_map>'
                  '<roughness>1</roughness><metalness>0</metalness></metal></pbr></material>')
        models.append(f'<model name="{name}"><static>true</static><pose>{" ".join(map(str,xyz))} 0 0 0</pose>'
                      f'<link name="link"><visual name="visual"><geometry>{geometry}</geometry>{material}</visual></link></model>')
    camera=dict(width=960,height=540,horizontal_fov_rad=1.22,near_m=.3,far_m=depth_far,rgb_far_m=rgb_far,
                near_clip_mode='occlude',
                xyz=[0,0,.762],rpy=[0,.4,0],rate_hz=30)
    world=f'''<sdf version="1.9"><world name="visibility_fixture">
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre</render_engine></plugin>
<physics name="default" type="dart"><max_step_size>.002</max_step_size><real_time_factor>1</real_time_factor></physics>
<gravity>0 0 0</gravity><scene><ambient>.8 .8 .8 1</ambient><background>.25 .4 .65 1</background><shadows>false</shadows></scene>
{''.join(models)}
<model name="fixture_camera"><pose>0 0 .762 0 .4 0</pose><link name="link"><kinematic>true</kinematic><gravity>false</gravity>
<sensor name="rgbd" type="rgbd_camera"><always_on>true</always_on><update_rate>30</update_rate><topic>/fixture</topic>
<camera><horizontal_fov>1.22</horizontal_fov><image><width>960</width><height>540</height><format>R8G8B8</format></image>
<clip><near>.3</near><far>{rgb_far}</far></clip><depth_camera><clip><near>.3</near><far>{depth_far}</far></clip></depth_camera>
</camera></sensor></link>
<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose>
<publish_link_pose>false</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>30</update_frequency></plugin>
</model></world></sdf>'''
    (output/'world.sdf').write_text(world)
    document=dict(version=3,evaluator_only=True,camera=camera,objects=items,background_rgb=[.25,.4,.65],
                  rock_vertices=vertices.tolist(),rock_faces=faces.tolist(),
                  asset_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted(output.iterdir()) if p.suffix in ('.png','.dae')},
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output/'fixture.json').write_text(json.dumps(document,indent=2))
    return document


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--rgb-far',type=float,default=8)
    parser.add_argument('--asset-root',type=Path)
    args=parser.parse_args()
    generate(args.output,args.rgb_far,asset_root=args.asset_root)
