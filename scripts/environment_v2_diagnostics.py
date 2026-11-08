#!/usr/bin/env python3
"""Evaluator-only fixed-camera diagnostics; never starts a rover or training capture.

The checked host guard must supervise the hardware subcommand. Scene geometry
is read only by this evaluator. Nothing is published to navigation or control.
"""
import argparse
from collections import OrderedDict,deque
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def prepare(bundle,output):
    from mars_training_environment.common import quaternion,write_json
    from mars_training_environment.generate import write_world
    from mars_training_environment.terrain import Terrain
    output.mkdir(parents=True,exist_ok=False)
    files=list((bundle/'scenes').glob('*/scene.json'))
    if len(files)!=1:raise ValueError('A single qualified representative is required')
    original=json.loads(files[0].read_text());assets=json.loads((bundle/'assets/catalog.json').read_text())
    cases=[]
    for route in original['reachability']['routes']:
        if route['category'].endswith(':near') or route['category']=='rock:outer':
            x,y=route['staging_xy'];tx,ty=route['target_xy'];yaw=math.atan2(ty-y,tx-x)
            cases.append((route['category'].replace(':','-'),original,[x,y],yaw))
    background=deepcopy(original);background.update(objects=[],regions=[],empty=True,dust_textures={},ground={'layers':[]})
    cases.append(('background',background,[12.,-12.],0.))
    records=[]
    for name,scene,xy,yaw in cases:
        folder=output/name;folder.mkdir()
        field=Terrain(scene['terrain']['heights'],40.)
        base=[*xy,float(field.height(*xy))+.08]
        camera=[base[0]+.2*math.cos(yaw),base[1]+.2*math.sin(yaw),base[2]+.682]
        world=folder/'world.sdf';write_world(bundle,scene,assets,output_path=world)
        tree=ET.parse(world);root=tree.getroot().find('world');root.set('name','visibility_fixture')
        model=ET.SubElement(root,'model',name='fixture_camera')
        ET.SubElement(model,'pose').text=' '.join(map(str,[*camera,0.,.4,yaw]))
        link=ET.SubElement(model,'link',name='link')
        ET.SubElement(link,'kinematic').text='true';ET.SubElement(link,'gravity').text='false'
        sensor=ET.SubElement(link,'sensor',name='rgbd',type='rgbd_camera')
        ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='30'
        ET.SubElement(sensor,'topic').text='/fixture'
        cam=ET.SubElement(sensor,'camera');ET.SubElement(cam,'horizontal_fov').text='1.22'
        image=ET.SubElement(cam,'image')
        for tag,value in [('width','960'),('height','540'),('format','R8G8B8')]:ET.SubElement(image,tag).text=value
        clips=ET.SubElement(cam,'clip');ET.SubElement(clips,'near').text='.3';ET.SubElement(clips,'far').text='20'
        clips=ET.SubElement(ET.SubElement(cam,'depth_camera'),'clip')
        ET.SubElement(clips,'near').text='.3';ET.SubElement(clips,'far').text='8'
        plugin=ET.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
        for tag,value in [('publish_model_pose','true'),('publish_link_pose','false'),('use_pose_vector_msg','true'),('update_frequency','30')]:
            ET.SubElement(plugin,tag).text=value
        tree.write(world,encoding='unicode')
        metadata=dict(diagnostic_only=True,training_images=False,name=name,bundle=str(bundle),scene=scene,
                      base_xyz=base,base_quaternion=quaternion([0.,0.,yaw]),camera_xyz=camera)
        write_json(folder/'fixture.json',metadata);records.append(name)
    write_json(output/'plan.json',dict(diagnostic_only=True,cases=records))


def capture(fixture,output):
    from gz.msgs10.camera_info_pb2 import CameraInfo
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    from google.protobuf.json_format import MessageToJson
    from phase1_capture_fixture import stamp,decode,camera_intrinsics
    from phase1_pose_sync import measured_pose_bracket
    from mars_training_environment.annotation import Annotator,check_sky_rgb
    from mars_training_environment.common import write_json
    from mars_training_environment.environment_v2 import export_class_id
    from mars_training_environment.framing import configured_extrinsics
    metadata=json.loads(fixture.read_text());scene=metadata['scene']
    if not metadata.get('diagnostic_only') or not os.environ.get('GZ_PARTITION','').startswith('rover-phase1-'):
        raise RuntimeError('Isolated evaluator fixture required')
    output.mkdir(parents=True,exist_ok=False)
    node=Node();lock=threading.Lock();rgb=OrderedDict();depth=OrderedDict();poses=deque(maxlen=200);cal=[]
    def cache(msg,store):
        with lock:
            store[stamp(msg)]=msg
            while len(store)>60:store.popitem(last=False)
    def pose(msg):
        with lock:
            for value in msg.pose:
                if value.name=='fixture_camera':
                    p,q=value.position,value.orientation
                    poses.append((stamp(value) or stamp(msg),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
    def info(msg):
        with lock:
            if not cal:cal.append(MessageToJson(msg))
    if not all((node.subscribe(Image,'/fixture/image',lambda m:cache(m,rgb)),
                node.subscribe(Image,'/fixture/depth_image',lambda m:cache(m,depth)),
                node.subscribe(CameraInfo,'/fixture/camera_info',info),
                node.subscribe(Pose_V,'/model/fixture_camera/pose',pose))):raise RuntimeError('Fixture subscriptions failed')
    deadline=time.monotonic()+90;chosen=None
    while time.monotonic()<deadline:
        with lock:
            for ts in sorted(set(rgb)&set(depth),reverse=True):
                bracket=measured_pose_bracket(poses,ts)
                if cal and bracket and bracket['stable']:
                    chosen=(ts,rgb[ts],depth[ts],bracket);break
        if chosen:break
        time.sleep(.02)
    if not chosen:raise RuntimeError('No exact-stamp RGB-D with a stable measured camera pose')
    ts,rgb_msg,depth_msg,bracket=chosen
    if np.linalg.norm(np.array(bracket['pose'][1])-metadata['camera_xyz'])>1e-6:
        raise RuntimeError('Measured camera moved from the declared evaluator pose')
    pixels,values=decode(rgb_msg,depth_msg);intrinsics=camera_intrinsics(json.loads(cal[0]))
    if (pixels.shape[:2]!=(540,960) or pixels[::8,::8].std(axis=(0,1)).max()<1
            or not (np.isfinite(values)&(values>.3)&(values<=8)).any()):raise RuntimeError('Camera diagnostic failed')
    annotator=Annotator(metadata['bundle'],scene)
    labels,objects,quality,instances=annotator.annotate(metadata['base_xyz'],metadata['base_quaternion'],
        configured_extrinsics(scene['config']['camera']),intrinsics,values)
    check_sky_rgb(pixels.tobytes(),annotator.last_instances,objects,quality['visibility_check']['objects'])
    lookup={o['name']:o for o in scene['objects']+scene['regions']}
    exports=[dict(label,export_class_id=export_class_id(lookup[label['name']])) for label in labels]
    if metadata['name']=='background' and labels:raise RuntimeError('Background acquired a target')
    cv2.imwrite(str(output/'rgb.png'),cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR))
    overlay=cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
    for label in exports:
        cv2.rectangle(overlay,(int(label['xmin']),int(label['ymin'])),(int(label['xmax']),int(label['ymax'])),(0,255,255),1)
        cv2.putText(overlay,str(label['export_class_id']),(int(label['xmin']),max(12,int(label['ymin']))),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
    cv2.imwrite(str(output/'annotations.png'),overlay)
    np.savez_compressed(output/'geometry.npz',depth=values,instances=annotator.last_instances,expected_depth=annotator.last_expected_depth)
    report=dict(diagnostic_only=True,training_images=False,stamp_ns=ts,measured_pose_bracket=bracket,
        labels=exports,objects=objects,quality=quality,intrinsics=intrinsics,
        camera_profile=scene['config']['camera'],input_rgb_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
        input_depth_sha256=hashlib.sha256(values.tobytes()).hexdigest())
    write_json(output/'result.json',report)
    print(json.dumps(dict(name=metadata['name'],labels=len(labels),export_classes=sorted({r['export_class_id'] for r in exports}))))


def run(fixtures,output):
    if os.environ.get('LIBGL_ALWAYS_SOFTWARE')!='0' or not os.environ.get('DISPLAY'):
        raise RuntimeError('Qualified attended desktop path required')
    output.mkdir(parents=True,exist_ok=False)
    plan=json.loads((fixtures/'plan.json').read_text());records=[]
    for index,name in enumerate(plan['cases']):
        folder=output/name;folder.mkdir();env=dict(os.environ,GZ_PARTITION='rover-phase1-env-v2-'+str(index))
        with (folder/'simulator.log').open('xb') as log:
            process=subprocess.Popen(['gz','sim','-r','-s','--render-engine-server','ogre',str(fixtures/name/'world.sdf')],
                env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                with (folder/'capture.log').open('xb') as capture_log:
                    subprocess.run([sys.executable,__file__,'capture',str(fixtures/name/'fixture.json'),str(folder/'capture')],
                                   env=env,stdout=capture_log,stderr=subprocess.STDOUT,timeout=150,check=True)
                proof=subprocess.run([sys.executable,str(Path(__file__).with_name('phase1_renderer_provenance.py'))],
                                     env=env,capture_output=True,text=True,timeout=30)
                (folder/'renderer-provenance.json').write_text(proof.stdout)
                (folder/'renderer-provenance.stderr').write_text(proof.stderr)
                if proof.returncode:raise RuntimeError('Mapped renderer provenance failed')
                records.append(name)
            finally:
                if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=10)
    classes=set()
    for name in records:
        r=json.loads((output/name/'capture/result.json').read_text())
        classes.update(v['export_class_id'] for v in r['labels'])
    result=dict(result='ALL_FOUR_CLASSES_RENDERED_PENDING_LABEL_REVIEW' if classes==set(range(4)) else 'FAIL',
                cases=records,export_classes=sorted(classes),diagnostic_only=True,training_images=False)
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    if classes!=set(range(4)):raise RuntimeError('Rendered diagnostic coverage is incomplete')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['prepare','capture','run'])
    p.add_argument('input',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();globals()[a.mode](a.input,a.output)
