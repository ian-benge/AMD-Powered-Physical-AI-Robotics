#!/usr/bin/env python3
"""Isolated multi-process 960x540 RGB/depth transport fixture; no control topics."""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time


def worker(role,output):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init();node=Node('demo_transport_'+role);streams={'rgb':[],'depth':[]}
    if role=='reader':
        def receive(name,data):
            order='<' if data[1]&1 else '>';sec,nano=struct.unpack_from(order+'iI',data,4)
            streams[name].append([time.monotonic(),sec+nano/1e9])
        subscriptions=[node.create_subscription(Image,'/demo_transport_fixture/'+name,
            lambda data,n=name:receive(n,data),qos_profile_sensor_data,raw=True) for name in streams]
        until=time.monotonic()+9.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        output.write_text(json.dumps(streams)+'\n')
    else:
        pubs={n:node.create_publisher(Image,'/demo_transport_fixture/'+n,qos_profile_sensor_data) for n in streams}
        images={}
        for name in streams:
            image=Image();image.width=960;image.height=540;image.encoding='bgr8' if name=='rgb' else '32FC1'
            image.step=960*(3 if name=='rgb' else 4);image.data=bytes(image.step*540);image.header.frame_id='fixture_only'
            images[name]=image
        time.sleep(1.)
        start=time.monotonic()
        for i in range(90):
            due=start+i/15
            while time.monotonic()<due:time.sleep(max(0.,min(.002,due-time.monotonic())))
            sec,nano=divmod(int(time.monotonic()*1e9),1_000_000_000)
            for name,image in images.items():
                image.header.stamp.sec=sec;image.header.stamp.nanosec=nano;pubs[name].publish(image)
    node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--worker',choices=('reader','writer'));a=p.parse_args()
    if a.worker:return worker(a.worker,a.output)
    if a.output.exists():raise ValueError('Preserve the previous throughput report')
    profile=Path(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']);assert profile.is_file()
    env=dict(os.environ,ROS_DOMAIN_ID='218',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    raw=a.output.with_suffix('.samples.json')
    reader=subprocess.Popen([sys.executable,__file__,'--worker','reader','--output',str(raw)],env=env)
    try:
        time.sleep(.4)
        subprocess.run([sys.executable,__file__,'--worker','writer','--output',str(raw)],env=env,check=True,timeout=15)
        reader.wait(timeout=15)
        if reader.returncode:raise RuntimeError('Transport reader failed')
        samples=json.loads(raw.read_text());report=dict(fixture_only=True,motion_requested=False,streams={},passed=True)
        for name,rows in samples.items():
            rate=(len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else 0.
            gap=max((b[0]-a[0] for a,b in zip(rows,rows[1:])),default=999.)
            report['streams'][name]=dict(count=len(rows),wall_hz=rate,max_gap_seconds=gap)
            report['passed'] &= len(rows)>=80 and rate>=12 and gap<=.6
        a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
        if not report['passed']:raise RuntimeError('Large-image DDS throughput failed')
    finally:
        if reader.poll() is None:reader.terminate();reader.wait(timeout=5)


if __name__=='__main__':main()
