#!/usr/bin/env python3
"""Capture real ROS depth, then compare CPU and ROCm terrain on identical frames."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np


def capture(folder):
    import rclpy
    import message_filters
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener, TransformException
    from rover_navigation.ros_utils import from_grid
    rclpy.init()
    node=Node('terrain_gpu_capture')
    buffer=Buffer(); listener=TransformListener(buffer,node)
    pending=[]; captured=[]; last=-math.inf; grid=[None]
    depth=message_filters.Subscriber(node,Image,'/oak/depth/image_raw',qos_profile=qos_profile_sensor_data)
    info=message_filters.Subscriber(node,CameraInfo,'/oak/camera_info',qos_profile=qos_profile_sensor_data)
    sync=message_filters.ApproximateTimeSynchronizer([depth,info],5,.015)
    sync.registerCallback(lambda d,i:pending.append((d,i)) if len(pending)<10 else None)
    node.create_subscription(OccupancyGrid,'/navigation/terrain_map',lambda m:grid.__setitem__(0,from_grid(m)),1)
    def rigid(tf):
        t,q=tf.transform.translation,tf.transform.rotation
        return [t.x,t.y,t.z],[q.x,q.y,q.z,q.w]
    started=time.monotonic()
    try:
        while len(captured)<10 and time.monotonic()-started<30:
            rclpy.spin_once(node,timeout_sec=.03)
            for d,i in list(pending):
                stamp=d.header.stamp.sec+d.header.stamp.nanosec/1e9
                if stamp<=last+.19:
                    pending.remove((d,i));continue
                try:
                    when=Time.from_msg(d.header.stamp)
                    tf=buffer.lookup_transform('odom',d.header.frame_id,when)
                    pose=buffer.lookup_transform('odom','base_link',when)
                    inverse=buffer.lookup_transform('odom','map',Time())
                except TransformException:
                    continue
                translation,quaternion=rigid(tf)
                inv_t,inv_q=rigid(inverse)
                meta=dict(encoding=d.encoding,width=d.width,height=d.height,step=d.step,
                    bigendian=bool(d.is_bigendian),k=list(i.k),translation=translation,quaternion=quaternion,
                    center=[pose.transform.translation.x,pose.transform.translation.y],
                    resolution=.025,size=10.,max_slope=12.,max_step=.03)
                np.savez(folder/f'frame_{len(captured):03d}.npz', data=np.frombuffer(bytes(d.data),np.uint8),
                    meta=np.frombuffer(json.dumps(meta).encode(),np.uint8), inverse=np.array([inv_t+inv_q]))
                captured.append(stamp);last=stamp;pending.remove((d,i))
                print('captured',len(captured),flush=True)
                break
        if len(captured)!=10:
            raise RuntimeError('Could not capture ten time-aligned live depth frames')
        (folder/'capture.json').write_text(json.dumps(dict(stamps=captured,source='live ROS RGB-D + TF'),indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


def benchmark(folder):
    from rover_navigation.terrain_compute import CpuTerrainCompute,RocmTerrainCompute
    from rover_navigation.terrain_service import RemoteTerrainCompute
    from rover_navigation.core import TerrainMemory,terrain_reasons
    cpu,gpu,remote=CpuTerrainCompute(),RocmTerrainCompute(),RemoteTerrainCompute()
    frames=[]
    for p in sorted(folder.glob('frame_*.npz')):
        with np.load(p,allow_pickle=False) as d:
            frames.append((d['data'].tobytes(),json.loads(d['meta'].tobytes()),d['inverse'][0]))
    if len(frames)!=10:
        raise ValueError('Exactly ten recorded frames required')
    memory=TerrainMemory(20.,.025)
    differences=[]
    for raw,kw,inverse in frames:
        a,al,ac=cpu.depth(raw,**kw);b,bl,bc=remote.depth(raw,**kw)
        unknown=int(np.count_nonzero((a.data<0)!=(b.data<0)))
        hazards=int(np.count_nonzero((a.data>=90)!=(b.data>=90)))
        cost=int(np.max(np.abs(a.data.astype(int)-b.data.astype(int))))
        for name in al:
            np.testing.assert_allclose(al[name],bl[name],atol=2e-7,rtol=1e-8,equal_nan=True)
        if unknown or hazards or cost>1 or ac!=bc:
            raise AssertionError(f'Unsafe depth parity: unknown={unknown}, hazards={hazards}, costs={cost}')
        memory.remember(a,terrain_reasons(a,al))
        expected=cpu.project(memory,inverse[:3],inverse[3:],20.,.05)
        actual=remote.project(memory,inverse[:3],inverse[3:],20.,.05)
        for e,v in zip(expected,actual):np.testing.assert_array_equal(e.data,v.data)
        differences.append(dict(unknown=unknown,hazards=hazards,max_cost_difference=cost,points=ac))
    def measure(backend):
        values=[]
        for i in range(35):
            raw,kw,inverse=frames[i%10]
            started=time.perf_counter()
            backend.depth(raw,**kw)
            middle=time.perf_counter()
            backend.project(memory,inverse[:3],inverse[3:],20.,.05)
            end=time.perf_counter()
            if i>=5:values.append([(middle-started)*1000,(end-middle)*1000,(end-started)*1000])
        def stats(v):return dict(mean=float(np.mean(v)),median=float(np.median(v)),p95=float(np.percentile(v,95)))
        return dict(depth_ms=stats([v[0] for v in values]),projection_ms=stats([v[1] for v in values]),
                    total_ms=stats([v[2] for v in values]))
    report=dict(metadata=gpu.metadata,frames=10,parity=differences,
                cpu=measure(cpu),rocm_direct=measure(gpu),rocm_rpc=measure(remote))
    (folder/'benchmark.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['capture','benchmark'])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    (capture if args.mode=='capture' else benchmark)(args.output)
