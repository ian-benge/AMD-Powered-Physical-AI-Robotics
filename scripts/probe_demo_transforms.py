#!/usr/bin/env python3
"""Exercise real tf2 for both SLAM localization and the fixed demo map."""
from types import SimpleNamespace as NS
import math

from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer
from rover_navigation.demo_node import DemoMission


def insert(buffer,parent,child,seconds,x,y=0.,*,static=False):
    message=TransformStamped()
    nanoseconds=round(seconds*1e9)
    message.header.stamp.sec,message.header.stamp.nanosec=divmod(nanoseconds,1_000_000_000)
    message.header.frame_id=parent;message.child_frame_id=child
    message.transform.translation.x=x;message.transform.translation.y=y;message.transform.rotation.w=1.
    publish = buffer.set_transform_static if static else buffer.set_transform
    publish(message,'isolated_demo_fixture')


def main():
    buffer=Buffer()
    insert(buffer,'map','odom',9.5,.2);insert(buffer,'map','odom',10.1,.5)
    insert(buffer,'odom','drive_center',9.7,2.);insert(buffer,'odom','drive_center',9.98,2.)
    node=NS(tf=buffer,get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    # Newest map->odom is 100 ms in the future. The valid full-chain pose is
    # 20 ms behind the clock; correction at that time is interpolated to .44 m.
    for target,expected in (('map',1.44),('odom',.56)):
        result=DemoMission.transform_pose(node,[1.,2.,.3],target)
        assert math.isclose(result[0],expected,abs_tol=1e-8),result
        assert result[1:]==[2.,.3],result
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=11_000_000_000))
    try:DemoMission.transform_pose(node,[1.,2.,.3])
    except ValueError as error:assert 'stale' in str(error)
    else:raise AssertionError('Stale full-chain pose was accepted')
    print('ACTUAL_TF2_REPROJECTION_PASS: interpolated at fresh pose time; stale chain rejected; no motion')

    stable = Buffer()
    insert(stable,'map','odom',0.,0.,static=True)
    insert(stable,'odom','drive_center',143.,.14,3.226)
    insert(stable,'odom','drive_center',149.,.13,3.220)
    # A separate diagnostic graph can jump without moving the operational map.
    insert(stable,'rtabmap_map','graph_reference',143.,0.)
    insert(stable,'rtabmap_map','graph_reference',149.,.095,-2.797)
    node.tf=stable
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=149_050_000_000))
    home=[.129953,-.00024,0.]
    assert DemoMission.transform_pose(node,home)==home
    assert DemoMission.pose(node)==[.13,3.22,0.]
    image_time=Time(nanoseconds=149_050_000_000)
    image_tf=stable.lookup_transform('map','odom',image_time)
    assert Time.from_msg(image_tf.header.stamp).nanoseconds==image_time.nanoseconds
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=150_000_000_000))
    try:DemoMission.pose(node)
    except ValueError as error:assert 'stale' in str(error)
    else:raise AssertionError('Fixed map accepted stale odometry')
    print('ACTUAL_TF2_FIXED_MAP_PASS: isolated map jump, image-time projection, and stale odometry rejected')


if __name__=='__main__':main()
