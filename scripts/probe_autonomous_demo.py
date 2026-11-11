#!/usr/bin/env python3
"""Actual ROS constructor/service smoke check, with no sensors or motion publisher."""
import json
from pathlib import Path
import tempfile
import time
import threading
from http.server import ThreadingHTTPServer
from urllib.request import Request,urlopen

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rover_navigation.demo_node import DemoMission
from rover_navigation_interfaces.srv import DemoCommand
from rover_navigation.mission_controls import Controls,handler


def main():
    with tempfile.TemporaryDirectory() as folder:
        output=Path(folder)/'demo.jsonl'
        rclpy.init(args=['--ros-args','-p','session:=smoke-session','-p','demo_profile:=rehearsal',
            '-p','demo_output:='+str(output),'-p','navigation_speed:=0.05','-p','demo_mode:=true'])
        node=DemoMission();executor=SingleThreadedExecutor();executor.add_node(node)
        try:
            end=time.monotonic()+.8
            while time.monotonic()<end:executor.spin_once(timeout_sec=.05)
            assert node.latest_status['phase']=='ready'
            assert node.latest_status['home'] is None
            assert node.goal is None and not node.pending
            client=node.create_client(DemoCommand,'/navigation/demo_command')
            assert client.wait_for_service(timeout_sec=1.)
            req=DemoCommand.Request();req.command='heartbeat';req.session_id='smoke-session'
            req.mission_id=node.run.mission_id;req.client_token='smoke-operator-token';req.sequence=1
            future=client.call_async(req);executor.spin_until_future_complete(future,timeout_sec=2.)
            assert future.done() and future.result().success
            req.command='start_exploration'
            future=client.call_async(req);executor.spin_until_future_complete(future,timeout_sec=2.)
            assert future.done() and not future.result().success
            assert node.run.phase=='ready' and node.goal is None
            events=[json.loads(line) for line in output.read_text().splitlines()]
            assert [r['kind'] for r in events]==['configuration','command_rejected']
            assert events[-1]['command']=='start_exploration'
            # Match the qualified rgbd_odometry publisher's actual offered QoS.
            # The previous reliable subscription silently received no odometry.
            publisher=node.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
            odometry=Odometry();odometry.header.frame_id='odom';odometry.child_frame_id='base_link'
            odometry.pose.pose.orientation.w=1.
            end=time.monotonic()+3.
            while time.monotonic()<end and node.odometry_pose is None:
                publisher.publish(odometry);executor.spin_once(timeout_sec=.05)
            assert node.odometry_pose==[0.,0.,0.] and not node.odometry_error
            assert node.health_details()['checks']['estimated_odometry']
            print('ACTUAL_ROS_BEST_EFFORT_ODOMETRY_RECEIVED_PASS')
            controls=Controls();executor.add_node(controls)
            server=ThreadingHTTPServer(('127.0.0.1',0),handler(controls))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
            try:
                request=Request(f'http://127.0.0.1:{server.server_port}/demo/heartbeat',
                    data=json.dumps(dict(session_id='smoke-session',mission_id=node.run.mission_id,
                        client_token='smoke-operator-token',epoch=0,sequence=2)).encode(),
                    headers={'Content-Type':'application/json'})
                with urlopen(request,timeout=5) as reply:assert json.load(reply)['success']
            finally:
                server.shutdown();server.server_close();executor.shutdown();spin.join(timeout=2)
                controls.destroy_node()
            print('ACTUAL_ROS_DEMO_CONSTRUCTOR_SERVICE_AND_NO_SENSOR_REFUSAL_PASS')
        finally:
            node.worker.shutdown(wait=False,cancel_futures=True);node.stream.close()
            executor.remove_node(node);node.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
