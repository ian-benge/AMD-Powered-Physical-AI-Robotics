"""Exercise capture-loop ordering without starting ROS, Gazebo or a GPU."""
from collections import deque
import importlib.util
import sys
import tempfile
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

try:
    from mars_training_environment import capture
except ModuleNotFoundError as error:
    if error.name.split('.')[0] not in {'rclpy','gz','nav_msgs','sensor_msgs','vision_msgs'}:
        raise
    # Exercise production ordering in CPU checks, with middleware import doubles
    # confined to an isolated module. Actual ROS installs use the normal import.
    doubles={name:ModuleType(name) for name in ['rclpy','rclpy.node','rclpy.qos',
        'gz','gz.msgs10','gz.msgs10.boolean_pb2','gz.msgs10.pose_pb2','gz.transport13',
        'nav_msgs','nav_msgs.msg','sensor_msgs','sensor_msgs.msg','vision_msgs','vision_msgs.msg']}
    for module in doubles.values():module.__path__=[]
    doubles['rclpy'].spin_once=Mock()
    doubles['rclpy.node'].Node=type('Node',(),{})
    doubles['rclpy.qos'].QoSProfile=Mock()
    doubles['rclpy.qos'].HistoryPolicy=S(KEEP_LAST=1)
    doubles['rclpy.qos'].ReliabilityPolicy=S(RELIABLE=1)
    for module,names in [('gz.msgs10.boolean_pb2',['Boolean']),('gz.msgs10.pose_pb2',['Pose']),
                         ('gz.transport13',['Node']),('nav_msgs.msg',['Odometry']),
                         ('sensor_msgs.msg',['CameraInfo','Image']),('vision_msgs.msg',['Detection2DArray'])]:
        for name in names:setattr(doubles[module],name,type(name,(),{}))
    source=Path(__file__).resolve().parents[1]/'mars_training_environment/capture.py'
    spec=importlib.util.spec_from_file_location('mars_training_environment._capture_ordering_test',source)
    capture=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,doubles):spec.loader.exec_module(capture)
Capture,stamp=capture.Capture,capture.stamp


def message(seconds,xy=(0.,0.)):
    sec=int(seconds);nsec=round((seconds-sec)*1e9)
    return S(header=S(stamp=S(sec=sec,nanosec=nsec)),
             pose=S(pose=S(position=S(x=xy[0],y=xy[1],z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))))


class PoseFeedbackTests(unittest.TestCase):
    def node(self):
        return S(images=deque([message(1.)]),odometry=deque([message(1.02)]))

    def test_barrier_drains_pending_feedback_instead_of_reusing_pre_annotation_time(self):
        node=self.node();pending=[('images',message(3.)),('odometry',message(3.04)),
                                  ('images',message(4.)),('odometry',message(4.02))]
        def spin(_,timeout_sec):
            self.assertEqual(timeout_sec,0.)
            if pending:
                name,value=pending.pop(0);getattr(node,name).append(value)
        with patch.object(capture.rclpy,'spin_once',side_effect=spin) as calls:
            self.assertAlmostEqual(Capture.pose_feedback_barrier(node),4.02)
        self.assertLessEqual(calls.call_count,256)

    def test_out_of_order_tail_cannot_move_the_barrier_backwards(self):
        node=self.node();node.images.append(message(5.));node.images.append(message(4.))
        with patch.object(capture.rclpy,'spin_once'):
            self.assertEqual(Capture.pose_feedback_barrier(node),5.)

    def test_yaw_only_pose_waits_for_feedback_beyond_the_drained_barrier_then_settles(self):
        node=self.node();node.depths=deque([object()]);node.boxes=deque()
        node.requires_box_messages=False;node.cap=dict(settle_sim_sec=.7)
        node.scene=dict(id='scene',config_sha256='config',content_sha256='scene',views=[
            dict(id='view_000',xyz=[0.,0.,.09],rpy=[0.,0.,.035])])
        node.box_contract={};node.records=[];node.clearer_view=Mock(return_value=None)
        node.get_logger=Mock(return_value=Mock())
        def barrier():
            node.images.append(message(4.));node.odometry.append(message(4.02))
            return 4.02
        node.pose_feedback_barrier=Mock(side_effect=barrier)
        node.set_pose=Mock(return_value=True)
        sample=(message(4.8),None,None,None,None)
        node.synchronized=Mock(return_value=sample)
        def wait(predicate,reason,timeout=None):
            if reason=='Rover did not reach the requested viewpoint':
                # Same XY and newer than the old image still do NOT establish
                # arrival: this odometry was already present before set_pose.
                self.assertFalse(predicate())
                node.odometry.append(message(4.04))
                return predicate()
            return predicate()
        node.wait=wait
        node.save=Mock(side_effect=lambda *args:node.records.append('metadata/frame.json'))
        with tempfile.TemporaryDirectory() as folder:
            node.root=Path(folder)
            # No real ROS waits occur in this ordering fixture. Supply elapsed
            # wall time independently of the host's monotonic clock resolution.
            with patch.object(capture.time,'monotonic',side_effect=[0.,.1,.2,1.]):
                Capture.run(node)
        timing=node.save.call_args.args[0]['capture_timing']
        self.assertEqual(timing['feedback_barrier_sim_sec'],4.02)
        self.assertAlmostEqual(timing['pose_arrival_sim_sec'],4.04)
        self.assertAlmostEqual(timing['capture_after_sim_sec'],4.74)
        self.assertGreater(stamp(sample[0]),timing['capture_after_sim_sec'])
        self.assertEqual(node.synchronized.call_count,1)
        self.assertAlmostEqual(node.synchronized.call_args.args[0],4.74)


if __name__=='__main__':unittest.main()
