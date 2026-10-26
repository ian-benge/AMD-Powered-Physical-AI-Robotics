"""Latest-frame rover detector, with exact image headers and explicit readiness."""
import json
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from .detectors import TorchDetector, draw_detections
from .detector_service import RemoteDetector
from .live_core import image_to_bgr


class RockDetector(Node):
    def __init__(self):
        super().__init__('rock_detector')
        for name, value in dict(image_topic='/oak/image_raw', detection_topic='/detections',
                                backend='none', model_path='', detector_url='http://127.0.0.1:8775',
                                confidence=.25).items():
            self.declare_parameter(name, value)
        self.backend, self.model_path = (self.get_parameter(k).value for k in ('backend', 'model_path'))
        if self.backend not in ('none', 'ultralytics', 'remote_http'):
            raise ValueError('Detector backend must be none, ultralytics or remote_http')
        self.condition = threading.Condition()
        self.pending = self.completed = None
        self.closed = False
        self.model = None
        self.error = 'No trained detector configured' if self.backend == 'none' else 'Loading trained detector'
        self.processed = 0
        self.pub = self.create_publisher(Detection2DArray, self.get_parameter('detection_topic').value, 1)
        self.annotated_pub = self.create_publisher(CompressedImage, '/navigation/annotated_image/compressed', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image, qos_profile_sensor_data)
        self.create_timer(.02, self.publish_completed)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def _on_image(self, msg):
        with self.condition:
            self.pending = msg, time.monotonic()
            self.condition.notify()

    def run(self):
        while not self.closed:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.closed)
                if self.closed:
                    return
                msg, received = self.pending
                self.pending = None
            if self.backend == 'none':
                continue
            try:
                if self.model is None:
                    if not self.model_path:
                        raise ValueError('A trained model path is required')
                    self.model = (RemoteDetector(self.get_parameter('detector_url').value, self.model_path)
                                  if self.backend == 'remote_http' else TorchDetector(self.model_path))
                if time.monotonic()-received > 1.:
                    continue
                image = image_to_bgr(msg)
                result = self.model.detect(image, self.get_parameter('confidence').value)
                detections = detection_message(result.detections, msg.header)
                ok, jpeg = cv2.imencode('.jpg', draw_detections(image, result.detections), [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    raise ValueError('Annotated image encoding failed')
                annotated = CompressedImage()
                annotated.header, annotated.format, annotated.data = msg.header, 'jpeg', jpeg.tobytes()
                with self.condition:
                    self.completed = detections, annotated, received, result.timings
                    self.processed += 1
                    self.error = ''
            except Exception as error:
                self.error = str(error)
                self.model = None
                with self.condition:
                    self.condition.wait_for(lambda: self.closed, timeout=1.)

    def publish_completed(self):
        with self.condition:
            complete, self.completed = self.completed, None
        if complete and time.monotonic()-complete[2] <= 1.:
            self.pub.publish(complete[0])
            self.annotated_pub.publish(complete[1])
        status = String()
        status.data = json.dumps(dict(ready=self.model is not None and not self.error, backend=self.backend,
                                     error=self.error, processed_frames=self.processed,
                                     model=getattr(self.model, 'metadata', None)))
        self.status_pub.publish(status)

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.worker.join(timeout=3.)


def _set_box_center(bbox, x, y):
    center = bbox.center
    if hasattr(center, 'position'):
        center.position.x, center.position.y = float(x), float(y)
    else:
        center.x, center.y = float(x), float(y)
    if hasattr(center, 'theta'):
        center.theta = 0.


def detection_message(rows, header):
    output = Detection2DArray()
    output.header = header
    for row in rows:
        detection = Detection2D()
        detection.header = header
        x1, y1, x2, y2 = row['xyxy']
        _set_box_center(detection.bbox, (x1+x2)/2, (y1+y2)/2)
        detection.bbox.size_x, detection.bbox.size_y = float(x2-x1), float(y2-y1)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = row['class_name']
        hypothesis.hypothesis.score = float(row['confidence'])
        detection.results.append(hypothesis)
        output.detections.append(detection)
    return output


def main():
    cv2.setNumThreads(1)
    rclpy.init()
    node = RockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
