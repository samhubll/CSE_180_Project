# Copyright 2026 CSE180 student
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish person boxes; the starter threshold and detection range need measurement."""

from pathlib import Path

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class PersonDetector(Node):
    """Run a local YOLO model on the latest image at a limited rate."""

    def __init__(self):
        super().__init__('person_detector')
        self.declare_parameter('image_topic', '/rgbd_camera/image')
        self.declare_parameter('detections_topic', '/people/detections')
        self.declare_parameter('model_path', '/opt/cse180-models/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('device', 'cpu')

        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        model_path = Path(self.get_parameter('model_path').value).expanduser()
        self.confidence = float(self.get_parameter('confidence_threshold').value)
        max_rate_hz = float(self.get_parameter('max_rate_hz').value)
        self.device = self.get_parameter('device').value
        if not 0.0 <= self.confidence <= 1.0:
            raise RuntimeError('confidence_threshold must be between 0 and 1')
        if not 0.0 < max_rate_hz < float('inf'):
            raise RuntimeError('max_rate_hz must be positive and finite')
        if not model_path.is_file():
            raise RuntimeError(
                f'Model weights are missing: {model_path}. '
                'Install the weights or set the model_path parameter.')

        try:
            from ultralytics import YOLO
            self.model = YOLO(str(model_path))
        except Exception as exc:
            raise RuntimeError(
                'Could not load the person detector. Run this node with the '
                f'configured Ultralytics environment. Details: {exc}') from exc
        names = self.model.names
        labels = names.items() if isinstance(names, dict) else enumerate(names)
        person_ids = [int(key) for key, name in labels if name == 'person']
        if not person_ids:
            raise RuntimeError('The selected model has no class named "person"')
        self.person_class_id = person_ids[0]

        self.bridge = CvBridge()
        self.latest_image = None
        self.publisher = self.create_publisher(
            Detection2DArray, detections_topic, 10)
        self.subscription = self.create_subscription(
            Image, image_topic, self.receive_image, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max_rate_hz, self.process_latest)
        self.get_logger().info(
            f'Loaded {model_path}; {image_topic} -> {detections_topic}, '
            f'at most {max_rate_hz:g} Hz on {self.device}. '
            'Threshold and reliable detection range require measurement.')

    def receive_image(self, message):
        """Keep the most recent image without running inference in the callback."""
        self.latest_image = message

    def process_latest(self):
        """Infer once for each new frame and preserve its timestamp and frame."""
        message = self.latest_image
        if message is None:
            return
        self.latest_image = None
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Cannot convert camera image: {exc}')
            return
        try:
            result = self.model.predict(
                source=image,
                classes=[self.person_class_id],
                conf=self.confidence,
                device=self.device,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(f'Person inference failed: {exc}')
            return

        output = Detection2DArray()
        output.header = message.header
        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                if class_id != self.person_class_id or confidence < self.confidence:
                    continue
                x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
                detection = Detection2D()
                detection.header = message.header
                detection.bbox.center.position.x = float((x_min + x_max) / 2.0)
                detection.bbox.center.position.y = float((y_min + y_max) / 2.0)
                detection.bbox.size_x = float(x_max - x_min)
                detection.bbox.size_y = float(y_max - y_min)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = 'person'
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)
                output.detections.append(detection)
        self.publisher.publish(output)


def main(args=None):
    """Start the detector, reporting startup errors without fabricated results."""
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        rclpy.logging.get_logger('person_detector').fatal(str(exc))
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
