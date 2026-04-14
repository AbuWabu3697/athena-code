#!/usr/bin/env python3

from typing import Optional, List

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloRosNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_node')

        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('target_class', 'bottle')

        self.conf_thres: float = float(
            self.get_parameter('conf_thres').get_parameter_value().double_value
        )
        self.model_path: str = (
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        self.image_topic: str = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )
        self.target_class: str = (
            self.get_parameter('target_class').get_parameter_value().string_value
        )

        self.bridge = CvBridge()

        if YOLO is None:
            self.get_logger().error(
                'Ultralytics is not installed. Install with: pip install ultralytics'
            )
            raise RuntimeError('Missing ultralytics package')

        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO model from: {self.model_path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model: {exc}')
            raise

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.target_found_pub = self.create_publisher(
            Bool,
            '/yolo/target_found',
            10
        )

        self.get_logger().info('YOLO ROS node initialized')
        self.get_logger().info(f'Subscribing to image topic: {self.image_topic}')
        self.get_logger().info(f'Target class: {self.target_class}')
        self.get_logger().info(f'Confidence threshold: {self.conf_thres:.2f}')

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            self.publish_target_found(False)
            return

        try:
            results = self.model(frame, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            self.publish_target_found(False)
            return

        target_found = False

        if results:
            result = results[0]
            boxes = getattr(result, 'boxes', None)
            names = getattr(result, 'names', {})

            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    conf = float(box.conf[0].item())
                    class_id = int(box.cls[0].item())

                    if conf < self.conf_thres:
                        continue

                    class_name = names[class_id] if class_id in names else str(class_id)

                    if class_name == self.target_class:
                        target_found = True
                        break

        self.publish_target_found(target_found)

    def publish_target_found(self, found: bool) -> None:
        msg = Bool()
        msg.data = found
        self.target_found_pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = YoloRosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Fatal error in yolo_node: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()