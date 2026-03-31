#!/usr/bin/env python3

from typing import Optional, List

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

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
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_image_topic', '/yolo/annotated_image')

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
        self.publish_annotated_image: bool = (
            self.get_parameter('publish_annotated_image')
            .get_parameter_value()
            .bool_value
        )
        self.annotated_image_topic: str = (
            self.get_parameter('annotated_image_topic')
            .get_parameter_value()
            .string_value
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

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )

        self.target_found_pub = self.create_publisher(
            Bool,
            '/yolo/target_found',
            10
        )

        self.target_label_pub = self.create_publisher(
            String,
            '/yolo/target_label',
            10
        )

        self.target_center_pub = self.create_publisher(
            PointStamped,
            '/yolo/target_center',
            10
        )

        self.annotated_image_pub = self.create_publisher(
            Image,
            self.annotated_image_topic,
            10
        )

        self.get_logger().info('YOLO ROS node initialized')
        self.get_logger().info(f'Subscribing to image topic: {self.image_topic}')
        self.get_logger().info(f'Target class for BT condition: {self.target_class}')
        self.get_logger().info(f'Confidence threshold: {self.conf_thres:.2f}')

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        try:
            results = self.model(frame, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        if not results:
            self.publish_empty_outputs(msg.header)
            return

        result = results[0]

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        target_found = False
        best_target_conf = -1.0
        best_target_center = None

        annotated_frame = frame.copy()

        boxes = getattr(result, 'boxes', None)
        names = getattr(result, 'names', {})

        if boxes is None or len(boxes) == 0:
            self.detections_pub.publish(detection_array)
            self.publish_target_outputs(
                msg.header,
                found=False,
                target_center=None,
                target_label=''
            )
            if self.publish_annotated_image:
                self.publish_annotated(annotated_frame, msg.header)
            return

        for box in boxes:
            conf = float(box.conf[0].item())
            class_id = int(box.cls[0].item())

            if conf < self.conf_thres:
                continue

            class_name = names[class_id] if class_id in names else str(class_id)

            xyxy = box.xyxy[0].tolist()
            x_min, y_min, x_max, y_max = xyxy

            center_x = (x_min + x_max) / 2.0
            center_y = (y_min + y_max) / 2.0
            width = x_max - x_min
            height = y_max - y_min

            detection_msg = Detection2D()
            detection_msg.header = msg.header
            detection_msg.bbox.center.position.x = center_x
            detection_msg.bbox.center.position.y = center_y
            detection_msg.bbox.size_x = width
            detection_msg.bbox.size_y = height

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = conf
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

            if class_name == self.target_class and conf > best_target_conf:
                target_found = True
                best_target_conf = conf
                best_target_center = (center_x, center_y)

            cv2.rectangle(
                annotated_frame,
                (int(x_min), int(y_min)),
                (int(x_max), int(y_max)),
                (0, 255, 0),
                2
            )
            cv2.putText(
                annotated_frame,
                f'{class_name} {conf:.2f}',
                (int(x_min), max(0, int(y_min) - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        self.detections_pub.publish(detection_array)

        self.publish_target_outputs(
            msg.header,
            found=target_found,
            target_center=best_target_center,
            target_label=self.target_class if target_found else ''
        )

        if self.publish_annotated_image:
            self.publish_annotated(annotated_frame, msg.header)

    def publish_empty_outputs(self, header) -> None:
        empty_array = Detection2DArray()
        empty_array.header = header
        self.detections_pub.publish(empty_array)

        self.publish_target_outputs(
            header,
            found=False,
            target_center=None,
            target_label=''
        )

    def publish_target_outputs(
        self,
        header,
        found: bool,
        target_center: Optional[tuple],
        target_label: str
    ) -> None:
        found_msg = Bool()
        found_msg.data = found
        self.target_found_pub.publish(found_msg)

        label_msg = String()
        label_msg.data = target_label
        self.target_label_pub.publish(label_msg)

        if found and target_center is not None:
            point_msg = PointStamped()
            point_msg.header = header
            point_msg.point.x = float(target_center[0])
            point_msg.point.y = float(target_center[1])
            point_msg.point.z = 0.0
            self.target_center_pub.publish(point_msg)

    def publish_annotated(self, frame, header) -> None:
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header = header
            self.annotated_image_pub.publish(img_msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to publish annotated image: {exc}')


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