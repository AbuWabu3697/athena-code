#!/usr/bin/env python3
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseWithCovariance


class YoloLiveLoggerNode(Node):
    def __init__(self):
        super().__init__("yolo_live_logger")

        pkg_share = get_package_share_directory("yolo_ros")
        default_model_path = os.path.join(pkg_share, "models", "best.pt")

        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("detections_topic", "/vision/detections_2d")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("default_frame_id", "")
        self.declare_parameter("include_source_img", False)

        self.model_path = self.get_parameter("model_path").value
        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.queue_size = int(self.get_parameter("queue_size").value)
        self.default_frame_id = self.get_parameter("default_frame_id").value
        self.include_source_img = bool(self.get_parameter("include_source_img").value)

        self.get_logger().info(f"[YOLO] Loading model from: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"[YOLO] Classes: {self.model.names}")

        self.bridge = CvBridge()

        self.pub = self.create_publisher(Detection2DArray, self.detections_topic, self.queue_size)
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, self.queue_size)

        self.get_logger().info(
            f"[ROS] Subscribing: {self.image_topic} | Publishing: {self.detections_topic} | conf_thres={self.conf_thres}"
        )
        if self.include_source_img:
            self.get_logger().warn("[ROS] include_source_img=True")

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        t0 = time.time()
        results = self.model(frame, verbose=False, conf=self.conf_thres)
        t1 = time.time()
        _fps = 1.0 / max(t1 - t0, 1e-6)

        out = Detection2DArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        if msg.header.frame_id:
            out.header.frame_id = msg.header.frame_id
        elif self.default_frame_id:
            out.header.frame_id = self.default_frame_id
        else:
            out.header.frame_id = ""

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                class_name = self.model.names.get(cls_id, str(cls_id))

                det_msg = self._make_detection2d(
                    header=out.header,
                    class_name=class_name,
                    conf=conf,
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                    source_img=(msg if self.include_source_img else None),
                )
                out.detections.append(det_msg)

        self.pub.publish(out)

    def _make_detection2d(
        self,
        header: Header,
        class_name: str,
        conf: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        source_img: Optional[Image],
    ) -> Detection2D:
        det = Detection2D()
        det.header = header
        det.id = ""

        bbox = BoundingBox2D()

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)

        bbox.center.position.x = cx
        bbox.center.position.y = cy
        bbox.center.theta = 0.0
        bbox.size_x = w
        bbox.size_y = h

        det.bbox = bbox

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_name)
        hyp.hypothesis.score = float(conf)
        hyp.pose = PoseWithCovariance()
        det.results.append(hyp)

        if source_img is not None:
            det.source_img = source_img

        return det


def main(args=None):
    rclpy.init(args=args)
    node = YoloLiveLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
