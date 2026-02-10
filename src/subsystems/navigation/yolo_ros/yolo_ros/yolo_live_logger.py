#!/usr/bin/env python3
"""
ROS2 YOLO Live Logger
- Subscribes to sensor_msgs/Image
- Runs Ultralytics YOLO inference
- Publishes vision_msgs/Detection2DArray on a topic

Matches message layout:
Detection2DArray:
  std_msgs/Header header
  vision_msgs/Detection2D[] detections

Detection2D:
  std_msgs/Header header
  vision_msgs/ObjectHypothesisWithPose[] results
  vision_msgs/BoundingBox2D bbox
  sensor_msgs/Image source_img
"""

import os
import time
import csv
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesisWithPose,
) 


from geometry_msgs.msg import PoseWithCovariance


class YoloLiveLoggerNode(Node):
    def __init__(self):
        super().__init__("yolo_live_logger")

        # -------------------- Parameters --------------------
        self.declare_parameter("model_path", "best.pt")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/vision/detections_2d")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("queue_size", 5)

        # frame_id behavior:
        # - If incoming image msg has header.frame_id: use it
        # - else use this param if provided
        self.declare_parameter("default_frame_id", "")

        # source_img behavior:
        # True  -> put full source image into every Detection2D (bandwidth heavy)
        # False -> leave source_img empty (recommended)
        self.declare_parameter("include_source_img", False)

        # Optional CSV logging (kept from your script)
        self.declare_parameter("save_csv", False)
        self.declare_parameter("output_dir", "yolo_output")

        # -------------------- Read parameters --------------------
        self.model_path = self.get_parameter("model_path").value
        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.queue_size = int(self.get_parameter("queue_size").value)

        self.default_frame_id = self.get_parameter("default_frame_id").value
        self.include_source_img = bool(self.get_parameter("include_source_img").value)

        self.save_csv = bool(self.get_parameter("save_csv").value)
        self.output_dir = self.get_parameter("output_dir").value

        # -------------------- Load YOLO --------------------
        weights_path = self._resolve_model_path(self.model_path)
        self.get_logger().info(f"[YOLO] Loading model from: {weights_path}")
        self.model = YOLO(weights_path)
        self.get_logger().info(f"[YOLO] Classes: {self.model.names}")

        # -------------------- ROS interfaces --------------------
        self.bridge = CvBridge()

        self.pub = self.create_publisher(
            Detection2DArray, self.detections_topic, self.queue_size
        )
        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_cb, self.queue_size
        )

        # -------------------- CSV logging --------------------
        self.csv_file: Optional[csv.FileIO] = None
        self.csv_writer: Optional[csv.writer] = None
        self.frame_counter = 0

        if self.save_csv:
            os.makedirs(self.output_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = os.path.join(self.output_dir, f"detections_log_{ts}.csv")

            self.csv_file = open(csv_path, mode="w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "frame_id", "timestamp", "class_id", "class_name",
                "confidence", "x1", "y1", "x2", "y2", "width_px", "height_px",
                "fps"
            ])
            self.get_logger().info(f"[CSV] Logging enabled: {csv_path}")

        self.get_logger().info(
            f"[ROS] Subscribing: {self.image_topic} | Publishing: {self.detections_topic} | conf_thres={self.conf_thres}"
        )
        if self.include_source_img:
            self.get_logger().warn(
                "[ROS] include_source_img=True (this can be VERY bandwidth heavy if there are many detections per frame)"
            )

    def _resolve_model_path(self, model_path: str) -> str:
        # If relative, resolve relative to this python file (same behavior as your script)
        if os.path.isabs(model_path):
            return model_path
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, model_path)

    def destroy_node(self):
        # close CSV cleanly
        try:
            if self.csv_file:
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()

    def image_cb(self, msg: Image):
        # Convert image -> OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        t_start = time.time()

        # Run YOLO
        # Ultralytics returns a list of Results
        results = self.model(frame, verbose=False, conf=self.conf_thres)

        t_end = time.time()
        fps = 1.0 / max(t_end - t_start, 1e-6)

        # Build Detection2DArray output
        out = Detection2DArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp

        if msg.header.frame_id:
            out.header.frame_id = msg.header.frame_id
        elif self.default_frame_id:
            out.header.frame_id = self.default_frame_id
        else:
            out.header.frame_id = ""

        # Fill detections[]
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
                    cls_id=cls_id,
                    conf=conf,
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                    source_img=(msg if self.include_source_img else None),
                )
                out.detections.append(det_msg)

                # Optional CSV log
                if self.csv_writer is not None:
                    self.csv_writer.writerow([
                        self.frame_counter,
                        datetime.now().isoformat(),
                        cls_id,
                        class_name,
                        f"{conf:.4f}",
                        int(x1), int(y1), int(x2), int(y2),
                        int(x2 - x1),
                        int(y2 - y1),
                        f"{fps:.2f}",
                    ])

        self.pub.publish(out)
        self.frame_counter += 1

    def _make_detection2d(
        self,
        header: Header,
        cls_id: int,
        conf: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        source_img: Optional[Image],
    ) -> Detection2D:
        det = Detection2D()
        det.header = header

        # ---- bbox (BoundingBox2D) ----
        bbox = BoundingBox2D()

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)

        # Different distros define bbox.center slightly differently.
        # Most common: center has x, y, theta (Pose2D-like)
        # We'll set x/y always, and theta only if it exists.
        bbox.center.x = cx
        bbox.center.y = cy
        if hasattr(bbox.center, "theta"):
            bbox.center.theta = 0.0

        bbox.size_x = w
        bbox.size_y = h
        det.bbox = bbox

        # ---- results (ObjectHypothesisWithPose[]) ----
        hyp = ObjectHypothesisWithPose()

        # class_id is typically string in vision_msgs hypothesis
        hyp.hypothesis.class_id = str(cls_id)
        hyp.hypothesis.score = float(conf)

        # pose is optional; identity pose-with-covariance is fine
        hyp.pose = PoseWithCovariance()

        det.results.append(hyp)

        # ---- source_img (sensor_msgs/Image) ----
        # Leave empty unless include_source_img=True
        if source_img is not None:
            det.source_img = source_img

        # Some versions include det.id; set empty if present
        if hasattr(det, "id"):
            det.id = ""

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
