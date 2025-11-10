#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from typing import List

from detectors.yolo11 import YOLO11Detector
from detectors.base import Det2D

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)
from geometry_msgs.msg import Pose2D  # ROS 2 Humble: Pose2D has x,y,theta

TOPIC_IMG = "/oakd/rgb/preview/image_raw"
TOPIC_DET = "/detections"


def draw_detections(img, dets: List[Det2D]):
    h, w = img.shape[:2]
    for d in dets:
        x1 = max(0, min(int(d.box.x1), w - 1))
        y1 = max(0, min(int(d.box.y1), h - 1))
        x2 = max(0, min(int(d.box.x2), w - 1))
        y2 = max(0, min(int(d.box.y2), h - 1))
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2, lineType=cv2.LINE_AA)
        label = f"{d.cls} {d.score:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        y0 = max(0, y1 - 8)
        cv2.rectangle(img, (x1, y0 - th - 4), (x1 + tw + 2, y0), (0, 255, 0), -1)
        cv2.putText(img, label, (x1 + 1, y0 - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return img


def dets_to_msg(dets: List[Det2D], header) -> Detection2DArray:
    """
    Det2D -> Detection2DArray (ROS 2 Humble)
      - bbox.center: geometry_msgs/Pose2D with x,y,theta
      - results[]: vision_msgs/ObjectHypothesisWithPose
          * .hypothesis: vision_msgs/ObjectHypothesis
              - .class_id: string label
              - .score: float64
          * .pose: PoseWithCovariance (leave default)
    """
    arr = Detection2DArray()
    arr.header = header

    for d in dets:
        x1, y1, x2, y2 = float(d.box.x1), float(d.box.y1), float(d.box.x2), float(d.box.y2)
        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)
        cx = x1 + 0.5 * w
        cy = y1 + 0.5 * h

        det = Detection2D()
        det.header = header

        # Build bbox without reassigning .center to a new object property
        bbox = BoundingBox2D()
        # center is already a Pose2D; set fields directly
        bbox.center.position.x = float(cx)
        bbox.center.position.y = float(cy)
        bbox.center.theta = 0.0
        bbox.size_x = float(w)
        bbox.size_y = float(h)
        det.bbox = bbox

        # Fill results using ObjectHypothesisWithPose.hypothesis
        hyp_with_pose = ObjectHypothesisWithPose()
        hyp_with_pose.hypothesis = ObjectHypothesis()
        hyp_with_pose.hypothesis.class_id = str(d.cls)
        hyp_with_pose.hypothesis.score = float(d.score)
        # hyp_with_pose.pose remains default (unset) for 2D detections
        det.results.append(hyp_with_pose)

        arr.detections.append(det)

    return arr


class DetectAndPublish(Node):
    def __init__(self):
        super().__init__("detect_and_publish")
        self.bridge = CvBridge()

        # Default: YOLO11
        self.detector = YOLO11Detector()
        self.detector.setup(
            model="yolo11n.pt",
            conf=0.4,
            iou=0.45,
            max_det=200,
            class_allowlist=None,
            warmup=True,
        )

        self.sub = self.create_subscription(Image, TOPIC_IMG, self.on_img, 10)
        self.pub = self.create_publisher(Detection2DArray, TOPIC_DET, 10)

        self.get_logger().info(f"Subscribed: {TOPIC_IMG}")
        self.get_logger().info(f"Publishing: {TOPIC_DET}")

    def on_img(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        dets = self.detector.detect(bgr)
        arr = dets_to_msg(dets, msg.header)
        self.pub.publish(arr)

        vis = draw_detections(bgr.copy(), dets)
        cv2.imshow("Detections (pub to /detections)", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quit requested")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = DetectAndPublish()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
