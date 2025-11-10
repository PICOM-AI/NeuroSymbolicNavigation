#!/usr/bin/env python3
"""
Single-flow pipeline:
- Subscribe to /oakd/rgb/preview/image_raw
- Run YOLO11Detector on each frame
- Draw results and display

Swap hint:
- To use your own detector later, replace YOLO11Detector with MyDetector
  and keep the rest identical. Your class must follow DetectorBase from base.py.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from typing import List

# --- Detector import (reference) ---
# Uses the provided YOLOv11 implementation which follows DetectorBase. :contentReference[oaicite:0]{index=0}
from detectors.yolo11 import YOLO11Detector
# Base data types for drawing. :contentReference[oaicite:1]{index=1}
from detectors.base import Det2D

TOPIC = "/oakd/rgb/preview/image_raw"

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


class DetectNode(Node):
    def __init__(self):
        super().__init__("detect_and_draw_yolo11")
        self.bridge = CvBridge()

        # --- Detector setup (reference) ---
        # Students: later replace next two lines with:
        #   from detectors.my_detector import MyDetector
        #   self.detector = MyDetector(); self.detector.setup(...)
        self.detector = YOLO11Detector()
        self.detector.setup(
            model="yolo11n.pt",   # change weight file as needed
            conf=0.4,
            iou=0.45,
            max_det=200,
            class_allowlist=None,
            warmup=True,
        )

        self.sub = self.create_subscription(Image, TOPIC, self.on_img, 10)
        self.get_logger().info(f"Subscribed to {TOPIC}")

    def on_img(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        dets = self.detector.detect(bgr)
        vis = bgr.copy()
        draw_detections(vis, dets)
        cv2.imshow("Detections (YOLO11)", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quit requested")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = DetectNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
