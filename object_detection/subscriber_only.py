#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

TOPIC = "/oakd/rgb/preview/image_raw"

class CamViewer(Node):
    def __init__(self):
        super().__init__("cam_viewer")
        self.bridge = CvBridge()
        # Reliable QoS is typical for camera streams. Adjust if needed.
        self.sub = self.create_subscription(Image, TOPIC, self.on_img, 10)
        self.get_logger().info(f"Subscribed to {TOPIC}")

    def on_img(self, msg: Image):
        # Convert ROS2 Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("RGB Preview", frame)
        # 1 ms wait keeps GUI responsive without blocking ROS executor
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quit requested")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = CamViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
