#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ZCrop(Node):
    def __init__(self):
        super().__init__('z_cropper')
        self.declare_parameter('input', '/lidar_points')
        self.declare_parameter('output', '/lidar_points_filtered')
        self.declare_parameter('min_z', -2.0)
        self.declare_parameter('max_z', 3.0)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output').value, 10)
        self.sub = self.create_subscription(PointCloud2, self.get_parameter('input').value, self.cb, 10)

    def cb(self, msg):
        pts = pc2.read_points(msg, field_names=None, skip_nans=True)
        min_z = float(self.get_parameter('min_z').value)
        max_z = float(self.get_parameter('max_z').value)
        filtered = [p for p in pts if min_z <= p[2] <= max_z]  # p[2] is z
        out = pc2.create_cloud(msg.header, msg.fields, filtered)
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ZCrop())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
