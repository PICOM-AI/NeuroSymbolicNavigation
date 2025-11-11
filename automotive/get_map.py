#!/usr/bin/env python3
# save_map_white_free.py
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import OccupancyGrid

class OneShotMapSaver(Node):
    def __init__(self, base_name='map', occ_thr=0.65):
        super().__init__('one_shot_map_saver')
        self.base = base_name
        self.occ_thr = occ_thr
        self.done = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map', self.cb, qos)
        self.get_logger().info('Waiting for /map ...')

    def cb(self, msg: OccupancyGrid):
        self.get_logger().info('Received /map. Saving...')
        pgm_path = f'{self.base}.pgm'
        yaml_path = f'{self.base}.yaml'
        self._write_pgm(msg, pgm_path)
        self._write_yaml(msg, pgm_path, yaml_path)
        self.get_logger().info(f'Saved {pgm_path} and {yaml_path}')
        self.done = True  # signal main loop to stop

    def _write_pgm(self, msg: OccupancyGrid, path: str):
        w, h = msg.info.width, msg.info.height
        data = msg.data
        occ_thr_100 = int(self.occ_thr * 100 + 0.5)

        pixels = bytearray(w * h)
        for y in range(h):
            src_row = h - 1 - y
            for x in range(w):
                v = data[src_row * w + x]
                pixels[y * w + x] = 0 if v >= occ_thr_100 else 255

        with open(path, 'wb') as f:
            f.write(b'P5\n')
            f.write(b'# CREATOR: save_map_white_free.py\n')
            f.write(f'{w} {h}\n255\n'.encode('ascii'))
            f.write(pixels)

    def _write_yaml(self, msg: OccupancyGrid, image_path: str, yaml_path: str):
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        o = msg.info.origin.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))

        yaml_text = (
            f"image: {os.path.basename(image_path)}\n"
            f"mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{ox}, {oy}, {yaw}]\n"
            f"negate: 0\n"
            f"occupied_thresh: {self.occ_thr}\n"
            f"free_thresh: 0.0\n"
        )
        with open(yaml_path, 'w') as f:
            f.write(yaml_text)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--base', default='map')
    parser.add_argument('--occ', type=float, default=0.65)
    args = parser.parse_args()

    rclpy.init()
    node = OneShotMapSaver(base_name=args.base, occ_thr=args.occ)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.2)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
