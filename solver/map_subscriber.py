#!/usr/bin/env python3
"""
map_subscriber.py

Subscribe to ROS2 topic /map (occupancy grid map 2D), print map information,
save to PGM bitmap file, then exit.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import OccupancyGrid


class MapSubscriber(Node):
    def __init__(self, output_file='map.pgm'):
        super().__init__('map_subscriber')
        self.output_file = output_file
        self.map_received = False

        # Use QoS profile suitable for map topics (TRANSIENT_LOCAL durability)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.get_logger().info('Waiting for /map topic...')

    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            return  # Only process once
        
        self.map_received = True
        self.get_logger().info('Received /map message')
        
        # Print map information
        self.print_map_info(msg)
        
        # Save to PGM file
        self.save_pgm(msg, self.output_file)
        
        self.get_logger().info(f'Map saved to {self.output_file}')
        self.get_logger().info('Exiting...')

    def print_map_info(self, msg: OccupancyGrid):
        """Print map information to console"""
        print("\n" + "="*60)
        print("MAP INFORMATION")
        print("="*60)
        print(f"Width:  {msg.info.width} pixels")
        print(f"Height: {msg.info.height} pixels")
        print(f"Resolution: {msg.info.resolution} m/pixel")
        print(f"Origin X: {msg.info.origin.position.x:.3f} m")
        print(f"Origin Y: {msg.info.origin.position.y:.3f} m")
        print(f"Origin Z: {msg.info.origin.position.z:.3f} m")
        
        # Calculate map bounds
        map_width_m = msg.info.width * msg.info.resolution
        map_height_m = msg.info.height * msg.info.resolution
        print(f"Map size: {map_width_m:.3f} m x {map_height_m:.3f} m")
        
        # Count occupancy values
        data = list(msg.data)
        unknown = data.count(-1)
        free = data.count(0)
        occupied = sum(1 for v in data if v > 0)
        
        print(f"\nOccupancy statistics:")
        print(f"  Unknown:  {unknown} pixels ({unknown*100.0/len(data):.1f}%)")
        print(f"  Free:     {free} pixels ({free*100.0/len(data):.1f}%)")
        print(f"  Occupied: {occupied} pixels ({occupied*100.0/len(data):.1f}%)")
        print("="*60 + "\n")

    def save_pgm(self, msg: OccupancyGrid, path: str):
        """Save occupancy grid to PGM file"""
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        
        # Convert occupancy grid data to PGM format
        # PGM: 0 = black (occupied), 255 = white (free), 205 = gray (unknown)
        pixels = bytearray(width * height)
        for y in range(height):
            # Flip Y axis (ROS map origin is bottom-left, PGM origin is top-left)
            src_row = height - 1 - y
            for x in range(width):
                idx = src_row * width + x
                val = data[idx]
                
                if val == -1:
                    # Unknown: gray (205)
                    pixels[y * width + x] = 205
                elif val == 0:
                    # Free: white (255)
                    pixels[y * width + x] = 255
                else:
                    # Occupied: black (0)
                    pixels[y * width + x] = 0
        
        # Write PGM file
        with open(path, 'wb') as f:
            f.write(b'P5\n')
            f.write(b'# CREATOR: map_subscriber.py\n')
            f.write(f'{width} {height}\n255\n'.encode('ascii'))
            f.write(pixels)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Subscribe to /map and save as PGM')
    parser.add_argument('--output', '-o', default='map.pgm', 
                       help='Output PGM file path (default: map.pgm)')
    args = parser.parse_args()

    rclpy.init()
    node = MapSubscriber(output_file=args.output)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # Spin until map is received
        while rclpy.ok() and not node.map_received:
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

