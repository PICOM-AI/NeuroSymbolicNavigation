#!/usr/bin/env python3
import math
import os
import subprocess
from typing import Optional
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from pathlib import Path


from irobot_create_msgs.action import RotateAngle, DriveDistance


class TB4Motion(Node):
    def __init__(self):
        super().__init__('tb4_motion_sync')
        self._rotate_ac = ActionClient(self, RotateAngle, '/rotate_angle')
        self._drive_ac = ActionClient(self, DriveDistance, '/drive_distance')
        self.origin_x = -10.1
        self.origin_y = -10.1
        self.cell_size = 0.2

    def set_map_params(self, origin_x, origin_y, cell_size, map_width, map_height):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.cell_size = cell_size
        self.map_width = map_width
        self.map_height = map_height

    # ---------- internal sync senders ----------

    def _wait_for_server(self, ac: ActionClient, name: str, timeout_sec: Optional[float] = 5.0):
        if not ac.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f'Action server {name} not available')

    def _send_rotate_sync(self, angle_rad: float, max_rot_speed: float = 0.5):
        """Block until rotation completes."""
        self._wait_for_server(self._rotate_ac, '/rotate_angle')
        goal = RotateAngle.Goal()
        goal.angle = float(angle_rad)
        goal.max_rotation_speed = float(max_rot_speed)

        send_future: Future = self._rotate_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('RotateAngle goal rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            raise RuntimeError('RotateAngle returned no result')
        return result.result.pose  # geometry_msgs/PoseStamped

    def _send_drive_sync(self, distance_m: float, max_trans_speed: float = 0.3):
        """Block until straight drive completes."""
        self._wait_for_server(self._drive_ac, '/drive_distance')
        goal = DriveDistance.Goal()
        goal.distance = float(distance_m)
        goal.max_translation_speed = float(max_trans_speed)

        send_future: Future = self._drive_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('DriveDistance goal rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            raise RuntimeError('DriveDistance returned no result')
        return result.result.pose  # geometry_msgs/PoseStamped

    # ---------- public sync helpers ----------

    def rotate_left_90(self, max_rot_speed: float = 2.0):
        return self._send_rotate_sync(math.pi / 2.0, max_rot_speed)

    def rotate_right_90(self, max_rot_speed: float = 2.0):
        return self._send_rotate_sync(-math.pi / 2.0, max_rot_speed)

    def turn_back_180(self, max_rot_speed: float = 2.0):
        return self._send_rotate_sync(math.pi, max_rot_speed)

    def forward(self, meters: float, max_trans_speed: float = 0.3):
        if meters < 0:
            # Use negative to drive backward if desired.
            pass
        pose = self._send_drive_sync(meters, max_trans_speed)
        return pose

    def move_forward(self, meters: float, max_trans_speed: float = 0.3):
        """Move forward by moving forward."""
        pose = self.forward(meters, max_trans_speed)
        return self.pose2cell(pose)

    def move_left(self, meters: float, max_trans_speed: float = 0.3):
        """Move left by rotating left 90, moving forward, rotating right 90."""
        self.rotate_left_90()
        pose = self.forward(meters, max_trans_speed)
        self.rotate_right_90()
        return self.pose2cell(pose)

    def move_right(self, meters: float, max_trans_speed: float = 0.3):
        """Move right by rotating right 90, moving forward, rotating left 90."""
        self.rotate_right_90()
        pose = self.forward(meters, max_trans_speed)
        self.rotate_left_90()
        return self.pose2cell(pose)

    def move_backward(self, meters: float, max_trans_speed: float = 0.3):
        """Move back by turning back 180, moving forward, turning back 180."""
        self.turn_back_180()
        pose = self.forward(meters, max_trans_speed)
        self.turn_back_180()
        return self.pose2cell(pose)

    def pose2cell(self, pose):
        """Convert a geometry_msgs/PoseStamped bottom-left to grid cell coordinates top-left."""
        x = pose.pose.position.x
        y = pose.pose.position.y
        cell_x = int((x - self.origin_x) / self.cell_size)
        cell_y = self.map_height - int((y - self.origin_y) / self.cell_size) # flip y-axis
        return (cell_x, cell_y)

    def cell2pose(self, cell_x, cell_y):
        """Convert grid cell coordinates (top-left) back to pose coordinates (bottom-left)."""
        x = self.origin_x + (cell_x) * self.cell_size -0.5
        y = self.origin_y + (self.map_height - cell_y) * self.cell_size + 0.1  # unflip y-axis
        return (x, y)

    def teleport_object(self, model_name, x, y, z=0.0, yaw=0.0, world="default"):
        """
        Teleport a Gazebo (Fortress) model to a specified pose using ign service.

        Args:
            model_name (str): Model name in Gazebo.
            x, y, z (float): Target position.
            yaw (float): Rotation about Z (radians).
            world (str): World name (default: "default").
        """
        import math

        # Convert yaw to quaternion (assuming flat rotation)
        qx, qy, qz, qw = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        cmd = [
            "ign", "service",
            "-s", f"/world/{world}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req",
            (
                f'name: "{model_name}", '
                f'position: {{x: {x}, y: {y}, z: {z}}}, '
                f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
            )
        ]

        subprocess.run(cmd, check=True)

    def spawn_new_object(self, sdf_path="box.sdf", name="obj", world="maze", x=0.0, y=0.0, z=0.1):
        """
        Spawn an SDF object into a Gazebo world using ros_gz_sim CLI.

        Args:
            sdf_path (str): Path to the SDF file.
            name (str): Model name.
            world (str): Target Gazebo world.
            x, y, z (float): Spawn coordinates.
        """
        sdf_file = Path(sdf_path)
        if not sdf_file.exists():
            print(f"SDF file '{sdf_file}' not found.")
            return False

        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", world,
            "-file", str(sdf_file),
            "-name", name,
            "-x", str(x),
            "-y", str(y),
            "-z", str(z)
        ]

        try:
            subprocess.run(cmd, check=True)
            print(f"Spawned '{name}' at ({x}, {y}, {z}) in world '{world}'.")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Failed to spawn '{name}': {e}")
            return False