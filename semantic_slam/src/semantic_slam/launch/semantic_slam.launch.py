# file: semantic_slam/launch/semantic_slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='semantic_slam',
            executable='semantic_slam_node',
            name='semantic_slam',
            parameters=[{
                'detector_type': 'yolo11',
                'detector.model': 'yolo11n.pt',
                'detector.conf': 0.45,
                'detector.iou': 0.45,
                'map_frame': 'map',
                'camera_frame': 'oakd_rgb_camera_optical_frame',
                'fuser.score_min': 0.45,
                'fuser.range_min_m': 0.3,
                'fuser.range_max_m': 6.0,
                'fuser.publish_markers': True,
                'fuser.depth_scale': 1.0,   # 32FC1 meters
                'fuser.median_window': 5,
                'fuser.ground_clamp': True,
                'fuser.yaw_bias_deg': 0.0,
                'sync_queue': 10,
                'sync_slop_sec': 0.10,
                'fuser.warn_stamp_delta_sec': 0.20
            }],
            output='screen'
        )
    ])
