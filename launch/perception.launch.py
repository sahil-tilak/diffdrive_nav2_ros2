"""Start the obstacle detector (use_sim_time false). Start it AFTER bringup.launch.py.

Usage:
  ros2 launch diffdrive_nav2_ros2 perception.launch.py

Subscribes /scan, publishes /detected_obstacles (geometry_msgs/PoseArray, frame odom, one per scan).
Parameters (defaults declared in src/perception/obstacle_detector_node.cpp):
  target_frame odom, range_min 0.15, range_max 8.0, gap_base 0.03, gap_ratio 0.112,
  min_points 3, max_length 0.6, occlusion_margin 0.3
Known bias: the published point is on the visible face of the object, up to 0.15 m in front of
the centre of a 0.3 m box.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diffdrive_nav2_ros2',
            executable='obstacle_detector_node',
            name='obstacle_detector',
            output='screen',
            # /clock is ~1 kHz; sim time cost 18% CPU vs 1.5% on wall time (percA_01 vs percB_01)
            parameters=[{'use_sim_time': False}],
        ),
    ])
