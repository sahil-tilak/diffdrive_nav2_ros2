"""Start the obstacle tracker (use_sim_time false). Start it AFTER perception.launch.py.

Usage:
  ros2 launch diffdrive_nav2_ros2 tracking.launch.py

Subscribes /detected_obstacles (PoseArray, odom), publishes /tracked_obstacles
(diffdrive_nav2_ros2/TrackedObstacleArray, moving tracks only) and /tracked_obstacles/markers
(MarkerArray, one arrow per track = velocity * 1 s).
Parameters (defaults declared in include/.../tracking/kalman_tracker.hpp, TrackerParams):
  q_accel 0.5, r_meas 0.05, gate 0.5, max_missed 5, min_speed 0.1, init_vel_var 1.0
dt comes from header stamps, so wall time for the node is fine.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diffdrive_nav2_ros2',
            executable='obstacle_tracker_node',
            name='obstacle_tracker',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
