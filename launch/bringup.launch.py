"""Start Gazebo, then Nav2 after nav2_delay, then (optionally) RViz after rviz_delay.

Does not send a goal.

Usage:
  ros2 launch diffdrive_nav2_ros2 bringup.launch.py headless:=false rviz:=true

Predictive costmap layer (default off = Nav2 gets config/nav2_params.yaml, exactly as before):
  ros2 launch diffdrive_nav2_ros2 bringup.launch.py predictive_layer:=true
  This merges config/predictive_layer.yaml into a temporary copy of nav2_params.yaml and gives
  Nav2 that copy. The path of the copy is printed when Nav2 starts. The layer needs
  perception.launch.py and tracking.launch.py running to have anything to mark.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _merge(base, overlay):
    """Deep merge: dicts are merged, everything else (lists, scalars) is replaced."""
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def generate_launch_description():
    pkg_share = get_package_share_directory('diffdrive_nav2_ros2')
    nav2_share = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_share, 'maps', 'test_world_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    nav2_delay = LaunchConfiguration('nav2_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')
    moving_obstacle = LaunchConfiguration('moving_obstacle')

    args = [
        DeclareLaunchArgument(
            'moving_obstacle', default_value='false',
            description='true = moving box world + obstacle script (see gazebo.launch.py)'),
        DeclareLaunchArgument(
            'predictive_layer', default_value='false',
            description='true = add the predictive costmap layer to the local costmap'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = run Gazebo without its window'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true = start RViz after Nav2'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='15',
            description='Seconds after Gazebo starts before Nav2 starts'),
        DeclareLaunchArgument(
            'rviz_delay', default_value='15',
            description='Seconds after Nav2 starts before RViz starts'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'headless': headless,
                          'moving_obstacle': moving_obstacle}.items(),
    )

    def nav2_include(params):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': params,
                'use_sim_time': 'true',
            }.items(),
        )

    def make_nav2(context):
        # predictive_layer false: the same params file path as before this argument existed.
        if LaunchConfiguration('predictive_layer').perform(context).lower() != 'true':
            return [nav2_include(params_file)]
        overlay_file = os.path.join(pkg_share, 'config', 'predictive_layer.yaml')
        with open(params_file) as f:
            params = yaml.safe_load(f)
        with open(overlay_file) as f:
            overlay = yaml.safe_load(f)
        with tempfile.NamedTemporaryFile(
                'w', prefix='nav2_params_predictive_', suffix='.yaml', delete=False) as tmp:
            yaml.safe_dump(_merge(params, overlay), tmp, sort_keys=False)
        return [
            LogInfo(msg='predictive_layer:=true, Nav2 params file: ' + tmp.name),
            nav2_include(tmp.name),
        ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz),
    )

    # The RViz timer is nested inside the Nav2 timer, so rviz_delay counts
    # from when Nav2 starts, not from when Gazebo starts.
    start_rviz = TimerAction(period=rviz_delay, actions=[rviz_node])
    start_nav2 = TimerAction(
        period=nav2_delay, actions=[OpaqueFunction(function=make_nav2), start_rviz])

    return LaunchDescription(args + [gazebo, start_nav2])
