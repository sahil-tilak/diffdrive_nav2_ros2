import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def _bridge_and_obstacle(context):
    """Bridge node (one process) and, if moving_obstacle:=true, the obstacle node.

    With moving_obstacle:=false the bridge arguments are exactly the old five.
    """
    moving = LaunchConfiguration('moving_obstacle').perform(context) == 'true'
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        'tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ]
    actions = []
    if moving:
        bridge_args.append(
            '/model/moving_box/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist')
        actions.append(Node(
            package='diffdrive_nav2_ros2',
            executable='moving_obstacle.py',
            name='moving_obstacle',
            output='screen',
            # wall time on purpose: use_sim_time costs ~60 % of a core (/clock at ~1 kHz)
        ))
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen'
    )
    return [bridge] + actions


def generate_launch_description():
    pkg_share = get_package_share_directory('diffdrive_nav2_ros2')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'test_world.sdf')
    moving_world_file = os.path.join(pkg_share, 'worlds', 'test_world_moving.sdf')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = run Gazebo without its window (-s)')
    moving_arg = DeclareLaunchArgument(
        'moving_obstacle', default_value='false',
        description='true = use worlds/test_world_moving.sdf (moving box, no '
                    'unmapped_box), bridge its cmd_vel and start moving_obstacle.py')
    gz_args = PythonExpression([
        "('-r -s ' if '", LaunchConfiguration('headless'),
        "' == 'true' else '-r ') + ('", moving_world_file, "' if '",
        LaunchConfiguration('moving_obstacle'), "' == 'true' else '", world_file, "')"])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diffdrive_robot',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        moving_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        OpaqueFunction(function=_bridge_and_obstacle),
    ])