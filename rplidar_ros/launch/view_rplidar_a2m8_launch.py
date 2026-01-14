from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogWarn, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def try_launch_lidar(context, *args, **kwargs):
    actions = []
    try:
        lidar_pkg = get_package_share_directory('rplidar_ros')
        lidar_launch = os.path.join(
            lidar_pkg,
            'launch',
            'view_rplisar_a2m8_launch.py'
        )

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch)
            )
        )

    except Exception as e:
        actions.append(
            LogWarn(
                msg=f"[WARNING] LiDAR launch failed: {str(e)}"
            )
        )

    return actions


def generate_launch_description():

    # -------- Robot Description --------
    pkg_ebot_description = get_package_share_directory('ebot_description')
    xacro_file = os.path.join(
        pkg_ebot_description,
        'models',
        'ebot',
        'ebot_description.xacro'
    )

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([

        # -------- Robot State Publisher --------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # -------- Joint State Publisher --------
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # -------- Serial Interface --------
        Node(
            package='ebot_serial',
            executable='cmdvel_to_arduino',
            name='cmdvel_to_arduino',
            output='screen'
        ),

        Node(
            package='ebot_serial',
            executable='encoder_odimetery',
            name='encoder_odimetery',
            output='screen'
        ),

        # -------- LiDAR (Optional) --------
        OpaqueFunction(function=try_launch_lidar),
    ])