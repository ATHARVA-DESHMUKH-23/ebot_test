from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --------------------
    # Robot Description
    # --------------------
    pkg_ebot_description = get_package_share_directory('ebot_description')
    xacro_file = os.path.join(
        pkg_ebot_description,
        'models',
        'ebot',
        'ebot_description.xacro'
    )

    robot_description = Command(['xacro ', xacro_file])

    # --------------------
    # RPLidar Launch
    # --------------------
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'view_rplidar_a2m8_launch.py'
    )

    # --------------------
    # Magnetometer Script
    # --------------------
    magne_script = os.path.join(
        get_package_share_directory('ebot_serial'),
        'scripts',
        'magne.py'
    )

    return LaunchDescription([

        # --------------------
        # Robot State Publisher
        # --------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # --------------------
        # Fake Hardware Interface
        # --------------------
        Node(
            package='ebot_serial',
            executable='fake_hw_interface',
            name='fake_hw_interface',
            output='screen'
        ),

        # --------------------
        # Encoder Odometry
        # --------------------
        Node(
            package='ebot_serial',
            executable='encoder_odimetery',
            name='encoder_odimetery',
            output='screen'
        ),

        # --------------------
        # Magnetometer Calibration Script
        # --------------------
        ExecuteProcess(
            cmd=['python3', magne_script],
            output='screen'
        ),

        # --------------------
        # RPLidar
        # --------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),
    ])
