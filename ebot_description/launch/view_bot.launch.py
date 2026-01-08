from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Path to xacro file
    pkg_ebot_description = get_package_share_directory('ebot_description')
    xacro_file = os.path.join(
        pkg_ebot_description,
        'models',
        'ebot',
        'ebot_description.xacro'
    )

    # Robot description parameter
    robot_description = Command([
        'xacro ', xacro_file
    ])

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Joint State Publisher (no GUI)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])
