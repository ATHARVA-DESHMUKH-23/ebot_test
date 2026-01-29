from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    xacro_file = os.path.join(
        FindPackageShare('ebot_description').find('ebot_description'),
        'models',
        'ebot',
        'ebot_description.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    simple_kdl_test = Node(
        package='ebot_serial',
        executable='cartesian_servo_kdl',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([
        simple_kdl_test
    ])
