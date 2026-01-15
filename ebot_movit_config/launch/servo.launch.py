from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory("ebot_movit_config"),
        "config",
        "servo.yaml"
    )

    return LaunchDescription([
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            parameters=[config_path],
            output="screen",
        )
    ])
