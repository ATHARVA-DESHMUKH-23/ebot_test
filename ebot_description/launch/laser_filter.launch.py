from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",   # ⭐ THIS LINE FIXES EVERYTHING
            parameters=[
                os.path.expanduser(
                    "~/EBOT_TEST/src/ebot_description/config/scan_filter.yaml"
                )
            ],
            remappings=[
                ("scan", "/scan"),
                ("scan_filtered", "/scan_filtered")
            ],
            output="screen"
        )
    ])