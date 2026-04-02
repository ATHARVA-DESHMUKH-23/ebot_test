from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_ebot = get_package_share_directory('ebot_description')

    # -----------------------------
    # Robot Launch
    # -----------------------------
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ebot, 'launch', 'view_bot.launch.py')
        )
    )

    # -----------------------------
    # Navigation Stack (SLAM + Nav2)
    # -----------------------------
    nav_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ebot, 'launch', 'navigation_stack.launch.py')
        )
    )

    # -----------------------------
    # Web Interface
    # -----------------------------
    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ebot, 'launch', 'web.launch.py')
        )
    )

    # -----------------------------
    # Launch Order
    # -----------------------------
    return LaunchDescription([

        # 1. Robot first (immediate)
        robot_launch,

        # 2. Navigation after robot stabilizes
        TimerAction(
            period=8.0,
            actions=[nav_stack_launch]
        ),

        # 3. Web interface (can start anytime, but delayed for clarity)
        TimerAction(
            period=10.0,
            actions=[web_launch]
        ),
    ])