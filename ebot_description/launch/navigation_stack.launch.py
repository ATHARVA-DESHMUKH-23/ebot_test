from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_ebot = get_package_share_directory('ebot_description')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # -----------------------------
    # SLAM Toolbox (Localization)
    # -----------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                pkg_ebot,
                'config',
                'lidar_slam.yaml'
            )
        }.items()
    )

    # -----------------------------
    # Nav2
    # -----------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': '/home/ebot/EBOT_TEST/map/office_map.yaml',
            'params_file': os.path.join(
                pkg_ebot,
                'config',
                'nav2_params_full.yaml'
            )
        }.items()
    )

    # -----------------------------
    # Launch Order (IMPORTANT)
    # -----------------------------
    return LaunchDescription([

        # Start SLAM first
        slam_launch,

        # Delay Nav2 so map + TF is ready
        TimerAction(
            period=2.0,
            actions=[nav2_launch]
        ),
    ])