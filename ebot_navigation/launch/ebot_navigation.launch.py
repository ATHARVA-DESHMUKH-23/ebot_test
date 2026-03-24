from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- SLAM TOOLBOX ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'slam_params_file': 'src/ebot_description/config/lidar_slam.yaml'
        }.items()
    )

    # --- NAV2 ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': '/home/rajvardhan/mobile_manipulator/src/ebot_description/config/nav2_param_full_twist_stamped.yaml'
        }.items()
    )

    # --- DELAY NAV2 BY 5 SECONDS ---
    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        slam_launch,
        delayed_nav2
    ])