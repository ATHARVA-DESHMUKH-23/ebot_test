from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_rosbridge = get_package_share_directory('rosbridge_server')

    # -----------------------------
    # Rosbridge WebSocket
    # -----------------------------
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                pkg_rosbridge,
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        ),
        launch_arguments={
            'delay_between_messages': '0.0'
        }.items()
    )

    # -----------------------------
    # Web Server (Joystick UI)
    # -----------------------------
    web_server = ExecuteProcess(
        cmd=[
            'python3', '-m', 'http.server', '8080',
            '--directory', 'JoyStick'
        ],
        output='screen'
    )

    # -----------------------------
    # Launch Description
    # -----------------------------
    return LaunchDescription([
        rosbridge_launch,
        web_server
    ])