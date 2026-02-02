from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    carto_config_dir = os.path.join(
        get_package_share_directory('carto_config'), 'config'
    )

    return LaunchDescription([

        # ---------------- STATIC TFs ----------------

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0',
                       'base_link','laser']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0',
                       'base_link','imu_link']
        ),

        # ---------------- IMU ----------------
        Node(
            package='ebot_slam',
            executable='imu_node',
            output='screen'
        ),

        # ---------------- 3D SCAN BUILDER ----------------
        Node(
            package='ebot_slam',
            executable='scan_to_3d',
            output='screen'
        ),
    ])
