from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    carto_config_dir = os.path.join(
        get_package_share_directory('carto_config'), 'config'
    )

    return LaunchDescription([

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', carto_config_dir,
                '-configuration_basename', 'carto.lua'
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu/data_raw')
            ]
        ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            arguments=[
                '-resolution', '0.05',
                '-publish_period_sec', '0.5'
            ]
        )
    ])
