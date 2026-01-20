from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0', '0', '0.15', '0', '0', '0', '1', 'base_link', 'laser']
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        laser_tf,
        imu_tf
    ])
