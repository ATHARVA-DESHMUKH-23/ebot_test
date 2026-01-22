from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ebot_slam')

    # ekf_yaml = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_yaml = os.path.join(pkg_share, 'config', 'slam_lidar_only.yaml')

    # static TFs: base_link -> imu_link and base_link -> laser
    static_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_imu',
        output='screen',
        arguments=['0.0','0.0','0.05','0','0','0','1','base_link','imu_link']
    )

    static_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_laser',
        output='screen',
        arguments=['0.0','0.0','0.15','0','0','0','1','base_link','laser']
    )

    # IMU node (your MPU node)
    imu_node = Node(
        package='ebot_slam',
        executable='imu_node',   # ensure your package install makes this executable available
        name='imu_node',
        output='screen',
    )

    # # fake odom node
    # fake_odom_node = Node(
    #     package='ebot_slam',
    #     executable='fake_odom',
    #     name='fake_odom',
    #     output='screen',
    # )

    # EKF filter node (robot_localization)
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_yaml],
    #     # remap its default output /odometry/filtered to /odom so slam_toolbox keeps using /odom
    #     remappings=[('odometry/filtered', 'odom')]
    # )

    # slam_toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_yaml]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    )

    # TimerAction can help start EKF after imu and odom are up; slight delay reduces TF drops
    # delayed_ekf = TimerAction(period=0.5, actions=[ekf_node])
    delayed_slam = TimerAction(period=1.0, actions=[slam_node, lifecycle_manager])

    return LaunchDescription([
        static_imu,
        static_laser,
        imu_node,
        slam_node,
        lifecycle_manager
    ])
