from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('ebot_slam')

    ekf_yaml  = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_yaml = os.path.join(pkg_share, 'config', 'slam_lidar_only.yaml')

    # ----------------------------
    # Static TFs
    # ----------------------------
    static_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_imu',
        arguments=['0','0','0.05','0','0','0','base_link','imu_link'],
    )

    static_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_laser',
        arguments=['0','0','0.15','0','0','0','base_link','laser'],
    )

    # ----------------------------
    # IMU
    # ----------------------------
    imu_node = Node(
        package='ebot_slam',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )

    # ----------------------------
    # rf2o (NO TF)
    # ----------------------------
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'freq': 15.0,
        }],
    )

    # ----------------------------
    # EKF (owns odom → base_link)
    # ----------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        remappings=[('odometry/filtered', '/odom')],
    )

    # ----------------------------
    # SLAM Toolbox
    # ----------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_yaml],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['slam_toolbox']
        }],
    )

    return LaunchDescription([
        static_imu,
        static_laser,
        imu_node,
        rf2o_node,
        TimerAction(period=2.0, actions=[ekf_node]),
        TimerAction(period=4.0, actions=[slam_node, lifecycle_manager]),
    ])
