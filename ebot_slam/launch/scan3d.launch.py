from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0',
                    'map','odom']
        ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0','0','0','0','0','0',
        #             'odom','base_link']
        # ),
        # # -------- base_link → laser --------
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0','0','0','0','0','0',
        #                'base_link','laser']
        # ),

        # # -------- base_link → imu_link --------
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0','0','0','0','0','0',
        #                'base_link','imu_link']
        # ),
    
        # -------- IMU EKF --------
        Node(
            package='ebot_slam',
            executable='imu_ekf',
            output='screen'
        ),
        
        # -------- Scan to 3D --------
        Node(
            package='ebot_slam',
            executable='scan_to_3d',
            output='screen'
        ),

        Node(
            package='ebot_serial',
            executable='start_3d_lidar',
            output='screen'
        ),
    ])
