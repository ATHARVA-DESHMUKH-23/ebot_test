from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -----------------------------
    # Set GZ_SIM_RESOURCE_PATH
    # -----------------------------
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH'),
            ':',
            '/home/rajvardhan/mobile_manipulator/src/ebot_description/models'
        ]
    )

    # -----------------------------
    # Gazebo Harmonic
    # -----------------------------
    world_file = PathJoinSubstitution([
        FindPackageShare('ebot_description'),
        'worlds',
        'officemap3.world'
    ])

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # -----------------------------
    # Clock bridge (CRITICAL)
    # -----------------------------
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            '/camera_head/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_head/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_head/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_head/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

   
    # -----------------------------
    # Launch
    # -----------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        set_gz_resource_path,
        gz_sim,
        clock_bridge,

    ])
