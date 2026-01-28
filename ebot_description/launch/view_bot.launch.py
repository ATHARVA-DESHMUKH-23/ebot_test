from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------------
    # Paths
    # -----------------------------
    pkg_ebot_description = get_package_share_directory('ebot_description')

    xacro_file = os.path.join(
        pkg_ebot_description,
        'models',
        'ebot',
        'ebot_description.xacro'
    )

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('ebot_description'),
        'config',
        'arm_controllers.yaml'
    ])

    # -----------------------------
    # Robot description
    # -----------------------------
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # -----------------------------
    # ros2_control node
    # -----------------------------
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml
        ]
    )

    # -----------------------------
    # Controller spawners
    # -----------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    # Optional (enable when base is ready)
    # diff_drive_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_drive_controller'],
    #     output='screen'
    # )

    # -----------------------------
    # Delay controllers until ros2_control is ready
    # -----------------------------
    delayed_controller_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            # diff_drive_controller_spawner,
        ]
    )

    # -----------------------------
    # Launch description
    # -----------------------------
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        delayed_controller_spawners
    ])
