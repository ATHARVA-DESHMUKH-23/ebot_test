from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------------
    # XACRO (NO Gazebo)
    # -----------------------------
    xacro_file = PathJoinSubstitution([
        FindPackageShare('ebot_description'),
        'models', 'ebot',
        'ebot_description.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                xacro_file,
                ' use_gazebo:=false',
                ' use_hardware:=true',
                ' use_gripper:=true'
            ]),
            value_type=str
        )
    }

    # -----------------------------
    # Controllers YAML
    # -----------------------------
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('ebot_description'),
        'config',
        'arm_controllers.yaml'
    ])

    # -----------------------------
    # ros2_control (REAL HARDWARE)
    # -----------------------------
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controllers_yaml
        ],
        output='screen'
    )

    # -----------------------------
    # Controller spawners (ARM ONLY)
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

    # -----------------------------
    # Robot State Publisher (REAL TIME)
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': False}
        ],
        output='screen'
    )

    # -----------------------------
    # Launch description
    # -----------------------------
    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        robot_state_publisher
    ])
