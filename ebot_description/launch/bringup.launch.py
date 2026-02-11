from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

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

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml
        ],
        output='screen'
    )

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

    # -----------------------------
    # rosbridge websocket
    # -----------------------------
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'delay_between_messages': 0.0
        }]
    )

    # -----------------------------
    # ebot_serial nodes
    # -----------------------------
    cartesian_servo = Node(
        package='ebot_serial',
        executable='cartesian_servo',
        output='screen'
    )

    ee_velocity_allocator = Node(
        package='ebot_serial',
        executable='ee_velocity_allocator',
        output='screen'
    )

    cmdvel_to_arduino = Node(
        package='ebot_serial',
        executable='cmdvel_to_arduino',
        output='screen'
    )

    fake_hw_interface = Node(
        package='ebot_serial',
        executable='fake_hw_interface',
        output='screen'
    )

    encoder_odometry = Node(
        package='ebot_serial',
        executable='encoder_odimetery',
        output='screen'
    )

    # -----------------------------
    # ebot_navigation nodes
    # -----------------------------
    odom_controller = Node(
        package='ebot_navigation',
        executable='odom_controller',
        output='screen'
    )

    odom_nav_action_server = Node(
        package='ebot_navigation',
        executable='odom_nav_action_server',
        output='screen'
    )

    # -----------------------------
    # KDL / FSM nodes
    # -----------------------------
    kdl_cartesian_point_controller = Node(
        package='kdl_fk_cpp',
        executable='kdl_cartesian_point_controller',
        output='screen'
    )

    pick_fsm_node = Node(
        package='kdl_fk_cpp',
        executable='pick_fsm_node',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,

        # rosbridge
        rosbridge_websocket,

        # -----------------------------
        # Uncomment as needed
        # -----------------------------
        # cartesian_servo,
        # ee_velocity_allocator,

        cmdvel_to_arduino,
        fake_hw_interface,
        encoder_odometry,

        odom_controller,
        odom_nav_action_server,

        # kdl_cartesian_point_controller,
        pick_fsm_node,
    ])
