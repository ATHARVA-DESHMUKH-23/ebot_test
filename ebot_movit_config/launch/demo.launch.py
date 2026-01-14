from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ebot",
        package_name="ebot_movit_config"
    ).to_moveit_configs()

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([
                FindPackageShare("ebot_movit_config"),
                "config",
                "ros2_controllers.yaml"
            ]),
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    return LaunchDescription([
        ros2_control_node,
        joint_state_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        *generate_demo_launch(moveit_config).entities
    ])
