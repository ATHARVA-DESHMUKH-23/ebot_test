from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("wx250s")
        .robot_description(file_path="config/wx250s.urdf.xacro")
        .to_moveit_configs()
    )

    # --------------------
    # ros2_control YAML (MANUAL LOAD)
    # --------------------
    ros2_controllers_yaml = os.path.join(
        get_package_share_directory("wx250s_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # --------------------
    # Robot State Publisher
    # --------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # --------------------
    # ros2_control
    # --------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_yaml,
        ],
        output="screen",
    )

    # --------------------
    # Controllers
    # --------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wx250s_arm_controller"],
        output="screen",
    )

    # --------------------
    # Move Group
    # --------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([
        rsp,
        ros2_control_node,
        joint_state_broadcaster,
        arm_controller,
        move_group,
    ])
