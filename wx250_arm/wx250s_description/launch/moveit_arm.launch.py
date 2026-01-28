from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("wx250s")
        .robot_description(file_path="config/wx250s.urdf.xacro")
        .to_moveit_configs()
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
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.ros2_controllers_path,
        ],
        output="screen",
    )

    # --------------------
    # Controllers
    # --------------------
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wx250s_arm_controller"],
        output="screen",
    )

    # --------------------
    # MoveIt move_group
    # --------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        rsp,
        ros2_control,
        jsb,
        arm_ctrl,
        move_group,
    ])
