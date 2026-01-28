from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("wx250s")
        .robot_description(file_path="config/wx250s.urdf.xacro")
        .to_moveit_configs()
    )

    servo_params = PathJoinSubstitution([
        FindPackageShare("wx250s_moveit_config"),
        "config",
        "servo.yaml"
    ])

    return LaunchDescription([
        Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            output="screen",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])
