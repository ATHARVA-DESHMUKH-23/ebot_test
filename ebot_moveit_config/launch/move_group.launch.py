from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_pkg = get_package_share_directory("ebot_description")
    moveit_pkg = get_package_share_directory("ebot_moveit_config")

    robot_description = {
        "robot_description": Command([
            "xacro ",
            os.path.join(description_pkg, "models/ebot/ebot.urdf.xacro")
        ])
    }

    robot_description_semantic = {
        "robot_description_semantic": open(
            os.path.join(moveit_pkg, "config/ebot.srdf")
        ).read()
    }

    kinematics_yaml = os.path.join(moveit_pkg, "config/kinematics.yaml")
    ompl_yaml = os.path.join(moveit_pkg, "config/ompl_planning.yaml")
    controllers_yaml = os.path.join(moveit_pkg, "config/moveit_controllers.yaml")

    return LaunchDescription([

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_yaml,
                controllers_yaml,
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
            ],
        )
    ])
