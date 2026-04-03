from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Robot description (URDF from xacro)
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
                ' use_gazebo:=true'
            ]),
            value_type=str
        )
    }

    # SRDF
    srdf_file = os.path.join(
        get_package_share_directory("moveo_moveit_config"),
        "config",
        "moveo_urdf.srdf"
    )

    with open(srdf_file, 'r') as f:
        robot_description_semantic = {
            'robot_description_semantic': f.read()
        }

    moveit_config = get_package_share_directory("moveo_moveit_config")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            os.path.join(moveit_config, "config", "kinematics.yaml"),
            os.path.join(moveit_config, "config", "ompl_planning.yaml"),
            os.path.join(moveit_config, "config", "joint_limits.yaml"),
            os.path.join(moveit_config, "config", "planning_pipeline.yaml"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription([
        move_group,
        rviz
    ])