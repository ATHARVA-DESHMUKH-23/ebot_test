#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    def configure_setup(context):

        use_sim_time = (
            LaunchConfiguration("use_sim_time")
            .perform(context)
            .lower() == "true"
        )

        robot_name_str = "moveo"

        package_name_moveit_config = "moveo_moveit_config"

        pkg_share_moveit_config = FindPackageShare(
            package=package_name_moveit_config
        ).find(package_name_moveit_config)

        config_path = os.path.join(
            pkg_share_moveit_config,
            "config",
            robot_name_str
        )

        joint_limits_file_path = os.path.join(
            config_path,
            "joint_limits.yaml"
        )

        kinematics_file_path = os.path.join(
            config_path,
            "kinematics.yaml"
        )

        moveit_controllers_file_path = os.path.join(
            config_path,
            "moveit_controllers.yaml"
        )

        srdf_model_path = os.path.join(
            config_path,
            f"{robot_name_str}.srdf"
        )

        pilz_cartesian_limits_file_path = os.path.join(
            config_path,
            "pilz_cartesian_limits.yaml"
        )

        moveit_config = (
            MoveItConfigsBuilder(
                robot_name_str,
                package_name=package_name_moveit_config
            )
            .trajectory_execution(
                file_path=moveit_controllers_file_path
            )
            .robot_description_semantic(
                file_path=srdf_model_path
            )
            .joint_limits(
                file_path=joint_limits_file_path
            )
            .robot_description_kinematics(
                file_path=kinematics_file_path
            )
            .planning_pipelines(
                pipelines=["ompl"],
                default_planning_pipeline="ompl"
            )
            .pilz_cartesian_limits(
                file_path=pilz_cartesian_limits_file_path
            )
            .to_moveit_configs()
        )

        servo_params = {
            "moveit_servo": {

                "move_group_name": "arm",

                "planning_frame": "ebot_base_link",

                "ee_frame_name": "Link_5",

                "robot_link_command_frame": "Link_5",

                "command_in_type": "speed_units",

                "scale": {
                    "linear": 0.1,
                    "rotational": 0.2,
                    "joint": 0.5
                },

                "publish_period": 0.02,

                "low_latency_mode": False,

                "incoming_command_timeout": 0.1,

                "num_outgoing_halt_msgs_to_publish": 4,

                "lower_singularity_threshold": 17.0,

                "hard_stop_singularity_threshold": 30.0,

                "leaving_singularity_threshold_multiplier": 2.0,

                "joint_limit_margin": 0.1,

                "command_out_topic":
                    "/arm_controller/joint_trajectory",

                "command_out_type":
                    "trajectory_msgs/JointTrajectory",

                "publish_joint_positions": True,

                "publish_joint_velocities": True,

                "publish_joint_accelerations": False,

                "joint_topic": "/joint_states",

                "status_topic": "~/status",

                "cartesian_command_in_topic":
                    "~/delta_twist_cmds",

                "joint_command_in_topic":
                    "~/delta_joint_cmds",

                "pose_command_in_topic":
                    "~/pose_target_cmds",

                "check_collisions": True,

                "collision_check_rate": 10.0,

                "self_collision_proximity_threshold": 0.01,

                "scene_collision_proximity_threshold": 0.02,

                "use_smoothing": False,

                "smoothing_filter_plugin_name":
                    "online_signal_smoothing::ButterworthFilterPlugin"
            }
        }

        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            output="screen",
            parameters=[
                servo_params,
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
            ],
        )

        return [servo_node]

    return LaunchDescription([
        declare_use_sim_time,
        OpaqueFunction(function=configure_setup)
    ])
