from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # -----------------------------
    # XACRO
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
                ' use_gazebo:=true'
            ]),
            value_type=str
        )
    }

    # -----------------------------
    # Gazebo Harmonic
    # -----------------------------
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # -----------------------------
    # Clock bridge (CRITICAL)
    # -----------------------------
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # -----------------------------
    # Spawn robot
    # -----------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ebot'
        ],
        output='screen'
    )

    # -----------------------------
    # Controllers (spawners ONLY)
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

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-ros-args',
            '--remap',
            '/diff_drive_controller/cmd_vel:=/cmd_vel'
        ],
        output='screen'
    )

    # -----------------------------
    # Robot State Publisher (DELAYED)
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    delayed_rsp = TimerAction(
        period=2.0,   # wait for controllers + joint_states
        actions=[robot_state_publisher]
    )

    # -----------------------------
    # Launch
    # -----------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        gz_sim,
        clock_bridge,
        spawn_robot,

        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        diff_drive_controller_spawner,

        delayed_rsp
    ])
