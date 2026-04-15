from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # -----------------------------
    # XACRO (NO Gazebo)
    # -----------------------------
    pkg_ebot_description = get_package_share_directory('ebot_description')

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
                ' use_gripper:=true',
                ' use_arm:=true'
            ]),
            value_type=str
        )
    }

    # --------------------
    # RPLidar Launch
    # --------------------
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'view_rplidar_a2m8_launch.py'
    )

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
        robot_state_publisher,
        # --------------------
        # 2. Fake HW + Encoder (after 3 sec)
        # --------------------
        TimerAction(
            period=3.0,
            actions=[

                Node(
                    package='ebot_serial',
                    executable='fake_hw_interface',
                    name='fake_hw_interface',
                    output='screen'
                ),

                Node(
                    package='ebot_serial',
                    executable='encoder_odimetery',
                    name='encoder_odimetery',
                    output='screen'
                ),
            ]
        ),


         # --------------------
        # 3. IMU + Magnetometer (after 5 sec total)
        # (3 sec + 2 sec)
        # --------------------
        TimerAction(
            period=4.0,
            actions=[

                Node(
                    package='ebot_slam',
                    executable='imu_yaw',
                    name='imu_yaw',
                    output='screen'
                ),

                Node(
                    package='ebot_slam',
                    executable='mag',
                    name='mag',
                    output='screen'
                ),
            ]
        ),

       # --------------------
        # 4. Arduino Communication (after 5 sec)
        # (same stage as IMU → no delay between them)
        # --------------------
        TimerAction(
            period=5.0,
            actions=[

                Node(
                    package='ebot_serial',
                    executable='ebot_yaw',
                    name='ebot_yaw',
                    output='screen'
                ),

                Node(
                    package='ebot_serial',
                    executable='cmdvel_to_arduino',
                    name='cmdvel_to_arduino',
                    output='screen'
                ),
            ]
        ),

        
        # --------------------
        # Magnetometer Calibration Script
        # --------------------
        # ExecuteProcess(
        #     cmd=['python3', magne_script],
        #     output='screen'
        # ),

        # --------------------
        # RPLidar
        # --------------------
         # --------------------
        # 5. RPLidar (after 6 sec)
        # --------------------
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rplidar_launch_file)
                )
            ]
        ),
       
        # --------------------
        # 6. EKF (LAST, after 10 sec)
        # --------------------
        TimerAction(
            period=10.0,
            actions=[

                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[os.path.join(
                        pkg_ebot_description,
                        'config',
                        'ekf_config.yaml'
                    )]
                )
            ]
        ),
    ])
