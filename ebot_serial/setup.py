from setuptools import find_packages, setup

package_name = 'ebot_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rajvardhan',
    maintainer_email='atharvad2366@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmdvel_to_arduino = ebot_serial.cmdvel_to_ardiuno:main',
            'twist_to_stamped = ebot_serial.twist_to_stamped:main',
            'cmdvel_to_odom = ebot_serial.cmdvel_to_odom:main',
            'fake_hw_interface = ebot_serial.fake_hw_interface:main',
            'encoder_odimetery = ebot_serial.encoder_odimetery:main',
            'cartesian_servo = ebot_serial.cartesian_servo:main',
            'keyboard_ee_vel = ebot_serial.keyboard_ee_vel:main',
            'ee_plotter = ebot_serial.ee_ploter:main',
            'ee_velocity_estimator = ebot_serial.ee_velocity_estimator:main',
            'ee_velocity_allocator = ebot_serial.ee_velocity_allocator:main',
            'cartesian_servo_kdl = ebot_serial.cartesian_servo_kdl:main',
            'cartesian_point = ebot_serial.cartesian_point:main',
            'kdl_fk_test = ebot_serial.kdl_fk:main',
            'aruco_perception = ebot_serial.aruco_perception:main',
            'aruco_state_machine = ebot_serial.aruco_state_machine:main',
        ],
    },
)

# ros2 run robot_state_publisher robot_state_publisher   --ros-args -p robot_description:="$(xacro ~/EBOT_TEST/src/ebot_description/models/ebot/ebot_description.xacro)"
# ros2 run joint_state_publisher joint_state_publisher
