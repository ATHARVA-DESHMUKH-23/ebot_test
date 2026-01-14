from setuptools import find_packages, setup

package_name = 'ebot_moveit_scripts'

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
            'moveit_demo = ebot_moveit_scripts.moveit_demo:main',
            'arm_pose_commander = ebot_moveit_scripts.pose_goal:main',
        ],
    },
)
