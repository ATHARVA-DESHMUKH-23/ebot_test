from setuptools import setup
import os
from glob import glob

package_name = 'ebot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebot',
    maintainer_email='ebot@todo.todo',
    description='LiDAR + EKF + IMU SLAM package',
    license='Apache-2.0',
    tests_require=['pytest'],

    # 🔴 THIS WAS MISSING — THIS IS THE FIX
    entry_points={
        'console_scripts': [
            'fake_odom = ebot_slam.fake_odom:main',
            'imu_node = ebot_slam.imu_node:main',
        ],
    },
)
