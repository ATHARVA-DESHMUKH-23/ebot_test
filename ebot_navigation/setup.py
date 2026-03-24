from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ebot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # ✅ THIS IS IMPORTANT
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
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
            'odom_nav_action_server = ebot_navigation.odom_nav_action_server:main',
            'odom_controller = ebot_navigation.odom_controller:main',
        ],
    },
)
