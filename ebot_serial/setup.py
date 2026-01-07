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

        ],
    },
)
