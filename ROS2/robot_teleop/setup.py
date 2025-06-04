from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
        ('share/' + package_name + '/config', ['config/teleop_params.yaml']),
        ('lib/' + package_name, glob('scripts/*')),  # Копировать скрипты в lib директорию
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='seko',
    maintainer_email='seko@example.com',
    description='ROS2 пакет для телеуправления роботом через Arduino',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = robot_teleop.teleop_node:main',
        ],
    },
)
