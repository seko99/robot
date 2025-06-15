from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_odometry'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odometry.launch.py']),
        ('share/' + package_name + '/config', ['config/odometry_params.yaml']),
        ('lib/' + package_name, glob('scripts/*')),  # Копировать скрипты в lib директорию
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'nav_msgs',
        'tf2_ros',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='seko',
    maintainer_email='seko@example.com',
    description='ROS2 пакет для публикации одометрии и управления роботом с интегрированным телеуправлением',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = robot_odometry.odometry_node:main',
        ],
    },
)
