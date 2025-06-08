from setuptools import setup
import os
from glob import glob

package_name = 'lds01rr_lidar_ros2'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 driver for LDS01RR Lidar',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = lds01rr_lidar_ros2.lidar_node:main',
            'lidar_node_v2 = lds01rr_lidar_ros2.lds01rr_node_v2:main',
            'test_driver_v2 = lds01rr_lidar_ros2.test_driver_v2:main',
        ],
    },
) 