from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_sonar'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sonar.launch.py']),
        ('share/' + package_name + '/config', ['config/sonar_params.yaml']),
        ('lib/' + package_name, glob('scripts/*')),  # Копировать скрипты в lib директорию
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='seko',
    maintainer_email='seko@example.com',
    description='ROS2 пакет для чтения данных с ультразвукового датчика через Arduino',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node = robot_sonar.sonar_node:main',
        ],
    },
)
