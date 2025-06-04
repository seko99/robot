#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Аргументы
        DeclareLaunchArgument(
            'odometry_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for odometry Arduino'
        ),
        DeclareLaunchArgument(
            'teleop_serial_port', 
            default_value='/dev/ttyUSB0',
            description='Serial port for teleop Arduino'
        ),
        DeclareLaunchArgument(
            'sonar_serial_port',
            default_value='/dev/ttyUSB1', 
            description='Serial port for sonar Arduino'
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.3',
            description='Distance between wheels in meters'
        ),
        DeclareLaunchArgument(
            'use_sonar',
            default_value='true',
            description='Whether to launch sonar node'
        ),
        
        # Запуск одометрии
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('odometry_serial_port'),
                'baud_rate': 115200,
                'wheel_base': LaunchConfiguration('wheel_base'),
            }]
        ),
        
        # Запуск телеуправления
        Node(
            package='robot_teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('teleop_serial_port'),
                'baud_rate': 115200,
                'max_linear': 0.5,
                'max_angular': 1.57,
                'wheel_base': LaunchConfiguration('wheel_base'),
                'max_motor_speed': 255,
            }]
        ),
        
        # Запуск сонара (условно)
        Node(
            package='robot_sonar',
            executable='sonar_node',
            name='sonar_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sonar')),
            parameters=[{
                'serial_port': LaunchConfiguration('sonar_serial_port'),
                'baud_rate': 115200,
                'frame_id': 'sonar_link',
                'min_range': 0.02,
                'max_range': 4.0,
                'field_of_view': 0.1,
                'publish_rate': 10.0,
            }]
        ),
    ])
