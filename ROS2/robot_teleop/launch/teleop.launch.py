#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Arduino connection'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'max_linear',
            default_value='0.5',
            description='Maximum linear velocity in m/s'
        ),
        DeclareLaunchArgument(
            'max_angular',
            default_value='1.57',
            description='Maximum angular velocity in rad/s'
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.3',
            description='Distance between wheels in meters'
        ),
        
        Node(
            package='robot_teleop',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'max_linear': LaunchConfiguration('max_linear'),
                'max_angular': LaunchConfiguration('max_angular'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'max_motor_speed': 255,
            }]
        )
    ])
