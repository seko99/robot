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
            'wheel_base',
            default_value='0.3',
            description='Distance between wheels in meters'
        ),
        
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'wheel_base': LaunchConfiguration('wheel_base'),
            }]
        )
    ])
