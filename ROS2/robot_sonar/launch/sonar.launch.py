#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for Arduino connection'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='sonar_link',
            description='Frame ID for the sonar sensor'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Publishing rate in Hz'
        ),
        
        Node(
            package='robot_sonar',
            executable='sonar_node',
            name='sonar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'min_range': 0.02,  # 2 cm
                'max_range': 4.0,   # 400 cm
                'field_of_view': 0.1,  # радиан
                'publish_rate': LaunchConfiguration('publish_rate'),
            }]
        )
    ])
