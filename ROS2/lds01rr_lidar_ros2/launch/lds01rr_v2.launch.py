#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for LDS01RR lidar'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='lidar_link',
            description='Frame ID for lidar data'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Publishing rate in Hz'
        ),
        DeclareLaunchArgument(
            'angle_min',
            default_value='0.0',
            description='Minimum scan angle in radians'
        ),
        DeclareLaunchArgument(
            'angle_max',
            default_value='6.28318',
            description='Maximum scan angle in radians (2*pi)'
        ),
        DeclareLaunchArgument(
            'range_min',
            default_value='0.05',
            description='Minimum range in meters'
        ),
        DeclareLaunchArgument(
            'range_max',
            default_value='15.0',
            description='Maximum range in meters'
        ),
        
        # Launch the LDS01RR node v2
        Node(
            package='lds01rr_lidar_ros2',
            executable='lidar_node_v2',
            name='lds01rr_lidar_v2',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'angle_min': LaunchConfiguration('angle_min'),
                'angle_max': LaunchConfiguration('angle_max'),
                'range_min': LaunchConfiguration('range_min'),
                'range_max': LaunchConfiguration('range_max'),
            }]
        )
    ]) 