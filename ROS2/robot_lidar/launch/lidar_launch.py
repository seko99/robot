#!/usr/bin/env python3
"""
Launch file for LDS01RR Lidar Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS5',
        description='Serial port for the lidar'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='lidar',
        description='Frame ID for the lidar'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transform'
    )
    
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='base_link',
        description='Parent frame for TF transform'
    )
    
    # Create lidar node
    lidar_node = Node(
        package='robot_lidar',
        executable='lidar_node',
        name='robot-lidar',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
            'parent_frame': LaunchConfiguration('parent_frame'),
        }]
    )
    
    return LaunchDescription([
        port_arg,
        baud_rate_arg,
        frame_id_arg,
        publish_tf_arg,
        parent_frame_arg,
        lidar_node,
    ]) 