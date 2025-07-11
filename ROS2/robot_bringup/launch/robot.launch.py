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
            'sonar_serial_port',
            default_value='/dev/ttyUSB1', 
            description='Serial port for sonar Arduino'
        ),
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyS5', 
            description='Serial port for LDS01RR lidar'
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
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Whether to launch lidar node'
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
            'simulation_mode',
            default_value='false',
            description='Run in simulation mode (no serial connections)'
        ),
        DeclareLaunchArgument(
            'lidar_max_frequency',
            default_value='10.0',
            description='Maximum lidar publish frequency in Hz'
        ),
        DeclareLaunchArgument(
            'lidar_min_points',
            default_value='180',
            description='Minimum points required for lidar scan'
        ),
        DeclareLaunchArgument(
            'lidar_partial_scans',
            default_value='false',
            description='Enable partial scan publishing for higher frequency'
        ),
        
        # Запуск одометрии с интегрированным teleop
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('odometry_serial_port'),
                'baud_rate': 115200,
                'wheel_base': LaunchConfiguration('wheel_base'),
                'max_linear': LaunchConfiguration('max_linear'),
                'max_angular': LaunchConfiguration('max_angular'),
                'max_motor_speed': 255,
                'simulation_mode': LaunchConfiguration('simulation_mode'),
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
        
        # Запуск лидара (условно)
        Node(
            package='robot_lidar',
            executable='lidar_node',
            name='robot_lidar',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_serial_port'),
                'baud_rate': 115200,
                'frame_id': 'lidar',
                'publish_tf': True,
                'parent_frame': 'base_link',
                'max_publish_frequency': LaunchConfiguration('lidar_max_frequency'),
                'min_points_for_scan': LaunchConfiguration('lidar_min_points'),
                'allow_partial_scans': LaunchConfiguration('lidar_partial_scans'),
            }]
        ),
    ])
