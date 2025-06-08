#!/usr/bin/env python3
"""
LDS01RR Lidar ROS2 Node v2
Uses the corrected driver based on LDS.c protocol
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import time
from lds01rr_driver_v2 import LDS01RRDriverV2

class LDS01RRNodeV2(Node):
    def __init__(self):
        super().__init__('lds01rr_node_v2')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', 6.28318)  # 2*pi
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 15.0)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        
        # Create publishers
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.speed_publisher = self.create_publisher(Float32, 'lidar_speed', 10)
        
        # Initialize lidar driver
        try:
            self.driver = LDS01RRDriverV2(serial_port, baud_rate)
            self.get_logger().info(f'Connected to LDS01RR on {serial_port} at {baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to LDS01RR: {e}')
            raise
        
        # Create timer for polling lidar
        timer_period = 1.0 / publish_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('LDS01RR Node v2 started')
        
    def timer_callback(self):
        """Timer callback to poll lidar and publish data"""
        try:
            # Poll for new scan data
            measurements = self.driver.poll()
            
            if measurements:
                # Convert to LaserScan message
                scan_msg = self.create_laser_scan_msg(measurements)
                if scan_msg:
                    self.scan_publisher.publish(scan_msg)
                    
                    # Log scan info
                    valid_ranges = [r for r in scan_msg.ranges if r > 0 and r != float('inf')]
                    self.get_logger().debug(f'Published scan with {len(valid_ranges)}/{len(scan_msg.ranges)} valid points')
            
            # Publish motor speed
            speed = self.driver.get_speed_rpm()
            if speed > 0:
                speed_msg = Float32()
                speed_msg.data = speed
                self.speed_publisher.publish(speed_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
    
    def create_laser_scan_msg(self, measurements):
        """
        Create LaserScan message from measurement data
        
        Args:
            measurements: List of MeasurementData objects
            
        Returns:
            LaserScan message or None if no valid data
        """
        if not measurements:
            return None
        
        # Get scan data dictionary
        scan_data = self.driver.get_scan_data_dict(measurements)
        if not scan_data:
            return None
        
        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # Set scan parameters
        scan_msg.angle_min = float(self.angle_min)
        scan_msg.angle_max = float(self.angle_max)
        scan_msg.angle_increment = float(scan_data.get('angle_increment', 0.0))
        scan_msg.time_increment = float(scan_data.get('time_increment', 0.0))
        scan_msg.scan_time = float(scan_data.get('scan_time', 0.2))
        scan_msg.range_min = float(self.range_min)
        scan_msg.range_max = float(self.range_max)
        
        # Set range and intensity data
        ranges = scan_data.get('ranges', [])
        intensities = scan_data.get('intensities', [])
        
        scan_msg.ranges = [float(r) for r in ranges]
        scan_msg.intensities = [float(i) for i in intensities]
        
        return scan_msg
    
    def destroy_node(self):
        """Clean shutdown"""
        try:
            if hasattr(self, 'driver'):
                self.driver.disconnect()
                self.get_logger().info('LDS01RR driver disconnected')
        except Exception as e:
            self.get_logger().error(f'Error disconnecting driver: {e}')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LDS01RRNodeV2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 