#!/usr/bin/env python3
"""
ROS2 Node for LDS01RR Lidar
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import threading
import time

from .lds01rr_driver_v2 import LDS01RRDriverV2


class LDS01RRLidarNode(Node):
    def __init__(self):
        super().__init__('robot_lidar')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyS5')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'lidar')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('parent_frame', 'base_link')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        
        # Create publisher
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        # Create TF broadcaster if needed
        self.tf_broadcaster = None
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize lidar driver
        try:
            self.driver = LDS01RRDriverV2(serial_port=self.serial_port, baud_rate=self.baud_rate)
            self.get_logger().info(f'LDS01RR driver initialized on {self.serial_port} at {self.baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LDS01RR driver: {e}')
            return
        
        # Start polling thread
        self.running = True
        self.poll_thread = threading.Thread(target=self._poll_loop)
        self.poll_thread.daemon = True
        self.poll_thread.start()
        
        self.get_logger().info('LDS01RR Lidar Node started')
    
    def _poll_loop(self):
        """Main polling loop for lidar data"""
        while self.running and rclpy.ok():
            try:
                points = self.driver.poll()
                if points is not None and len(points) > 0:
                    self._publish_scan(points)
            except Exception as e:
                self.get_logger().error(f'Error in poll loop: {e}')
                time.sleep(0.1)
    
    def _publish_scan(self, points):
        """
        Publish laser scan message
        
        Args:
            points: List of MeasuredPoint objects
        """
        try:
            # Process lidar data
            scan_data = self.driver.get_scan_data_dict(points)
            if not scan_data:
                return
            
            # Create LaserScan message
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            msg.angle_min = scan_data['angle_min']
            msg.angle_max = scan_data['angle_max']
            msg.angle_increment = scan_data['angle_increment']
            msg.time_increment = scan_data['time_increment']
            msg.scan_time = scan_data['scan_time']
            msg.range_min = scan_data['range_min']
            msg.range_max = scan_data['range_max']
            
            msg.ranges = scan_data['ranges']
            msg.intensities = scan_data['intensities']
            
            # Publish scan
            self.scan_publisher.publish(msg)
            
            # Publish TF if enabled
            if self.publish_tf and self.tf_broadcaster:
                self._publish_tf(msg.header.stamp)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing scan: {e}')
    
    def _publish_tf(self, timestamp):
        """
        Publish transform from base_link to lidar
        
        Args:
            timestamp: Timestamp for the transform
        """
        try:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.frame_id
            
            # Set transform (identity transform - lidar at robot center)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Error publishing TF: {e}')
    
    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'driver'):
            self.driver.disconnect()
        if hasattr(self, 'poll_thread'):
            self.poll_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = LDS01RRLidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.get_logger().info('Shutting down robot_lidar node')
            except:
                pass
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main() 