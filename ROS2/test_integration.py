#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class TestIntegrationNode(Node):
    def __init__(self):
        super().__init__('test_integration_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.get_logger().info("Starting integration test...")
        
        # Таймер для отправки команд
        self.create_timer(2.0, self.send_test_commands)
        self.command_count = 0
        
    def odom_callback(self, msg):
        self.get_logger().info(f"Odometry received: x={msg.pose.pose.position.x:.3f}, "
                             f"y={msg.pose.pose.position.y:.3f}, "
                             f"linear_vel={msg.twist.twist.linear.x:.3f}")
    
    def send_test_commands(self):
        if self.command_count >= 5:
            self.get_logger().info("Test completed!")
            return
            
        twist = Twist()
        
        if self.command_count == 0:
            twist.linear.x = 0.1
            self.get_logger().info("Sending forward command")
        elif self.command_count == 1:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info("Sending turn command")
        elif self.command_count == 2:
            twist.linear.x = -0.1
            self.get_logger().info("Sending backward command")
        elif self.command_count == 3:
            twist.linear.x = 0.0
            twist.angular.z = -0.5
            self.get_logger().info("Sending reverse turn command")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Sending stop command")
        
        self.cmd_vel_pub.publish(twist)
        self.command_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestIntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 