#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import serial
import struct
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.3)  # Расстояние между колесами
        
        # Подключение к Arduino
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise
        
        # Формат структуры SensorData из Motors.ino
        # struct SensorData: uint32_t timestamp, float left_odometry, float right_odometry, 
        #                   uint16_t left_ticks, uint16_t right_ticks, bool left_h2_state, 
        #                   bool right_h2_state, bool left_flag, bool right_flag
        self.sensor_data_format = '<IffHH????'
        self.sensor_data_size = struct.calcsize(self.sensor_data_format)
        
        # Инициализация переменных одометрии
        self.left_prev = 0.0
        self.right_prev = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # ROS интерфейсы
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Таймер для чтения данных
        self.create_timer(0.05, self.read_serial)  # 20 Гц

    def read_serial(self):
        """Чтение бинарных данных от Arduino"""
        if self.ser.in_waiting >= self.sensor_data_size:
            try:
                data = self.ser.read(self.sensor_data_size)
                if len(data) == self.sensor_data_size:
                    # Распаковка структуры SensorData
                    unpacked = struct.unpack(self.sensor_data_format, data)
                    timestamp, left_odo, right_odo, left_ticks, right_ticks, left_h2, right_h2, left_flag, right_flag = unpacked
                    
                    # Обрабатываем одометрию (расстояния уже готовые в метрах)
                    self.process_odometry(left_odo, right_odo)
                    
            except struct.error as e:
                self.get_logger().error(f"Struct unpack error: {e}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

    def process_odometry(self, left, right):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Вычисление пройденного расстояния (расстояния уже в метрах от Arduino)
        d_left = left - self.left_prev
        d_right = right - self.right_prev
        self.left_prev = left
        self.right_prev = right
        
        # Кинематика дифференциального привода
        wheel_base = self.get_parameter('wheel_base').value
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / wheel_base
        
        # Интегрирование положения
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta
        
        # Нормализация угла
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Создание сообщения Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Заполнение позиции
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.theta)
        
        # Заполнение скорости
        if dt > 0:
            odom_msg.twist.twist.linear.x = d_center / dt
            odom_msg.twist.twist.angular.z = d_theta / dt
        
        # Публикация
        self.odom_pub.publish(odom_msg)
        self.publish_tf(odom_msg)
        self.last_time = current_time

    def publish_tf(self, odom_msg):
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def destroy_node(self):
        """Корректное завершение работы"""
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Disconnected from Arduino")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Получен сигнал остановки...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
