#!/usr/bin/env python3

import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

DATA_FORMAT = '<Bf'
DATA_SIZE = struct.calcsize(DATA_FORMAT)

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')
        
        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'sonar_link')
        self.declare_parameter('min_range', 0.02)  # 2 cm
        self.declare_parameter('max_range', 4.0)   # 400 cm
        self.declare_parameter('field_of_view', 0.1)  # радиан
        self.declare_parameter('publish_rate', 10.0)  # Гц
        
        # Получение параметров
        self.serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        # Создание publisher
        self.publisher = self.create_publisher(Range, '/ultrasonic_sensor/range', 10)
        
        # Подключение к Arduino
        try:
            self.serial = serial.Serial(self.serial_port, baudrate=baud_rate, timeout=1.0)
            self.get_logger().info(f"Opened serial port: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {self.serial_port}: {e}")
            raise
        
        # Таймер для чтения данных
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.read_serial_data)

    def read_serial_data(self):
        try:
            if self.serial.in_waiting >= DATA_SIZE:
                data = self.serial.read(DATA_SIZE)
                if len(data) == DATA_SIZE:
                    cmd, us_distance = struct.unpack(DATA_FORMAT, data)
                    
                    self.get_logger().debug(
                        f"Received: cmd={cmd}, distance={us_distance} cm",
                        throttle_duration_sec=1.0
                    )
                    
                    self.publish_sonar_data(us_distance)
                else:
                    self.get_logger().warn("Incomplete data read")
            
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def publish_sonar_data(self, distance_cm):
        # Получение параметров
        frame_id = self.get_parameter('frame_id').value
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        field_of_view = self.get_parameter('field_of_view').value
        
        # Создание сообщения Range
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = field_of_view
        msg.min_range = min_range
        msg.max_range = max_range
        
        # Преобразование из см в метры
        distance_m = float(distance_cm) / 100.0
        
        # Проверка валидности измерения
        if min_range <= distance_m <= max_range:
            msg.range = distance_m
        else:
            # Если измерение вне диапазона, устанавливаем специальные значения
            if distance_m < min_range:
                msg.range = min_range - 0.01  # Слишком близко
            else:
                msg.range = float('inf')  # Слишком далеко или нет отражения
        
        self.publisher.publish(msg)

    def destroy_node(self):
        """Корректное завершение работы"""
        if hasattr(self, 'serial') and self.serial and self.serial.is_open:
            self.serial.close()
            self.get_logger().info("Disconnected from Arduino")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Получен сигнал остановки...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
