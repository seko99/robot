#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time

# Команды для моторов (из Motors.ino)
CMD_LEFT_FORWARD = 0
CMD_LEFT_BACKWARD = 1
CMD_RIGHT_FORWARD = 2
CMD_RIGHT_BACKWARD = 3
CMD_LEFT_STOP = 4
CMD_RIGHT_STOP = 5
CMD_STOP_MOTORS = 6

# Форматы структур данных
COMMAND_FORMAT = '<BB'  # struct Command: uint8_t cmd, uint8_t value

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear', 0.5)  # м/с
        self.declare_parameter('max_angular', 1.57)  # рад/с
        self.declare_parameter('wheel_base', 0.3)  # м
        self.declare_parameter('max_motor_speed', 255)  # максимальная скорость мотора PWM
        
        # Подключение к Arduino
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)  # Ждем инициализации Arduino
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise
        
        # Подписка на команды
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
    def send_command(self, cmd, value):
        """Отправка команды мотору через бинарный протокол"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("Serial порт не открыт!")
            return False
        
        try:
            packed_command = struct.pack(COMMAND_FORMAT, cmd, value)
            self.ser.write(packed_command)
            return True
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки команды: {e}")
            return False

    def cmd_vel_callback(self, msg):
        # Параметры
        wheel_base = self.get_parameter('wheel_base').value
        max_linear = self.get_parameter('max_linear').value
        max_angular = self.get_parameter('max_angular').value
        max_motor_speed = self.get_parameter('max_motor_speed').value
        
        # Ограничение скоростей
        linear = max(min(msg.linear.x, max_linear), -max_linear)
        angular = max(min(msg.angular.z, max_angular), -max_angular)
        
        # Преобразование в скорости колес (дифференциальный привод)
        left = linear - (angular * wheel_base) / 2.0
        right = linear + (angular * wheel_base) / 2.0
        
        # Нормализация скоростей
        max_speed = max(abs(left), abs(right))
        if max_speed > max_linear:
            left = left * max_linear / max_speed
            right = right * max_linear / max_speed
        
        # Преобразование в PWM значения (0-255)
        left_pwm = int(abs(left) * max_motor_speed / max_linear)
        right_pwm = int(abs(right) * max_motor_speed / max_linear)
        
        # Ограничение PWM значений
        left_pwm = max(0, min(255, left_pwm))
        right_pwm = max(0, min(255, right_pwm))
        
        # Определение направления и отправка команд
        if left == 0:
            self.send_command(CMD_LEFT_STOP, 0)
        elif left > 0:
            self.send_command(CMD_LEFT_FORWARD, left_pwm)
        else:
            self.send_command(CMD_LEFT_BACKWARD, left_pwm)
            
        if right == 0:
            self.send_command(CMD_RIGHT_STOP, 0)
        elif right > 0:
            self.send_command(CMD_RIGHT_FORWARD, right_pwm)
        else:
            self.send_command(CMD_RIGHT_BACKWARD, right_pwm)
    
    def destroy_node(self):
        """Корректное завершение работы"""
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.send_command(CMD_STOP_MOTORS, 0)  # Останавливаем моторы
            self.ser.close()
            self.get_logger().info("Disconnected from Arduino")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Получен сигнал остановки...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
