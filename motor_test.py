#!/usr/bin/env python3
import serial
import struct
import time
import sys
import threading
from datetime import datetime

# Настройки serial порта
SERIAL_PORT = '/dev/ttyUSB0'  # Измените на нужный порт
BAUD_RATE = 115200

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
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

# struct SensorData: long timestamp, float left_odometry, float right_odometry, int left_ticks, int right_ticks, 
#                   bool left_h2_state, bool right_h2_state, bool left_flag, bool right_flag
SENSOR_DATA_FORMAT = '<IffHH????'  # long + 2 float + 2 int + 4 bool
SENSOR_DATA_SIZE = struct.calcsize(SENSOR_DATA_FORMAT)

class MotorController:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.timestamp = 0
        self.left_odometry = 0.0
        self.right_odometry = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_h2_state = False
        self.right_h2_state = False
        self.left_flag = False
        self.right_flag = False
        
    def connect(self):
        """Подключение к serial порту"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)  # Ждем инициализации Arduino
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Подключен к {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Ошибка подключения к {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Отключение от serial порта"""
        if self.ser and self.ser.is_open:
            self.send_command(CMD_STOP_MOTORS, 0)  # Останавливаем моторы
            self.ser.close()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Отключен от {self.port}")
    
    def send_command(self, cmd, value):
        """Отправка команды мотору"""
        if not self.ser or not self.ser.is_open:
            print("Serial порт не открыт!")
            return False
        
        try:
            packed_command = struct.pack(COMMAND_FORMAT, cmd, value)
            self.ser.write(packed_command)
            
            cmd_names = {
                CMD_LEFT_FORWARD: "Левый мотор вперед",
                CMD_LEFT_BACKWARD: "Левый мотор назад", 
                CMD_RIGHT_FORWARD: "Правый мотор вперед",
                CMD_RIGHT_BACKWARD: "Правый мотор назад",
                CMD_LEFT_STOP: "Остановка левого мотора",
                CMD_RIGHT_STOP: "Остановка правого мотора",
                CMD_STOP_MOTORS: "Остановка всех моторов"
            }
            
            cmd_name = cmd_names.get(cmd, f"Неизвестная команда {cmd}")
            print(f"[{datetime.now().strftime('%H:%M:%S')}] {cmd_name} (скорость: {value})")
            return True
            
        except Exception as e:
            print(f"Ошибка отправки команды: {e}")
            return False
    
    def read_sensor_data(self):
        """Чтение данных одометрии"""
        if not self.ser or not self.ser.is_open:
            return
        
        try:
            # Проверяем, есть ли данные для чтения
            if self.ser.in_waiting >= SENSOR_DATA_SIZE:
                data = self.ser.read(SENSOR_DATA_SIZE)
                if len(data) == SENSOR_DATA_SIZE:
                    unpacked = struct.unpack(SENSOR_DATA_FORMAT, data)
                    timestamp, left_odo, right_odo, left_ticks, right_ticks, left_h2, right_h2, left_flag, right_flag = unpacked
                    
                    # Обновляем внутренние переменные
                    self.timestamp = timestamp
                    self.left_odometry = left_odo
                    self.right_odometry = right_odo
                    self.left_ticks = left_ticks
                    self.right_ticks = right_ticks
                    self.left_h2_state = left_h2
                    self.right_h2_state = right_h2
                    self.left_flag = left_flag
                    self.right_flag = right_flag
                    
                    # Выводим основные данные
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] T:{timestamp}ms Одометрия - Левая: {left_odo:.3f}м, Правая: {right_odo:.3f}м")
                    #print(f"    Тики: L={left_ticks}, R={right_ticks} | H2: L={left_h2}, R={right_h2} | Флаги: L={left_flag}, R={right_flag}")
        except Exception as e:
            print(f"Ошибка чтения данных сенсоров: {e}")
    
    def sensor_reader_thread(self):
        """Поток для постоянного чтения данных сенсоров"""
        while self.running:
            self.read_sensor_data()
            time.sleep(0.01)  # Небольшая задержка
    
    def motor_control_loop(self):
        """Основной цикл управления моторами"""
        motor_speed = 150  # Скорость моторов (0-255)
        state = 0  # 0 - левый мотор, 1 - правый мотор
        
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Начинаем тест моторов...")
        print("Каждые 3 секунды будет включаться попеременно левый и правый мотор")
        print("Для остановки нажмите Ctrl+C")
        
        try:
            while self.running:
                if state == 0:
                    # Включаем левый мотор вперед
                    self.send_command(CMD_RIGHT_STOP, 0)  # Останавливаем правый
                    time.sleep(0.1)
                    self.send_command(CMD_LEFT_FORWARD, motor_speed)
                    state = 1
                else:
                    # Включаем правый мотор вперед  
                    self.send_command(CMD_LEFT_STOP, 0)  # Останавливаем левый
                    time.sleep(0.1)
                    self.send_command(CMD_RIGHT_FORWARD, motor_speed)
                    state = 0
                
                # Ждем 3 секундыжно протестировать
                time.sleep(3.0)
                
        except KeyboardInterrupt:
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Получен сигнал остановки...")
        finally:
            self.send_command(CMD_STOP_MOTORS, 0)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Все моторы остановлены")
    
    def run(self):
        """Запуск программы"""
        if not self.connect():
            return
        
        self.running = True
        
        # Запускаем поток для чтения данных сенсоров
        sensor_thread = threading.Thread(target=self.sensor_reader_thread, daemon=True)
        sensor_thread.start()

        motor_speed = 150  # Скорость моторов (0-255)

        self.send_command(CMD_LEFT_FORWARD, motor_speed)
        self.send_command(CMD_RIGHT_FORWARD, motor_speed)

        time.sleep(10.0)

        self.send_command(CMD_LEFT_STOP, 0)  # Останавливаем правый
        self.send_command(CMD_RIGHT_STOP, 0)  # Останавливаем правый

#        try:
#            # Запускаем основной цикл управления моторами
#            self.motor_control_loop()
#        finally:
#            self.running = False
#            self.disconnect()

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = SERIAL_PORT
    
    print("=== Тест моторов с одометрией ===")
    print(f"Порт: {port}")
    print(f"Скорость: {BAUD_RATE}")
    print()
    
    controller = MotorController(port)
    controller.run()

if __name__ == "__main__":
    main()
