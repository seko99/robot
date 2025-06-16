#!/usr/bin/env python3

import serial
import struct
import time

def read_sensor_data(port='/dev/ttyUSB1', baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        
        print(f"Подключен к {port} на скорости {baudrate}")
        print("Ожидание данных с датчиков...")
        print("Ctrl+C для выхода")
        
        while True:
            if ser.in_waiting >= 5:
                data = ser.read(5)
                if len(data) == 5:
                    cmd, us_distance = struct.unpack('<Bf', data)
                    print(f"CMD: {cmd}, Расстояние: {us_distance:.2f} см")
            
            time.sleep(0.1)
                    
    except serial.SerialException as e:
        print(f"Ошибка подключения к {port}: {e}")
        print("Попробуйте другой порт: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyACM0")
    except KeyboardInterrupt:
        print("\nЗавершение работы")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
    read_sensor_data(port) 