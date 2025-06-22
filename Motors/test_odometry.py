#!/usr/bin/env python3

import serial
import struct
import time
import sys

SENSOR_DATA_FORMAT = '<Iffll????'
SENSOR_DATA_SIZE = struct.calcsize(SENSOR_DATA_FORMAT)

def format_time(timestamp_ms):
    hours = (timestamp_ms // (1000 * 60 * 60)) % 24
    minutes = (timestamp_ms // (1000 * 60)) % 60
    seconds = (timestamp_ms // 1000) % 60
    milliseconds = timestamp_ms % 1000
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}.{milliseconds:03d}"

def print_sensor_data(data):
    timestamp, left_odo, right_odo, left_ticks, right_ticks, left_h2, right_h2, left_flag, right_flag = data
    
    current_time = format_time(timestamp)
    
    print(f"\n╔═══════════════ ОДОМЕТРИЯ РОБОТА [{current_time}] ═══════════════╗")
    print(f"║ Время (мс): {timestamp:>10} │ Время: {current_time:>12} ║")
    print(f"╠═══════════════════════════════════════════════════════════════╣")
    print(f"║ РАССТОЯНИЯ (метры):                                           ║")
    print(f"║   Левое колесо:  {left_odo:>10.6f} м                            ║")
    print(f"║   Правое колесо: {right_odo:>10.6f} м                            ║")
    print(f"║   Разность:      {abs(left_odo - right_odo):>10.6f} м                            ║")
    print(f"╠═══════════════════════════════════════════════════════════════╣")
    print(f"║ ЭНКОДЕРЫ (тики):                                              ║")
    print(f"║   Левый:   {left_ticks:>6} тиков                                ║") 
    print(f"║   Правый:  {right_ticks:>6} тиков                                ║")
    print(f"║   Разность: {abs(left_ticks - right_ticks):>5} тиков                                ║")
    print(f"╠═══════════════════════════════════════════════════════════════╣")
    print(f"║ СОСТОЯНИЯ ЭНКОДЕРОВ:                                          ║")
    print(f"║   Левый H2:    {'HIGH' if left_h2 else 'LOW ':>4}                                ║")
    print(f"║   Правый H2:   {'HIGH' if right_h2 else 'LOW ':>4}                                ║")
    print(f"║   Левый флаг:  {'SET ' if left_flag else 'CLEAR':>5}                                ║")
    print(f"║   Правый флаг: {'SET ' if right_flag else 'CLEAR':>5}                                ║")
    print(f"╚═══════════════════════════════════════════════════════════════╝")

    if left_odo != 0 or right_odo != 0:
        center_distance = (left_odo + right_odo) / 2.0
        wheel_base = 0.265  # из параметров одометрии
        rotation = (right_odo - left_odo) / wheel_base
        
        print(f"║ ДВИЖЕНИЕ:                                                     ║")
        print(f"║   Центр масс:    {center_distance:>10.6f} м                           ║")
        print(f"║   Поворот:       {rotation:>10.6f} рад ({rotation * 180 / 3.14159:>6.2f}°)           ║")
        print(f"╚═══════════════════════════════════════════════════════════════╝")

def main():
    port = '/dev/ttyUSB0'
    baud = 115200
    
    print(f"Подключение к Arduino на {port} со скоростью {baud}...")
    print(f"Размер пакета данных: {SENSOR_DATA_SIZE} байт")
    print(f"Формат данных: {SENSOR_DATA_FORMAT}")
    print("Для выхода нажмите Ctrl+C")
    print("="*70)
    
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2)
        print(f"✓ Подключен к Arduino на {port}")
        print("Ожидание данных одометрии...")
        
        last_timestamp = 0
        packet_count = 0
        
        while True:
            if ser.in_waiting >= SENSOR_DATA_SIZE:
                try:
                    data = ser.read(SENSOR_DATA_SIZE)
                    if len(data) == SENSOR_DATA_SIZE:
                        unpacked = struct.unpack(SENSOR_DATA_FORMAT, data)
                        timestamp = unpacked[0]
                        
                        if timestamp != last_timestamp:
                            packet_count += 1
                            print_sensor_data(unpacked)
                            
                            if packet_count > 1:
                                dt = timestamp - last_timestamp
                                freq = 1000.0 / dt if dt > 0 else 0
                                print(f"Интервал: {dt}мс, Частота: {freq:.1f}Гц")
                            
                            last_timestamp = timestamp
                            print("-" * 70)
                        
                except struct.error as e:
                    print(f"Ошибка распаковки данных: {e}")
                    ser.flushInput()
                except Exception as e:
                    print(f"Ошибка чтения: {e}")
            else:
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\nПрограмма остановлена пользователем")
    except Exception as e:
        print(f"Ошибка подключения: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Отключен от Arduino")

if __name__ == '__main__':
    main() 