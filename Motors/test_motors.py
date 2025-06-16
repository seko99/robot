#!/usr/bin/env python3

import serial
import struct
import time

CMD_LEFT_FORWARD = 0
CMD_LEFT_BACKWARD = 1
CMD_RIGHT_FORWARD = 2
CMD_RIGHT_BACKWARD = 3
CMD_LEFT_STOP = 4
CMD_RIGHT_STOP = 5
CMD_STOP_MOTORS = 6

COMMAND_FORMAT = '<BB'

def send_command(ser, cmd, value):
    try:
        packed_command = struct.pack(COMMAND_FORMAT, cmd, value)
        ser.write(packed_command)
        return True
    except Exception as e:
        print(f"Ошибка отправки команды: {e}")
        return False

def stop_motors(ser):
    send_command(ser, CMD_STOP_MOTORS, 0)

def move_forward(ser, speed=150):
    send_command(ser, CMD_LEFT_FORWARD, speed)
    send_command(ser, CMD_RIGHT_FORWARD, speed)

def move_backward(ser, speed=150):
    send_command(ser, CMD_LEFT_BACKWARD, speed)
    send_command(ser, CMD_RIGHT_BACKWARD, speed)

def turn_right(ser, speed=150):
    send_command(ser, CMD_LEFT_FORWARD, speed)
    send_command(ser, CMD_RIGHT_BACKWARD, speed)

def turn_left(ser, speed=150):
    send_command(ser, CMD_LEFT_BACKWARD, speed)
    send_command(ser, CMD_RIGHT_FORWARD, speed)

def main():
    port = '/dev/ttyUSB0'
    baud = 115200
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)
        print(f"Подключен к Arduino на {port}")
        
        print("Тест моторов начинается...")
        
        print("Движение вперёд (5 сек)")
        move_forward(ser)
        time.sleep(5)
        stop_motors(ser)
        time.sleep(1)
        
        print("Поворот вправо (5 сек)")
        turn_right(ser)
        time.sleep(5)
        stop_motors(ser)
        time.sleep(1)
        
        print("Поворот влево (5 сек)")
        turn_left(ser)
        time.sleep(5)
        stop_motors(ser)
        time.sleep(1)
        
        print("Движение назад (5 сек)")
        move_backward(ser)
        time.sleep(5)
        stop_motors(ser)
        
        print("Тест завершён")
        
    except Exception as e:
        print(f"Ошибка подключения: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            stop_motors(ser)
            ser.close()
            print("Отключен от Arduino")

if __name__ == '__main__':
    main() 