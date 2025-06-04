import serial
import sys
import time

port = '/dev/ttyS0'
baudrate = 115200
bytes_counter = 0

try:
    with serial.Serial(port, baudrate, timeout=0.1) as ser:
        ser.reset_input_buffer()
        with open('serial_data.bin', 'ab') as f:
            while True:
                try:
                    if ser.in_waiting > 0:
                        bytes_to_read = min(ser.in_waiting, 1)
                        data = ser.read(bytes_to_read)
                        
                        if data:
                            f.write(data)
                            f.flush()
                            bytes_counter += len(data)
                            
                            if bytes_counter >= 256:
                                print('.', end='', flush=True)
                                bytes_counter = 0
                    else:
                        time.sleep(0.01)
                        
                except serial.SerialException as se:
                    print(f"\nSerial port error: {se}")
                    print("Reconnect...")
                    time.sleep(1)
                    ser.reset_input_buffer()
                    continue
except KeyboardInterrupt:
    print("\nUser terminated the program.")
except Exception as e:
    print(f"\Error: {e}")