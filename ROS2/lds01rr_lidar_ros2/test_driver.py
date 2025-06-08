#!/usr/bin/env python3
"""
Test script for LDS01RR driver without ROS2
"""

import sys
import time
from lds01rr_lidar_ros2.lds01rr_driver import LDS01RRDriver


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_driver.py <serial_port> [baud_rate]")
        print("Example: python3 test_driver.py /dev/ttyACM0 115200")
        return
    
    serial_port = sys.argv[1]
    baud_rate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    print(f"Testing LDS01RR driver on {serial_port} at {baud_rate} baud")
    
    try:
        driver = LDS01RRDriver(serial_port, baud_rate)
        print("Driver initialized successfully")
        
        scan_count = 0
        while scan_count < 10:  # Test 10 scans
            print(f"Waiting for scan {scan_count + 1}...")
            points = driver.poll()
            
            if points:
                scan_data = driver.process_lidar_data(points)
                if scan_data:
                    print(f"Scan {scan_count + 1}: {len(points)} points, "
                          f"ranges: {len(scan_data['ranges'])}")
                    
                    # Print some sample ranges
                    ranges = scan_data['ranges']
                    if len(ranges) >= 4:
                        print(f"  Sample ranges: "
                              f"0°={ranges[0]:.3f}m, "
                              f"90°={ranges[len(ranges)//4]:.3f}m, "
                              f"180°={ranges[len(ranges)//2]:.3f}m, "
                              f"270°={ranges[3*len(ranges)//4]:.3f}m")
                    
                    scan_count += 1
            else:
                print("No data received")
                time.sleep(0.1)
        
        print("Test completed successfully!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'driver' in locals():
            driver.disconnect()
            print("Driver disconnected")


if __name__ == '__main__':
    main() 