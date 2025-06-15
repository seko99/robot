#!/usr/bin/env python3
"""
Test script for LDS01RR Driver v2
"""

import time
from lds01rr_driver_v2 import LDS01RRDriverV2

def main():
    # Initialize driver (change port as needed)
    try:
        driver = LDS01RRDriverV2('/dev/ttyS5', 115200)
        print("Driver initialized successfully")
    except Exception as e:
        print(f"Failed to initialize driver: {e}")
        try:
            driver = LDS01RRDriverV2('/dev/ttyS5', 115200)
            print("Driver initialized with /dev/ttyS5")
        except Exception as e2:
            print(f"Failed to initialize driver with /dev/ttyS5: {e2}")
            return
    
    print("Starting lidar data collection...")
    scan_count = 0
    
    try:
        while True:  # Collect 10 scans for testing
            measurements = driver.poll()
            
            if measurements:
                scan_count += 1
                print(f"\nScan #{scan_count}: Received {len(measurements)} measurements")
                
                # Show some sample measurements
                if len(measurements) > 0:
                    print("Sample measurements:")
                    for i in range(0, min(10, len(measurements)), 2):
                        m = measurements[i]
                        print(f"  Angle: {m.angle_deg:6.1f}°, Distance: {m.distance_mm:4d}mm, Intensity: {m.intensity:4d}")
                
                # Get scan data in ROS format
                scan_data = driver.get_scan_data_dict(measurements)
                if scan_data:
                    ranges = scan_data['ranges']
                    valid_ranges = [r for r in ranges if r > 0 and r != float('inf')]
                    if valid_ranges:
                        print(f"Valid ranges: {len(valid_ranges)}/{len(ranges)}")
                        print(f"Range: {min(valid_ranges):.3f}m - {max(valid_ranges):.3f}m")
                
                # Get motor speed
                speed = driver.get_speed_rpm()
                if speed > 0:
                    print(f"Motor speed: {speed:.1f} RPM")
            
            #time.sleep(0.1)  # Small delay
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during operation: {e}")
    finally:
        driver.disconnect()
        print("Driver disconnected")

if __name__ == '__main__':
    main() 