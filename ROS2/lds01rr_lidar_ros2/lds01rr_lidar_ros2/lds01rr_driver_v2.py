#!/usr/bin/env python3
"""
LDS01RR Lidar Driver v2 for ROS2
Based on official Roborock Cullinan documentation
Supports 0xAA (info) and 0xFA (measurement) packet headers
"""

import serial
import struct
import math
from collections import deque
from enum import Enum
from typing import List, Optional, Dict, Tuple
import time


class PacketState(Enum):
    SYNC = 0
    INFO_PACKET = 1
    INFO_ESCAPE = 2
    MEAS_PACKET = 3


class MeasurementData:
    def __init__(self):
        self.distance_mm: int = 0
        self.intensity: int = 0
        self.angle_deg: float = 0.0
        self.quality: int = 0
        self.invalid_flag: bool = False
        self.strength_warning: bool = False


class LDS01RRDriverV2:
    def __init__(self, serial_port: str, baud_rate: int = 115200):
        """
        Initialize the LDS01RR driver v2
        
        Args:
            serial_port: Serial port device (e.g., '/dev/ttyS5')
            baud_rate: Baud rate for serial communication
        """
        self.serial_port_name = serial_port
        self.baud_rate = baud_rate
        self.serial_port: Optional[serial.Serial] = None
        self.shutting_down = False
        
        # Packet parsing state
        self.state = PacketState.SYNC
        self.packet_buffer = bytearray()
        self.packet_count = 0
        
        # Measurement data storage
        self.scan_data = []
        self.current_scan_complete = False
        
        # Speed calculation
        self.speed_sum = 0
        self.packet_cnt = 0
        self.last_angle = None
        
        # Constants according to documentation
        self.INFO_PACKET_SIZE = 85  # 84 + 1 sync byte
        self.MEAS_PACKET_SIZE = 22  # 22 bytes total
        
        # Initialize serial connection
        self._connect()
    
    def _connect(self):
        """Connect to the serial port"""
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to {self.serial_port_name} at {self.baud_rate} baud")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to connect to {self.serial_port_name}: {e}")
    
    def disconnect(self):
        """Disconnect from the serial port"""
        self.shutting_down = True
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def poll(self) -> Optional[List[MeasurementData]]:
        """
        Poll the lidar for new scan data
        
        Returns:
            List of MeasurementData objects if a complete scan is received, None otherwise
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("DEBUG: Serial port not available or not open")
            return None
        
        try:
            data = self.serial_port.read(1)
            if len(data) == 0:
                return None
            
            byte_val = data[0]
            packet_ready = self._parse_byte(byte_val)
            
            if packet_ready == 0:  # Measurement packet processed successfully
                if self.current_scan_complete:
                    print(f"DEBUG: Scan complete! Returning {len(self.scan_data)} measurements")
                    result = list(self.scan_data)
                    self.scan_data.clear()
                    self.current_scan_complete = False
                    return result
                        
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            return None
        
        return None
    
    def _parse_byte(self, byte_val: int) -> int:
        """
        Parse incoming byte according to LDS.c protocol from documentation
        
        Args:
            byte_val: Received byte
            
        Returns:
            0 if packet processed successfully, 1 if pending, -1 if error
        """
        if self.state == PacketState.SYNC:
            if byte_val == 0xAA:  # LDS information packet
                print("DEBUG: Info packet started (0xAA)")
                self.packet_buffer = bytearray([0xAA])
                self.packet_count = 1
                self.state = PacketState.INFO_PACKET
            elif byte_val == 0xFA:  # LDS measurement packet
                self.packet_buffer = bytearray([0xFA])
                self.packet_count = 1
                self.state = PacketState.MEAS_PACKET
            else:
                # Synchronization loss - ignore byte
                pass
                
        elif self.state == PacketState.INFO_PACKET:
            if byte_val != 0xAA:
                if byte_val != 0xA9:  # Normal data byte
                    self.packet_buffer.append(byte_val)
                    self.packet_count += 1
                    
                    if self.packet_count >= self.INFO_PACKET_SIZE:
                        # Full information packet received
                        print(f"DEBUG: Full info packet received ({self.packet_count} bytes)")
                        self.state = PacketState.SYNC
                        if self._check_info_packet():
                            self._process_info_packet()
                            return 0
                        else:
                            print("DEBUG: Info packet checksum failed")
                            return -1
                else:  # 0xA9 escape character
                    self.packet_buffer.append(0xA9)
                    # Don't increment packet_count yet
                    self.state = PacketState.INFO_ESCAPE
            else:  # Unexpected 0xAA
                self.packet_buffer.append(0xAA)
                self.packet_count += 1
                print("Unexpected 0xAA in info packet!")
                self.state = PacketState.SYNC
                
        elif self.state == PacketState.INFO_ESCAPE:
            if byte_val == 0x00:  # 0xA900 - represents 0xA9
                self.packet_count += 1
            elif byte_val == 0x01:  # 0xA901 - represents 0xAA
                self.packet_buffer[-1] = 0xAA  # Replace last 0xA9 with 0xAA
                self.packet_count += 1
            else:  # Error
                self.packet_buffer.append(byte_val)
                self.packet_count += 1
                print("Error in escape sequence")
                self.state = PacketState.SYNC
                return -1
            
            # Check if packet is complete
            if self.packet_count >= self.INFO_PACKET_SIZE:
                self.state = PacketState.SYNC
                if self._check_info_packet():
                    self._process_info_packet()
                    return 0
                else:
                    return -1
            else:
                self.state = PacketState.INFO_PACKET
                
        elif self.state == PacketState.MEAS_PACKET:
            self.packet_buffer.append(byte_val)
            self.packet_count += 1
            
            if self.packet_count >= self.MEAS_PACKET_SIZE:
                # Full measurement packet received
                self.state = PacketState.SYNC
                if self._check_meas_packet():
                    self._process_meas_packet()
                    return 0
                else:
                    #print("DEBUG: Measurement packet checksum failed")
                    return -1
        
        return 1  # Pending
    
    def _check_info_packet(self) -> bool:
        """
        Check information packet checksum according to documentation
        """
        if len(self.packet_buffer) < self.INFO_PACKET_SIZE:
            return False
        
        # Calculate checksum as specified in documentation
        checksum = 0
        # Skip the sync byte, process pairs starting from byte 1, size is 41 pairs
        for i in range(1, 83, 2):  # 83 = 1 + 41*2
            if i + 1 < len(self.packet_buffer):
                word = struct.unpack('<H', self.packet_buffer[i:i+2])[0]
                checksum += word
        
        checksum &= 0xFFFF  # Keep only 16 bits
        expected_checksum = struct.unpack('<H', self.packet_buffer[83:85])[0]
        
        if checksum == expected_checksum:
            return True
        else:
            print(f"Info packet checksum FAIL! Got: {checksum}, Expected: {expected_checksum}")
            return False
    
    def _check_meas_packet(self) -> bool:
        """
        Check measurement packet checksum according to documentation
        """
        if len(self.packet_buffer) < self.MEAS_PACKET_SIZE:
            return False
        
        # Calculate checksum as specified in documentation
        checksum = 0
        # Process pairs of bytes for first 20 bytes
        for i in range(0, 20, 2):
            word = struct.unpack('<H', self.packet_buffer[i:i+2])[0]
            checksum = ((checksum << 1) + word) & 0xFFFFFFFF  # Ограничение 32 битами
        
        checksum = (checksum + (checksum >> 15)) & 0x7FFF
        
        # Контрольная сумма в данных хранится с перевернутыми байтами
        # Если мы вычислили 0x1408, то в данных она будет как 0x0814
        expected_checksum_bytes = self.packet_buffer[20:22]
        expected_checksum = struct.unpack('<H', expected_checksum_bytes)[0]  # big-endian
        
        if checksum == expected_checksum:
            return True
        else:
            #print(f"Measurement packet checksum FAIL! Got: {hex(checksum)}, Expected: {hex(expected_checksum)}")
            return False
    
    def _process_info_packet(self):
        """Process information packet"""
        # Information packets contain device info, firmware version, etc.
        print("Received info packet")
        # Extract information if needed
        if len(self.packet_buffer) >= 5:
            packet_size = struct.unpack('<H', self.packet_buffer[1:3])[0]
            fw_version = struct.unpack('<H', self.packet_buffer[3:5])[0]
            major = (fw_version >> 12) & 0xF
            minor = fw_version & 0xFFF
            print(f"Firmware version: V{major}.{minor}")
    
    def _process_meas_packet(self):
        """
        Process measurement packet according to documentation format
        """
        if len(self.packet_buffer) < self.MEAS_PACKET_SIZE:
            return
        
        # Extract angle and speed from measurement packet
        # Bytes 0-1: 0xFA + angle index
        angle_index = self.packet_buffer[1]
        
        # Bytes 2-3: Speed (RPM * 64)
        speed_raw = struct.unpack('<H', self.packet_buffer[2:4])[0]
        self.speed_sum += speed_raw
        self.packet_cnt += 1
        
        # Extract 4 measurement points from bytes 4-19
        for i in range(4):
            offset = 4 + i * 4
            if offset + 3 < len(self.packet_buffer):
                # Parse distance and intensity according to documentation format
                # Byte 0: distance[7:0]
                # Byte 1: bit7=invalid, bit6=strength_warning, bit[5:0]=distance[13:8]
                # Byte 2: strength[7:0]  
                # Byte 3: strength[15:8]
                
                dist_low = self.packet_buffer[offset]
                dist_high_and_flags = self.packet_buffer[offset + 1]
                intensity_low = self.packet_buffer[offset + 2]
                intensity_high = self.packet_buffer[offset + 3]
                
                # Extract flags
                invalid_flag = bool(dist_high_and_flags & 0x80)
                strength_warning = bool(dist_high_and_flags & 0x40)
                
                # Extract distance (14 bits total)
                dist_high = dist_high_and_flags & 0x3F  # Lower 6 bits
                distance_raw = dist_low + (dist_high << 8)
                distance_mm = distance_raw * 0.25  # 0.25mm resolution according to docs
                
                # Extract intensity (16 bits)
                intensity = intensity_low + (intensity_high << 8)
                
                # Calculate angle (each packet covers 4 degrees, 90 packets total)
                # Index from 0xA0 to 0xF9 (90 packets total)
                if angle_index >= 0xA0 and angle_index <= 0xF9:
                    base_angle = (angle_index - 0xA0) * 4
                    angle_deg = base_angle + i
                    if angle_deg >= 360:
                        angle_deg -= 360
                else:
                    # Invalid angle index
                    continue

                #print(f"DEBUG: invalid_flag: {invalid_flag}, distance_mm: {distance_mm}, intensity: {intensity}, angle_deg: {angle_deg}")

                # Create measurement data point
                if not invalid_flag and distance_mm > 0:
                    point = MeasurementData()
                    point.distance_mm = int(distance_mm)
                    point.intensity = intensity
                    point.angle_deg = angle_deg
                    point.invalid_flag = invalid_flag
                    point.strength_warning = strength_warning
                    
                    self.scan_data.append(point)
                    #print(f"DEBUG: Added point #{len(self.scan_data)}: angle={angle_deg}°, dist={distance_mm}mm")
        
        # Check if we completed a full scan (360 points = 90 packets * 4 points)
        if len(self.scan_data) >= 360:
            print(f"DEBUG: Scan completed with {len(self.scan_data)} points!")
            self.current_scan_complete = True
            # Check for scan wrap-around
            if len(self.scan_data) > 400:  # Allow some overlap
                print(f"DEBUG: Trimming scan data from {len(self.scan_data)} to 360 points")
                # Keep only the most recent 360 points
                self.scan_data = self.scan_data[-360:]
    
    def get_scan_data_dict(self, measurements: List[MeasurementData]) -> Dict:
        """
        Convert measurement data to ROS2 LaserScan compatible format
        
        Args:
            measurements: List of measurement data points
            
        Returns:
            Dictionary containing scan data
        """
        if not measurements:
            return {}
        
        # Sort by angle to ensure proper ordering
        sorted_measurements = sorted(measurements, key=lambda x: x.angle_deg)
        
        # Create arrays for distances and intensities
        ranges = []
        intensities = []
        
        # Fill arrays with measurement data
        for measurement in sorted_measurements:
            if measurement.invalid_flag or measurement.distance_mm <= 0:
                distance_m = 0.0  # Invalid measurement
            else:
                distance_m = measurement.distance_mm / 1000.0
                # Apply range limits according to documentation (150mm - 6000mm)
                if distance_m < 0.15:
                    distance_m = 0.0  # Too close
                elif distance_m > 6.0:
                    distance_m = float('inf')  # Too far
            
            ranges.append(distance_m)
            intensities.append(float(measurement.intensity))
        
        # Calculate scan parameters
        angle_min = 0.0
        angle_max = 2.0 * math.pi
        angle_increment = angle_max / len(ranges) if ranges else 0.0
        
        # Estimate scan time and time increment based on documentation (5 Hz scan rate)
        scan_time = 1.0 / 5.0  # 5 Hz scan rate
        time_increment = scan_time / len(ranges) if ranges else 0.0
        
        scan_data = {
            'angle_min': angle_min,
            'angle_max': angle_max,
            'angle_increment': angle_increment,
            'time_increment': time_increment,
            'range_min': 0.15,  # 150mm minimum range
            'range_max': 6.0,   # 6000mm maximum range
            'scan_time': scan_time,
            'ranges': ranges,
            'intensities': intensities
        }
        
        return scan_data
    
    def get_speed_rpm(self) -> float:
        """
        Get current motor speed in RPM
        
        Returns:
            Speed in RPM
        """
        if self.packet_cnt > 0:
            speed_rpm = self.speed_sum / (self.packet_cnt * 64.0)
            # Reset counters
            self.speed_sum = 0
            self.packet_cnt = 0
            return speed_rpm
        return 0.0 