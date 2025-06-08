#!/usr/bin/env python3
"""
LDS01RR Lidar Driver for ROS2
Ported from C++ version by iliasam
"""

import serial
import struct
import math
from collections import deque
from enum import Enum
from typing import List, Optional
import time


class SyncState(Enum):
    NO_SYNC = 0
    RECEIVED_BYTE1 = 1
    RECEIVED_BYTE2 = 2
    RECEIVING_DATA = 3


class MeasuredPoint:
    def __init__(self):
        self.distance_mm: int = 0
        self.intensity: int = 0


class LDS01RRDriver:
    def __init__(self, serial_port: str, baud_rate: int = 115200):
        """
        Initialize the LDS01RR driver
        
        Args:
            serial_port: Serial port device (e.g., '/dev/ttyS5')
            baud_rate: Baud rate for serial communication
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_port: Optional[serial.Serial] = None
        self.shutting_down = False
        
        # Sync state variables
        self.curr_sync = SyncState.NO_SYNC
        self.packet_pos_cnt = 0
        self.expected_packet_size = 0
        self.curr_packet = deque()
        self.points_list = deque()
        
        # Initialize serial connection
        self._connect()
    
    def _connect(self):
        """Connect to the serial port"""
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to connect to {self.serial_port}: {e}")
    
    def disconnect(self):
        """Disconnect from the serial port"""
        self.shutting_down = True
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
    
    def poll(self) -> Optional[List[MeasuredPoint]]:
        """
        Poll the lidar for new scan data
        
        Returns:
            List of MeasuredPoint objects if a complete scan is received, None otherwise
        """
        if not self.serial_port or not self.serial_port.is_open:
            return None
        
        while not self.shutting_down:
            try:
                # Read one byte
                data = self.serial_port.read(1)
                if len(data) == 0:
                    continue
                
                byte_val = data[0]
                scan_ready = self._parse_byte(byte_val)
                
                if scan_ready:
                    result = list(self.points_list)
                    self.points_list.clear()  # Clear for next scan
                    return result
                    
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                return None
        
        return None
    
    def _parse_byte(self, rx_byte: int) -> bool:
        """
        Parse incoming byte and build packets
        
        Args:
            rx_byte: Received byte
            
        Returns:
            True if a complete scan is ready
        """
        if self.curr_sync == SyncState.NO_SYNC:
            if rx_byte == 0xE7:  # 231 dec
                self.curr_sync = SyncState.RECEIVED_BYTE1
                self.packet_pos_cnt = 1
                self.curr_packet.clear()
                self.curr_packet.append(rx_byte)
        
        elif self.curr_sync == SyncState.RECEIVED_BYTE1:
            if rx_byte == 0x7E:  # 126
                self.curr_sync = SyncState.RECEIVED_BYTE2
                self.packet_pos_cnt += 1
                self.curr_packet.append(rx_byte)
            else:
                self.curr_sync = SyncState.NO_SYNC
        
        elif self.curr_sync == SyncState.RECEIVED_BYTE2:
            self.expected_packet_size = rx_byte
            self.curr_sync = SyncState.RECEIVING_DATA
            self.packet_pos_cnt += 1
            self.curr_packet.append(rx_byte)
        
        elif self.curr_sync == SyncState.RECEIVING_DATA:
            self.curr_packet.append(rx_byte)
            self.packet_pos_cnt += 1
            
            if self.packet_pos_cnt >= self.expected_packet_size:
                self.curr_sync = SyncState.NO_SYNC
                
                if self.packet_pos_cnt == 34:
                    return self._parse_measurement_data_packet()
        
        return False
    
    def _parse_measurement_data_packet(self) -> bool:
        """
        Parse measurement data packet
        
        Returns:
            True if new scan is complete
        """
        packet_seq = self.curr_packet[4]
        angle_code = packet_seq - 160
        
        if angle_code < 0:
            return False
        
        # If angle_code is 0, we have completed a full scan
        scan_complete = False
        if angle_code == 0:
            scan_complete = True
        
        # Parse 4 measurement points from the packet
        for i in range(4):
            start = 16 + i * 4
            if start + 3 < len(self.curr_packet):
                point = self._parse_measured_data(
                    self.curr_packet[start],
                    self.curr_packet[start + 1],
                    self.curr_packet[start + 2],
                    self.curr_packet[start + 3]
                )
                self.points_list.append(point)
        
        if scan_complete:
            # Keep the points for this scan and clear for next
            # Note: points_list will be cleared after processing
            return True
        
        return False
    
    def _parse_measured_data(self, byte1: int, byte2: int, byte3: int, byte4: int) -> MeasuredPoint:
        """
        Parse measured data from 4 bytes
        
        Args:
            byte1, byte2: Distance bytes
            byte3, byte4: Intensity bytes
            
        Returns:
            MeasuredPoint object
        """
        point = MeasuredPoint()
        
        distance = byte1 + (byte2 * 256)
        intensity = byte3 + (byte4 * 256)
        
        # Check error flags
        if (byte2 & 128) != 0:
            point.distance_mm = -1  # Error flag
        elif (byte2 & 64) != 0:
            point.distance_mm = -2  # Error flag
        else:
            point.distance_mm = distance
        
        point.intensity = intensity
        
        return point
    
    def process_lidar_data(self, points_list: List[MeasuredPoint]) -> dict:
        """
        Process lidar data and convert to scan data
        
        Args:
            points_list: List of measured points
            
        Returns:
            Dictionary containing scan data
        """
        scan_points_cnt = len(points_list)
        if scan_points_cnt == 0:
            return {}
        
        angular_step = 360.0 / scan_points_cnt
        
        # Initialize distance buffer
        dist_buf = [0.0] * scan_points_cnt
        
        # Process each point
        for i, point in enumerate(points_list):
            dist_m = 0.0
            if point.distance_mm >= 0:
                dist_m = point.distance_mm / 1000.0
            
            angle_deg = 180.0 - i * angular_step
            
            # Correct angle
            corrected_angle_deg = angle_deg
            if corrected_angle_deg > 359.0:
                corrected_angle_deg = corrected_angle_deg - 359.0
            if corrected_angle_deg < 0.0:
                corrected_angle_deg = 360 + corrected_angle_deg
            
            pos = round(corrected_angle_deg / angular_step)
            if 0 <= pos < scan_points_cnt:
                dist_buf[pos] = dist_m
        
        # Prepare scan data
        scan_data = {
            'angle_min': 0.0,
            'angle_max': 2.0 * math.pi,
            'angle_increment': angular_step * math.pi / 180.0,
            'time_increment': 1.0 / (scan_points_cnt * 15.0),
            'range_min': 0.05,
            'range_max': 15.0,
            'scan_time': 1.0 / 5.0,
            'ranges': dist_buf,
            'intensities': [0.0] * scan_points_cnt  # Placeholder for intensities
        }
        
        return scan_data 