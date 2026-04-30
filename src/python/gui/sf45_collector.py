# --------------------------------------------------------------------------------------------------------------
# LightWare SF45/B - Data collector for 2D SLAM
# All units are FEET throughout (distances, map sizes, poses).
# Sends each scan over UDP instead of writing to a JSON file.
# --------------------------------------------------------------------------------------------------------------

import time
import serial
import json
import math
import os
import socket
import struct 

# --------------------------------------------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------------------------------------------

# Change this to match your actual serial port.
# Run:  ls /dev/serial/by-id/   or   ls /dev/ttyUSB* /dev/ttyACM*
# to find the correct path when the sensor is plugged in.
SERIAL_PORT = '/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-17174-if00'
SERIAL_BAUD = 921600

SCAN_SPEED = 5
UPDATE_RATE = 12

SCAN_LOW_DEG = -90.0
SCAN_HIGH_DEG = 90.0

LIDAR_MAX_RANGE_FT = 164.042
INVALID_SENTINEL_FT = LIDAR_MAX_RANGE_FT

TARGET_SCAN_SIZE = 300

POSE_X_FT = 2.290
POSE_Y_FT = -0.049
POSE_THETA_RAD = -0.463373

# --------------------------------------------------------------------------------------------------------------
# UDP helpers
# --------------------------------------------------------------------------------------------------------------

# FIX: sendto requires bytes, not a dict.
# We serialise the scan entry to JSON and encode to UTF-8 bytes.
# Each UDP packet carries exactly one scan entry (not the whole dataset)
# so packets stay small and the receiver can process them one at a time.

UDP_HOST = '100.119.158.85' 
UDP_PORT = 6000


# --------------------------------------------------------------------------------------------------------------
# LWNX library functions
# --------------------------------------------------------------------------------------------------------------
_packet_parse_state = 0
_packet_payload_size = 0
_packet_size = 0
_packet_data = []


def create_crc(data):
    crc = 0
    for i in data:
        code = crc >> 8
        code ^= int(i)
        code ^= code >> 4
        crc = crc << 8
        crc ^= code
        code = code << 5
        crc ^= code
        code = code << 7
        crc ^= code
        crc &= 0xFFFF
    return crc


def build_packet(command, write, data=[]):
    payload_length = 1 + len(data)
    flags = (payload_length << 6) | (write & 0x1)
    packet_bytes = [0xAA, flags & 0xFF, (flags >> 8) & 0xFF, command]
    packet_bytes.extend(data)
    crc = create_crc(packet_bytes)
    packet_bytes.append(crc & 0xFF)
    packet_bytes.append((crc >> 8) & 0xFF)
    return bytearray(packet_bytes)


def parse_packet(byte):
    global _packet_parse_state
    global _packet_payload_size
    global _packet_size
    global _packet_data

    if _packet_parse_state == 0:
        if byte == 0xAA:
            _packet_parse_state = 1
            _packet_data = [0xAA]

    elif _packet_parse_state == 1:
        _packet_parse_state = 2
        _packet_data.append(byte)

    elif _packet_parse_state == 2:
        _packet_parse_state = 3
        _packet_data.append(byte)
        _packet_payload_size = (_packet_data[1] | (_packet_data[2] << 8)) >> 6
        _packet_payload_size += 2
        _packet_size = 3
        if _packet_payload_size > 1019:
            _packet_parse_state = 0

    elif _packet_parse_state == 3:
        _packet_data.append(byte)
        _packet_size += 1
        _packet_payload_size -= 1
        if _packet_payload_size == 0:
            _packet_parse_state = 0
            crc = _packet_data[_packet_size -
                               2] | (_packet_data[_packet_size - 1] << 8)
            verify_crc = create_crc(_packet_data[0:-2])
            if crc == verify_crc:
                return True

    return False


def wait_for_packet(port, command, timeout=1):
    global _packet_parse_state
    global _packet_payload_size
    global _packet_size
    global _packet_data

    _packet_parse_state = 0
    _packet_data = []
    _packet_payload_size = 0
    _packet_size = 0

    end_time = time.time() + timeout

    while True:
        if time.time() >= end_time:
            return None

        c = port.read(1)

        if len(c) != 0:
            b = ord(c)
            if parse_packet(b):
                if _packet_data[3] == command:
                    return _packet_data


def execute_command(port, command, write, data=[], timeout=1):
    packet = build_packet(command, write, data)
    retries = 4

    while retries > 0:
        retries -= 1
        port.write(packet)
        response = wait_for_packet(port, command, timeout)
        if response is not None:
            return response

    raise Exception('LWNX command failed to receive a response.')


# --------------------------------------------------------------------------------------------------------------
# SF45 API helper functions
# --------------------------------------------------------------------------------------------------------------
def get_str16_from_packet(packet):
    str16 = ''
    for i in range(0, 16):
        if packet[4 + i] == 0:
            break
        else:
            str16 += chr(packet[4 + i])
    return str16


def print_product_information(port):
    response = execute_command(port, 0, 0, timeout=0.1)
    print('Product: ' + get_str16_from_packet(response))

    response = execute_command(port, 2, 0, timeout=0.1)
    print('Firmware: {}.{}.{}'.format(response[6], response[5], response[4]))

    response = execute_command(port, 3, 0, timeout=0.1)
    print('Serial: ' + get_str16_from_packet(response))


def set_scan_speed(port, speed):
    # Pack the integer into 2 bytes (little-endian unsigned short)
    data = list(struct.pack('<H', int(speed)))
    execute_command(port, 85, 1, data)

def set_update_rate(port, value):
    if value < 1 or value > 12:
        raise Exception('Invalid update rate value.')
    execute_command(port, 66, 1, [value])


def set_default_scan_low_angle(port, value):
    if value > -5 or value < -170:
        raise Exception('Invalid lower bound for Scan angle.')
    # Pack the float into 4 bytes (little-endian)
    data = list(struct.pack('<f', float(value)))
    execute_command(port, 98, 1, data)

def set_default_scan_high_angle(port, value):
    if value > 170 or value < 5:
        raise Exception('Invalid high bound for Scan angle.')
    # Pack the float into 4 bytes (little-endian)
    data = list(struct.pack('<f', float(value)))
    execute_command(port, 99, 1, data)


def set_default_distance_output(port, use_last_return=False):
    if use_last_return:
        execute_command(port, 27, 1, [1, 1, 0, 0])
    else:
        execute_command(port, 27, 1, [1, 1, 0, 0])

def set_distance_stream_enable(port, enable):
    if enable:
        execute_command(port, 30, 1, [5, 0, 0, 0])
    else:
        execute_command(port, 30, 1, [0, 0, 0, 0])


def wait_for_reading(port, timeout=1):
    """
    Returns (distance_ft, yaw_deg).
    The SF45/B packet 44 encodes distance as a 16-bit little-endian integer
    in centimetres. We convert directly to feet (1 cm = 0.0328084 ft).
    Returns (-1, 0) on timeout.
    """
    response = wait_for_packet(port, 44, timeout)

    # ---> UPDATE THIS CONDITION <---
    if response is None or len(response) < 8:
        return -1, 0

    distance_cm = (response[4] << 0) | (response[5] << 8)
    distance_ft = round(distance_cm * 0.0328084, 3)

    yaw_raw = (response[6] << 0) | (response[7] << 8)
    if yaw_raw > 32000:
        yaw_raw -= 65535
    yaw_deg = yaw_raw / 100.0

    return distance_ft, yaw_deg


# Pre-create a single socket and reuse it for every send.
# Creating a new socket per scan adds unnecessary overhead.
_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_scan_udp(scan_entry):
    """
    Serialise one scan entry dict to JSON bytes and fire it over UDP.
    scan_entry looks like:
        { "range": [...], "theta": float, "x": float, "y": float }

    UDP has a practical payload limit of ~65507 bytes.
    At 300 floats * ~8 bytes each = ~2400 bytes per scan, we are well within that.
    """
    payload = json.dumps(scan_entry).encode('utf-8')
    _udp_sock.sendto(payload, (UDP_HOST, UDP_PORT))
    print(f"  UDP sent {len(payload)} bytes to {UDP_HOST}:{UDP_PORT}")




# --------------------------------------------------------------------------------------------------------------
# Scan helpers
# --------------------------------------------------------------------------------------------------------------

def clamp_range(distance_ft):
    if distance_ft <= 0.0 or distance_ft >= LIDAR_MAX_RANGE_FT:
        return INVALID_SENTINEL_FT
    return round(distance_ft, 3)


def resample_scan(raw_pairs):
    if not raw_pairs:
        return [INVALID_SENTINEL_FT] * TARGET_SCAN_SIZE

    raw_pairs.sort(key=lambda p: p[0])
    angles = [p[0] for p in raw_pairs]
    ranges = [p[1] for p in raw_pairs]

    step = (SCAN_HIGH_DEG - SCAN_LOW_DEG) / (TARGET_SCAN_SIZE - 1)
    grid = []

    for i in range(TARGET_SCAN_SIZE):
        target_angle = SCAN_LOW_DEG + step * i
        best_idx = min(range(len(angles)), key=lambda j: abs(
            angles[j] - target_angle))
        if abs(angles[best_idx] - target_angle) <= step * 1.5:
            grid.append(clamp_range(ranges[best_idx]))
        else:
            grid.append(INVALID_SENTINEL_FT)

    return grid


# --------------------------------------------------------------------------------------------------------------
# Main application
# --------------------------------------------------------------------------------------------------------------
print('Running SF45/B LWNX sample.')

sensor_port = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
set_distance_stream_enable(sensor_port, False)
time.sleep(0.1)
sensor_port.reset_input_buffer()

set_scan_speed(sensor_port, SCAN_SPEED)
print_product_information(sensor_port)
set_update_rate(sensor_port, UPDATE_RATE)
set_default_scan_low_angle(sensor_port, SCAN_LOW_DEG)
set_default_scan_high_angle(sensor_port, SCAN_HIGH_DEG)
set_default_distance_output(sensor_port, use_last_return=False)
set_distance_stream_enable(sensor_port, True)

sensor_port.reset_input_buffer()
raw_pairs = []
in_scan = False
prev_yaw = None

print(f'Collecting scans | FOV=[{SCAN_LOW_DEG}, {SCAN_HIGH_DEG}] deg | '
      f'bins={TARGET_SCAN_SIZE} | max_range={LIDAR_MAX_RANGE_FT:.1f} ft')
print(f'Sending UDP -> {UDP_HOST}:{UDP_PORT}')
print('Press Ctrl-C to stop.\n')

try:
    while True:
        distance_ft, yaw_deg = wait_for_reading(sensor_port)

        if distance_ft == -1:
            print("LIDAR Timeout\n")
            continue

        if prev_yaw is not None:
            crossed_start = (prev_yaw < SCAN_LOW_DEG <= yaw_deg)

            if crossed_start:
                if in_scan and len(raw_pairs) > TARGET_SCAN_SIZE // 4:
                    scan = resample_scan(raw_pairs)
                    scan_entry = {
                        "range": scan,
                        "theta": float(POSE_THETA_RAD),
                        "x":     float(POSE_X_FT),
                        "y":     float(POSE_Y_FT),
                    }
                    send_scan_udp(scan_entry)
                    print(f"Sent scan: {len(raw_pairs)
                                        } raw -> {TARGET_SCAN_SIZE} bins")

                raw_pairs = []
                in_scan = True

        if in_scan and SCAN_LOW_DEG <= yaw_deg <= SCAN_HIGH_DEG:
            raw_pairs.append((yaw_deg, distance_ft))

        prev_yaw = yaw_deg

except KeyboardInterrupt:
    if in_scan and raw_pairs:
        scan = resample_scan(raw_pairs)
        scan_entry = {
            "range": scan,
            "theta": float(POSE_THETA_RAD),
            "x":     float(POSE_X_FT),
            "y":     float(POSE_Y_FT),
        }
        send_scan_udp(scan_entry)
        print(f"Final partial scan sent: {len(raw_pairs)} raw samples.")

    set_scan_speed(sensor_port, 0)
    set_distance_stream_enable(sensor_port, False)
    sensor_port.close()
    _udp_sock.close()
    print(f"\nStopped.")