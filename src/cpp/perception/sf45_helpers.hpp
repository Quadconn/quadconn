#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <csignal> 

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
const char* UDP_HOST = "100.127.242.40"; // change to host IP (for GUI)
const int UDP_PORT = 6005;
const char* SERIAL_PORT = "/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-17174-if00";
const int SERIAL_BAUD = B921600; // termios constant for 921600

const uint16_t SCAN_SPEED = 5;
const uint8_t UPDATE_RATE = 12; // 5000 Hz

const float SCAN_LOW_DEG = -160.0f;
const float SCAN_HIGH_DEG = 160.0f;

const float LIDAR_MAX_RANGE_FT = 164.042f;
const float INVALID_SENTINEL_FT = LIDAR_MAX_RANGE_FT;

const int TARGET_SCAN_SIZE = 300;

const float POSE_X_FT = 2.290f;
const float POSE_Y_FT = -0.049f;
const float POSE_THETA_RAD = -0.463373f;



// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------
// --- ZERO-COPY SHARED MEMORY FORMAT (RAW ONLY) ---
const uint32_t MAX_POINTS_PER_SWEEP = 5000;

#pragma pack(push, 1)
struct RawPair {
    float angle;
    float distance;
};

struct LidarSweep {
    uint32_t num_points; 
    RawPair points[MAX_POINTS_PER_SWEEP]; 
};
#pragma pack(pop)

uint16_t create_crc(const std::vector<uint8_t>& data) {
    uint16_t crc = 0;
    for (size_t i = 0; i < data.size() - 2; ++i) { // Exclude the 2 CRC bytes at the end
        uint16_t code = crc >> 8;
        code ^= data[i];
        code ^= code >> 4;
        crc = crc << 8;
        crc ^= code;
        code = code << 5;
        crc ^= code;
        code = code << 7;
        crc ^= code;
    }
    return crc;
}

std::vector<uint8_t> build_packet(uint8_t command, uint8_t write_flag, const std::vector<uint8_t>& data) {
    uint16_t payload_length = 1 + data.size();
    uint16_t flags = (payload_length << 6) | (write_flag & 0x1);
    
    std::vector<uint8_t> packet = {0xAA, static_cast<uint8_t>(flags & 0xFF), static_cast<uint8_t>((flags >> 8) & 0xFF), command};
    packet.insert(packet.end(), data.begin(), data.end());
    
    packet.push_back(0); // placeholder for crc low
    packet.push_back(0); // placeholder for crc high
    
    uint16_t crc = create_crc(packet);
    packet[packet.size() - 2] = crc & 0xFF;
    packet[packet.size() - 1] = (crc >> 8) & 0xFF;
    
    return packet;
}

bool parse_byte(uint8_t& byte, int& parse_state, int& payload_size, int& packet_size, std::vector<uint8_t>& packet_data) {
    if (parse_state == 0) {
        if (byte == 0xAA) {
            parse_state = 1;
            packet_data = {0xAA};
        }
    } else if (parse_state == 1) {
        parse_state = 2;
        packet_data.push_back(byte);
    } else if (parse_state == 2) {
        parse_state = 3;
        packet_data.push_back(byte);
        payload_size = (packet_data[1] | (packet_data[2] << 8)) >> 6;
        payload_size += 2;
        packet_size = 3;
        if (payload_size > 1019) parse_state = 0;
    } else if (parse_state == 3) {
        packet_data.push_back(byte);
        packet_size++;
        payload_size--;
        if (payload_size == 0) {
            parse_state = 0;
            uint16_t crc = packet_data[packet_size - 2] | (packet_data[packet_size - 1] << 8);
            uint16_t verify_crc = create_crc(packet_data);
            if (crc == verify_crc) return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------------
// Hardware Commands
// -----------------------------------------------------------------------------
void execute_command(int fd, uint8_t command, uint8_t write_flag, const std::vector<uint8_t>& data = {}) {
    std::vector<uint8_t> packet = build_packet(command, write_flag, data);
    write(fd, packet.data(), packet.size());
    usleep(10000); // 10ms wait for processing
}

void set_float_cmd(int fd, uint8_t cmd, float val) {
    std::vector<uint8_t> data(4);
    memcpy(data.data(), &val, 4);
    execute_command(fd, cmd, 1, data);
}

void set_uint16_cmd(int fd, uint8_t cmd, uint16_t val) {
    std::vector<uint8_t> data(2);
    memcpy(data.data(), &val, 2);
    execute_command(fd, cmd, 1, data);
}

// -----------------------------------------------------------------------------
// Serial & UDP Setup
// -----------------------------------------------------------------------------
int init_serial() {
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, SERIAL_BAUD);
    cfsetospeed(&options, SERIAL_BAUD);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// -----------------------------------------------------------------------------
// Processing
// -----------------------------------------------------------------------------
float clamp_range(float distance_ft, const float LIDAR_MAX_RANGE_FT) {
    if (distance_ft <= 0.0f || distance_ft >= LIDAR_MAX_RANGE_FT) return INVALID_SENTINEL_FT;
    return std::round(distance_ft * 1000.0f) / 1000.0f;
}


