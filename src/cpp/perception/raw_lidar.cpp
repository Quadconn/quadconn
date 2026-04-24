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
#include <chrono> // Added for high-precision dt tracking

// -----------------------------------------------------------------------------
// Signal Handling (Ctrl+C)
// -----------------------------------------------------------------------------
volatile sig_atomic_t keep_running = 1;

void sigint_handler(int dummy) {
    keep_running = 0;
}

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
const char* UDP_HOST = "100.127.242.40"; // change to host IP (for GUI)
const int UDP_PORT = 6005;
const char* SERIAL_PORT = "/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-17174-if00";
const int SERIAL_BAUD = B921600;

const uint16_t SCAN_SPEED = 5;
const uint8_t UPDATE_RATE = 12; // 5000 Hz

const float SCAN_LOW_DEG = -160.0f;
const float SCAN_HIGH_DEG = 160.0f;

// -----------------------------------------------------------------------------
// Data Structures (Dynamic Binary Format)
// -----------------------------------------------------------------------------
#pragma pack(push, 1)
struct RawPair {
    float angle;    // Raw angle in degrees
    float distance; // Raw distance in feet
};

struct RawPacketHeader {
    float x;
    float y;
    float theta;
    uint32_t num_points; // Crucial for dynamic unpacking in Python
};
#pragma pack(pop)

// -----------------------------------------------------------------------------
// LWNX Protocol Globals & Functions
// -----------------------------------------------------------------------------
int parse_state = 0;
int payload_size = 0;
int packet_size = 0;
std::vector<uint8_t> packet_data;

uint16_t create_crc(const std::vector<uint8_t>& data) {
    uint16_t crc = 0;
    for (size_t i = 0; i < data.size() - 2; ++i) {
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
    
    packet.push_back(0); 
    packet.push_back(0); 
    
    uint16_t crc = create_crc(packet);
    packet[packet.size() - 2] = crc & 0xFF;
    packet[packet.size() - 1] = (crc >> 8) & 0xFF;
    
    return packet;
}

bool parse_byte(uint8_t byte) {
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

void execute_command(int fd, uint8_t command, uint8_t write_flag, const std::vector<uint8_t>& data = {}) {
    std::vector<uint8_t> packet = build_packet(command, write_flag, data);
    write(fd, packet.data(), packet.size());
    usleep(10000); 
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
// Main Application
// -----------------------------------------------------------------------------
int main() {
    signal(SIGINT, sigint_handler);
    std::cout << "Starting SF45/B Raw Point Cloud C++ Node..." << std::endl;

    int serial_fd = init_serial();
    if (serial_fd == -1) {
        std::cerr << "Failed to open serial port." << std::endl;
        return 1;
    }

    // Initialize Hardware
    execute_command(serial_fd, 30, 1, {0, 0, 0, 0}); 
    usleep(100000);
    tcflush(serial_fd, TCIFLUSH); 

    set_uint16_cmd(serial_fd, 85, SCAN_SPEED);
    execute_command(serial_fd, 66, 1, {UPDATE_RATE});
    set_float_cmd(serial_fd, 98, SCAN_LOW_DEG);
    set_float_cmd(serial_fd, 99, SCAN_HIGH_DEG);
    execute_command(serial_fd, 27, 1, {1, 1, 0, 0});
    execute_command(serial_fd, 30, 1, {5, 0, 0, 0}); 
    tcflush(serial_fd, TCIFLUSH);

    // Initialize UDP
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_HOST, &dest_addr.sin_addr);

    std::vector<RawPair> raw_pairs;
    bool in_scan = false;
    float prev_yaw = 999.0f;

    // --- Dynamic Pose State ---
    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float pose_theta = 0.0f;
    
    // Timer for Kinematics (Delta T)
    auto last_time = std::chrono::steady_clock::now();

    std::cout << "Streaming raw points... Press Ctrl+C to stop." << std::endl;

    uint8_t byte;
    while (keep_running) {
        if (read(serial_fd, &byte, 1) > 0) {
            
            if (parse_byte(byte)) {
                if (packet_data[3] == 44 && packet_data.size() >= 8) { 
                    int16_t dist_cm = packet_data[4] | (packet_data[5] << 8);
                    float distance_ft = dist_cm * 0.0328084f;

                    int16_t yaw_raw = packet_data[6] | (packet_data[7] << 8);
                    float yaw_deg = yaw_raw / 100.0f;

                    if (prev_yaw != 999.0f) {
                        bool crossed_start = (prev_yaw < (SCAN_LOW_DEG + 1.0f) && yaw_deg >= (SCAN_LOW_DEG + 1.0f));

                        if (crossed_start) {
                            if (in_scan && raw_pairs.size() > 50) { // Safety threshold to prevent empty sends
                                
                                // 1. Calculate Delta T
                                auto current_time = std::chrono::steady_clock::now();
                                std::chrono::duration<float> dt_duration = current_time - last_time;
                                float dt = dt_duration.count();
                                last_time = current_time;

                                // 2. FETCH VELOCITIES HERE (Replace with actual sensor/encoder data)
                                float v_x = 0.0f;      // Forward velocity (feet/sec)
                                float v_y = 0.0f;      // Lateral strafing velocity (feet/sec)
                                float yaw_rate = 0.0f; // Spin rate (rads/sec)

                                // 3. Update Global Pose (Local Frame Kinematics)
                                pose_x += (v_x * std::cos(pose_theta) - v_y * std::sin(pose_theta)) * dt;
                                pose_y += (v_x * std::sin(pose_theta) + v_y * std::cos(pose_theta)) * dt;
                                pose_theta += yaw_rate * dt;

                                // 4. Create Header
                                RawPacketHeader header;
                                header.x = pose_x;
                                header.y = pose_y;
                                header.theta = pose_theta;
                                header.num_points = raw_pairs.size();

                                // 5. Determine exact memory footprint
                                size_t packet_size = sizeof(RawPacketHeader) + (raw_pairs.size() * sizeof(RawPair));

                                // 6. Allocate and pack dynamic buffer
                                std::vector<uint8_t> send_buffer(packet_size);
                                std::memcpy(send_buffer.data(), &header, sizeof(RawPacketHeader));
                                std::memcpy(send_buffer.data() + sizeof(RawPacketHeader), raw_pairs.data(), raw_pairs.size() * sizeof(RawPair));
                                
                                // 7. Broadcast
                                sendto(sock, send_buffer.data(), packet_size, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                                
                                std::cout << "Sent raw sweep: " << raw_pairs.size() << " points (" << packet_size << " bytes)" << std::endl;
                            }
                            raw_pairs.clear();
                            in_scan = true;
                        }
                    }

                    if (in_scan && yaw_deg >= SCAN_LOW_DEG && yaw_deg <= SCAN_HIGH_DEG) {
                        raw_pairs.push_back({yaw_deg, distance_ft});
                    }
                    prev_yaw = yaw_deg;
                }
            }
        } else {
            ::usleep(1000);
        }
    }

    // -----------------------------------------------------------------------------
    // Safe Shutdown
    // -----------------------------------------------------------------------------
    std::cout << "\nInterrupt received. Stopping motor and closing ports..." << std::endl;
    
    set_uint16_cmd(serial_fd, 85, 0); 
    execute_command(serial_fd, 30, 1, {0, 0, 0, 0}); 
    
    close(serial_fd);
    close(sock);
    
    std::cout << "Successfully shutdown." << std::endl;
    return 0;
}