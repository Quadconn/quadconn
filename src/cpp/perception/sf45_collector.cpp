// --------------------------------------------------------------------------------------------------------------
// LightWare SF45/B - Data collector for 2D SLAM
// All units are FEET throughout (distances).
// Sends each scan over UDP as {"range": [...]} - LiDAR data only.
// --------------------------------------------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <cstring>
#include <csignal>
#include <stdexcept>

// POSIX specific for Serial and UDP (Linux/macOS)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// --------------------------------------------------------------------------------------------------------------
// Configuration
// --------------------------------------------------------------------------------------------------------------
const std::string SERIAL_PORT = "/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-17174-if00";
const speed_t SERIAL_BAUD = B921600;

const int SCAN_SPEED = 5;
const int UPDATE_RATE = 12;

const float SCAN_LOW_DEG = -90.0f;
const float SCAN_HIGH_DEG = 90.0f;

const float LIDAR_MAX_RANGE_FT = 164.042f;
const float INVALID_SENTINEL_FT = LIDAR_MAX_RANGE_FT;

const int TARGET_SCAN_SIZE = 300;

const std::string UDP_HOST = "100.119.158.85";
const int UDP_PORT = 6000;

// --------------------------------------------------------------------------------------------------------------
// Global State & Ctrl-C Handling
// --------------------------------------------------------------------------------------------------------------
volatile sig_atomic_t running = 1;

void handle_sigint(int /*sig*/) {
    running = 0;
}

// --------------------------------------------------------------------------------------------------------------
// LWNX library functions
// --------------------------------------------------------------------------------------------------------------
uint16_t create_crc(const std::vector<uint8_t>& data) {
    uint16_t crc = 0;
    for (uint8_t i : data) {
        uint16_t code = crc >> 8;
        code ^= i;
        code ^= code >> 4;
        crc = crc << 8;
        crc ^= code;
        code = code << 5;
        crc ^= code;
        code = code << 7;
        crc ^= code;
        crc &= 0xFFFF;
    }
    return crc;
}

std::vector<uint8_t> build_packet(uint8_t command, uint8_t write_flag, const std::vector<uint8_t>& data = {}) {
    uint16_t payload_length = 1 + data.size();
    uint16_t flags = (payload_length << 6) | (write_flag & 0x1);
    
    std::vector<uint8_t> packet_bytes = {
        0xAA, 
        static_cast<uint8_t>(flags & 0xFF), 
        static_cast<uint8_t>((flags >> 8) & 0xFF), 
        command
    };
    packet_bytes.insert(packet_bytes.end(), data.begin(), data.end());
    
    uint16_t crc = create_crc(packet_bytes);
    packet_bytes.push_back(crc & 0xFF);
    packet_bytes.push_back((crc >> 8) & 0xFF);
    return packet_bytes;
}

// Wrap parser state in a struct to avoid dirty globals
struct LwnxParser {
    int state = 0;
    int payload_size = 0;
    int size = 0;
    std::vector<uint8_t> data;

    void reset() {
        state = 0;
        payload_size = 0;
        size = 0;
        data.clear();
    }

    bool parse_byte(uint8_t byte) {
        if (state == 0) {
            if (byte == 0xAA) {
                state = 1;
                data = {0xAA};
            }
        } else if (state == 1) {
            state = 2;
            data.push_back(byte);
        } else if (state == 2) {
            state = 3;
            data.push_back(byte);
            payload_size = (data[1] | (data[2] << 8)) >> 6;
            payload_size += 2;
            size = 3;
            if (payload_size > 1019) state = 0;
        } else if (state == 3) {
            data.push_back(byte);
            size++;
            payload_size--;
            if (payload_size == 0) {
                state = 0;
                uint16_t crc = data[size - 2] | (data[size - 1] << 8);
                std::vector<uint8_t> crc_data(data.begin(), data.end() - 2);
                uint16_t verify_crc = create_crc(crc_data);
                if (crc == verify_crc) {
                    return true;
                }
            }
        }
        return false;
    }
};

LwnxParser parser;

std::vector<uint8_t> wait_for_packet(int fd, uint8_t command, double timeout_sec = 1.0) {
    parser.reset();
    auto start = std::chrono::steady_clock::now();
    
    while (running) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start;
        if (elapsed.count() >= timeout_sec) return {};

        uint8_t b;
        int n = read(fd, &b, 1);
        if (n > 0) {
            if (parser.parse_byte(b)) {
                if (parser.data.size() > 3 && parser.data[3] == command) {
                    return parser.data;
                }
            }
        }
    }
    return {};
}

std::vector<uint8_t> execute_command(int fd, uint8_t command, uint8_t write_flag, const std::vector<uint8_t>& data = {}, double timeout_sec = 1.0) {
    auto packet = build_packet(command, write_flag, data);
    int retries = 4;
    
    while (retries > 0 && running) {
        retries--;
        write(fd, packet.data(), packet.size());
        auto response = wait_for_packet(fd, command, timeout_sec);
        if (!response.empty()) return response;
    }
    throw std::runtime_error("LWNX command failed to receive a response.");
}

// --------------------------------------------------------------------------------------------------------------
// SF45 API helper functions
// --------------------------------------------------------------------------------------------------------------
std::string get_str16_from_packet(const std::vector<uint8_t>& packet) {
    std::string str16 = "";
    for (int i = 0; i < 16; i++) {
        if (packet.size() <= static_cast<size_t>(4 + i) || packet[4 + i] == 0) break;
        str16 += static_cast<char>(packet[4 + i]);
    }
    return str16;
}

void print_product_information(int fd) {
    auto response = execute_command(fd, 0, 0, {}, 0.1);
    std::cout << "Product: " << get_str16_from_packet(response) << "\n";

    response = execute_command(fd, 2, 0, {}, 0.1);
    std::cout << "Firmware: " << (int)response[6] << "." << (int)response[5] << "." << (int)response[4] << "\n";

    response = execute_command(fd, 3, 0, {}, 0.1);
    std::cout << "Serial: " << get_str16_from_packet(response) << "\n";
}

template<typename T>
std::vector<uint8_t> pack_little_endian(T value) {
    std::vector<uint8_t> data(sizeof(T));
    std::memcpy(data.data(), &value, sizeof(T));
    return data;
}

void set_scan_speed(int fd, uint16_t speed) {
    execute_command(fd, 85, 1, pack_little_endian(speed));
}

void set_update_rate(int fd, uint8_t value) {
    if (value < 1 || value > 12) throw std::runtime_error("Invalid update rate value.");
    execute_command(fd, 66, 1, {value});
}

void set_default_scan_low_angle(int fd, float value) {
    if (value > -5.0f || value < -170.0f) throw std::runtime_error("Invalid lower bound for Scan angle.");
    execute_command(fd, 98, 1, pack_little_endian(value));
}

void set_default_scan_high_angle(int fd, float value) {
    if (value > 170.0f || value < 5.0f) throw std::runtime_error("Invalid high bound for Scan angle.");
    execute_command(fd, 99, 1, pack_little_endian(value));
}

void set_default_distance_output(int fd, bool /*use_last_return*/ = false) {
    execute_command(fd, 27, 1, {1, 1, 0, 0});
}

void set_distance_stream_enable(int fd, bool enable) {
    if (enable) execute_command(fd, 30, 1, {5, 0, 0, 0});
    else        execute_command(fd, 30, 1, {0, 0, 0, 0});
}

// Returns {distance_ft, yaw_deg}. Returns {-1.0f, 0.0f} on timeout.
std::pair<float, float> wait_for_reading(int fd, double timeout_sec = 1.0) {
    auto response = wait_for_packet(fd, 44, timeout_sec);

    if (response.empty() || response.size() < 8) {
        return {-1.0f, 0.0f};
    }

    uint16_t distance_cm = (response[4] << 0) | (response[5] << 8);
    float distance_ft = std::round(distance_cm * 0.0328084f * 1000.0f) / 1000.0f;

    int16_t yaw_raw = static_cast<int16_t>((response[6] << 0) | (response[7] << 8));
    float yaw_deg = yaw_raw / 100.0f;

    return {distance_ft, yaw_deg};
}

// --------------------------------------------------------------------------------------------------------------
// Serial and UDP Helpers
// --------------------------------------------------------------------------------------------------------------
int configure_serial(const std::string& port, speed_t baud) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) throw std::runtime_error("Failed to open serial port: " + port);

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) throw std::runtime_error("Error from tcgetattr");

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; 
    tty.c_lflag &= ~ECHOE; 
    tty.c_lflag &= ~ECHONL; 
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1; // 1 decisecond timeout
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) throw std::runtime_error("Error from tcsetattr");
    fcntl(fd, F_SETFL, 0); // Clear O_NDELAY
    return fd;
}

int udp_sock;
struct sockaddr_in udp_addr;

void setup_udp() {
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_HOST.c_str(), &udp_addr.sin_addr);
}

void send_scan_udp(const std::vector<float>& scan) {
    std::ostringstream oss;
    oss << "{\"range\": [";
    for (size_t i = 0; i < scan.size(); i++) {
        oss << scan[i];
        if (i < scan.size() - 1) oss << ", ";
    }
    oss << "]}";
    
    std::string payload = oss.str();
    sendto(udp_sock, payload.c_str(), payload.size(), 0, (struct sockaddr*)&udp_addr, sizeof(udp_addr));
    std::cout << "  UDP sent " << payload.size() << " bytes to " << UDP_HOST << ":" << UDP_PORT << "\n";
}

// --------------------------------------------------------------------------------------------------------------
// Scan helpers
// --------------------------------------------------------------------------------------------------------------
struct ScanPoint { float yaw; float distance; };

float clamp_range(float distance_ft) {
    if (distance_ft <= 0.0f || distance_ft >= LIDAR_MAX_RANGE_FT) {
        return INVALID_SENTINEL_FT;
    }
    return std::round(distance_ft * 1000.0f) / 1000.0f;
}

std::vector<float> resample_scan(std::vector<ScanPoint>& raw_pairs) {
    if (raw_pairs.empty()) {
        return std::vector<float>(TARGET_SCAN_SIZE, INVALID_SENTINEL_FT);
    }

    std::sort(raw_pairs.begin(), raw_pairs.end(), [](const ScanPoint& a, const ScanPoint& b) {
        return a.yaw < b.yaw;
    });

    float step = (SCAN_HIGH_DEG - SCAN_LOW_DEG) / (TARGET_SCAN_SIZE - 1);
    std::vector<float> grid;
    grid.reserve(TARGET_SCAN_SIZE);

    for (int i = 0; i < TARGET_SCAN_SIZE; i++) {
        float target_angle = SCAN_LOW_DEG + step * i;
        
        auto best_it = std::min_element(raw_pairs.begin(), raw_pairs.end(), 
            [target_angle](const ScanPoint& a, const ScanPoint& b) {
                return std::abs(a.yaw - target_angle) < std::abs(b.yaw - target_angle);
            }
        );

        if (std::abs(best_it->yaw - target_angle) <= step * 1.5f) {
            grid.push_back(clamp_range(best_it->distance));
        } else {
            grid.push_back(INVALID_SENTINEL_FT);
        }
    }
    return grid;
}

// --------------------------------------------------------------------------------------------------------------
// Main application
// --------------------------------------------------------------------------------------------------------------
int main() {
    std::signal(SIGINT, handle_sigint);

    std::cout << "Running SF45/B LWNX sample.\n";

    int fd;
    try {
        fd = configure_serial(SERIAL_PORT, SERIAL_BAUD);
    } catch (const std::exception& e) {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }

    setup_udp();

    try {
        set_distance_stream_enable(fd, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        tcflush(fd, TCIFLUSH);

        set_scan_speed(fd, SCAN_SPEED);
        print_product_information(fd);
        set_update_rate(fd, UPDATE_RATE);
        set_default_scan_low_angle(fd, SCAN_LOW_DEG);
        set_default_scan_high_angle(fd, SCAN_HIGH_DEG);
        set_default_distance_output(fd, false);
        set_distance_stream_enable(fd, true);

        tcflush(fd, TCIFLUSH);

        std::vector<ScanPoint> raw_pairs;
        bool in_scan = false;
        bool has_prev_yaw = false;
        float prev_yaw = 0.0f;

        std::cout << "Collecting scans | FOV=[" << SCAN_LOW_DEG << ", " << SCAN_HIGH_DEG << "] deg | "
                  << "bins=" << TARGET_SCAN_SIZE << " | max_range=" << LIDAR_MAX_RANGE_FT << " ft\n";
        std::cout << "Sending UDP -> " << UDP_HOST << ":" << UDP_PORT << "\n";
        std::cout << "Press Ctrl-C to stop.\n\n";

        while (running) {
            auto reading = wait_for_reading(fd);
            float distance_ft = reading.first;
            float yaw_deg = reading.second;

            if (distance_ft == -1.0f) {
                std::cout << "LIDAR Timeout\n\n";
                continue;
            }

            if (has_prev_yaw) {
                bool crossed_start = (prev_yaw < SCAN_LOW_DEG && SCAN_LOW_DEG <= yaw_deg);

                if (crossed_start) {
                    if (in_scan && raw_pairs.size() > TARGET_SCAN_SIZE / 4) {
                        auto scan = resample_scan(raw_pairs);
                        send_scan_udp(scan);
                        std::cout << "Sent scan: " << raw_pairs.size() << " raw -> " << TARGET_SCAN_SIZE << " bins\n";
                    }

                    raw_pairs.clear();
                    in_scan = true;
                }
            }

            if (in_scan && SCAN_LOW_DEG <= yaw_deg && yaw_deg <= SCAN_HIGH_DEG) {
                raw_pairs.push_back({yaw_deg, distance_ft});
            }

            prev_yaw = yaw_deg;
            has_prev_yaw = true;
        }

        // Cleanup block on Ctrl-C
        if (in_scan && !raw_pairs.empty()) {
            auto scan = resample_scan(raw_pairs);
            send_scan_udp(scan);
            std::cout << "Final partial scan sent: " << raw_pairs.size() << " raw samples.\n";
        }

    } catch (const std::exception& e) {
        std::cerr << "Runtime Error: " << e.what() << "\n";
    }

    // Graceful shutdown
    std::cout << "\nStopping LiDAR stream...\n";
    try {
        set_scan_speed(fd, 0);
        set_distance_stream_enable(fd, false);
    } catch (...) {} // Ignore errors on shutdown
    
    close(fd);
    close(udp_sock);
    std::cout << "Stopped.\n";

    return 0;
}