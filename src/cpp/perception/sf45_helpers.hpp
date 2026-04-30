#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <netinet/in.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
const char* const SERIAL_PORT = "/dev/serial/by-id/usb-LightWare_Optoelectronics_lwnx_device_38S45-17174-if00";
const int SERIAL_BAUD = 921600;

const int SCAN_SPEED = 5;
const int UPDATE_RATE = 12;

const float SCAN_LOW_DEG = -90.0f;
const float SCAN_HIGH_DEG = 90.0f;

const float LIDAR_MAX_RANGE_FT = 164.042f;
const float INVALID_SENTINEL_FT = LIDAR_MAX_RANGE_FT;

const int TARGET_SCAN_SIZE = 300;

const char* const UDP_HOST = "100.119.158.85";
const int UDP_PORT = 6000;

// -----------------------------------------------------------------------------
// Class Definition
// -----------------------------------------------------------------------------
class SF45Collector {
public:
    SF45Collector();
    ~SF45Collector();

    bool init();
    void run();
    void stop();

private:
    int serial_fd_;
    int udp_sock_;
    struct sockaddr_in udp_addr_;
    bool running_;

    // Parsing State
    int parse_state_;
    int payload_size_;
    int packet_size_;
    std::vector<uint8_t> packet_data_;

    // LWNX Protocol
    uint16_t create_crc(const std::vector<uint8_t>& data);
    std::vector<uint8_t> build_packet(uint8_t command, bool write, const std::vector<uint8_t>& data);
    bool parse_packet_byte(uint8_t byte);
    std::vector<uint8_t> wait_for_packet(uint8_t command, double timeout_sec = 1.0);
    std::vector<uint8_t> execute_command(uint8_t command, bool write_flag, const std::vector<uint8_t>& data = {}, double timeout_sec = 1.0);

    // SF45 API Helpers
    std::string get_str16_from_packet(const std::vector<uint8_t>& packet);
    void print_product_information();
    void set_scan_speed(uint16_t speed);
    void set_update_rate(uint8_t rate);
    void set_default_scan_low_angle(float angle);
    void set_default_scan_high_angle(float angle);
    void set_default_distance_output(bool use_last_return);
    void set_distance_stream_enable(bool enable);
    bool wait_for_reading(float& distance_ft, float& yaw_deg, double timeout_sec = 1.0);

    // Processing & Networking
    float clamp_range(float distance_ft);
    std::vector<float> resample_scan(std::vector<std::pair<float, float>>& raw_pairs);
    void send_scan_udp(const std::vector<float>& scan);
};