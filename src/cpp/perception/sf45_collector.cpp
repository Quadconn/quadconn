#include "quad_ipc.hpp"
#include "sf45_helpers.hpp"
// -----------------------------------------------------------------------------
// Signal Handling (Ctrl+C)
// -----------------------------------------------------------------------------
volatile sig_atomic_t keep_running = 1;

void sigint_handler(int dummy) {
    keep_running = 0;
}

// -----------------------------------------------------------------------------
// LWNX Protocol Globals & Functions
// -----------------------------------------------------------------------------
int parse_state = 0;
int payload_size = 0;
int packet_size = 0;
std::vector<uint8_t> packet_data;


// -----------------------------------------------------------------------------
// Main Application
// -----------------------------------------------------------------------------

int main() {
    signal(SIGINT, sigint_handler);
    std::cout << "Starting SF45/B Hardware Node (Raw Data Only)..." << std::endl;

    auto node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto service_name = iox2::ServiceName::create("Sensor/Lidar/SF45B_Raw").value();
    auto service = node.service_builder(service_name)
                       .publish_subscribe<LidarSweep>()
                       .open_or_create()
                       .value();

    auto publisher = service.publisher_builder().create().value();

    int serial_fd = init_serial(); // Make sure your init_serial function is defined above
    if (serial_fd == -1) return 1;

    // Hardware Initialization
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

    std::vector<RawPair> raw_pairs;
    bool in_scan = false;
    float prev_yaw = 999.0f;
    uint8_t byte;

    while (keep_running) {
            if (read(serial_fd, &byte, 1) > 0) {
                if (parse_byte(byte, parse_state, payload_size, packet_size, packet_data)) {
                    if (packet_data[3] == 44 && packet_data.size() >= 8) { 
                        int16_t dist_cm = packet_data[4] | (packet_data[5] << 8);
                        float distance_ft = dist_cm * 0.0328084f;
                        int16_t yaw_raw = packet_data[6] | (packet_data[7] << 8);
                        float yaw_deg = yaw_raw / 100.0f;

                        if (prev_yaw != 999.0f) {
                            bool crossed_start = (prev_yaw < (SCAN_LOW_DEG + 1.0f) && yaw_deg >= (SCAN_LOW_DEG + 1.0f));

                            if (crossed_start) {
                                if (in_scan && raw_pairs.size() > 50) { 
                                    // LOAN MEMORY AND PUBLISH RAW DATA ONLY
                                    auto sample_result = publisher.loan();
                                    if (sample_result.has_value()) {
                                        auto sample = std::move(sample_result.value());
                                        uint32_t count = std::min(static_cast<uint32_t>(raw_pairs.size()), MAX_POINTS_PER_SWEEP);
                                    // 1. Use .payload_mut() to access the struct (instead of ->)
                                    sample.payload_mut().num_points = count;

                                    // Directly copy serial data into shared memory
                                    std::memcpy(sample.payload_mut().points, raw_pairs.data(), count * sizeof(RawPair));
                                    
                                    // 2. Pass the sample into the Publisher's send method
                                    publisher.send(std::move(sample)).value();
                                    }
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

        set_uint16_cmd(serial_fd, 85, 0); 
        execute_command(serial_fd, 30, 1, {0, 0, 0, 0}); 
        close(serial_fd);
        return 0;
    }