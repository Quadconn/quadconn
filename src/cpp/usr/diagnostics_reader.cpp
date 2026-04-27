#include "iox2/iceoryx2.hpp"
#include "motor_diagnostics.hpp"
#include <iostream>

int main(int argc, char** argv) {
    using namespace iox2;

    // Create the Node
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();

    // Open the existing MotorDiagnosticsArray service
    auto s_service = node.service_builder(ServiceName::create("MotorDiagnosticsArray").value())
                    .publish_subscribe<MotorDiagnosticsArray>()
                    .open_or_create()
                    .value();

    auto subscriber = s_service.subscriber_builder().create().value();

    std::cout << "Listening for Motor Diagnostics..." << std::endl;

    while (true) {
        auto receive_result = subscriber.receive();
        
        if (receive_result.has_value()) {
            auto& sample_opt = receive_result.value();
            
            if (sample_opt.has_value()) {
                auto& sample = sample_opt.value();
                
                std::cout << "\n--- New Telemetry Frame ---" << std::endl;
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    const auto& m = sample.payload().motor_instance[i];
                    std::cout << "Motor " << i+1 
                              << " | Pos: " << m.position 
                              << " | Volt: " << m.voltage << "V" << std::endl; 
                }
            }
        } else {
            std::cerr << "Error receiving sample." << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}