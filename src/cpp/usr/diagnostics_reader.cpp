#include "iox2/iceoryx2.hpp"
#include "motor_diagnostics.hpp"

int main(int argc, char** argv) {
    using namespace iox2;

    constexpr bb::Duration UPDATE_RATE = bb::Duration::from_secs(1);

    /* START: BRACKET GUARD -- INIT NODE */
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();
    
    auto s_service = node.service_builder(ServiceName::create("MotorDiagnosticsArray").value())
                    .publish_subscribe<MotorDiagnosticsArray>()
                    .open_or_create()
                    .value();
    auto subscriber = s_service.subscriber_builder().create().value();
    /* END: BRACKET GUARD -- INIT NODE */

    while(node.wait(UPDATE_RATE).has_value()) {
        
        
        auto receive_result = subscriber.receive();
        if (!receive_result.has_value()) {
            std::cerr << "IPC Error: " << static_cast<int>(receive_result.error()) << "\n";
            continue; 
        }
        auto sample_opt = std::move(receive_result.value());

        // receive data (in sample.payload())
        if (sample_opt.has_value()) {
            const auto& sample = sample_opt.value();
            
            /* START: OPERATE ON DATA */
            for (int i = 0; i < MOTOR_COUNT; i++) {
                std::cout << "Node " << i+1 << ": position is "<< sample.payload().motor_instance[i].position << std::endl; 
                std::cout << "Node " << i+1 << ": voltage is " << sample.payload().motor_instance[i].voltage  << std::endl; 
            }
            /* END: OPERATE ON DATA */

        }
    }
    return 0;
}