#include "iox2/iceoryx2.hpp"
#include "motor_diagnostics.hpp"

int main(int argc, char** argv) {
    using namespace iox2;

    constexpr bb::Duration UPDATE_RATE = bb::Duration::from_secs(1);

    /* START: BRACKET GUARD -- INIT NODE */
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();
    
    auto s_service = node.service_builder(ServiceName::create("motor_diagnostics_array").value())
                    .publish_subscribe<motor_diagnostics_array>()
                    .open_or_create()
                    .value();
    auto subscriber = s_service.subscriber_builder().create().value();
    /* END: BRACKET GUARD -- INIT NODE */

    while(node.wait(UPDATE_RATE).has_value()) {
        
        /* START: USER CODE: */

        /* END: USER CODE */ 
        
        /* END: BRACKET GUARD -- SEND PUB VALUE */
        auto receive_result = subscriber.receive();
        if (!receive_result.has_value()) {
            std::cerr << "IPC Error: " << static_cast<int>(receive_result.error()) << "\n";
            continue; 
        }

        auto sample_opt = std::move(receive_result.value());
        // Check if we actually received a sample 
        if (sample_opt.has_value()) {
            const auto& sample = sample_opt.value();
            
            for (int i = 0; i < MOTOR_COUNT; i++) {
                std::cout << "Node " << i+1 << ": position is "<< sample.payload().motor_d[i].position << std::endl; 
                std::cout << "Node " << i+1 << ": voltage is " << sample.payload().motor_d[i].voltage  << std::endl; 
            }
            // Access Payload (Fix for '->')

        }
        /* END: BRACKET GUARD -- SEND PUB VALUE */
    }
    return 0;
}