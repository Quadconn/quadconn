#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include "moteus.h"
#include "gamepad_data.hpp"

#include "iox2/iceoryx2.hpp"


// Test with low delay (seconds)
#define CMD_DELAY 1.0

// Convert radians to turns
double rad2turns(double radians) {
    return radians / (2.0 * M_PI);
}

double get_time() {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    using namespace mjbots;
    using namespace iox2;
    constexpr bb::Duration UPDATE_RATE = bb::Duration::from_millis(10);

    // Setup arguments and transport
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) { args.push_back(argv[i]); }
    moteus::Controller::DefaultArgProcess(args);
    // Raw transport to send batch
    auto transport = moteus::Controller::MakeSingletonTransport({});
    // Setup controllers
    std::vector<int> motor_ids = {1, 2, 3};
    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (int id : motor_ids) {
        moteus::Controller::Options options;
        options.id = id;
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    // Stop everything to clear faults first
    for (auto& c : controllers) { c->SetStop(); }

    /* START: BRACKET GUARD -- SUB VALUE */
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();
    auto service = node.service_builder(ServiceName::create("gamepad_data").value())
                    .publish_subscribe<gamepad_data>()
                    .open_or_create()
                    .value();
    auto subscriber = service.subscriber_builder().create().value();
    /* END: BRACKET GUARD -- SUB VALUE */


    // Prepare batch of commands
    std::vector<moteus::CanFdFrame> command_frames;
    std::vector<double> angles = {0.0, 0.0, 0.0};

    // Execution loop
    std::cout << "waiting for data\n";
    while (node.wait(UPDATE_RATE).has_value()) {
    // 1. Receive the Result
    auto receive_result = subscriber.receive();

    // 2. Handle IPC Errors (Fix for 'has_error')
    if (!receive_result.has_value()) {
        std::cerr << "IPC Error: " << static_cast<int>(receive_result.error()) << "\n";
        continue; 
    }

    // 3. Extract the Option (Move semantics!)
    // We use std::move to transfer ownership out of the result
    auto sample_opt = std::move(receive_result.value());

    // 4. Check if we actually received a sample (Fix for Deleted Copy)
    if (sample_opt.has_value()) {
        
        // Take a REFERENCE to the sample to avoid copying (Zero-Copy)
        const auto& sample = sample_opt.value();
        
        std::cout << "Received gamepad_values: A{" << sample.payload() << std::endl;

        // 5. Access Payload (Fix for '->')
        if (sample.payload().A) {
            std::cout << "Received A:" << sample.payload().A << "\n";
            angles = {2 * M_PI, 2 * M_PI, 2 * M_PI};
        } else if (sample.payload().B) {
            angles = {0, 0, 0};
            std::cout << "Received B:" << sample.payload().B << "\n";
        }
                // ... Motor control logic continues ...
        for (size_t i = 0; i < controllers.size(); ++i) {
            moteus::PositionMode::Command cmd;
            
            cmd.position = rad2turns(angles[i]);
            
            
            cmd.velocity = 0.0; 
            command_frames.push_back(controllers[i]->MakePosition(cmd));
        }

    }
    

    std::vector<moteus::CanFdFrame> replies;
    transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
        // ::usleep(10000); 
    }
        
    // End safely
    for (auto& c : controllers) { c->SetStop(); }
    return 0;
}

