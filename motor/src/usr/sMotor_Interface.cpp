#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include "moteus.h"
#include "three_dof_theta.hpp"

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
    auto service = node.service_builder(ServiceName::create("three_dof_theta").value())
                    .publish_subscribe<three_dof_theta>()
                    .open_or_create()
                    .value();
    auto subscriber = service.subscriber_builder().create().value();
    /* END: BRACKET GUARD -- SUB VALUE */



    // Execution loop
    std::cout << "waiting for data\n";
    while (node.wait(UPDATE_RATE).has_value()) {
        
        // Prepare batch of commands
        std::vector<moteus::CanFdFrame> command_frames;
        std::vector<double> angles = {NAN, NAN, NAN};

        // pull value from nodes

        auto sample = subscriber.receive().value();
        if (sample.has_value()) {
            std::cout << "Received Thetas:" << sample->payload() << std::endl;
            const auto& values = sample;
            angles[0] = values->payload().theta1;
            angles[1] = values->payload().theta2;
            angles[2] = values->payload().theta3;
        }
        for (size_t i = 0; i < controllers.size(); ++i) {
            moteus::PositionMode::Command cmd;
            cmd.position = rad2turns(angles[i]);  // Grab from 3D vector
            cmd.velocity = 0.0;                        // Or calculated velocity
            
            command_frames.push_back(controllers[i]->MakePosition(cmd));
        }
        // Send batch
        std::vector<moteus::CanFdFrame> replies;
        transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
        // ::usleep(10000); 
    }
        
    // End safely
    for (auto& c : controllers) { c->SetStop(); }
    return 0;
}

