#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>

#include "moteus.h"
#include "three_dof_theta.hpp"
#include "iox2/iceoryx2.hpp"


// Test with low delay (seconds)
#define CMD_DELAY 1.0

// Convert radians to turns
double rad2turns(double radians) {
    return 9*(radians / (2.0 * M_PI));
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



    // Prepare batch of commands
    std::vector<moteus::CanFdFrame> command_frames;
    std::vector<double> angles = {0.0, 0.0, 0.0};
    std::map<int, moteus::Query::Result> servo_data;

    // Execution loop
    std::cout << "starting motor loop\n";
    const auto start = get_time();
    while (node.wait(UPDATE_RATE).has_value()) { 
        command_frames.clear();
        // attempt to receive value every 10 ms
        auto receive_result = subscriber.receive();

        
        if (!receive_result.has_value()) {
            std::cerr << "IPC Error: " << static_cast<int>(receive_result.error()) << "\n";
            continue; 
        }

        auto sample_opt = std::move(receive_result.value());
        // Check if we actually received a sample 
        if (sample_opt.has_value()) {
            
            // Take a REFERENCE to the sample to avoid copying (Zero-Copy)
            const auto& sample = sample_opt.value();
            

            // Access Payload (Fix for '->')
            angles = {sample.payload().theta1, 
                      sample.payload().theta2,
                      sample.payload().theta3};
        }
                // encode & send to motor
        for (size_t i = 0; i < controllers.size(); ++i) {
            moteus::PositionMode::Command cmd;
            cmd.position = rad2turns(angles[i]);
            cmd.velocity = 2; // std::numeric_limits<double>::quiet_NaN(); 
            command_frames.push_back(controllers[i]->MakePosition(cmd));
        }
        std::vector<moteus::CanFdFrame> replies;
        transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
        for (const auto& frame : replies) {
        servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
        }

        for (const auto& pair : servo_data) {
            const auto r = pair.second;
            std::cout << "position: " << r.position << ", velocity: " << r.velocity 
                    << ", torque: " << r.torque << ", fault: " << r.fault
                    << ", voltage: " << r.voltage << ", mode: " << static_cast<int>(r.mode) 
                    << ", trajectory_completed: "<< r.trajectory_complete << "\n";
            
        }

        
        // ::usleep(10000); 
    }
        
    // End safely
    for (auto& c : controllers) { c->SetStop(); }
    return 0;
}

