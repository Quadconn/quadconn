#include <iostream>
#include <unistd.h>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <map>

#include "moteus.h"
#include "three_dof_theta.hpp"
#include "motor_diagnostics.hpp"
#include "iox2/iceoryx2.hpp"



// Convert radians to turns
double rad2turns(double radians) {
    return 9*(radians / (2.0 * M_PI));
}

int main(int argc, char** argv) {
    using namespace mjbots;
    using namespace iox2;
    // how often the loop pools
    constexpr bb::Duration UPDATE_RATE = bb::Duration::from_millis(10);

    // Setup arguments and transport
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) { args.push_back(argv[i]); }
    moteus::Controller::DefaultArgProcess(args);
    // Raw transport to send batch

    // list fdcanusbs by id (9C92 tied to 3, 1889 tied to 4)
    auto transport = moteus::Controller::MakeSingletonTransport({});
    
    // mapping CAN-IDs to FD-CAN
    std::map<int, std::shared_ptr<mjbots::moteus::Fdcanusb>> id_can_mapping
        {   {4, std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00")},
            {3, std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00")}
         };

    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (auto& pair : id_can_mapping) {
        moteus::Controller::Options options{};
        options.id = pair.first;
        options.transport = pair.second;
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
    // pubbing motor diagnostics
    
    auto p_service = node.service_builder(ServiceName::create("motor_diagnostics_array").value())
                    .publish_subscribe<motor_diagnostics_array>()
                    .open_or_create()
                    .value();
    auto publisher = p_service.publisher_builder().create().value();
    /* END: BRACKET GUARD -- SUB VALUE */



    // Prepare batch of commands
    std::vector<moteus::CanFdFrame> command_frames;
    std::vector<double> angles = {0.0, 0.0, 0.0};
    std::map<int, moteus::Query::Result> servo_data;
    std::map<std::shared_ptr<moteus::Fdcanusb>, std::vector<moteus::CanFdFrame>> transport_batches;

    // Execution loop
    std::cout << "starting motor loop\n";

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
        // Instead of a single transport->BlockingCycle call:

        // 1. Group your frames by their specific transport
        std::map<std::shared_ptr<moteus::Fdcanusb>, std::vector<moteus::CanFdFrame>> transport_batches;

        for (size_t i = 0; i < controllers.size(); ++i) {
            moteus::PositionMode::Command cmd;
            cmd.position = rad2turns(angles[i]);
            cmd.velocity = std::numeric_limits<double>::quiet_NaN();
            command_frames.push_back(controllers[i]->MakePosition(cmd));
            std::cout << "sending value: " << cmd.position << " to node " << controllers[i]->options().id << std::endl;
            // ... setup cmd ...
            
            // Use the transport associated with THIS controller
            auto& controller_transport = id_can_mapping[controllers[i]->options().id];
            transport_batches[controller_transport].push_back(controllers[i]->MakePosition(cmd));
        }

        std::vector<moteus::CanFdFrame> replies;
        // 2. Execute a cycle for EACH transport
        for (auto& [bus, frames] : transport_batches) {
            bus->BlockingCycle(&frames[0], frames.size(), &replies);
            
            for (const auto& frame : replies) {
                servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
            }
        }   


        // Fill a diagnostics array at runtime (zero-initialized)
        motor_diagnostics_array diags{}; // all entries zeroed
        int index = 0;
        for (const auto& pair : servo_data) {
            const auto& r = pair.second;
            if (index >= 12) break; // safety: array has 12 entries
            diags.motor_d[index] = make_diag(r);
            ++index;
        }

        // Publish the full diagnostics array once per loop
        auto sample = publisher.loan_uninit().value();
        auto init_sample = sample.write_payload(diags);
        send(std::move(init_sample)).value();

        
        // ::usleep(10000); 
    }
        
    // End safely
    for (auto& c : controllers) { c->SetStop(); }
    return 0;
}

