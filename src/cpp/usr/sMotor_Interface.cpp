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
    
    // change usb-id depending on motor id used, map motor controller node ids to bus
    auto bus_a = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_188998B3-if00");
    auto bus_b = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_9C92C905-if00");
    std::map<int, std::shared_ptr<moteus::Fdcanusb>> id_to_bus = {
        {4, std::move(bus_a)},
        {3, std::move(bus_b)}
    };

    // Initialize Controllers
    std::vector<std::shared_ptr<moteus::Controller>> controllers;
    for (auto const& [id, bus] : id_to_bus) {
        moteus::Controller::Options options{};
        options.id = id;
        options.transport = bus; 
        controllers.push_back(std::make_shared<moteus::Controller>(options));
    }

    // Stop everything to clear faults first
    for (auto& c : controllers) { c->SetStop(); }

    /* START: BRACKET GUARD -- SUB VALUE */
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();
    auto service = node.service_builder(ServiceName::create("three_dof_theta").value())
                    .publish_subscribe<ThreeDoFTheta>()
                    .open_or_create()
                    .value();
    auto subscriber = service.subscriber_builder().create().value();
    // publishing motor diagnostics
    auto p_service = node.service_builder(ServiceName::create("motor_diagnostics_array").value())
                    .publish_subscribe<MotorDiagnosticsArray>()
                    .open_or_create()
                    .value();
    auto publisher = p_service.publisher_builder().create().value();
    /* END: BRACKET GUARD -- SUB VALUE */



    // Prepare batch of commands
    std::vector<double> target_angles = {0.0, 0.0, 0.0};
    std::map<int, moteus::Query::Result> servo_data;
    std::map<std::shared_ptr<moteus::Fdcanusb>, std::vector<moteus::CanFdFrame>> bus_cmd;

    // Execution loop
    std::cout << "starting motor loop\n";
    
    // attempt to receive value every 10 ms
    while (node.wait(UPDATE_RATE).has_value()) { 
        
        // clear on new loop
        servo_data.clear();
        bus_cmd.clear();

        auto receive_result = subscriber.receive();
        if (receive_result.has_value()) {
            auto sample_opt = std::move(receive_result.value());
            if (sample_opt.has_value()) {
                const auto& p = sample_opt.value().payload();
                target_angles = {p.theta1, p.theta2, p.theta3};
            }
        }

        //  
        
        


        for (size_t i = 0; i < controllers.size(); ++i) {
            moteus::PositionMode::Command cmd;
            cmd.position = rad2turns(target_angles[i]);
            cmd.velocity = std::numeric_limits<double>::quiet_NaN(); 
            auto frame = controllers[i]->MakePosition(cmd);
            // first holds value of transport path, second holds value of command frame
            bus_cmd[id_to_bus[controllers[i]->options().id]].push_back(frame);
        }

        std::vector<moteus::CanFdFrame> replies;
        //  Execute a cycle for EACH transport
        for (auto& [bus, frames] : bus_cmd) {
            bus->BlockingCycle(&frames[0], frames.size(), &replies);
            
            for (const auto& frame : replies) {
                servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
            }
        }   


        MotorDiagnosticsArray diags{};
        for (auto const& [id, result] : servo_data) {
            int target_idx = id - 1;


            if (target_idx >= 0 && target_idx < MOTOR_COUNT) {
                diags.motor_instance[target_idx] = motor_info::make_diag(result);
            }
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

